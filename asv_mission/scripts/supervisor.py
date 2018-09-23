#!/usr/bin/python
import rospy
from asv_messages.msg import Plan, Task, Float64Stamped
from asv_messages.srv import RequestPlan
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Quaternion, PoseStamped, Pose, Vector3
from topic_tools.srv import MuxSelect
from missionplanning import hierarchy
from tf_conversions import transformations as tf
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from pygeodesy import utm
from scipy.stats import norm
from math import sqrt

class Supervisor:
    def __init__(self):
        self.mode = "idle"
        self.plan = None
        self.hptask = False
        self.hptime = 0.0
        self._energy_measured_sigma = rospy.get_param('~power_module_std',0.634184) # the standard deviation of the power module
        self._task_soft_limit = rospy.get_param('~task_soft_limit',0.5) # the limit for the task survival function before replan is considered.
        self._task_hard_limit = rospy.get_param('~task_hard_limit',0.4) # the limit for a task when a replan is triggered.
        self._plan_soft_limit = rospy.get_param('~plan_soft_limit',0.5) # the limit for the plan survival function when a replan is triggered.
        self._plan_hard_limit = rospy.get_param('~plan_hard_limit',0.4) # the limit for the vehicle survival function.
        self._vehicle_soft_limit = rospy.get_param('~vehicle_soft_limit',0.55) # the cutoff for the vehicle before considering survival options.
        self._battery_capacity = rospy.get_param('~battery_capacity',7*12*3600) # the energy capacity of the vehicle's battery system.
        self._task_nd = None
        self._plan_nd = None
        self._energy_measured_mu = None
        self._completed_mu = []
        self._completed_std = []
        rospy.init_node("supervisor")
        self.task_pub = rospy.Publisher('guidance/task',GeoPoseStamped,queue_size=10)
        self.status_pub = rospy.Publisher("asv/status",String,queue_size=10)
        self.marker_pub = rospy.Publisher("mission/markers",MarkerArray,queue_size=10)
        self.pause = rospy.Service('supervisor/pause', SetBool, self.pausePlayCallback)
        self.start = rospy.Service('supervisor/start',Trigger,self.startPlanCallback)
        self.nexttask = rospy.Service('supervisor/request_new', Trigger, self.nextTaskCallback)
        
        rospy.Subscriber("energy/aggregate",Float64Stamped,self.energySampleCallback)
        rospy.Subscriber("mission/plan",Plan,self.receivePlanCallback)
        rospy.Subscriber("guidance/operator",PoseStamped,self.receiveOperatorCallback)
        rospy.Subscriber("joy",Joy,self.receiveManualCallback)
        rospy.spin()
    
    def energySampleCallback(self,msg):
        # UPDATE THE ENERGY USED FOR THE CURRENT TASK
        self._energy_measured_mu = msg.data
        if not self._task_nd is None and not self._plan_nd is None:
            # GET THE SURVIVAL FUNCTION OF THE TASK
            if not self._task_nd is None:
                # The intersection probability density function is
                intersection_mu = self.intersection_mean(self._task_nd.mean(),msg.data,self._task_nd.var(),self._energy_measured_sigma**2)
                survival_task = self._task_nd.sf(intersection_mu)

            # GET THE SURVIVAL FUNCTION OF THE PLAN
            if not self._plan_nd is None:
                # accumulate the completed task energies (+ the current)
                depleted_mu = sum(self._completed_mu)+self._energy_measured_mu
                depleted_var = sum([x**2 for x in self._completed_std])+self._energy_measured_sigma
                # have to sum up all of the tasks expected energies that have yet to be done in the plan
                tasks_todo = [self.plan.plan[i] for i in self.plan._todo]
                todo_mu = sum([x.cost_mu for x in tasks_todo if x.action!='ROOT'])
                todo_var = sum([x.cost_std**2 for x in tasks_todo if x.action!='ROOT'])
                # sum the depleted mean with the todo mean, var with var
                mixed_mu = todo_mu+depleted_mu
                mixed_var = depleted_var+todo_var
                intersection_mu = self.intersection_mean(self._plan_nd.mean(),mixed_mu,self._plan_nd.var(),mixed_var)
                survival_plan = self._plan_nd.sf(intersection_mu)
                survival_vehicle = norm.sf(self._battery_capacity,loc=mixed_mu,scale=sqrt(mixed_var))

            replan_flag = False
            # ASSESS THE SURVIVAL FUNCTIONS OF THE VEHICLE, PLAN, AND TASK IN THAT PRIORITY
            # IF THE VEHICLE IS STILL SAFE
            if survival_vehicle > self._vehicle_soft_limit:
                # VEHICLE IS FINE, MOVE ON TO PLAN
                pass
            else:
                # VEHICLE IN DANGER ZONE, MOVE ON TO EMERGENCY RECOURSES
                rospy.logwarn("Vehicle in danger zone, returning home.")
                pass
            if survival_plan > self._plan_hard_limit:
                # PLAN HASN'T FAILED YET
                if survival_plan > self._plan_soft_limit:
                    # PLAN IS STILL GOOD
                    pass
                else:
                    # PLAN IS APPROACHING FAILURE, TIGHTEN TASK CONSTRAINTS
                    tmp = self._task_soft_limit+0.1
                    self._task_soft_limit = max(0.55,tmp)
            else:
                # PLAN HAS FAILED, TRIGGER REPLAN.
                rospy.logwarn("Plan has failed, triggering replan.")
                pass
            if survival_task > self._task_hard_limit:
                # TASK HASN'T FAILED YET
                if survival_task > self._task_soft_limit:
                    # TASK IS STILL GOOD
                    pass
                else:
                    # TASK IS APPROACHING FAILURE, LOG A WARNING
                    rospy.logwarn("Task is approaching failure.")
            else:
                # TASK HAS FAILED, TRIGGER REPLAN.
                rospy.logwarn("Task has failed, trigerring replan.")
            formatstring = 3*"| {} \t|\t"
            rospy.loginfo(formatstring.format(survival_vehicle,survival_plan,survival_task))       

    def intersection_mean(self,mu1,mu2,var1,var2):
        return var2*mu1/(var1+var2)+var1*mu2/(var1+var2)

    def receiveManualCallback(self,joy_msg):
        if self.mode=="manual":
            return
        else:
            self.mode="manual"
            rospy.logwarn("Switching from {} to manual override.".format(self.mode))
            self.status_pub.publish("ASV currently in mode: {}".format(self.mode))
            try:
                rospy.wait_for_service('control_topic_mux/select',5.0)
            except Exception as exc:
                rospy.logerr(exc)
                return
            service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
            response=service_handle("tau_com/override")
            rospy.logdebug("{}: Switching to manual override, now publishing {} from {}".format(rospy.get_name(),rospy.resolve_name("override"),response.prev_topic))

    def receiveOperatorCallback(self,geo_msg):
        rospy.logwarn("Switching from {} to operator override.".format(self.mode))
        self.mode="operator"
        self.status_pub.publish("ASV currently in mode: {}".format(self.mode))
        try:
            rospy.wait_for_service('control_topic_mux/select',5.0)
        except Exception as exc:
            rospy.logerr(exc)
            return
        service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
        response=service_handle("tau_com/AP")
        rospy.logdebug("{}: Switching to operator override, now publishing {} from {}".format(rospy.get_name(),rospy.resolve_name("AP"),response.prev_topic))

    def parseTask(self,task):
        geo_msg = GeoPoseStamped()
        geo_msg.header.frame_id="utm"
        geo_msg.header.stamp = rospy.Time.now()
        self.checkValidTask(task)
        if task.action == "WP":
            # waypoint task, configure for AP mode to a GPS waypoint.
            if self.changeMode("/tau_com/AP"):
                geo_msg.pose.position.altitude=-1
                geo_msg.pose.position.latitude=task.data[0]
                geo_msg.pose.position.longitude=task.data[1]
                geo_msg.pose.orientation = Quaternion(0,0,0,1)
                self.task_pub.publish(geo_msg)
                return [True,"Waypoint Task Selected, executing."]
            else:
                return [False,"Mode not configured to AP."]
        elif task.action == "HP":
            # hold position task, configure DP mode at a GPS waypoint and orientation for a set time.
            if self.changeMode("/tau_com/DP"):
                geo_msg.pose.position.altitude=0
                geo_msg.pose.position.latitude=task.data[0]
                geo_msg.pose.position.longitude=task.data[1]
                geo_msg.pose.orientation = Quaternion(*tf.quaternion_from_euler(0,0,task.data[2]))
                self.hptime = task.data[3]
                self.hptask = True
                self.task_pub.publish(geo_msg)
                return [True,"Hold Pose Task Selected, executing."]
            else:
                return [False,"Mode not configured to DP."]
        elif task.action == "ROOT":
            # root task reached, notify the operator that the mission is complete.
            if self.changeMode("__none"):
                self.mode="idle"
                self.status_pub.publish("ASV currently in mode: {}".format(self.mode))
                rospy.loginfo("Final task completed, now idling.")
                return [True,"Final task completed, now idling."]
            else:
                rospy.logerr("Couldn't disable output topic, but mission complete.")
                return [False,"Couldn't Idle for some reason?"]
                
        elif task.action == "START":
            # start task, is a waypoint task to move the vehicle to the starting position.
            if self.changeMode("/tau_com/AP"):
                geo_msg.pose.position.altitude=-1
                geo_msg.pose.position.latitude=task.data[0]
                geo_msg.pose.position.longitude=task.data[1]
                geo_msg.pose.orientation = Quaternion(0,0,0,1)
                self.task_pub.publish(geo_msg)
                rospy.loginfo("Moving to starting waypoint of plan.")
                return [True,"AP mode to starting point."]
            else:
                rospy.logerr("Couldn't configure to AP mode.")
                return [False,"Couldn't configure to AP mode."]
        elif task.action == "HOME":
            # start task, is a waypoint task to move the vehicle to the starting position.
            if self.changeMode("/tau_com/AP"):
                geo_msg.pose.position.altitude=-1
                geo_msg.pose.position.latitude=task.data[0]
                geo_msg.pose.position.longitude=task.data[1]
                geo_msg.pose.orientation = Quaternion(0,0,0,1)
                self.task_pub.publish(geo_msg)
                rospy.loginfo("Moving to rendezvous point.")
                return [True,"AP mode to home point."]
            else:
                rospy.logerr("Couldn't configure to AP mode.")
                return [False,"Couldn't configure to AP mode."]
        else:
            rospy.logerr("Task of type {} is not supported, getting next task.".format(task.action))
            self.plan.taskDone()
            return self.parseTask(self.plan.getTask())

    def changeMode(self,request):
        try:
            rospy.wait_for_service("control_topic_mux/select",5.0)
        except:
            rospy.logerr("No control_topic_mux/select service found, is controlmanager.py running?")
            return False
        service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
        service_handle(request)
        return True

    def receivePlanCallback(self,plan_msg):
        # parse the received Plan message
        self.plan = hierarchy.Hierarchy(plan_msg)
        self.home_task = self.plan.home
        root_task = self.plan.plan[self.plan._root]
        self._plan_nd = norm(root_task.cost_mu,root_task.cost_std)
        self.updateMissionMarkers()
        rospy.loginfo("Plan received, waiting for start mission service on {}".format(rospy.resolve_name("supervisor/start")))

    def updateMissionMarkers(self):
        array_msg = MarkerArray()
        array_msg.markers = []
        for idx,task in enumerate(self.plan.plan):
            marker_msg = Marker()
            marker_msg.header.stamp = rospy.Time.now()
            marker_msg.header.frame_id = "utm"
            if task.action=="WP" or task.action=="HOME":
                marker_msg.ns = "mission/markers"
                marker_msg.id = idx
                marker_msg.type = 2
                marker_msg.action = 0
                UTM = utm.toUtm(task.data[0],task.data[1])
                marker_msg.pose = Pose(Vector3(UTM.easting,UTM.northing,0),Quaternion(0,0,0,1))
                marker_msg.scale = Vector3(5,5,5)
                if idx in self.plan._complete:
                    marker_msg.color = ColorRGBA(0,1,0,0.8) #GREEN
                else:
                    marker_msg.color = ColorRGBA(1,0,0,0.8) #RED
                array_msg.markers.append(marker_msg)
        self.marker_pub.publish(array_msg)

    def nextTaskCallback(self,request):
        # new task requested from the guidance node.
        if self.hptask:
            rospy.loginfo("Pose reached, holding for {} seconds...".format(self.hptime))
            rospy.sleep(rospy.Duration(self.hptime))
            self.hptask = False
        self.plan.taskDone()
        self._completed_mu.append(self._energy_measured_mu)
        self._completed_std.append(self._energy_measured_sigma)
        self.updateMissionMarkers()
        rospy.loginfo("Task done, getting next...")
        return self.parseTask(self.plan.getTask())

    def pausePlayCallback(self,request):
        # if we want to start/resume the mission
        if not request.data:
            if self.plan is None:
                response = [False,"No Mission to resume"]
            else:
                self.mode="mission"
                self.status_pub.publish("ASV currently in mode: {}".format(self.mode))
                # put command in here to unpause the mission on the guidance side
                try:
                    rospy.wait_for_service("guidance/resume",4.0)
                except:
                    response = [False, "No guidance/resume service"]
                else:
                    service_handle = rospy.ServiceProxy("guidance/resume",Trigger)
                    service_handle()
                    response = self.parseTask(self.plan.getTask())
        # else we want to stop the mission
        else:
            self.mode="idle"
            self.status_pub.publish("ASV currently in mode: {}".format(self.mode))
            try:
                rospy.wait_for_service("control_topic_mux/select",4.0)
            except:
                response = [False, "No control_topic_mux/select service"]
            else:
                service_handle = rospy.ServiceProxy("control_topic_mux/select",MuxSelect)
                service_handle("__none")
                response = [True,"Mission paused."]
        return response

    def checkValidTask(self,task_msg):
        if task_msg.cost_mu > 0:
            self._task_nd = norm(task_msg.cost_mu,task_msg.cost_std)
        else:
            self._task_nd = None

    def resetEnergy(self):
        try:
            rospy.wait_for_service("energy/reset_aggregator",4.0)
        except:
            return False
        else:
            service_handle = rospy.ServiceProxy("energy/reset_aggregator",Trigger)
            success,_ = service_handle()
        return success

    def startPlanCallback(self,request):
        if self.plan is None:
            return [False,"No mission loaded"]
        else:
            task_msg = self.plan.getTask()
            self.mode="mission"
            self.status_pub.publish("ASV currently in mode: {}".format(self.mode))
            try:
                rospy.wait_for_service("guidance/resume",4.0)
            except:
                response = [False, "No guidance/resume service"]
            else:
                service_handle = rospy.ServiceProxy("guidance/resume",Trigger)
                response = service_handle()
            response = self.parseTask(task_msg)
            if response[0]:
                return [True,"Mission starting: {}".format(response[1])]
            else:
                return [False, "Mission failed to start: {}".format(response[1])]

if __name__=="__main__":
    try:
        supervisor = Supervisor()
    except rospy.ROSInterruptException:
        pass
