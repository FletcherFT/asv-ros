#!/usr/bin/python
import rospy
from asv_messages.msg import Plan, Task, Float64Stamped, Readings
from asv_messages.srv import PlanService, PlanServiceRequest, UTMService
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String, Bool, Int8
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
        self._inrange = True
        self._energy_measured_sigma = rospy.get_param('~power_module_std',0.634184) # the standard deviation of the power module
        self._task_soft_limit = rospy.get_param('~task_soft_limit',0.5) # the limit for the task survival function before replan is considered.
        self._task_hard_limit = rospy.get_param('~task_hard_limit',0.4) # the limit for a task when a replan is triggered.
        self._plan_soft_limit = rospy.get_param('~plan_soft_limit',0.5) # the limit for the plan survival function when a replan is triggered.
        self._plan_hard_limit = rospy.get_param('~plan_hard_limit',0.4) # the limit for the vehicle survival function.
        self._vehicle_soft_limit = rospy.get_param('~vehicle_soft_limit',0.55) # the cutoff for the vehicle before considering survival options.
        self._battery_capacity = rospy.get_param('~battery_capacity',7*12*3600) # the energy capacity of the vehicle's battery system.
        self._battery_variance = rospy.get_param('~battery_variance',0.25*self._battery_capacity) # variance of the battery
        self._vehicle_nd = norm(self._battery_capacity,sqrt(self._battery_variance))
        self._task_nd = None
        self._plan_nd = None
        self._energy_measured_mu = 0
        self._energy_measured_std = self._energy_measured_sigma
        self._energy_measured_total = 0
        self._completed_mu = []
        self._completed_std = []
        self._reset_flag = False
        self._energy_datum = 0
        self._replan_flag = False
        self.aware = False
        rospy.init_node("supervisor")
        self.survival_pub = rospy.Publisher('supervisor/survival',Readings,queue_size=10)
        self.recourse_pub = rospy.Publisher('supervisor/recourse',Int8,queue_size=10)
        self.task_pub = rospy.Publisher('guidance/task',GeoPoseStamped,queue_size=10)
        self.status_pub = rospy.Publisher("asv/status",String,queue_size=10)
        self.marker_pub = rospy.Publisher("mission/markers",MarkerArray,queue_size=10)
        self.pause = rospy.Service('supervisor/pause', SetBool, self.pausePlayCallback)
        self.start = rospy.Service('supervisor/start',Trigger,self.startPlanCallback)
        self.nexttask = rospy.Service('supervisor/request_new', Trigger, self.nextTaskCallback)

        rospy.Subscriber("failsafe/inrange",Bool,self.rangeStateCallback)
        rospy.Subscriber("energy/aggregate",Float64Stamped,self.energySampleCallback)
        rospy.Subscriber("mission/plan",Plan,self.receivePlanCallback)
        rospy.Subscriber("guidance/operator",PoseStamped,self.receiveOperatorCallback)
        rospy.Subscriber("joy",Joy,self.receiveManualCallback)
        rospy.spin()
    
    def rangeStateCallback(self,msg):
        self._inrange = msg.data

    def energySampleCallback(self,msg):
        # UPDATE THE ENERGY USED FOR THE CURRENT TASK
        if self.mode == "mission":
            self._energy_measured_total = msg.data
            self._energy_measured_mu = self._energy_measured_total-self._energy_datum
            self._energy_measured_std += self._energy_measured_sigma

            if not self._task_nd is None and not self._plan_nd is None:
                # GET THE SURVIVAL FUNCTION OF THE TASK
                # The intersection probability density function is
                intersection_mu = self.intersection_mean(self._task_nd.mean(),self._energy_measured_mu,self._task_nd.var(),self._energy_measured_std**2)
                rospy.logdebug("Task intersection mu: {}".format(intersection_mu))
                survival_task = self._task_nd.sf(intersection_mu)

                # GET THE SURVIVAL FUNCTION OF THE PLAN
                # accumulate the completed task energies (+ the current)
                depleted_mu = sum(self._completed_mu)+self._energy_measured_mu
                depleted_var = sum([x**2 for x in self._completed_std])+self._energy_measured_sigma
                rospy.logdebug("Depleted mu: {}\t Depleted var: {}".format(depleted_mu,depleted_var))
                # have to sum up all of the tasks expected energies that have yet to be done in the plan (excluding the current one)
                tasks_todo = [self.plan.plan[i] for idx,i in enumerate(self.plan._todo) if self.plan._todo[idx]!=self.plan._current]
                todo_mu = sum([x.cost_mu for x in tasks_todo if x.action!='ROOT'])
                todo_var = sum([x.cost_std**2 for x in tasks_todo if x.action!='ROOT'])
                rospy.logdebug("Remaining mu: {}\t Remaining var: {}".format(todo_mu,todo_var))
                # sum the depleted mean with the todo mean, var with var
                hybrid_mu = todo_mu+depleted_mu
                hybrid_var = depleted_var+todo_var
                rospy.logdebug("Hybrid mu: {}\t Hybrid std: {}".format(hybrid_mu,hybrid_var))
                intersection_mu = self.intersection_mean(self._plan_nd.mean(),hybrid_mu,self._plan_nd.var(),hybrid_var)
                rospy.logdebug("Plan intersection mu: {}".format(intersection_mu))
                survival_plan = self._plan_nd.sf(intersection_mu)
                rospy.logdebug("Battery cap: {}".format(self._battery_capacity))
                survival_vehicle = self._vehicle_nd.sf(hybrid_mu)
                msg = Readings()
                msg.header.frame_id="supervisor"
                msg.header.stamp = rospy.Time.now()
                msg.data = [survival_vehicle,survival_plan,survival_task]
                self.survival_pub.publish(msg)
                # ASSESS THE SURVIVAL FUNCTIONS OF THE VEHICLE, PLAN, AND TASK IN THAT PRIORITY
                # IF THE VEHICLE IS STILL SAFE
                if survival_vehicle > self._vehicle_soft_limit:
                    # VEHICLE IS FINE, MOVE ON TO PLAN
                    pass
                else:
                    # VEHICLE IN DANGER ZONE, MOVE ON TO EMERGENCY RECOURSES
                    rospy.logwarn("Vehicle in danger zone, returning home.")
                    self.replanRecourse(0)
                    pass
                if survival_plan > self._plan_hard_limit:
                    # PLAN HASN'T FAILED YET
                    if survival_plan > self._plan_soft_limit:
                        # PLAN IS STILL GOOD
                        rospy.sleep(rospy.Duration(0.01))
                    else:
                        # PLAN IS APPROACHING FAILURE, TIGHTEN TASK CONSTRAINTS
                        tmp = self._task_soft_limit+0.1
                        self._task_soft_limit = max(0.55,tmp)
                        rospy.logwarn("Plan approaching failure, tightening task constraints.")
                else:
                    # PLAN HAS FAILED, TRIGGER REPLAN.
                    rospy.logwarn("Plan has failed, triggering replan.")
                    self.replanRecourse(1)
                if survival_task > self._task_hard_limit:
                    # TASK HASN'T FAILED YET
                    if survival_task > self._task_soft_limit:
                        # TASK IS STILL GOOD
                        rospy.sleep(rospy.Duration(0.01))
                    else:
                        # TASK IS APPROACHING FAILURE, LOG A WARNING
                        rospy.logwarn("Task is approaching failure.")
                else:
                    # TASK HAS FAILED, TRIGGER REPLAN.
                    rospy.logwarn("Task has failed, trigerring replan.")
                    self.replanRecourse(2)
                formatstring = 3*"| {} \t"+"|"
                rospy.logdebug(formatstring.format(survival_vehicle,survival_plan,survival_task)) 
                


    def replanRecourse(self,fault):
        # Log the replan recourse action.
        self.recourse_pub.publish(fault)
        if self.aware:
            self.mode = "recourse"
            # fault types
            # 0: battery in danger
            # 1: plan in danger
            # 2: task in danger
            if fault==0:
                self.parseTask(self.plan.plan[self.plan.home])
                # in this one, we just try to go home
            elif fault==1 or fault==2:
                flag = False
                # check for in range
                if self._inrange:
                    # get the current position of the vehicle
                    try:
                        rospy.wait_for_service('guidance/utmrequest',4.0)
                    except:
                        rospy.logerr("No utmrequest service.")
                        return
                    else:
                        service_handle = rospy.ServiceProxy("guidance/utmrequest",UTMService)
                        response = service_handle()
                    if response.success:
                        request = PlanServiceRequest()
                        request.fault = fault
                        request.northing = response.northing
                        request.easting = response.easting
                        request.latitude = response.latitude
                        request.longitude = response.longitude
                        request.cost_mu = self._completed_mu
                        request.cost_std = self._completed_std
                        request.complete = self.plan._complete 
                    else:
                        rospy.logerr("{} no utm position to use!".format(rospy.get_name))
                else:
                    # vehicle not in range, send to home point.
                    self.parseTask(self.plan.plan[self.plan.home])
                    while not self._inrange and not rospy.is_shutdown():
                        rospy.sleep(rospy.Duration(0.5))
                    try:
                        rospy.wait_for_service('guidance/utmrequest',4.0)
                    except:
                        rospy.logerr("No utmrequest service.")
                        return
                    else:
                        service_handle = rospy.ServiceProxy("guidance/utmrequest",UTMService)
                        response = service_handle()
                    if response.success:
                        request = PlanServiceRequest()
                        request.fault = fault
                        request.northing = response.northing
                        request.easting = response.easting
                        request.latitude = response.latitude
                        request.longitude = response.longitude
                        request.cost_mu = self._completed_mu
                        request.cost_std = self._completed_std
                        request.complete = self.plan._complete
                    else:
                        rospy.logerr("{} no utm position to use!".format(rospy.get_name))
                while not flag:
                    try:
                        rospy.wait_for_service('mission/recourse',5.0)
                    except:
                        rospy.logerr("No mission/recourse service... is MATLAB running? Trying again in 5 s")
                        flag = False
                        rospy.sleep(5.0)
                    else:
                        service_handle = rospy.ServiceProxy("mission/recourse",PlanService)
                        response = service_handle(request)
                        flag = True
                rospy.loginfo("{} Recourse Plan Requested.".format(rospy.get_name()))
            else:
                rospy.logerr("What fault is this -> {}".format(fault))

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
        # new task, so make sure the energy aggregator is suppressed until reset
        self._energy_datum = self._energy_measured_total
        self._energy_measured_std = self._energy_measured_sigma
        if self.aware:
            if task.action!='START':
                if task.cost_mu==0 or task.cost_std ==0:
                    rospy.logwarn("Buggy task, skipping. {}".format(task))
                    self.plan.taskDone()
                    self.parseTask(self.plan.getTask())
                    return [True,"Going to next task."]
        
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
                self._start_flag = True
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
            # home task, is a waypoint task to move the vehicle to the starting position.
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
        self.aware = plan_msg.aware
        self.home_task = self.plan.home
        root_task = self.plan.plan[self.plan._root]
        self._plan_nd = norm(root_task.cost_mu,root_task.cost_std)
        self.updateMissionMarkers()
        rospy.loginfo("Plan received, waiting for start mission service on {}".format(rospy.resolve_name("supervisor/start")))
        if self.mode == "recourse":
            self.parseTask(self.plan.getTask())
            self.mode = "mission"
            rospy.loginfo("Commencing plan with recourse!")

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
        # if the vehicle is not in recourse mode
        if not self.mode=='recourse':
            if self.hptask:
                rospy.loginfo("Pose reached, holding for {} seconds...".format(self.hptime))
                rospy.sleep(rospy.Duration(self.hptime))
                self.hptask = False
            if self._start_flag:
                self._start_flag = False
            else:
                self._completed_mu.append(self._energy_measured_mu)
                self._completed_std.append(self._energy_measured_sigma)
            self.plan.taskDone()
            self.updateMissionMarkers()
            rospy.loginfo("Task done, getting next...")
            return self.parseTask(self.plan.getTask())
        else:
            self.changeMode("__none")
            rospy.logwarn("Vehicle in recourse mode and has reached HOME point.. waiting for plan.")
            return [False,"Vehicle in recourse mode and has reached HOME point.. waiting for plan."]

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
            self.changeMode("__none")
            response = [True,"Mission paused."]
        return response

    def checkValidTask(self,task_msg):
        if task_msg.cost_mu > 0 and task_msg.action!='ROOT':
            self._task_nd = norm(task_msg.cost_mu,task_msg.cost_std)
        else:
            self._task_nd = None

    def startPlanCallback(self,request):
        # starting a new plan, so reset the energy counter
        self._energy_datum = self._energy_measured_total
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
