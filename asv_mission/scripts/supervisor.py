#!/usr/bin/python
import rospy
import numpy as np
from asv_messages.msg import Plan, Task, Float64Stamped, Readings
from asv_messages.srv import PlanService, PlanServiceRequest, UTMService
from sensor_msgs.msg import Joy, BatteryState
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

        self._task_warn = rospy.get_param('~task_warn',0.46) # Threshold for when warnings are generated for the task.
        self._task_limit = rospy.get_param('~task_limit',0.45) # the limit for the task survival function before recourse.
        self._plan_warn = rospy.get_param('~plan_warn',0.48) # Threshold for when warnings are generated for the plan.
        self._plan_limit = rospy.get_param('~plan_limit',0.475) # Threshold for the plan survival function before recourse.
        self._battery_warn = rospy.get_param('~battery_warn',0.51) # Threshold for when warnings are generated for the battery..
        self._battery_limit = rospy.get_param('~battery_limit',0.5) # the limit of the battery before recourse.
        self._voltage_warn = rospy.get_param('~voltage_warn',10.5) # Threshold for warning that the voltage is getting low.
        self._voltage_limit = rospy.get_param('~voltage_limit',10.2) # Threshold for recourse due to low battery voltage.

        self._timing_lock = False
        self._replan_lock = False

        self._battery_energy_capacity = rospy.get_param('~battery_capacity',7*12*3600) # the energy capacity of the vehicle's battery system.
        self._battery_energy_variance = rospy.get_param('~battery_variance',0.25*self._battery_energy_capacity) # variance of the battery
        self._battery_energy_remaining = self._battery_energy_capacity
        self._percent_remain = 1.0

        self._battery_nd = norm(self._battery_energy_capacity,sqrt(self._battery_energy_variance)) # normal distribution of the battery.
        self._task_nd = None
        self._plan_nd = None
        self._energy_measured_mu = 0
        self._energy_measured_std = 0
        self._energy_measured_total = 0
        self._completed_mu = []
        self._completed_std = []
        self._reset_flag = False
        self._task_energy_datum = 0
        self._plan_energy_datum = 0
        self._aware = False
        self._emergency = False

        rospy.init_node("supervisor")
        rospy.loginfo("Supervisor Started with:\n Task Limit: {}\n Plan Limit: {}\n Battery Limit: {}\n Voltage Limit: {}".format(self._task_limit,self._plan_limit,self._battery_limit,self._voltage_limit))

        self.survival_pub = rospy.Publisher('supervisor/survival',Readings,queue_size=10)
        self.recourse_pub = rospy.Publisher('supervisor/recourse',Int8,queue_size=10)
        self.task_pub = rospy.Publisher('guidance/task',GeoPoseStamped,queue_size=10)
        self.status_pub = rospy.Publisher("asv/status",String,queue_size=10)
        self.marker_pub = rospy.Publisher("mission/markers",MarkerArray,queue_size=10)
        self.pause = rospy.Service('supervisor/pause', SetBool, self.pausePlayCallback)
        self.start = rospy.Service('supervisor/start',Trigger,self.startPlanCallback)
        self.nexttask = rospy.Service('supervisor/request_new', Trigger, self.nextTaskCallback)
        self.hold = rospy.Service('supervisor/hold',Trigger,self.holdPositionCallback)

        rospy.Subscriber("failsafe/inrange",Bool,self.rangeStateCallback)
        rospy.Subscriber("energy/aggregate",Readings,self.energySampleCallback)
        rospy.Subscriber("throttled/battery",BatteryState,self.batterySampleCallback)
        rospy.Subscriber("mission/plan",Plan,self.receivePlanCallback)
        rospy.Subscriber("guidance/operator",PoseStamped,self.receiveOperatorCallback)
        rospy.Subscriber("guidance/percent_remain",Float64Stamped,self.receivePercentCallback)
        rospy.Subscriber("joy",Joy,self.receiveManualCallback)

        rospy.spin()
    
    def intersection_mean(self,mu1,mu2,var1,var2):
        return var2*mu1/(var1+var2)+var1*mu2/(var1+var2)

    def batterySampleCallback(self,msg):
        if msg.voltage > self._voltage_warn:
            pass
        elif msg.voltage < self._voltage_limit:
            rospy.logerr("Battery voltage below limit. Recourse.")
            self.recourse(3)
        else:
            rospy.logwarn("Battery approaching voltage limit.")

    def receivePercentCallback(self,msg):
        self._percent_remain = msg.data

    def rangeStateCallback(self,msg):
        self._inrange = msg.data

    def energySampleCallback(self,msg):
        # UPDATE THE TOTAL MEASURED CONSUMED ENERGY
        self._energy_measured_total = msg.data[0]
        self._energy_measured_std = msg.data[1]

        # ALWAYS GET THE SURVIVAL OF THE BATTERY
        battery_intersection_mu = self.intersection_mean(self._battery_nd.mean(),self._energy_measured_total,self._battery_nd.var(),self._energy_measured_std**2)
        self._battery_survival = self._battery_nd.sf(battery_intersection_mu)

        # IF THE VEHICLE IS CURRENTLY IN A MISSION, DO THE FOLLOWING
        if self.mode == "mission":
            # UPDATE THE ENERGY USED FOR THE CURRENT TASK
            # GET THE CONSUMED ENERGY FOR THE TASK
            self._task_energy_measured_mu = self._energy_measured_total-self._task_energy_datum
            self._task_energy_measured_std = self._energy_measured_std
            # UPDATE THE ENERGY USED FOR THE CURRENT PLAN
            # GET THE CONSUMED ENERGY FOR THE PLAN
            self._plan_energy_measured_mu = self._energy_measured_total-self._plan_energy_datum
            self._plan_energy_measured_std = self._energy_measured_std

            # IF THERE ARE TASK AND PLAN DISTRIBUTIONS (DECLARED WHEN PLAN STARTED)
            if not self._task_nd is None and not self._plan_nd is None:
                # GET THE SURVIVAL FUNCTION OF THE TASK
                # The intersection probability density function mean is
                task_intersection_mu = self.intersection_mean(self._task_nd.mean(),self._task_energy_measured_mu,self._task_nd.var(),self._task_energy_measured_std**2)
                # The cumulative probability that the energy consumed for current task is the energy predicted for the task is:
                self._task_survival = self._task_nd.sf(task_intersection_mu)
                # PREDICT THE ENERGY REMAINING FOR THE CURRENT TASK
                self._task_energy_remaining = self._task_energy_measured_mu * (1 + self._percent_remain)
                
                # The intersection probability density function mean is
                plan_intersection_mu = self.intersection_mean(self._plan_nd.mean(),self._plan_energy_measured_mu,self._plan_nd.var(),self._plan_energy_measured_std**2)
                # The cumulative proability that the energy consumed for current task is the energy predicted for the task is:
                self._plan_survival = self._plan_nd.sf(plan_intersection_mu)

                # PUBLISH THE SURVIVAL FUNCTIONS
                msg = Readings()
                msg.header.frame_id="[battery,plan,task]"
                msg.header.stamp = rospy.Time.now()
                msg.data = [self._battery_survival,self._plan_survival,self._task_survival]
                self.survival_pub.publish(msg)

                # ASSESS THE SURVIVAL FUNCTIONS OF THE BATTERY, PLAN, AND TASK IN THAT PRIORITY
                # BATTERY SURVIVAL
                if self._battery_survival > self._battery_warn:
                    # NO BATTERY SURVIVAL ISSUE
                    pass
                elif self._battery_survival < self._battery_limit:
                    # BATTERY SURVIVAL LIMIT CROSSED, TIME FOR RECOURSE
                    rospy.logerr("Energy consumed for battery crossed battery threshold, recourse initiated.")
                    self.recourse(0)
                else:
                    # BATTERY SURVIVAL WARN LIMIT CROSSED, LOG WARNING
                    rospy.logwarn("Energy consumed for battery since startup approaching battery energy threshold.")

                if not self._start_flag:
                    # PLAN SURVIVAL
                    if self._plan_survival > self._plan_warn:
                        # NO PLAN SURVIVAL ISSUE
                        pass
                    elif self._plan_survival < self._plan_limit:
                        # PLAN SURVIVAL LIMIT CROSSED, TIME FOR RECOURSE
                        rospy.logerr("Energy consumed for plan crossed plan threshold, recourse initiated.")
                        self.recourse(1)
                    else:
                        # PLAN SURVIVAL WARN LIMIT CROSSED, LOG WARNING
                        rospy.logwarn("Energy consumed for plan approaching plan threshold.")

                    # TASK SURVIVAL
                    if self._task_survival > self._task_warn:
                        # NO TASK SURVIVAL ISSUE
                        pass
                    elif self._task_survival < self._task_limit:
                        rospy.logerr("Energy consumed for task crossed task threshold, recourse initiated.")
                        self.recourse(2)
                    else:
                        rospy.logwarn("Energy consumed for task approaching task threshold.")
                

    def recourse(self,fault):
        # Log the replan recourse action.
        self.recourse_pub.publish(fault)
        if self._aware:
            # fault types
            # 0: battery in danger
            # 1: plan in danger
            # 2: task in danger
            # 3: battery voltage in danger, motors won't work so idle and send distress.

            if fault==0:
                # when the battery threshold is crossed, just return home.
                self.mode = "recourse"
                self.parseTask(self.plan.plan[self.plan.home])
            elif not self._timing_lock and fault==1:
                # when the plan budget threshold is crossed, first try to give a replan
                self.mode = "recourse"
                if not self._replan_lock:
                    if self._inrange:
                        # first get the vehicle to hold current position
                        try:
                            rospy.wait_for_service("guidance/hold",4.0)
                        except:
                            response = [False, "No guidance/hold service"]
                        else:
                            service_handle = rospy.ServiceProxy("guidance/hold",Trigger)
                            service_handle()
                            self.mode = "recourse"

                        # Get the current position of the vehicle
                        try:
                            rospy.wait_for_service('guidance/utmrequest',4.0)
                        except:
                            rospy.logerr("No utmrequest service.")
                        else:
                            service_handle = rospy.ServiceProxy("guidance/utmrequest",UTMService)
                            response = service_handle()
                        # If the position was obtained from guidance, fill out the replan request form
                        if response.success:
                            request = PlanServiceRequest()
                            request.fault = fault
                            # the amount of energy burned doing the current plan, not overall!
                            request.burned = self._plan_energy_measured_mu
                            request.northing = response.northing
                            request.easting = response.easting
                            request.latitude = response.latitude
                            request.longitude = response.longitude
                            request.cost_mu = self._completed_mu
                            request.cost_std = self._completed_std
                            request.completed = self.plan._complete 
                            request.skipped = self.plan._skipped
                        else:
                            rospy.logerr("{} no utm position to use!".format(rospy.get_name))

                        # Now request a new plan from MATLAB
                        try:
                            rospy.wait_for_service('mission/recourse',5.0)
                        except:
                            rospy.logerr("No mission/recourse service... is MATLAB running? Trying again in 5 s")
                            # vehicle not in range, decide whether to skip task or not.
                            if self.skipOrSkipNot():
                                rospy.logwarn("Plan fail, not in range, skipping task")
                                self.plan.skipTask()
                                self.mode="mission"
                                self.updateMissionMarkers()
                                self.parseTask(self.plan.getTask())
                            else:
                                rospy.logwarn("Plan fail, not in range, not skipping task")
                                self.mode="mission"
                        else:
                            service_handle = rospy.ServiceProxy("mission/recourse",PlanService)
                            response = service_handle(request)
                            rospy.loginfo("{} Recourse Plan Requested.".format(rospy.get_name()))
                            # lock the vehicle from calling another replan until it has completed a task
                            self._replan_lock = True

                    else:
                        # vehicle not in range, decide whether to skip task or not.
                        if self.skipOrSkipNot():
                            rospy.logwarn("Plan fail, not in range, skipping task")
                            self.plan.skipTask()
                            self.mode="mission"
                            self.updateMissionMarkers()
                            self.parseTask(self.plan.getTask())
                        else:
                            rospy.logwarn("Plan fail, not in range, not skipping task")
                            self.mode="mission"
            elif not self._timing_lock and fault==2:
                self.mode = "recourse"
                # DECIDE ON SKIPPING TASK OR NOT
                if self.skipOrSkipNot():
                        rospy.logwarn("Task fail, skipping task")
                        self.plan.skipTask()
                        self.mode="mission"
                        self.updateMissionMarkers()
                        self.parseTask(self.plan.getTask())
                else:
                    rospy.logwarn("Task fail, not skipping task")
                    self.mode="mission"
            elif fault==3:
                self.mode = "recourse"
                rospy.logfatal("Voltage Dropout")
                if not self._emergency:
                    rospy.logfatal("Voltage Dropout, cutting power to motors!")
                    self.changeMode("__none")
                    self._emergency = True
            else:
                rospy.logerr("What fault is this -> {}".format(fault))

    def skipOrSkipNot(self):
        # iteratively compare energy remaining
        todo = self.plan._todo
        costs = [ self.plan.plan[x].cost_mu for x in todo]
        sorted_costs = sorted(costs)
        sorted_todo = [todo[x] for x in np.argsort(costs)] 
        viable = []
        e = 0
        for idx,i in enumerate(sorted_todo):
            if self._task_energy_remaining < e:
                break
            else:
                e += sorted_costs[idx]
                viable.append(i)
        viable_rewards = [ self.plan.plan[x].reward for x in viable]

        if sum(viable_rewards) > self.plan.plan[self.plan._current].reward:
            return True
        else:
            return False


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
        self._task_energy_measured_mu = 0
        self._task_energy_measured_std = self._energy_measured_std
        self._task_energy_datum = self._energy_measured_total

        # IF THIS IS A PLAN THAT REQUIRES SUPERVISION
        if self._aware:
            # if the task isn't the starting task.
            if task.action!='START':
                # if the energy cost mu or std = 0, then this task is buggy.
                if task.cost_mu==0 or task.cost_std ==0:
                    rospy.logwarn("Buggy task, skipping. {}".format(task))
                    self.plan.skipTask()
                    self.parseTask(self.plan.getTask())
                    return [True,"Going to next task."]
        
        # GET WAYPOINT COMPONENTS READY TO SEND TO THE GUIDANCE NODE
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
            self.plan.skipTask()
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
        rospy.loginfo("{}: Got plan!".format("SUPERVISOR"))
        self.plan = hierarchy.Hierarchy(plan_msg)
        self._aware = plan_msg.aware
        self.home_task = self.plan.home
        root_task = self.plan.plan[self.plan._root]
        self._plan_nd = norm(root_task.cost_mu,root_task.cost_std)

        self.updateMissionMarkers()
        rospy.loginfo("Plan received, waiting for start mission service on {}".format(rospy.resolve_name("supervisor/start")))
        if self.mode == "recourse":
            rospy.loginfo("Commencing plan with recourse!")
            self.startPlanCallback(None)

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
                elif idx in self.plan._skipped:
                    marker_msg.color = ColorRGBA(1.0,1.0,0,0.8) #YELLOW
                else:
                    marker_msg.color = ColorRGBA(1,0,0,0.8) #RED
                array_msg.markers.append(marker_msg)
        self.marker_pub.publish(array_msg)

    def nextTaskCallback(self,request):
        # flip a lock to stop the supervisor from skipping or replanning
        self._timing_lock = True
        # new task requested from the guidance node.
        # if the vehicle is not in recourse mode
        if not self.mode=='recourse':
            if self.hptask:
                rospy.loginfo("Pose reached, holding for {} seconds...".format(self.hptime))
                rospy.sleep(rospy.Duration(self.hptime))
                self.hptask = False
            # IF THE TASK JUST FINISHED WAS THE STARTING TASK
            if self._start_flag:
                self._plan_energy_measured_mu = 0 
                self._plan_energy_measured_std = self._energy_measured_std
                self._plan_energy_datum = self._energy_measured_total
                self._start_flag = False
            else:
                self._completed_mu.append(self._task_energy_measured_mu)
                self._completed_std.append(self._task_energy_measured_std)
                # turn off the replan lock if a task that wasn't the starting task is completed
                if self._replan_lock:
                    self._replan_lock = False
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

    def holdPositionCallback(self,request):
        self.mode="HOLD"
        try:
            rospy.wait_for_service("guidance/hold",4.0)
        except:
            response = [False, "No guidance/hold service"]
        else:
            service_handle = rospy.ServiceProxy("guidance/hold",Trigger)
            return service_handle()

    def checkValidTask(self,task_msg):
        if task_msg.cost_mu > 0 and task_msg.action!='ROOT':
            self._task_nd = norm(task_msg.cost_mu,task_msg.cost_std)
        else:
            self._task_nd = None

    def startPlanCallback(self,request):
        # starting a new plan, so reset the energy counter
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
