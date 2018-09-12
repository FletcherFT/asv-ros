#!/usr/bin/python
import rospy
from asv_mission.msg import Plan, Task
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger, SetBool
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Quaternion, PoseStamped
from topic_tools.srv import MuxSelect
from missionplanning import hierarchy
from tf_conversions import transformations as tf

class Supervisor:
    def __init__(self):
        self.paused = False
        self.manual = False
        self._flag = True
        self.plan = None
        self.hptask = False
        self.hptime = 0.0
        rospy.init_node("supervisor")
        self.task_pub = rospy.Publisher('guidance/task',GeoPoseStamped,queue_size=10)
        self.pause = rospy.Service('supervisor/pause', SetBool, self.pausePlayCallback)
        self.start = rospy.Service('supervisor/start',Trigger,self.startPlanCallback)
        self.nexttask = rospy.Service('supervisor/request_new', Trigger, self.nextTaskCallback)

        rospy.Subscriber("mission/plan",Plan,self.receivePlanCallback)
        rospy.Subscriber("guidance/operator",PoseStamped,self.receiveOperatorCallback)
        rospy.Subscriber("joy",Joy,self.receiveManualCallback)
        rospy.spin()
    
    def receiveManualCallback(self,joy_msg):
        # if we've received a joystick message and the mission isn't paused, pause the mission and configure for override mode.
        flag = False
        if not self.paused:
            self.manual = True
            self.paused = True
            flag = True
        elif self.paused and not self.manual:
            # joystick received, but vehicle in operator override mode, so override that with manual control.
            self.manual = True
            flag = True
        if flag:
            try:
                rospy.wait_for_service('control_topic_mux/select',5.0)
            except Exception as exc:
                rospy.logerr(exc)
                return
            service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
            response=service_handle("tau_com/override")
            rospy.logdebug("{}: Switching to manual override, now publishing {} from {}".format(rospy.get_name(),rospy.resolve_name("override"),response.prev_topic))

    def receiveOperatorCallback(self,geo_msg):
        # if we've receive an operator override message and the mission isn't paused or in manual override, pause it.
        flag = False
        if not self.paused and not self.manual:
            self.paused = True
            flag = True
        elif self.paused and not self.manual:
            flag = True
        elif self.paused and self.manual:
            rospy.logwarn("In manual override mode, you must call resume service to get out of it.")
        if flag:
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
        if task.action == "wp":
            # waypoint task, configure for AP mode to a GPS waypoint.
            try:
                rospy.wait_for_service("control_topic_mux/select",5.0)
            except:
                rospy.logerr("No control_topic_mux/select service found, is controlmanager.py running?")
                return [False,"No control_topic_mux/select service found, is controlmanager.py running?"]
            service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
            service_handle("/tau_com/AP")
            geo_msg.pose.position.altitude=-1
            geo_msg.pose.position.latitude=task.data[0]
            geo_msg.pose.position.longitude=task.data[1]
            geo_msg.pose.orientation = Quaternion(0,0,0,1)
            self.task_pub.publish(geo_msg)
            return [True,"Waypoint Task Selected, executing."]
        elif task.action == "hp":
            # hold position task, configure DP mode at a GPS waypoint and orientation for a set time.
            try:
                rospy.wait_for_service("control_topic_mux/select",5.0)
            except:
                rospy.logerr("No control_topic_mux/select service found, is controlmanager.py running?")
                return [False,"No control_topic_mux/select service found, is controlmanager.py running?"]
            service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
            service_handle("/tau_com/DP")
            geo_msg.pose.position.altitude=0
            geo_msg.pose.position.latitude=task.data[0]
            geo_msg.pose.position.longitude=task.data[1]
            geo_msg.pose.orientation = Quaternion(*tf.quaternion_from_euler(0,0,task.data[2]))
            self.hptime = task.data[3]
            self.hptask = True
            self.task_pub.publish(geo_msg)
            return [True,"Hold Pose Task Selected, executing."]
        elif task.action == "root":
            # root task reached, notify the operator that the mission is complete.
            try:
                rospy.wait_for_service("control_topic_mux/select",5.0)
            except:
                rospy.logerr("No control_topic_mux/select service found, is controlmanager.py running?")
                return [False,"No control_topic_mux/select service found, is controlmanager.py running?"]
            service_handle = rospy.ServiceProxy('control_topic_mux/select', MuxSelect)
            service_handle("__none")
            rospy.loginfo("Final task completed, now idling.")
            return [True,"Final task completed, now idling."]
        else:
            rospy.logerr("Task of type {} is not supported, getting next task.".format(task.action))
            self.plan.taskDone()
            return self.parseTask(self.plan.getTask())

    def receivePlanCallback(self,plan_msg):
        # parse the received Plan message
        self.plan = hierarchy.Hierarchy(plan_msg)
        rospy.loginfo("Plan received, waiting for start mission service on {}".format(rospy.resolve_name("supervisor/start")))

    def nextTaskCallback(self,request):
        # new task requested from the guidance node.
        if self.hptask:
            rospy.loginfo("Pose reached, holding for {} seconds...".format(self.hptime))
            rospy.sleep(rospy.Duration(self.hptime))
            self.hptask = False
        self.plan.taskDone()
        rospy.loginfo("Task done, getting next...")
        return self.parseTask(self.plan.getTask())

    def pausePlayCallback(self,request):
        # if we want to start/resume the mission
        if not request.data:
            if self.plan is None:
                response = [False,"No Mission to resume"]
            else:
                self.paused = False
                self.manual = False
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
            self.paused = True
            self.manual = True
            try:
                rospy.wait_for_service("control_topic_mux/select",4.0)
            except:
                response = [False, "No control_topic_mux/select service"]
            else:
                service_handle = rospy.ServiceProxy("control_topic_mux/select",MuxSelect)
                service_handle("/tau_com/override")
                response = [True,"Mission paused."]
        return response

    def startPlanCallback(self,request):
        if self.plan is None:
            return [False,"No mission loaded"]
        else:
            self.manual = False
            self.paused = False
            try:
                rospy.wait_for_service("guidance/resume",4.0)
            except:
                response = [False, "No guidance/resume service"]
            else:
                service_handle = rospy.ServiceProxy("guidance/resume",Trigger)
                response = service_handle()
            response = self.parseTask(self.plan.getTask())
            if response[0]:
                return [True,"Mission starting: {}".format(response[1])]
            else:
                return [False, "Mission failed to start: {}".format(response[1])]

if __name__=="__main__":
    try:
        supervisor = Supervisor()
    except rospy.ROSInterruptException:
        pass