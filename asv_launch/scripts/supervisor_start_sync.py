#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from asv_messages.srv import PlanService, PlanServiceResponse
from std_msgs.msg import String

def main():
    rospy.init_node("fake_supervisor_service_caller")
    rospy.Service('mission/recourse', PlanService, planCallback)
    rospy.Subscriber('test/status', String, statusCallback)
    rospy.spin()

def statusCallback(msg):
    if msg.data == "ASV currently in mode: mission":
        callSupervisor()

def planCallback(request):
    response = PlanServiceResponse()
    response.success=True
    response.message="Fooled ya."
    return response

def callSupervisor():
    try:
        rospy.wait_for_service('supervisor/start',5.0)
    except:
        rospy.logerr("{}: supervisor/start unavailable?".format(rospy.get_name()))
        return False
    else:
        handle = rospy.ServiceProxy('supervisor/start',Trigger)
        response = handle()
        rospy.loginfo("{}: {}".format(rospy.get_name(),response))
        return True


if __name__=="__main__":
	main()