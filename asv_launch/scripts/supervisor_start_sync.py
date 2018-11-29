#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger
from asv_messages.srv import PlanService, PlanServiceResponse

def main():
    rospy.init_node("fake_supervisor_service_caller")
    rospy.Service('mission/recourse', PlanService, planCallback)
    flag = False
    while not rospy.is_shutdown():
        if rospy.get_time() > 1542086798.2 and not flag:
            flag = callSupervisor()

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