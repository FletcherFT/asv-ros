#!/usr/bin/python
import rospy
from asv_messages.msg import Plan, Task
from asv_messages.srv import PlanService, PlanServiceRequest
from missionplanning import hierarchy


def receivePlanCallback(plan_msg):
    rospy.loginfo("Plan received: {}".format(plan_msg))
    plan = hierarchy.Hierarchy(plan_msg)
    plan.getTask()
    for i in range(5):
        plan.taskDone()
        plan.getTask()
    plan.skipTask()
    plan.getTask()
    for i in range(3):
        plan.taskDone()
        plan.getTask()

    request = PlanServiceRequest()
    request.burned = 100000
    request.completed = plan._complete
    request.skipped = plan._skipped
    request.fault = 1
    request.cost_mu = [500,600,700,800,900,1000,1100,1200]
    request.cost_std = [5,5,5,5,5,5,5,5]
    request.northing = 5413398.3861+5
    request.easting = 515754.7752+5
    request.latitude = -41.430824465374300
    request.longitude = 147.188178502

    try:
        rospy.wait_for_service("mission/recourse",5.0)
    except:
        rospy.logerr("no service")
    else:
        service_handle = rospy.ServiceProxy("mission/recourse",PlanService)
        response = service_handle(request)

def main():
    rospy.init_node("testmissionlistener")
    rospy.Subscriber("mission/plan",Plan,receivePlanCallback)
    rospy.spin()

if __name__=="__main__":
    main()
