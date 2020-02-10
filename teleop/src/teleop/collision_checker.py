import rospy
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.srv import GetStateValidityRequest


class CollisionChecker(object):
    def __init__(self):
        # Initialize collision checker - not that this does not stop collisions, it merely stops the arm after we have already hit something - good enough if obstacles are inflated enough
        try:
            rospy.wait_for_service("check_state_validity", timeout=5)
        except rospy.ROSException:
             rospy.logwarn("[check_collisions_node] Done waiting for /check_state_validity service... service not found")
             rospy.logwarn("shutting down...")
             rospy.signal_shutdown("service unavailable")
        except rospy.ROSInterruptException:
            pass
        self.coll_client = rospy.ServiceProxy("check_state_validity", GetStateValidity)
        print("============ Connected to state validity service")

    def is_state_valid(self, robot_state, group_name):
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = robot_state
        gsvr.group_name = group_name
        resp = self.coll_client(gsvr)
        if not resp.valid:
            print('collision!')
            return False
        return True
