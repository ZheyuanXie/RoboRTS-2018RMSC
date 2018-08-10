#! /usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from rmsc_messages.msg import NavToAction, NavToActionGoal
from actionlib_msgs.msg import GoalStatus
import tf

def help():
    print '''
    1 - SendGoal
    2 - CancelGoal
    3 - Exit
    '''

def done_cb(terminal_state,result):
    print "DONE:",terminal_state, result

def active_cb():
    print "GOAL RCV"

if __name__ == "__main__":
    rospy.init_node("nav_to_client")
    ac_ = SimpleActionClient("nav_to_node_action", NavToAction)
    ac_.wait_for_server()
    print 'sever connected!'
    help()
    while not rospy.is_shutdown():
        cmd = raw_input('cmd:')
        if cmd == '1':
            x = input('target_x:')
            y = input('target_y:')
            yaw = input('yaw:')
            g = NavToActionGoal()
            q = tf.transformations.quaternion_from_euler(0,0,yaw)
            g.goal.navgoal.pose.position.x = x
            g.goal.navgoal.pose.position.y = y
            g.goal.navgoal.pose.orientation.z = q[2]
            g.goal.navgoal.pose.orientation.w = q[3]
            ac_.send_goal(g.goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=None)
            # ac_.wait_for_result()
        elif cmd == '2':
            ac_.cancel_all_goals()
        elif cmd == '3':
            break
        else:
            print 'Invalid Command!'
