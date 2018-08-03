#! /usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from messages.msg import NavToAction, NavToActionGoal

def help():
    print '''
    1 - SendGoal
    2 - CancelGoal
    3 - Exit
    '''

if __name__ == "__main__":
    rospy.init_node("navto_action_client_node")
    ac_ = SimpleActionClient("navigation_node_action", NavToAction)
    ac_.wait_for_server()
    print 'sever connected!'
    help()
    while not rospy.is_shutdown():
        cmd = raw_input('cmd:')
        if cmd == '1':
            x = input('target_x:')
            y = input('target_y:')
            g = NavToActionGoal()
            g.goal.navgoal.pose.position.x = x
            g.goal.navgoal.pose.position.y = y
            g.goal.navgoal.pose.orientation.w = 1
            ac_.send_goal(g.goal)
            # ac_.wait_for_result()
        elif cmd == '2':
            ac_.cancel_all_goals()
        elif cmd == '3':
            break
        else:
            print 'Invalid Command!'
