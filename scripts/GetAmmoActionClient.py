#! /usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from messages.msg import GetAmmoAction, GetAmmoActionGoal

def help():
    print '''
    1 - SendGoal
    2 - CancelGoal
    3 - Exit
    '''

if __name__ == "__main__":
    rospy.init_node("get_ammo_action_client_node")
    ac_ = SimpleActionClient("get_ammo", GetAmmoAction)
    ac_.wait_for_server()
    print 'sever connected!'
    help()
    while not rospy.is_shutdown():
        cmd = raw_input('cmd:')
        if cmd == '1':
            ai = input('ammobox index:')
            g = GetAmmoActionGoal()
            g.goal.ammobox_index = ai
            ac_.send_goal(g.goal)
        elif cmd == '2':
            ac_.cancel_all_goals()
        elif cmd == '3':
            break
        else:
            print 'Invalid Command!'
