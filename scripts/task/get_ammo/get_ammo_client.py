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

def done_cb(terminal_state,result):
    print "DONE:",terminal_state, result

def active_cb():
    print "GOAL RCV"

if __name__ == "__main__":
    rospy.init_node("get_ammo_client")
    ac_ = SimpleActionClient("get_ammo_node_action", GetAmmoAction)
    ac_.wait_for_server()
    print 'sever connected!'
    help()
    while not rospy.is_shutdown():
        cmd = raw_input('cmd:')
        if cmd == '1':
            ai = input('ammobox index:')
            g = GetAmmoActionGoal()
            g.goal.ammobox_index = ai
            ac_.send_goal(g.goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=None)
        elif cmd == '2':
            ac_.cancel_all_goals()
        elif cmd == '3':
            break
        else:
            print 'Invalid Command!'
