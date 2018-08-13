#! /usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from rmsc_messages.msg import MapMuxAction, MapMuxGoal

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
    rospy.init_node("map_mux_client")
    ac_ = SimpleActionClient("map_mux_node_action", MapMuxAction)
    ac_.wait_for_server()
    print 'sever connected!'
    help()
    while not rospy.is_shutdown():
        cmd = raw_input('cmd:')
        if cmd == '1':
            mi = input('map index:')
            g = MapMuxGoal()
            g.map_index = mi
            ac_.send_goal(g, done_cb=done_cb, active_cb=active_cb, feedback_cb=None)
        elif cmd == '2':
            ac_.cancel_all_goals()
        elif cmd == '3':
            break
        else:
            print 'Invalid Command!'
