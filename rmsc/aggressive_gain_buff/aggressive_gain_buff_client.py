#! /usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from rmsc_messages.msg import AggressiveGainBuffAction, AggressiveGainBuffActionGoal

def help():
    print '''
    1 - SendGoal
    2 - CancelGoal
    3 - Exit
    '''

if __name__ == "__main__":
    rospy.init_node("aggressive_gain_buff_client")
    ac_ = SimpleActionClient("aggressive_gain_buff_action", AggressiveGainBuffAction)
    ac_.wait_for_server()
    print 'Aggressive Gain Buff Sever Connected!'
    help()
    while not rospy.is_shutdown():
        cmd = raw_input('cmd:')
        if cmd == '1':
            route_index = input('Input Route Index:')
            g = AggressiveGainBuffActionGoal()
            g.goal.route_index = route_index
            ac_.send_goal(g.goal)
        elif cmd == '2':
            ac_.cancel_all_goals()
        elif cmd == '3':
            break
        else:
            print 'Invalid Command!'
