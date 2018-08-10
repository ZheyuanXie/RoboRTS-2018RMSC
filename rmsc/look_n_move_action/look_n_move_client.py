#! /usr/bin/env python
import rospy
import tf
from actionlib import SimpleActionClient
from rmsc_messages.msg import LookAndMoveAction, LookAndMoveActionGoal

def help():
    print '''
    1 - SendGoal
    2 - CancelGoal
    3 - Exit
    '''

if __name__ == "__main__":
    rospy.init_node("look_n_move_client")
    ac_ = SimpleActionClient("look_n_move_node_action", LookAndMoveAction)
    ac_.wait_for_server()
    print 'sever connected!'
    help()
    while not rospy.is_shutdown():
        cmd = raw_input('cmd:')
        if cmd == '1':
            x = input('offset_x:')
            y = input('offset_y:')
            angle = input('offset_angle:')
            g = LookAndMoveActionGoal()
            q = tf.transformations.quaternion_from_euler(0.,0.,angle)
            g.goal.relative_pose.pose.position.x = x
            g.goal.relative_pose.pose.position.y = y
            g.goal.relative_pose.pose.orientation.z = q[2]
            g.goal.relative_pose.pose.orientation.w = q[3]
            ac_.send_goal(g.goal)
        elif cmd == '2':
            ac_.cancel_all_goals()
        elif cmd == '3':
            break
        else:
            print 'Invalid Command!'
