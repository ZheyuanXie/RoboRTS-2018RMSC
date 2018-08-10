#!/usr/bin/env python
import rospy
from gripper import GripperController
from rmsc_messages.msg import GripperCmd

def help():
    print('''
    ------------------
    0 - RELAX
    1 - CLOSE
    2 - OPEN
    3 - POSITION CTRL
    4 - VELOCITY CTRL
    5 - NORMAL
    6 - GRIP HIGH
    7 - GRIP LOW
    h - help
    q - Exit
    ------------------
            ''')

if __name__ == "__main__":
    rospy.init_node('test_gripper_node')
    ctrl = GripperController()
    help()
    while not rospy.is_shutdown():
        cmd = raw_input('cmd:').lower()
        if cmd == '0':
            ctrl.SetState(GripperCmd.RELAX)
            print 'RELAX'
        elif cmd == '1':
            ctrl.SetState(GripperCmd.CLOSE)
            print 'CLOSE'
        elif cmd == '2':
            ctrl.SetState(GripperCmd.OPEN)
            print 'OPEN'
        elif cmd == '3':
            m1_pos = input(' - motor1 position:')
            m2_pos = input(' - motor2 position:')
            ctrl.SetPosition(m1_pos, m2_pos)
            print 'POS: %.3f, %.3f'%(m1_pos, m2_pos)
        elif cmd == '4':
            m1_vel = input(' - motor1 velocity:')
            m2_vel = input(' - motor2 velocity:')
            ctrl.SetVelocity(m1_vel, m2_vel)
            print 'POS: %.3f, %.3f'%(m1_vel, m2_vel)
        elif cmd == '5':
            ctrl.SetState(GripperCmd.NORMAL)
            print 'NORMAL'
        elif cmd == '6':
            ctrl.SetState(GripperCmd.GRIP_HIGH)
            print 'AUTOGRIP HIGH'
        elif cmd == '7':
            ctrl.SetState(GripperCmd.GRIP_LOW)
            print 'AUTOGRIP LOW'
        elif cmd == 'h': help()
        elif cmd == 'q': break
        else: print 'Invalid Command.'
        print '---------------------'