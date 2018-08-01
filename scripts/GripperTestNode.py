#!/usr/bin/env python
import rospy
from GripperControl import GripperController, GripperState

if __name__ == "__main__":
    rospy.init_node('test_gripper_node')
    ctrl = GripperController()
    print('''
------------------
1 - Normal Position
2 - Prepare Position
3 - Lower Ready Position
4 - Upper Ready Position
[ - Grasp
] - Release
h - help
x - Exit
------------------
        ''')
    while(1):
        cmd = raw_input('cmd:').lower()
        if cmd == '1':
            ctrl.SendGripperCmd(GripperState.Normal)
            print 'Set to Normal Position.'
        elif cmd == '2':
            ctrl.SendGripperCmd(GripperState.Ready_Low)
            print 'Set to Prepare Position.'
        elif cmd == '3':
            ctrl.SendGripperCmd(GripperState.Ready_High)
            print 'Set to Lower Ready Position.'
        elif cmd == '4':
            ctrl.SendGripperCmd(GripperState.Prepare)
            print 'Set to Upper Ready Position.'
        elif cmd == '[':
            ctrl.SendGripperCmd(GripperState.Release)
            print 'Release!'
        elif cmd == ']':
            ctrl.SendGripperCmd(GripperState.Grasp)
            print 'Grasp!'
        elif cmd == 'r':
            ctrl.SendGripperCmd(GripperState.Relax)
            print "Relax!"
        elif cmd == 'x':
            break
        else:
            print 'Invalid Command.'