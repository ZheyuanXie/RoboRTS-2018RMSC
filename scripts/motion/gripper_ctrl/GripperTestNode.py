#!/usr/bin/env python
import rospy
from GripperControl import GripperController, GripperState

def help():
    print('''
    ------------------
    q - Normal Position
    w - Prepare Position
    e - Lower Ready Position
    r - Upper Ready Position
    5 - Normal Position
    6 - Get Ammobox Macro (HIGH)
    7 - Get Ammobox Macro (LOW)
    [ - Grasp
    ] - Release
    h - help
    x - Exit
    ------------------
            ''')

if __name__ == "__main__":
    rospy.init_node('test_gripper_node')
    ctrl = GripperController()
    help()
    while(1):
        cmd = raw_input('cmd:').lower()
        if cmd == 'q':
            ctrl.SendGripperCmd(GripperState.Normal)
            print 'Set to Normal Position.'
        elif cmd == 'w':
            ctrl.SendGripperCmd(GripperState.Prepare)
            print 'Set to Prepare Position.'
        elif cmd == 'e':
            ctrl.SendGripperCmd(GripperState.Ready_Low)
            print 'Set to Lower Ready Position.'
        elif cmd == 'r':
            ctrl.SendGripperCmd(GripperState.Ready_High)
            print 'Set to Upper Ready Position.'
        elif cmd == '5':
            ctrl.SendGripperCmd(GripperState.Normal)
            print '5'
        elif cmd == '6':
            ctrl.SendGripperCmd(GripperState.Get_High)
            print '6'
        elif cmd == '7':
            ctrl.SendGripperCmd(GripperState.Get_Low)
            print '7'
        elif cmd == '[':
            ctrl.SendGripperCmd(GripperState.Release)
            print 'Release!'
        elif cmd == ']':
            ctrl.SendGripperCmd(GripperState.Grasp)
            print 'Grasp!'
        elif cmd == 'r':
            ctrl.SendGripperCmd(GripperState.Relax)
            print "Relax!"
        elif cmd == 'h':
            help()
        elif cmd == 'x':
            break
        else:
            print 'Invalid Command.'