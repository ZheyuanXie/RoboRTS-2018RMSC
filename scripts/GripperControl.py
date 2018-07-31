import rospy
from messages.msg import GripperCmd, GripperInfo

class GripperState:
    Normal      = (3,90,450)    # Normal State
    Ready_Low   = (3,0,10)
    Prepare_Low = (3,10,450)
    Release     = (1,0,0)       # Require Gas Tank
    Grasp       = (2,0,0)       # Require Gas Tank

class GripperController:
    def __init__(self, cmd_topic_name='/cmd_grip', fb_topic_name='/gripper'):
        self.pub_cmd = rospy.Publisher(cmd_topic_name, GripperCmd, queue_size=1)
        self.sub_fb = rospy.Subscriber(fb_topic_name, GripperInfo, callback=self.GripperCB)
        self.touch = False
        rospy.sleep(1.)
        self.SendGripperCmd(GripperState.Release)
        self.SendGripperCmd(GripperState.Normal)
        rospy.sleep(1.)

    def SendGripperCmd(self, cmd, repeat=3):
        gcmd = GripperCmd()
        gcmd.cmd = cmd[0]
        gcmd.motor1_ref = cmd[1]
        gcmd.motor2_ref = cmd[2]
        for i in range(repeat):
            self.pub_cmd.publish(gcmd)
    
    def GripperCB(self, data):
        self.touch = data.mode