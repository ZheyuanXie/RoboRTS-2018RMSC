import rospy
from messages.msg import GripperCmd, GripperInfo

CMD_REPEAT = 3

class GripperController:
    def __init__(self, cmd_topic_name='/cmd_grip', fb_topic_name='/gripper'):
        self.pub_cmd = rospy.Publisher(cmd_topic_name, GripperCmd, queue_size=1)
        self.sub_fb = rospy.Subscriber(fb_topic_name, GripperInfo, callback=self.FeedbackCB)
        self.feedback = False

    def SetState(self, state):
        self.SendGripperCmd(state, 0., 0.)
    
    def SetVelocity(self, m1_vel, m2_vel):
        self.SendGripperCmd(GripperCmd.VEL_CTRL, m1_vel, m2_vel)
    
    def SetPosition(self, m1_pos, m2_pos):
        self.SendGripperCmd(GripperCmd.POS_CTRL, m1_pos, m2_pos)
    
    def FeedbackCB(self, data):
        self.feedback = data.mode
    
    def SendGripperCmd(self, cmd, m1_ref, m2_ref, repeat=CMD_REPEAT):
        gcmd = GripperCmd()
        gcmd.cmd = cmd
        gcmd.motor1_ref = m1_ref
        gcmd.motor2_ref = m2_ref
        for _ in range(repeat):
            self.pub_cmd.publish(gcmd)