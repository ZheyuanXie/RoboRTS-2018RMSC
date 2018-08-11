#!/usr/bin/env python
import rospy
from messages.msg import GameInfo
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

PLAY_SOUND = False

def help():
    print '''
    1 - Start Game
    2 - End Game
    3 - Exit
    '''

if __name__ == "__main__":
    rospy.init_node("fake_referee")
    soundhandle = SoundClient()
    pub_game_info = rospy.Publisher("/fake_game_info",GameInfo,queue_size=1)
    info = GameInfo()
    while not rospy.is_shutdown():
        help()
        cmd = raw_input('>')
        if cmd == '1':

            info.game_process = 3
            rate = rospy.Rate(1)
            for cnt in range(5,0,-1):
                rospy.loginfo('Countdown:%d'%(cnt))
                info.remain_time = cnt
                pub_game_info.publish(info)
                if PLAY_SOUND:
                    soundhandle.stopAll()
                    soundhandle.say(str(cnt))
                rate.sleep()
            info.game_process = 4
            info.remain_time = 0
            rate = rospy.Rate(30)
            if PLAY_SOUND:
                soundhandle.stopAll()
                soundhandle.say('Game Start!')
            rospy.loginfo('Game Start!')
            for _ in range(30):
                pub_game_info.publish(info)
                rate.sleep()        
        elif cmd == '2':
            info = GameInfo()
            info.game_process = 0
            pub_game_info.publish(info)
            rospy.loginfo('Game Terminate!')
        elif cmd == '3':
            break