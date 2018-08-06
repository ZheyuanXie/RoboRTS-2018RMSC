#!/usr/bin/env python
import rospy
from messages.msg import ConditionOverride

if __name__ == "__main__":
    rospy.init_node("condtion_overide_node")
    pub_condition_ = rospy.Publisher("/debug/condition_override", ConditionOverride, queue_size=1)
    rate = rospy.Rate(10)
    condition = ConditionOverride()
    condition.condition_override_enable = True
    condition.game_stop_condition_ = False
    while not rospy.is_shutdown():
        pub_condition_.publish(condition)
        rate.sleep()