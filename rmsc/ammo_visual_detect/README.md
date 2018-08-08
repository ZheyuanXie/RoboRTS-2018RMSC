# Setup
 1. 复制整个工作空间文件夹到Home目录
 2. 删除devel和build文件夹
 2. catkin_make 本工作空间。这个过程可能会报错找不到fatal error: mag_track/MagTrackActionGoal.h: 没有那个文件或目录。如果出现立刻再次执行一次catkin_make命令一般可以解决。
 3. 如果对mag_track包内的action_msg进行过修改，需要手动将devel/include/mag_track内所有的*.h文件复制到/src/mag_track/action_msg目录下。然后再进行编译。
 4. source devel/setup.bash
 5. rosrun mag_track mag_track_node 或者 rosrun mag_scan mag_scan_node
 6. 如果使用mag_track包，还需要运行mag_track_client并且发送命令“1”，才会真正运行算法。
 7. 可以通过改每个包内的proto/arguments.prototest实现一些基本的配置而不需要重新编译。

Created by Wang Zheng