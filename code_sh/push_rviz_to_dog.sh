#! /bin/bash
cp /home/ycz/ROS_Code/dog_ws/src/A1_ctrl/doggy_control/config/rviz /home/ycz/Code/A1_ctrl/doggy_control/config/ -r
sshpass -p 123 scp -r /home/ycz/Code/A1_ctrl/doggy_control/config/rviz unitree@tx2:/home/unitree/A1_ctrl/doggy_control/config/
echo done.