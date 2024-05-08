#! /bin/bash
echo copying Code to ROS_ws...
rm /home/ycz/ROS_Code/dog_ws/src/A1_ctrl/* -rf
for file in `ls /home/ycz/Code/A1_ctrl/`
do
  cp /home/ycz/Code/A1_ctrl/$file /home/ycz/ROS_Code/dog_ws/src/A1_ctrl/ -r 
done
echo ssh copying Code to dog...
for file in `ls /home/ycz/Code/A1_ctrl/`
do
  sshpass -p 123 scp -P2233 -r /home/ycz/Code/A1_ctrl/$file root@nuc:/root/A1_ctrl_ws/src/dog_ctrl
done
