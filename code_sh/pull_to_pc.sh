#! /bin/bash
rm /home/ycz/Code/A1_ctrl/* -rf
echo ssh copying dog to Code...
sshpass -p 123 scp -P2233 -r root@nuc:/root/A1_ctrl_ws/src/dog_ctrl/* /home/ycz/Code/A1_ctrl
echo copying Code to ROS_ws...
rm /home/ycz/ROS_Code/dog_ws/src/A1_ctrl/* -rf
for file in `ls /home/ycz/Code/A1_ctrl/`
do
  cp /home/ycz/Code/A1_ctrl/$file /home/ycz/ROS_Code/dog_ws/src/A1_ctrl/ -r 
done
echo done.