# my_bot
https://github.com/MOGI-ROS

rsp_gz.launch.py         : output gz
rso_gz_robot_2.launch.py : includes rsp_gz.launch.py and spawn robot.

echo "# my_bot" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/kiinging/my_bot.git
git push -u origin main



ros2 run rqt_tf_tree rqt_tf_tree

ros2 run teleop_twist_keyboard teleop_twist_keyboard 

ros2 node info /ekf_filter_node

ros2 pkg list | grep twist_mux

ros2 topic info -v /cmd_vel

apt-cache policy ros-jazzy-twist-mux

px aus | grep gz
kill -9 xxxx


ros2 control list_hardware_interfaces
ros2 control list_controllers
