export ROS_MASTER_URI=http://elroy.local:11311
export ROS_IP=$(ifconfig wlp7s0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')
export ROS_HOSTNAME=$(ifconfig wlp7s0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')
