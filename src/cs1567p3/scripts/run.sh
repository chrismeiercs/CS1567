cd ../../..;
catkin_make;
source devel/setup.bash;
cd src/cs1567p3/scripts;
rosrun cs1567p3 WallFollower.py;
