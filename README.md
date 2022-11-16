# hand_mediapipe_ros

## ROS for python3.7.9 environment

- Create a new conda environment with python3.7.9 called mediapipe\
  `conda create -n mediapipe python=3.7.9`
- Add python3 path in bashrc\
  `echo "export PATH="/home/${user_name}/anaconda3/envs/mediapipe/bin:$PATH" >> ~/.bashrc`
- Install mediapipe, catkin-tools and rospkg in mediapipe environment\
  `pip install mediapipe`\
  `pip install catkin-tools`\
  `pip install rospkg`
 
## cv_brige for python3.7.9 in ros
- Install dependence\
  `pip install rosdep rosinstall catkin_pkg rospkg numpy pyyaml opencv-python`
- Init a catkin workspace for build cv_brige called ros_cv_bridge\
  `mkdir -p ros_cv_bridge/src && cd ros_cv_bridge/src`\
  `catkin_init_workspace`\
  `git clone https://gitee.com/irvingao/vision_opencv.git`\
  `cd ../`\
  `export CPLUS_INCLUDE_PATH=/home/${user_name}/anaconda3/include/python3.7m`\
  `catkin_make install -DCMAKE_BUILD_TYPE=Release -DSETUPTOOLS_DEB_LAYOUT=OFF -DPYTHON_EXECUTABLE=/home/${user_name}/anaconda3/bin/python`
- Add the cv_brige to bashrc\
  `echo "source ~/ros_cv_bridge/install/setup.bash --extend" >> ~/.bashrc`
## build
`catkin_make -DPYTHON_EXECUTABLE=/home//${user_name}/anaconda3/bin/python`
