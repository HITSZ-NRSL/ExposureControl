# Automated Exposure Control
For robots’ robust localization in varying illumination environments, the study proposes a novel automated camera-exposure control framework to capture the best-exposed images.
![image](videos/video.gif)

  Performance comparison with various image quality metrics.
![image](images/Figure13.png#pic_center)

  Qualitative performance evaluation with different  exposure control methods.
![image](images/Figure16.png#pic_center)

# Settings
- System: ubuntu 16.04
- C++ version: c++14
- OpenCV 3.3.4
- g++/gcc >= 7.0
    - [How to upgrade your g++ and gcc?](https://www.zybuluo.com/iStarLee/note/1260368)
- cmake >= 3.10
    - [upgrade your cmake](https://www.zybuluo.com/iStarLee/note/1739997)

# C/C++
* **Feature Match**<br>

  The  test code is to compare the matching performance of  different Image Quality Metrics, corresponds to the results in table 2.
```
cd src/FeatureMatch
mkdir build
cd build 
./KLT_EXPOSURE
```
* **Automated Exposure Control**<br>

  The  test code is utilized to control the exposure parameters of Intel D435i camera. To dynamically control the exposure parameters of D435i, please install or build from source the SDK as https://github.com/IntelRealSense/librealsense.
```
git clone https://github.com/HITSZ-NRSL/ExposureControl.git
cd ExposureControl 
catkin_make
source ./devel/setup.bash
# launch camera.
roslaunch realsense2_camera rs_camera.launch

# run camera dynamic_reconfigure.
rosrun dynamic_reconfigure dynparam_d435i.py

# run camera exposure control code.
rosrun stander_tutorials ExposureControl 
```
## Citation

If you use our work, please cite:
```
@ARTICLE{AURO-D-21-00050R1,
         author={Yu Wang, Haoyao Chen,Shiwu Zhang and Wencan Lu},
         journal={Autonomous Robots},
         title={Automated Camera-Exposure Control for Robust Localization in Varying Illumination Environments},
         year={2022},
         note={Accept.}}
```
## LICENSE
The source code is released under GPLv3 license.