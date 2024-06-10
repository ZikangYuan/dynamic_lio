# Dynamic_LIO

**Dynamic-LIO** is a LiDAR-inertial odometry for dynamic driving scenarios. The front-end odometry is designed based on the framework of [**SR-LIO**](https://github.com/ZikangYuan/sr_lio), and the back-end loop closure utilizes [**SC-A-LOAM**](https://github.com/gisbi-kim/SC-A-LOAM).

## Related Work

A Fast Dynamic Point Detection Method for LiDAR-Inertial Odometry in Driving Scenarios

Authors: [*Zikang Yuan*](https://scholar.google.com/citations?hl=zh-CN&user=acxdM9gAAAAJ), *Xiaoxiang Wang*, *Jingying Wu* and [*Xin Yang*](https://scholar.google.com/citations?user=lsz8OOYAAAAJ&hl=zh-CN)

[SR-LIO: LiDAR-Inertial Odometry with Sweep Reconstruction](https://arxiv.org/abs/2210.10424)

Authors: [*Zikang Yuan*](https://scholar.google.com/citations?hl=zh-CN&user=acxdM9gAAAAJ), [*Fengtian Lang*](https://scholar.google.com/citations?hl=zh-CN&user=zwgGSkEAAAAJ&view_op=list_works&gmla=ABEO0Yrl4-YPuowyntSYyCW760yxM5-IWkF8FGV4t9bs9qz1oWrqnlHmPdbt7LMcMDc04kl2puqRR4FaZvaCUONsX7MQhuAC6a--VS2pTsuwj-CyKgWp3iWDP2TS0I__Zui5da4), *Tianle Xu* and [*Xin Yang*](https://scholar.google.com/citations?user=lsz8OOYAAAAJ&hl=zh-CN)

## Installation

### 1. Requirements

> GCC >= 7.5.0
>
> Cmake >= 3.16.0
> 
> [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.3.4
>
> [OpenCV](https://github.com/opencv/opencv) >= 3.3
>
> [PCL](https://pointclouds.org/downloads/) == 1.8 for Ubuntu 18.04, and == 1.10 for Ubuntu 20.04
>
> [GTSAM](https://github.com/borglab/gtsam/tree/4.0.3) == 4.0.3 for Ubuntu 20.04
>
> [ROS](http://wiki.ros.org/ROS/Installation)

##### Have Tested On:

| OS    | GCC  | Cmake | Eigen3 | OpenCV | PCL | GTSAM |
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
| Ubuntu 20.04 | 9.4.0  | 3.16.3 | 3.3.7 | 4.2.0 | 1.10.0 | 4.0.3 |

### 2. Create ROS workspace

```bash
mkdir -p ~/Dynamic-LIO/src
cd Dynamic-LIO/src
```

### 3. Clone the directory and build

```bash
git clone https://github.com/ZikangYuan/dynamic_lio.git
cd ..
catkin_make
```

## Run on Public Datasets

Noted:

A. The warning message "Failed to find match for field 'time'." doesn't matter. It can be ignored.

B. **Please create a folder named "output" in "sr_lio" folder before running.** When **Dynamic-LIO** is running, the estimated pose of odometry is recorded in real time in the **pose.txt** located in the **output folder**, and the estimated pose with loop closure is recorded in the **final_pose.txt** located in the **output folder** after finishing a sequence.

###  1. Run on [*ULHK-CA*](https://github.com/weisongwen/UrbanLoco)

Please go to the workspace of **Dynamic-LIO** and open two terminals, type the following command in terminal 1:

```bash
cd Dynamic-LIO
source devel/setup.bash
roslaunch sr_lio lio_ulhk_CA.launch
```

and type the following command in terminal 2:

```bash
cd Dynamic-LIO
source devel/setup.bash
roslaunch aloam_velodyne loop_closure.launch
```

Then open the terminal in the path of the bag file, and type:

```bash
rosbag play SEQUENCE_NAME.bag --clock -d 1.0
```

###  2. Run on [*NTU_VIRAL*](https://ntu-aris.github.io/ntu_viral_dataset/)

Please go to the workspace of **SR-LIVO** and type:

```bash
cd SR-LIVO
source devel/setup.bash
roslaunch sr_livo livo_ntu.launch
```

Then open the terminal in the path of the bag file, and type:

```bash
rosbag play SEQUENCE_NAME.bag --clock -d 1.0
```
