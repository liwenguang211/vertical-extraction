# ** Voxel Projection-based Method for Vertical Feature Extraction From Mobile LiDAR Point Clouds**
# **1. Introduction**
**Voxel Projection-based Method** This paper introduces a voxel projection-based image construction method that forms the basis for a feature extraction technique aimed at extracting feature from sparse and noisy 3D point clouds. Additionally, a image based silent feature extraction is employed to extract vertical features. The image based feature extraction method proposed in the paper is beneficial for the current system and may offer an intuitive solution for infrastructure monitoring and urban digital twin applications. A PCA based noise remove method greatly improves the accuracy of extraction results. Discovering an effective vertical feature extraction method has been validated as beneficial for modeling and locating the environments.


# **2. Prerequisites**

## **2.1 Ubuntu and [ROS](https://www.ros.org/)**
We tested our code on Ubuntu18.04 with ros melodic and Ubuntu20.04 with noetic. Additional ROS package is required:
```
sudo apt-get install ros-xxx-pcl-conversions
```

## **2.2 Eigen**
Following the official [Eigen installation](eigen.tuxfamily.org/index.php?title=Main_Page), or directly install Eigen by:
```
sudo apt-get install libeigen3-dev
```
## **2.3. ceres-solver (version>=2.1)**
Please kindly install ceres-solver by following the guide on [ceres Installation](http://ceres-solver.org/installation.html). Notice that the version of ceres-solver should higher than [ceres-solver 2.1.0](https://github.com/ceres-solver/ceres-solver/releases/tag/2.1.0)

## **2.4. GTSAM**
Following the official [GTSAM installation](https://gtsam.org/get_started/), or directly install GTSAM 4.x stable release by:
```
# Add PPA
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update  # not necessary since Bionic
# Install:
sudo apt install libgtsam-dev libgtsam-unstable-dev
```
**!! IMPORTANT !!**: Please do not install the GTSAM of ***develop branch***, which are not compatible with our code! We are still figuring out this issue.


## **2.5 Prepare for the data**
Since this repo does not implement any method (i.e., LOAM, LIO, etc) for solving the pose for registering the LiDAR scan. So, you have to prepare one set of data for reproducing our results, include: ** the LiDAR point cloud data.**


# **3. Examples**
This reposity contains implementations of Stable Triangle Descriptor, as well as demos for vertical extraction and loop closure correction. For the **complete pipline of online LiDAR SLAM**, we will release this code along with the release of the **extended version**.

## **3.1. Example-1: vertical extraction with KITTI Odometry dataset**


Then, you should modify the **demo_verticalExt.launch** file
- Set the **pointCloudsPath** to your pointsCloud
- Set the **result_path** to your result save path
the result will be saved as "tree_filtered.pcd" 
  "ground.pcd"
  "corner.pcd"
```
cd $Catkin_ROS_DIR
source deve/setup.bash
roslaunch verticalDetector demo_verticalExt.launch
```


# **Contact Us**
We are still working on improving the performance and reliability of our codes. For any technical issues, please contact us via email Chongjian Yuan < ycj1ATconnect.hku.hk >, Jiarong Lin < ziv.lin.ljrATgmail.com >.




# **License**
The source code of this package is released under [**GPLv2**](http://www.gnu.org/licenses/) license. We only allow it free for personal and academic usage. For commercial use, please contact us to negotiate a different license.


