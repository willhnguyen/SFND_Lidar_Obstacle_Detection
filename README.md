# Sensor Fusion Self-Driving Car: Lidar Obstacle Detection

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

In this project, students are tasked with processing and visualizing LiDAR data. LiDAR captures high resolution data by casting a series of lasers and sensing the returned signals. On self-driving cars, the LiDAR sensor is usually a rotating device located at the top of car, but may also be accompanied by additional LiDAR sensors around the sides of the car.

LiDAR data processing involves filtering and clustering point cloud data. By doing so, we can separate the various entities on the road (e.g. road, cars, street signs) from one another. Identifying these entities is important as it influences path planning and decisions that ensure passenger safety.

The first step in point cloud clustering is identifying the road. Assuming a flat surface, the road can be segmented by using RANSAC to identify the majority of the points that lie on the same plane. RANSAC for planes involves randomly choosing a series of 3 points to define a plane, collecting the points that lie on the defined plane, and choosing the plane that contains the most points.

The second step in point cloud clustering is separating non-road entities from one another using Euclidean clustering. Euclidean clustering involves identifying and grouping points that are close to other points within a defined distance threshold. To ensure this computation happens quickly, a Kd-tree can be used to limit search to the most relevant regions.


## Development Setup
The lidar obstacle detection project requires the [Point Cloud Library](https://pointclouds.org) to be installed.

### Ubuntu 
```bash
sudo apt update
sudo apt install libpcl-dev
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh)

2. install PCL
```bash
brew update
brew tap brewsci/science
brew options pcl
brew install pcl
```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)

## Build and Run
```brew
cd ~
git clone https://github.com/willhnguyen/SFND_Lidar_Obstacle_Detection.git
cd SFND_Lidar_Obstacle_Detection
mkdir build && cd build
cmake ..
make
./environment
```

## Notes
Euclidean clustering is implemented two different ways. The first way is akin to breadth first search involving a queue of nodes to be visited. The second is akin to depth first search involving a recursive call stack. The depth first search method seems to be a second or so faster. Both implementations run within 6-9 milliseconds.

The implemented plane segmentation using RANSAC is a bottleneck with a runtime of around 25 milliseconds when max iterations is set to 30. Although this number seems low, it hasn't chosen a bad plane fitting as of yet.

With the selected hyperparameters, cars are successfully segmented without gaining or losing cluster identification during a run (as long as the car is visible and not occluded by other cars or objects). However, poles possibly representing street signs may disappear a few times.
