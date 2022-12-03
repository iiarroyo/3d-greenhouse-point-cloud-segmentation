# Point cloud segmentation for agricultural scenarios using a 16 channels-LiDAR sensor with SqueezeSeg network architecture
This repository is based on [SqueezeSegs](https://github.com/navarrs/squeeze-seg) modifications from [navarrs](https://github.com/navarrs). Scripts for transforming point clouds are in `pc_utils/`.
## Specs
- Ubuntu 20.04
- python 2.7
- tensorflow 1.14


## Using our code
### Visualizing
1. Clone this repository
```
$ git clone https://github.com/iiarroyo/3d-greenhouse-point-cloud-segmentation
```
2. Download our [data](#download-dataset).
3. Visualize data
```
$ roscore
$ rosrun squeezeseg-vlp16  visualize.py --net squeezeSeg16  --classes "ng"
$ rviz
```

### Transforming point clouds
Point clouds to spherical projections
```
$ python pc_utils/semkitti2npy.py 
```
Spherical projection to image
```
$ python pc_utils/semkitti2npy.py 
```
## Download dataset
You can download our annotated point clouds [here](https://drive.google.com/drive/folders/1WCo5nAX9yEOOIaUxEV8B3qri4LREub1N?usp=sharing) under `/velodyne` and `/labels`, or as spherical projections under `npy/`.