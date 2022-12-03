#! /home/iiarroyo/miniconda3/envs/python27/bin/python
import rospy
import os.path
import os
import sys
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as rosImage
from PIL import Image

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

from itertools import count
def _normalize(x):
  return (x - x.min())/(x.max() - x.min())

def crop_cloud(cloud, fov=(-45, 45)):
    '''
    function: crop cloud by fov of (-45, 45) at azimuth 
    input: cloud: (n, 4)-->(x, y, z, i)
    ouput: cloud: (m, 4)-->(x, y, z, i) (m ~= n/4)
    ''' 
    x = cloud[:, 0]
    y = cloud[:, 1]
    azi = np.arctan2(y, x).reshape(len(x), 1) * 180.0 / np.pi
    cloud_xyzia = np.concatenate((cloud, azi), axis=1)#(x, y, z, i, azimuth)
    crop_index = (cloud_xyzia[:, 4] > fov[0]) &  (cloud_xyzia[:, 4] < fov[1])#fov
    crop_points = cloud_xyzia[crop_index]
    return crop_points[:, :4] # (x, y, z, i)


def projectCloud2Image(cloud, width=512, height=16, fov=(-45, 45)):
    '''
    function: project cloud to spherical image
    input:  cloud:           (n, 4)      --> (x, y, z, i)
    output: spherical image: (64, 512, 5)--> (x, y, z, i, r)
    '''
    n = cloud.shape[0]
    x = cloud[:, 0].reshape(n, 1)
    y = cloud[:, 1].reshape(n, 1)
    z = cloud[:, 2].reshape(n, 1)
    r = np.sqrt(x**2 + y**2 + z**2).reshape(n, 1)

    yaw = np.arctan2(y, x).reshape(n, 1)
    pitch = np.arcsin(z/r).reshape(n, 1)

    #compute resolution of the cloud at each direction
    resolution_w = (yaw.max() - yaw.min()) / (width - 1)
    resolution_h = (pitch.max() - pitch.min()) / (height - 1)

    #compute each point's grid index in the image
    index_w = np.floor(((yaw - yaw.min()) / resolution_w)).astype(np.int)
    index_h = np.floor((pitch - pitch.min()) / resolution_h).astype(np.int)

    cloud_xyzir = np.concatenate((cloud, r), axis=1) #(x,y,z,i,r)
    
    spherical_image = np.zeros((height, width, 5))
    spherical_image[index_h, index_w, :] = cloud_xyzir[:, np.newaxis, :] #broadcast
    spherical_image = spherical_image[::-1, ::-1, :] #reverse image
    return spherical_image


def velo_callback(msg):
    # print("\nvelo_callback---------start")
    pcl_msg = pc2.read_points(msg, skip_nans=False, field_names=(
    x_channel, y_channel, z_channel, i_channel))
    cloud_input = np.array(list(pcl_msg), dtype=np.float32)
    print("\ncloud_input.shape = ", cloud_input.shape)

    #intensity must be(0, 1), normalize intensity in order for (0, 255)
    cloud_input[:, 3] = _normalize(cloud_input[:, 3])

    #crop and project cloud to spherical image
    crop_points = crop_cloud(cloud_input) #[-45, 45], (x,y,z,i)
    print("crop_points.shape = ", crop_points.shape)
    feature_image = projectCloud2Image(crop_points)#(64, 512, 5), 5->(x,y,z,i,range)
    print("feature_image.shape = ", feature_image.shape)
    # print(f"vineyard1_{next(iterator):04d}.npy")

    with open(f"centralhall1_{next(iterator):04d}.npy", 'wb') as fp:
        np.save(fp, feature_image)



if __name__ == '__main__':
    print("[+] squeezeseg_ros_node has started!")
    rospy.init_node('squeezeseg_ros_node')

    #read parameters from launch file
    pub_topic = "/velodyne_points_squeeze"
    sub_topic = "/velodyne_points"
    frame_id = "velodyne"
    x_channel = "x"
    y_channel = "y"
    z_channel = "z"
    i_channel = "intensity"

    # device_id = rospy.get_param('device_id')


    # publish and subscribe
    sub_velo_ = rospy.Subscriber(sub_topic, PointCloud2, velo_callback, queue_size=10)
    pub_velo_ = rospy.Publisher(pub_topic, PointCloud2, queue_size=10)
    iterator = count(start = 1, step = 1)

                            
    rospy.spin()
