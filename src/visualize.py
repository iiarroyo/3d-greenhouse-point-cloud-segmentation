#!/home/iiarroyo/miniconda3/envs/python27/bin/python
"""
Segmentation on LiDAR point cloud using SqueezeSeg Neural Network
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import tensorflow as tf
import rospy
import os
from ros.npy_visualizer1 import VisualizeNode

FLAGS = tf.app.flags.FLAGS
tf.app.flags.DEFINE_string('checkpoint', '../log/log16_G_CRF/train/model.ckpt-16500', """Path to the model parameter file.""")
tf.app.flags.DEFINE_string('input_path', '../data/vlp16_npy/*', """Input lidar scan to be detected.""")
tf.app.flags.DEFINE_string('gpu', '0', """gpu id.""")
tf.app.flags.DEFINE_string("topic_pcl", "/squeeze_seg/points", """Topic to publish point clouds.""")
tf.app.flags.DEFINE_string("topic_label", "/squeeze_seg/label_map", """Topic to pusblish label maps.""")
tf.app.flags.DEFINE_string("topic_intensity", "/squeeze_seg/intensity_map", """Topic to pusblish intensity maps.""")
tf.app.flags.DEFINE_string("topic_range", "/squeeze_seg/range_map", """Topic to pusblish range maps.""")
tf.app.flags.DEFINE_integer("rate", 20, """Publishing rate.""")
tf.app.flags.DEFINE_string('classes', 'ext', """Extended classes.""")
tf.app.flags.DEFINE_integer('crf', 1, """Using CRF""")
tf.app.flags.DEFINE_string('net', 'squeezeSeg', """Neural net architecture.""")

def main(argv=None):

	node = VisualizeNode(FLAGS=FLAGS) 
	rospy.logwarn('finished.')

if __name__ == '__main__':
    tf.app.run()
