#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import math
import sys
import pickle
import numpy as np

from scipy.interpolate import RegularGridInterpolator
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from selfie_map_processing.cfg import MapProcessingConfig

UPDATE_RATE = 50

def generate_interpolation(lookup_table, resolution, origin):
    dim_x = lookup_table.shape[1] * resolution
    dim_y = lookup_table.shape[0] * resolution

    start_x = map_data['origin'][0]
    start_y = map_data['origin'][1]

    stop_x = start_x + dim_x
    stop_y = start_y + dim_y

    x = np.linspace(start_x, stop_x, dim_x/resolution, endpoint=False)
    y = np.linspace(start_y, stop_y, dim_y/resolution, endpoint=False)

    # Account for y direction being inverted in images
    lookup_table = np.flipud(lookup_table)

    return RegularGridInterpolator((y, x), lookup_table)

def config_callback(config, level):
    global L
    L = config['L']

    rospy.loginfo("Reconfigure request: L=" + str(L) + "m")

    return config

if __name__ == '__main__':
    rospy.init_node('selfie_offset_publisher')

    srv = Server(MapProcessingConfig, config_callback)

    # Read output of map preprocessing
    filename = sys.argv[1]
    with open(filename, 'rb') as f:
        map_data = pickle.load(f)

    eval_offset = generate_interpolation(map_data['offsets'],
                                         map_data['resolution'],
                                         map_data['origin'])

    eval_direction = generate_interpolation(map_data['directions'],
                                            map_data['resolution'],
                                            map_data['origin'])

    rospy.loginfo('Map data loaded')

    # Announce topic publisher
    offset_pub = rospy.Publisher('steering_state', Float64, queue_size=UPDATE_RATE)

    # Configure transform listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Set update/publishing rate
    rate = rospy.Rate(UPDATE_RATE)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        try:
            pose = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        x = pose.translation.x
        y = pose.translation.y

        quaternion = (
            pose.rotation.x,
            pose.rotation.y,
            pose.rotation.z,
            pose.rotation.w
        )

        euler = tf_conversions.transformations.euler_from_quaternion(quaternion)

        offset = eval_offset([y, x])[0]
        path_direction = eval_direction([y, x])[0]

        theta = path_direction - euler[2]

        offset_pub.publish(Float64(offset + L*math.sin(theta)))

        rate.sleep()
