#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import math
import sys
import pickle
import numpy as np

from scipy.interpolate import RegularGridInterpolator
from selfie_map_processing.msg import PathWithMeta
from std_msgs.msg import Float64
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.server import Server
from selfie_map_processing.cfg import MapProcessingConfig

UPDATE_RATE = 50

def generate_interpolation(lookup_table, resolution, origin, method='linear'):
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

    return RegularGridInterpolator((y, x), lookup_table, method=method)

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

    eval_closest = generate_interpolation(map_data['closests'],
                                          map_data['resolution'],
                                          map_data['origin'],
                                          method='nearest')

    pathpoints = map_data['pathpoints']
    directions = map_data['directions']
    widths = map_data['widths']

    rospy.loginfo('Map data loaded')

    # Announce topic publishers
    offset_pub = rospy.Publisher('steering_state', Float64, queue_size=UPDATE_RATE)
    path_pub = rospy.Publisher('path', Path, queue_size=UPDATE_RATE)
    path_with_meta_pub = rospy.Publisher('path_with_meta', PathWithMeta, queue_size=UPDATE_RATE)
    width_pub = rospy.Publisher('track_width', Float64, queue_size=UPDATE_RATE)

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
        closest_idx = int(eval_closest([y, x])[0])
        path_direction = directions[closest_idx]
        theta = euler[2] - path_direction

        offset_pub.publish(Float64(offset + L*math.sin(theta)))

        path_with_meta = PathWithMeta()
        path_with_meta.path = Path()
        path_with_meta.path.header.frame_id = 'map'
        path_with_meta.path.poses = []
        path_with_meta.track_width = []

        for i in range(20):
            point_idx = (closest_idx + i) % len(pathpoints)
            point = pathpoints[point_idx]

            pose = PoseStamped()
            pose.header.frame_id = 'map'

            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            path_with_meta.path.poses.append(pose)

            width = widths[point_idx]
            path_with_meta.track_width.append(width)

        path_with_meta_pub.publish(path_with_meta)
        path_pub.publish(path_with_meta.path)

        width = widths[closest_idx]
        width_pub.publish(width)

        rate.sleep()