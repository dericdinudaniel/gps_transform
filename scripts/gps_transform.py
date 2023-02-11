#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import NavSatFix

origin_latitude = None
origin_longitude = None


def gps_callback(msg: NavSatFix):
    # rospy.loginfo('Latitude: %f, Longitude: %f, Altitude: %f' %
    #               (msg.latitude, msg.longitude, msg.altitude))
    global origin_latitude, origin_longitude

    if origin_longitude is None or origin_latitude is None:
        origin_longitude = msg.longitude
        origin_latitude = msg.latitude
        rospy.loginfo('Origin set to: %f, %f' %
                      (origin_longitude, origin_latitude))
    else:
        xpoint_lat = origin_latitude
        xpoint_long = msg.longitude

        ypoint_lat = msg.latitude
        ypoint_long = origin_longitude

        x_dist = haversine_distance(
            origin_latitude, origin_longitude, xpoint_lat, xpoint_long)
        y_dist = haversine_distance(
            origin_latitude, origin_longitude, ypoint_lat, ypoint_long)

        rospy.loginfo('X: %f, Y: %f' % (x_dist, y_dist))


def reset_origin():
    origin_longitude = None
    origin_latitude = None


def haversine_distance(lat1, long1, lat2, long2):
    R = 6371e3  # earth's radius in meters

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(long2 - long1)

    a = math.sin(delta_phi / 2)**2 + \
        math.cos(phi1) * math.cos(phi2) * \
        math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c

    return distance


if __name__ == '__main__':
    rospy.init_node('gps_transform')

    sub = rospy.Subscriber('/gps/fix', NavSatFix, callback=gps_callback)
    rospy.loginfo('GPS transform node started')

    rospy.spin()
