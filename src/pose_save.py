#!/usr/bin/python
import rospy 
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import tf
import matplotlib.pyplot as plt

import csv
import numpy

class Plot:
    def __init__(self):
        rospy.init_node('pose_save_node', anonymous=True)

        rospy.Subscriber('ndt_pose', PoseStamped,  lambda y: self.ndt_pose_callback(y))
        rospy.Subscriber('ekf_pose', PoseStamped,  lambda z: self.ekf_pose_callback(z))

        self.ndt_yaw_=[]
        self.ndt_count_=[]
        self.ndt_x_=[]
        self.ndt_y_=[]
        self.ndt_z_=[]
        self.ekf_yaw_=[]
        self.ekf_count_=[]
        self.ekf_x_=[]
        self.ekf_y_=[]
        self.ekf_z_=[]
        self.ndt_number_=0
        self.ekf_number_=0
        self.ref_x_=[]
        self.ref_y_=[]
        self.ref_z_=[]
        

    def point_callback(self, point_msg):
        self.ref_x_.append(point_msg.point.x)
        self.ref_y_.append(point_msg.point.y)
        self.ref_z_.append(point_msg.point.z)
        
    def ndt_pose_callback(self, pose_msg):
        print("ndt")
        e = tf.transformations.euler_from_quaternion((pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z,pose_msg.pose.orientation.w))
        self.ndt_yaw_.append(e[2])
        self.ndt_x_.append(pose_msg.pose.position.x)
        self.ndt_y_.append(pose_msg.pose.position.y)
        self.ndt_z_.append(pose_msg.pose.position.z)
        self.ndt_count_.append(self.ndt_number_)
        self.ndt_number_ += 1
    
    def ekf_pose_callback(self, pose_msg):
        print("ekf")
        e = tf.transformations.euler_from_quaternion((pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z,pose_msg.pose.orientation.w))
        self.ekf_yaw_.append(e[2])
        self.ekf_x_.append(pose_msg.pose.position.x)
        self.ekf_y_.append(pose_msg.pose.position.y)
        self.ekf_z_.append(pose_msg.pose.position.z)
        self.ekf_count_.append(self.ndt_number_)
        self.ekf_number_ += 1
        
    
if __name__ == '__main__':
    c=Plot()
    rospy.spin()

    with open('/home/mapiv-intern/catkin_ws/src/ntt_west_plot/src/csv/ekf_pose.csv','w')  as fe:
        writer=csv.writer(fe)
        writer.writerow(['x','y','z'])
        for i in range(len(c.ekf_x_)):
            writer.writerow([c.ekf_x_[i],c.ekf_y_[i],c.ekf_z_[i]])
        
    with open('/home/mapiv-intern/catkin_ws/src/ntt_west_plot/src/csv/ndt_pose.csv','w')  as fn:
        writer=csv.writer(fn)
        writer.writerow(['x','y','z'])
        for i in range(len(c.ndt_x_)):
            writer.writerow([c.ndt_x_[i],c.ndt_y_[i],c.ndt_z_[i]])
