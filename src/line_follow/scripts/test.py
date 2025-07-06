import rospy
from geometry_msgs.msg import Twist
import cv2
import time
from ztestnav2025.srv import lidar_process,lidar_processRequest,lidar_processResponse

rospy.init_node("test")

vel_msg = Twist()
vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
lidar_client = rospy.ServiceProxy("lidar_process/lidar_process", lidar_process)
lidar_Req = lidar_processRequest()
lidar_Req.lidar_process_start = -1

flag = 1
integration_y = 0
integration_z = 0
while flag:
    lidar_resp = lidar_client.call(lidar_Req)
    if(lidar_resp.lidar_results[0]!=-1):
        integration_y = max(min(integration_y + lidar_resp.lidar_results[1],-0.1),0.1)
        integration_z = max(min(integration_z + lidar_resp.lidar_results[2]/lidar_resp.lidar_results[3],-0.1),0.1)
        vel_msg.linear.y = max(min(lidar_resp.lidar_results[1]+integration_y, 0.1), -0.1)
        vel_msg.angular.z = max(min((lidar_resp.lidar_results[2]/lidar_resp.lidar_results[3]+integration_z) * -1, 0.1), -0.1)
        
        print(vel_msg.linear.y)
        print(vel_msg.angular.z)
        print(integration_y)
        print(integration_z)
        vel_publisher.publish(vel_msg) 
        if abs(vel_msg.linear.y) > 0.08:
            integration_y = 0
        if abs(vel_msg.angular.z) > 0.08:
            integration_z = 0
        if abs(vel_msg.linear.y) < 0.03 and abs(vel_msg.angular.z) < 0.05:
            print("done")
            break
start = time.time()
while flag :
    if time.time()-start > 2:
        flag = 0
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_msg.linear.y = 0.3
    vel_publisher.publish(vel_msg)
flag = 1
start = time.time()
while flag :
    if time.time()-start > 1.2:
        flag = 0
    vel_msg.linear.x = 0.4
    vel_msg.angular.z = 0
    vel_msg.linear.y = 0
    vel_publisher.publish(vel_msg)
flag = 1
start = time.time()
while flag :
    if time.time()-start > 2:
        flag = 0
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_msg.linear.y = -0.3
    vel_publisher.publish(vel_msg)
vel_msg.linear.x = 0
vel_msg.angular.z = 0
vel_msg.linear.y = 0
vel_publisher.publish(vel_msg)