#!/usr/bin/env python
print("importing libraries")
import rosbag #Lz4
import rospy
import sys, os
import argparse

import numpy as np
import csv
import pandas as pd  

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField, Imu, NavSatFix
from std_msgs.msg import Header

from pypcd_imp import pypcd
from glob import glob


#structure
# path/location/'frames_split'/{sequence}/GPS.csv
# path/location/'frames_split'/{sequence}GYRO.csv
# path/location/'frames_split'/{sequence}pcd/{pcd_timestamp:>19}.pcd

#setup the argument list
parser = argparse.ArgumentParser(description='Create a ROS bag using the LiDAR pcd and imu data.')
parser.add_argument('--path',  metavar='folder', nargs='?', help='Data folder')
parser.add_argument('--location',  metavar='location', nargs='?', help='Data folder')
#parser.add_argument('--output-bag', metavar='output_bag',  default="output.bag", help='ROS bag file %(default)s')

#print help if no argument is specified
if len(sys.argv)<2:
    parser.print_help()
    sys.exit(0)

# parse the args
parsed = parser.parse_args()
def createImuMessge(timestamp_int, omega, alpha, imu_name):
    timestamp = rospy.Time(int(timestamp_int//1000000000), int(timestamp_int%1000000000))
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.header.frame_id = imu_name
    rosimu.angular_velocity.x = float(omega.iloc[0])
    rosimu.angular_velocity.y = float(omega.iloc[1])
    rosimu.angular_velocity.z = float(omega.iloc[2])
    rosimu.linear_acceleration.x = float(alpha.iloc[0])
    rosimu.linear_acceleration.y = float(alpha.iloc[1])
    rosimu.linear_acceleration.z = float(alpha.iloc[2])
    return rosimu, timestamp

def split_time(pcd):
    ##get time from pcd file name
    pcd = pcd.split('/')[-1]
    pcd = pcd.split('.')[0]
    time = int(pcd) #str -> int
    secs = time // 1000000000   # 1_000_000_000 #sec
    nsecs = (time % 1000000000 ) # 1_000_000_000) #nsec
    return secs, nsecs

#create the bag
def getGpsCsvFiles(timestamp_int, latitude, longitude, altitude):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time(int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]))

    gps = NavSatFix()
    gps.header.stamp = timestamp
    gps.header.frame_id = 'gps'
    gps.latitude = float(latitude)
    gps.longitude = float(longitude)
    gps.altitude = float(altitude)
    gps.status.status = 1

    return gps, timestamp

def pcd2msgBypypcd(pcd_path):
    cloud = pypcd.PointCloud.from_path(pcd_path)
    #print(cloud.pc_data.dtype.names)
    x = np.asarray(cloud.pc_data["x"])
    y = np.asarray(cloud.pc_data["y"])
    z = np.asarray(cloud.pc_data["z"])
    intensity = np.asarray(cloud.pc_data["intensity"])
    #time = np.asarray(cloud.pc_data["time"])
    #data = np.vstack((x,y,z,intensity,time))
    data = np.vstack((x,y,z,intensity))
    data = data.transpose()
    secs, nsecs = split_time(pcd)
    timestamp = rospy.Time(secs=secs, nsecs=nsecs) 
    fields =[PointField('x', 0, PointField.FLOAT32, 1),
         PointField('y', 4, PointField.FLOAT32, 1),
         PointField('z', 8, PointField.FLOAT32, 1),
         PointField('intensity', 12, PointField.FLOAT32,1),
         #PointField('time', 16, PointField.FLOAT32,1)
    ]
    header = Header()
    header.frame_id = "velodyne"
    pc2 = point_cloud2.create_cloud(header, fields, data) 
    pc2.header.stamp = timestamp
    return pc2, timestamp

location = parsed.location
#try:
folder_paths = []
folder_path = os.path.join(parsed.path, location, 'frames_split')
print(folder_path)
for root, dirs, files in os.walk(folder_path):
    if len(files)== 0:
        for dir_name in dirs:                    
            partial_path = os.path.join(root, dir_name)
            folder_paths.append(partial_path)
        break  
folder_paths.sort()

for idx, sequence_folder in enumerate(folder_paths):
    print(idx, sequence_folder)
    bag = rosbag.Bag(sequence_folder + '/' + location + str(idx) + '.bag', 'w')
    print("bag location : ", sequence_folder,  ', name: ', location+str(idx))
        
    # write imu data
    imu_topic = ""
    imu_frame = ""
    try:
        print(sequence_folder  + '/GYRO.csv')
        gyro = pd.read_csv(sequence_folder  + '/GYRO.csv') 
    except:
        print("this folder is empty")
        continue

    topic = 'imu_link'
    for index, row in gyro.iterrows():
        imumsg, timestamp = createImuMessge(row.iloc[0], row.iloc[1:4], row.iloc[4:7], topic) #timestamp, gyro, accelerometer
        bag.write("imu_raw", imumsg, timestamp)
    print("imu end")

    # write lidar data 
    pcd_folders = np.sort(glob(sequence_folder +  '/pcd/*'))
    for pcd in pcd_folders:
        pc_data, timestamp = pcd2msgBypypcd(pcd)
        bag.write('points_raw', pc_data, timestamp)
    print("pcd end")

    # write GPS data 
    gpsfiles = sequence_folder + '/GPS.csv'
    gps_topic = 'gps_fix'
    with open(gpsfiles, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        headers = next(reader, None)
        last_camera_frame = -1
        cnt = 0
        for row in reader:
            if last_camera_frame == int(row[0]):
                continue
            last_camera_frame = int(row[0])
            gpsmsg, timestamp = getGpsCsvFiles(row[0], row[1], row[2], row[3]) #timestamp, lat, long, alt
            bag.write(gps_topic, gpsmsg, timestamp)
    print(f"{sequence_folder} end")
    bag.close()
        
#except:
#    print("error?")
