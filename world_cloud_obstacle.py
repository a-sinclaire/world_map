import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
import numpy as np
from std_msgs.msg import Header as HeaderMsg
import time

world_pub = rospy.Publisher('/world_cloud_full', PointCloud2, queue_size=1)
world_region_pub = rospy.Publisher('/world_cloud_region', PointCloud2, queue_size=1)


def callback(world_cloud_msg):
    # convert to o3d
    t1 = time.time()
    world_cloud_o3d = o3d.geometry.PointCloud()
    arr = np.array(list(pc2.read_points(world_cloud_msg)))[:, 0:3]
    world_cloud_o3d.points = o3d.utility.Vector3dVector(arr)
    t2 = time.time()

    # down-sample (not sure if this is needed, might just be slow)
    t3 = time.time()
    # THIS IS THE WORLD MAP
    world_cloud_o3d_filtered = world_cloud_o3d.voxel_down_sample(voxel_size=0.07)
    t4 = time.time()

    # create bounding box we use to crop cloud
    t5 = time.time()
    # TODO: make bounds relative to camera position
    bounding_box = o3d.geometry.AxisAlignedBoundingBox()
    bounding_box.min_bound = [-1, -1, -1]
    bounding_box.max_bound = [1, 1, 1]
    # strip away unnecessary points not near scooter using crop
    world_cloud_o3d_crop = world_cloud_o3d_filtered.crop(bounding_box)
    t6 = time.time()

    # convert o3d cloud to ros msg
    t7 = time.time()
    # publish down-sampled world cloud region near scooter
    header = HeaderMsg()
    header.frame_id = '/camera_link'
    header.stamp = rospy.Time.now()
    cloud_arr = np.array(world_cloud_o3d_crop.points)
    ros_cloud = pc2.create_cloud_xyz32(header, cloud_arr)
    world_region_pub.publish(ros_cloud)
    t8 = time.time()

    # TODO: orientation of cloud may be an issue...
    # can we maybe align it? we know the odom tf should be identical to the tf of the camera capturing data

    # TODO: use arm.py code to add obstacle cloud to openrave

    time_message = "Converting to o3d cloud: {} seconds\nDown-Sampling: {} seconds\nCropping: {} seconds\nConverting " \
                   "to ros-msg: {} seconds "
    print(time_message.format(t2 - t1, t4 - t3, t6 - t5, t8 - t7))
    total = "Total time: {} seconds\n"
    print(total.format(t2 - t1 + t4 - t3 + t6 - t5 + t8 - t7))


if __name__ == '__main__':
    # TODO: have a launch file that auto starts rtab? make rtab downsample?
    rospy.init_node('world_cloud_obstacle')

    # subscribe to rtab /cloud_map
    rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, callback, queue_size=1)

    rospy.spin()

    # TODO: NOTES
    #   the slowest part by far is converting from pointcloud2 to o3d
    #   it will get slower as time goes on due to the increasing size of the world pointcloud
    #   perhaps we could purge parts of the cloud that are far away
    #   perhaps we could downsample it to an extreme amount
    #   perhaps both. but all would need to be done prior to conversion which will be difficult
    #   #
    #   this script will only need to run once the scooter is stopped
    #   so we only need to load the cloud once.
    #   instead of listening to a topic rtab can save the world cloud to a file that we load in
    #   then convert to o3d and do everything else
    #   #
    #   also need to see if can get rtab working with occipital structure cameras -- otherwise maybe mount realsense lol
