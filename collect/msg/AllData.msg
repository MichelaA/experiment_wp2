
std_msgs/Header header
## contact point and wrench on each thimble
intrinsic_tactile_toolbox/TactileInfo[] thimbles

#phase space markers
visualization_msgs/MarkerArray LEDs

#point cloud of the scene
sensor_msgs/PointCloud2 scene

#tranformations
geometry_msgs/TransformStamped world_wrist_link
geometry_msgs/TransformStamped object_star_link
