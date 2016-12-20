# Dependency in this package on:
sudo apt-get install libgsl0-dev 

This package is useful for finding the extrinsic parameters of a laser scanner. The calibration needs to be provided with a set of point clouds along with a "ground truth" pose estimate of the sensor in world frame, this is obtained wither with a ground truth system or a mapping framework. To generate these files, the offline fuser could be used.
Not that there is a bit of a catch 22 here incase the ground truth is specified in a vehicle coordinate system, since this is required in order to generate the point cloud (needed if the vehicles moves  / rotates fast). An offset of 0,0,0 would work but it would make more sence to run the calibration in two steps where the generation of pointclouds utilize the sensor offset. Note that the sensory time offset should be set to zero when these files are generated (otherwise these will be added to the offset we want to calibrate afterwards anyway).


# Example - this will generate a set of point cloud files from a VLP16 using a ground truth /tf link (no correction / mapping will be performed here).
rosrun ndt_offline fuser3d_offline --Dd 0.0004 --Dt 0.0004 --Cd 0.001 --Ct 0.01 --Td 0.001 --Tt 0.01 --nosubmaps --resolution 2.0 --visualize --base-name tent --dir-name ../tent --velodyne_config_file /home/han/data/VLP16db.yaml --tf_gt_link /EKF --tf_base_link /EKF --tf_world_frame /World --size-xy 500 --size-z 70 --VCEnov16 --resolution_local_factor 0.9 --fuse-incomplete --max_range 130 --min_range 3 --alive --nb_scan_msgs 1 --filter-fov --hori-max 2. --hori-min -2. --use-odometry --velodyne_packets_topic /VLP16/velodyne_packets --save_clouds --disable_reg --min_dist 0.5


# Example - this will run the calibration.     
rosrun ndt_calibration ndt_calib --gt_file tentDd0.0004Dt0.0004Cd0.001Ct0.01Td0.001Tt0.01_res2_SC0_mindist0.5_sensorcutoff70_stepcontrol1_neighbours2_rlf0.9_gt.txt --est_sensorpose_file tentDd0.0004Dt0.0004Cd0.001Ct0.01Td0.001Tt0.01_res2_SC0_mindist0.5_sensorcutoff70_stepcontrol1_neighbours2_rlf0.9_sensorpose_est.txt --x 3.64925 --y 0.00668733 --z 1.17057819451374 --ex 0.0003905 --ey 0.0331966 --ez 0.0105067 --sensor_time_offset -0.06886 --score_type 3 --objective_type 1 --max_translation 10 --min_rotation 0.5 --bag_file ../tent/tent.bag --cx --ct --cy --cez

# This arguments controls which of the sensor poses and time offset that should be enabled in the optimization.
# --cx --cy --cez --cey --cex --cz --ct
