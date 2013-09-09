#!/bin/bash

#NDT2NDT
echo "----------------RUNINH TESTS FOR D2D-----------------------"
#valgrind --tool=callgrind --callgrind-out-file=profiler ./test_ndt_mapper 60 /data/datasets/real/aass_loop/cloud bLaser 0 bD2D 1
./test_ndt_mapper 60 /data/datasets/real/aass_loop/cloud bLaser 0 bD2D 1
mv /data/datasets/real/aass_loop/cloud.dat results/aass_D2D_nohist.dat
mv /data/datasets/real/aass_loop/cloud.g2o results/g2o/aass_D2D_nohist.g2o
mv /data/datasets/real/aass_loop/cloud.dat.times results/m/aass_D2D_nohist_times.m
/home/tsv/code/oru-ros-pkg/perception_oru/pointcloud_vrml/bin/./trajectory /data/datasets/ground\ truth/aass.dat results/aass_D2D_nohist.dat
mv /data/datasets/ground\ truth/aass.dat_res.m results/m/aass_D2D_nohist_errors.m
