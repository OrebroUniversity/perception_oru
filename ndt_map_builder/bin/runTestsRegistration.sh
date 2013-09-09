#!/bin/bash

#NDT2NDT

echo "----------------RUNINH TESTS FOR D2D-----------------------"
./test_ndt_mapper 60 ~/datasets/real/aass_loop/cloud bLaser 0 bD2D 1
mv ~/datasets/real/aass_loop/cloud.dat results/aass_D2D_nohist.dat
mv ~/datasets/real/aass_loop/cloud.g2o results/g2o/aass_D2D_nohist.g2o
mv ~/datasets/real/aass_loop/cloud.dat.times results/m/aass_D2D_nohist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/aass.dat results/aass_D2D_nohist.dat
mv ~/datasets/ground\ truth/aass.dat_res.m results/m/aass_D2D_nohist_errors.m

./test_ndt_mapper 60 ~/datasets/real/aass_loop/cloud bLaser 1 bD2D 1
mv ~/datasets/real/aass_loop/cloud.dat results/aass_D2D_hist.dat
mv ~/datasets/real/aass_loop/cloud.g2o results/g2o/aass_D2D_hist.g2o
mv ~/datasets/real/aass_loop/cloud.dat.times results/m/aass_D2D_hist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/aass.dat results/aass_D2D_hist.dat
mv ~/datasets/ground\ truth/aass.dat_res.m results/m/aass_D2D_hist_errors.m


./test_ndt_mapper 923 ~/datasets/real/hannover2/cloud bLaser 0 bD2D 0
mv ~/datasets/real/hannover2/cloud.dat results/hannover2_D2D_nohist.dat
mv ~/datasets/real/hannover2/cloud.g2o results/g2o/hannover2_D2D_nohist.g2o
mv ~/datasets/real/hannover2/cloud.dat.times results/m/hannover2_D2D_nohist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/hannover2.dat results/hannover2_D2D_nohist.dat
mv ~/datasets/ground\ truth/hannover2.dat_res.m results/m/hannover2_D2D_nohist_errors.m

./test_ndt_mapper 923 ~/datasets/real/hannover2/cloud bLaser 1 bD2D 0
mv ~/datasets/real/hannover2/cloud.dat results/hannover2_D2D_hist.dat
mv ~/datasets/real/hannover2/cloud.g2o results/g2o/hannover2_D2D_hist.g2o
mv ~/datasets/real/hannover2/cloud.dat.times results/m/hannover2_D2D_hist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/hannover2.dat results/hannover2_D2D_hist.dat
mv ~/datasets/ground\ truth/hannover2.dat_res.m results/m/hannover2_D2D_hist_errors.m

#PNT2NDT

echo "----------------RUNINH TESTS FOR P2D-----------------------"
./test_ndt_mapper 60 ~/datasets/real/aass_loop/cloud bLaser 0 bP2D 1
mv ~/datasets/real/aass_loop/cloud.dat results/aass_P2D_nohist.dat
mv ~/datasets/real/aass_loop/cloud.g2o results/g2o/aass_P2D_nohist.g2o
mv ~/datasets/real/aass_loop/cloud.dat.times results/m/aass_P2D_nohist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/aass.dat results/aass_P2D_nohist.dat
mv ~/datasets/ground\ truth/aass.dat_res.m results/m/aass_P2D_nohist_errors.m

./test_ndt_mapper 60 ~/datasets/real/aass_loop/cloud bLaser 1 bP2D 1
mv ~/datasets/real/aass_loop/cloud.dat results/aass_P2D_hist.dat
mv ~/datasets/real/aass_loop/cloud.g2o results/g2o/aass_P2D_hist.g2o
mv ~/datasets/real/aass_loop/cloud.dat.times results/m/aass_P2D_hist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/aass.dat results/aass_P2D_hist.dat
mv ~/datasets/ground\ truth/aass.dat_res.m results/m/aass_P2D_hist_errors.m


./test_ndt_mapper 923 ~/datasets/real/hannover2/cloud bLaser 0 bP2D 0
mv ~/datasets/real/hannover2/cloud.dat results/hannover2_P2D_nohist.dat
mv ~/datasets/real/hannover2/cloud.g2o results/g2o/hannover2_P2D_nohist.g2o
mv ~/datasets/real/hannover2/cloud.dat.times results/m/hannover2_P2D_nohist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/hannover2.dat results/hannover2_P2D_nohist.dat
mv ~/datasets/ground\ truth/hannover2.dat_res.m results/m/hannover2_P2D_nohist_errors.m

./test_ndt_mapper 923 ~/datasets/real/hannover2/cloud bLaser 1 bP2D 0
mv ~/datasets/real/hannover2/cloud.dat results/hannover2_P2D_hist.dat
mv ~/datasets/real/hannover2/cloud.g2o results/g2o/hannover2_P2D_hist.g2o
mv ~/datasets/real/hannover2/cloud.dat.times results/m/hannover2_P2D_hist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/hannover2.dat results/hannover2_P2D_hist.dat
mv ~/datasets/ground\ truth/hannover2.dat_res.m results/m/hannover2_P2D_hist_errors.m

#ICP
echo "----------------RUNINH TESTS FOR ICP-----------------------"
./test_ndt_mapper 60 ~/datasets/real/aass_loop/cloud bLaser 0 bICP 1
mv ~/datasets/real/aass_loop/cloud.dat results/aass_ICP_nohist.dat
mv ~/datasets/real/aass_loop/cloud.g2o results/g2o/aass_ICP_nohist.g2o
mv ~/datasets/real/aass_loop/cloud.dat.times results/m/aass_ICP_nohist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/aass.dat results/aass_ICP_nohist.dat
mv ~/datasets/ground\ truth/aass.dat_res.m results/m/aass_ICP_nohist_errors.m

./test_ndt_mapper 60 ~/datasets/real/aass_loop/cloud bLaser 1 bICP 1
mv ~/datasets/real/aass_loop/cloud.dat results/aass_ICP_hist.dat
mv ~/datasets/real/aass_loop/cloud.g2o results/g2o/aass_ICP_hist.g2o
mv ~/datasets/real/aass_loop/cloud.dat.times results/m/aass_ICP_hist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/aass.dat results/aass_ICP_hist.dat
mv ~/datasets/ground\ truth/aass.dat_res.m results/m/aass_ICP_hist_errors.m


./test_ndt_mapper 923 ~/datasets/real/hannover2/cloud bLaser 0 bICP 0
mv ~/datasets/real/hannover2/cloud.dat results/hannover2_ICP_nohist.dat
mv ~/datasets/real/hannover2/cloud.g2o results/g2o/hannover2_ICP_nohist.g2o
mv ~/datasets/real/hannover2/cloud.dat.times results/m/hannover2_ICP_nohist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/hannover2.dat results/hannover2_ICP_nohist.dat
mv ~/datasets/ground\ truth/hannover2.dat_res.m results/m/hannover2_ICP_nohist_errors.m

./test_ndt_mapper 923 ~/datasets/real/hannover2/cloud bLaser 1 bICP 0
mv ~/datasets/real/hannover2/cloud.dat results/hannover2_ICP_hist.dat
mv ~/datasets/real/hannover2/cloud.g2o results/g2o/hannover2_ICP_hist.g2o
mv ~/datasets/real/hannover2/cloud.dat.times results/m/hannover2_ICP_hist_times.m
/home/tsv/code/oru-ros-pkg/pointcloud_vrml/bin/./trajectory ~/datasets/ground\ truth/hannover2.dat results/hannover2_ICP_hist.dat
mv ~/datasets/ground\ truth/hannover2.dat_res.m results/m/hannover2_ICP_hist_errors.m

