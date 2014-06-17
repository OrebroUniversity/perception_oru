echo '============================================== support 20'
pwd
echo '--------- ONLY RANSAC'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4ransac_s20.txt --fixed_delta --delta_unit f --plot s4ransac.png --plot_rot s4ransac_rot.png
echo '--------- NDT D2D FEATURE'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval1_s20.txt --fixed_delta --delta_unit f --plot s4d2d_incremental.png --plot_rot s4d2d_incremental_rot.png 
echo '--------- NDT D2D FEATURE - with DI estimation'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval3_s20.txt --fixed_delta --delta_unit f --plot s4d2d_di.png --plot_rot s4d2d_di_rot.png
echo '--------- NDT D2D FEATURE - no feature association'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval4_s20.txt --fixed_delta --delta_unit f --plot s4d2d_noassoc.png --plot_rot s4d2d_noassoc_rot.png
echo '--------- NDT D2D FEATURE TRIMMED'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval6_s20.txt --fixed_delta --delta_unit f --plot s4d2d_incremental_tr.png --plot_rot s4d2d_incremental_tr_rot.png  
echo '--------- NDT D2D FEATURE TRIMMED - with DI estimation'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval7_s20.txt --fixed_delta --delta_unit f --plot s4d2d_di_tr.png --plot_rot s4d2d_di_tr_rot.png
echo '--------- NDT D2D FEATURE - max dist 6'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval8_s20.txt --fixed_delta --delta_unit f --plot s4d2d_max6.png --plot_rot s4d2d_max6.png

echo '============================================== support 15'
pwd
echo '--------- ONLY RANSAC'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4ransac.txt --fixed_delta --delta_unit f --plot s4ransac.png --plot_rot s4ransac_rot.png
echo '--------- NDT D2D FEATURE'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval1.txt --fixed_delta --delta_unit f --plot s4d2d_incremental.png --plot_rot s4d2d_incremental_rot.png 
echo '--------- NDT D2D FEATURE - with DI estimation'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval3.txt --fixed_delta --delta_unit f --plot s4d2d_di.png --plot_rot s4d2d_di_rot.png
echo '--------- NDT D2D FEATURE - no feature association'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval4.txt --fixed_delta --delta_unit f --plot s4d2d_noassoc.png --plot_rot s4d2d_noassoc_rot.png
echo '--------- NDT D2D FEATURE TRIMMED'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval6.txt --fixed_delta --delta_unit f --plot s4d2d_incremental_tr.png --plot_rot s4d2d_incremental_tr_rot.png  
echo '--------- NDT D2D FEATURE TRIMMED - with DI estimation'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval7.txt --fixed_delta --delta_unit f --plot s4d2d_di_tr.png --plot_rot s4d2d_di_tr_rot.png
echo '--------- NDT D2D FEATURE - max dist 6'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval8.txt --fixed_delta --delta_unit f --plot s4d2d_max6.png --plot_rot s4d2d_max6.png


echo '============================================== support 10'
pwd
echo '--------- ONLY RANSAC'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4ransac_s10.txt --fixed_delta --delta_unit f --plot s4ransac.png --plot_rot s4ransac_rot.png
echo '--------- NDT D2D FEATURE'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval1_s10.txt --fixed_delta --delta_unit f --plot s4d2d_incremental.png --plot_rot s4d2d_incremental_rot.png 
echo '--------- NDT D2D FEATURE - with DI estimation'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval3_s10.txt --fixed_delta --delta_unit f --plot s4d2d_di.png --plot_rot s4d2d_di_rot.png
echo '--------- NDT D2D FEATURE - no feature association'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval4_s10.txt --fixed_delta --delta_unit f --plot s4d2d_noassoc.png --plot_rot s4d2d_noassoc_rot.png
echo '--------- NDT D2D FEATURE TRIMMED'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval6_s10.txt --fixed_delta --delta_unit f --plot s4d2d_incremental_tr.png --plot_rot s4d2d_incremental_tr_rot.png  
echo '--------- NDT D2D FEATURE TRIMMED - with DI estimation'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval7_s10.txt --fixed_delta --delta_unit f --plot s4d2d_di_tr.png --plot_rot s4d2d_di_tr_rot.png
echo '--------- NDT D2D FEATURE - max dist 6'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s4eval8_s10.txt --fixed_delta --delta_unit f --plot s4d2d_max6.png --plot_rot s4d2d_max6.png




# echo '--------- ONLY RANSAC'
# rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s2ransac.txt --fixed_delta --delta_unit f --verbose --plot s2ransac.png
# echo '--------- NDT D2D FEATURE'
# rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s2eval1.txt --fixed_delta --delta_unit f --verbose --plot s2d2d_incremental.png
# echo '--------- NDT D2D FEATURE - with DI estimation'
# rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s2eval3.txt --fixed_delta --delta_unit f --verbose --plot s2d2d_di.png
# echo '--------- NDT D2D FEATURE - no feature association'
# rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt s2eval4.txt --fixed_delta --delta_unit f --verbose --plot s2d2d_noassoc.png


echo '============================================== full size - support 15'
echo '--------- ONLY RANSAC'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt ransac.txt --fixed_delta --delta_unit f  --plot ransac.png
echo '--------- NDT D2D FEATURE'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt eval1.txt --fixed_delta --delta_unit f  --plot d2d_incremental.png
#echo '--------- NDT D2D FEATURE - old pairwise registration code'
#rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt eval2.txt --fixed_delta --delta_unit f  --plot d2d.png
echo '--------- NDT D2D FEATURE - with DI estimation'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt eval3.txt --fixed_delta --delta_unit f  --plot d2d_di.png
echo '--------- NDT D2D FEATURE - no feature association'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt eval4.txt --fixed_delta --delta_unit f  --plot d2d_noassoc.png
echo '--------- NDT D2D FEATURE - full d2d'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt eval5.txt --fixed_delta --delta_unit f --plot d2d_full.png
echo '--------- NDT D2D FEATURE TRIMMED'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt eval6.txt --fixed_delta --delta_unit f --plot d2d_incremental_tr.png --plot_rot d2d_incremental_tr_rot.png  
echo '--------- NDT D2D FEATURE TRIMMED - with DI estimation'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt eval7.txt --fixed_delta --delta_unit f --plot d2d_di_tr.png --plot_rot d2d_di_tr_rot.png
echo '--------- NDT D2D FEATURE - max dist 6'
rosrun ndt_feature_reg evaluate_rpe.py groundtruth.txt eval8.txt --fixed_delta --delta_unit f --plot d2d_max6.png --plot_rot d2d_max6.png
