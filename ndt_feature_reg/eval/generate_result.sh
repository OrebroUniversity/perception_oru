# # This is called from generate_all_results.sh from each data directory, place all calls to generate results files here

# evaluation of different support sizes.
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=2 --output s4eval1_s2.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=4 --output s4eval1_s4.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=6 --output s4eval1_s6.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=8 --output s4eval1_s8.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=10 --output s4eval1_s10.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=12 --output s4eval1_s12.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=14 --output s4eval1_s14.txt
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=16 --output s4eval1_s16.txt
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=18 --output s4eval1_s18.txt
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=20 --output s4eval1_s20.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=22 --output s4eval1_s22.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=24 --output s4eval1_s24.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=26 --output s4eval1_s26.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=28 --output s4eval1_s28.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=30 --output s4eval1_s30.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=32 --output s4eval1_s32.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=34 --output s4eval1_s34.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=36 --output s4eval1_s36.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=38 --output s4eval1_s38.txt
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=40 --output s4eval1_s40.txt


rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=20 --output s4ransac_s20.txt --skip_matching
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=20 --output s4eval1_s20.txt
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=20 --output s4eval3_s20.txt --estimate_di
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=20 --output s4eval4_s20.txt --match_no_association
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=20 --output s4eval5_s20.txt --match_full
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=20 --output s4eval6_s20.txt --trim_factor=0.9
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=20 --output s4eval7_s20.txt --trim_factor=0.9 --estimate_di
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=20 --output s4eval8_s20.txt --max_kp_dist=6



# # -------------------------

rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4ransac.txt --skip_matching
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval1.txt
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval3.txt --estimate_di
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval4.txt --match_no_association
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval5.txt --match_full
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval6.txt --trim_factor=0.9
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval7.txt --trim_factor=0.9 --estimate_di
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval8.txt --max_kp_dist=6
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval9.txt --max_kp_dist=2
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval10.txt --max_kp_dist=3
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval11.txt --max_kp_dist=4
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval12.txt --max_kp_dist=5
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval13.txt --max_kp_dist=7
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval14.txt --max_kp_dist=8
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval15.txt --max_kp_dist=9
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval16.txt --skip_matching --non_mean
# rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=15 --output s4eval17.txt --non_mean

# # --------------------------

rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=10 --output s4ransac_s10.txt --skip_matching
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=10 --output s4eval1_s10.txt
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=10 --output s4eval3_s10.txt --estimate_di
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=10 --output s4eval4_s10.txt --match_no_association
#rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=10 --output s4eval5_s10.txt --match_full
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=10 --output s4eval6_s10.txt --trim_factor=0.9
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=10 --output s4eval7_s10.txt --trim_factor=0.9 --estimate_di
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.25 --detector_thresh=200 --support_size=10 --output s4eval8_s10.txt --max_kp_dist=6

# # ---------------------------

# #rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.5 --output s2ransac.txt --time_output s2ransac_times.txt --skip_matching
# #rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.5 --output s2eval1.txt --time_output s2eval1_times.txt
# #rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.5 --output s2eval3.txt --time_output s2eval3_times.txt --estimate_di
# #rosrun ndt_feature_reg ndt_feature_eval --association association.txt --img_scale 0.5 --output s2eval4.txt --time_output s2eval4_times.txt --match_no_association


# # -----------------------------

rosrun ndt_feature_reg ndt_feature_eval --association association.txt --support_size=15 --output ransac.txt --time_output ransac_times.txt --skip_matching 
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --support_size=15 --output eval1.txt --time_output eval1_times.txt
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --support_size=15 --output eval3.txt --time_output eval3_times.txt --estimate_di
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --support_size=15 --output eval4.txt --time_output eval4_times.txt --match_no_association
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --support_size=15 --output eval5.txt --time_output eval5_times.txt --match_full
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --support_size=15 --output eval6.txt --trim_factor=0.9
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --support_size=15 --output eval7.txt --trim_factor=0.9 --estimate_di
rosrun ndt_feature_reg ndt_feature_eval --association association.txt --support_size=15 --output eval8.txt --max_kp_dist=6

