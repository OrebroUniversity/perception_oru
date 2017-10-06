#!/bin/bash
FILES=./*Ts.gt
# d2d_est_end='Ts.d2d'
# d2d_sc_est_end='Ts.d2d_sc'
# filter_est_end='Ts.filter'
est_end='.Ts.filter'
png_end='.png'
txt_end='.txt'
rot='rot'
pos='pos'
rpe='.rpe'
ate='.ate'
for gt in $FILES
do
  # Remove the ending _gt.txt
    base_name=${gt%.Ts.gt}
    echo "Processing $base_name files..."

    est=$base_name$est_end
    echo $est

    atepng=$base_name$est_end$ate$png_end
    ateres=$base_name$est_end$ate

    rperes=$base_name$est_end$rpe

    # Processing 
    echo "rosrun ndt_feature_reg evaluate_ate.py $est $gt --plot $atepng --verbose > $ateres"
          rosrun ndt_feature_reg evaluate_ate.py $est $gt --plot $atepng --verbose > $ateres
    echo "rosrun ndt_feature_reg evaluate_rpe.py $est $gt --plot $est$pos$png_end --plot_rot $est$rot$png_end --fixed_delta --verbose > $rperes"
          rosrun ndt_feature_reg evaluate_rpe.py $est $gt --plot $est$pos$png_end --plot_rot $est$rot$png_end --fixed_delta --verbose > $rperes

done
