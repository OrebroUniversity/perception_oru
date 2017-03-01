#!/bin/bash
FILES=./*Ts.gt
# d2d_est_end='Ts.d2d'
# d2d_sc_est_end='Ts.d2d_sc'
# filter_est_end='Ts.filter'
est_end='.Ts.d2d_sc'
png_end='.png'
txt_end='.txt'
rot='rot'
pos='pos'
rpe='rpe'
for gt in $FILES
do
  # Remove the ending _gt.txt
    base_name=${gt%.Ts.gt}
    echo "Processing $base_name files..."

    est=$base_name$est_end
    echo $est

    png=$base_name$est_end$png_end
    txt=$base_name$est_end$txt_end

    # Processing 
    echo "rosrun ndt_feature_reg evaluate_ate.py $est $gt --plot $png --verbose > $txt"
          rosrun ndt_feature_reg evaluate_ate.py $est $gt --plot $png --verbose > $txt
    echo "rosrun ndt_feature_reg evaluate_rpe.py $est $gt --plot $est$pos$png_end --plot_rot $est$rot$png_end --fixed_delta --verbose > $est$rpe$txt_end"
          rosrun ndt_feature_reg evaluate_rpe.py $est $gt --plot $est$pos$png_end --plot_rot $est$rot$png_end --fixed_delta --verbose > $est$rpe$txt_end

done
