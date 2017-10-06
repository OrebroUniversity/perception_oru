#!/bin/bash
FILES=./*_gt.txt
est_end='_est.txt'
png_end='.png'
txt_end='.txt'
rot='rot'
pos='pos'
rpe='rpe'
for gt in $FILES
do
  # Remove the ending _gt.txt
    base_name=${gt%_gt.txt}
    echo "Processing $base_name files..."

    est=$base_name$est_end
    echo $est

    png=$base_name$png_end
    txt=$base_name$txt_end

    # Processing 
    echo "rosrun ndt_feature_reg evaluate_ate.py $est $gt --plot $png --verbose > $txt"
    rosrun ndt_feature_reg evaluate_ate.py $est $gt --plot $png --verbose > $txt
    echo "rosrun ndt_feature_reg evaluate_rpe.py $est $gt --plot $base_name$pos$png_end --plot_rot $base_name$rot$png_end --fixed_delta > $base_name$rpe$txt_end"
    rosrun ndt_feature_reg evaluate_rpe.py $est $gt --plot $base_name$pos$png_end --plot_rot $base_name$rot$png_end --fixed_delta --verbose > $base_name$rpe$txt_end

done
