#!/bin/bash
FILES=./*_gt.txt
time_end='_addTime.txt'
txt_end='_timestat.txt'

for gt in $FILES
do
  # Remove the ending _gt.txt
    base_name=${gt%_gt.txt}
    echo "Processing $base_name files..."

    time=$base_name$time_end
    echo $time

    txt=$base_name$txt_end

    # Processing 
    echo "rosrun semrob_rosbag_tools get_vector_stat --infile $time > $txt"
    rosrun semrob_rosbag_tools get_vector_stat --infile $time > $txt

done
