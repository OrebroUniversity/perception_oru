#!/bin/bash
FILES=./*_gt.txt
est_end='_est.txt'
png_end='.png'
rpe_end='rpe.txt'
rot='rot'
pos='pos'
rpe='rpe'

echo "# Parsed rpe data using parse_stat_rpe.sh..." > data_rpe.txt

for gt in $FILES
do
  # Remove the ending _gt.txt
    base_name=${gt%_gt.txt}
    echo "Processing $base_name files..."

    rpe_txt=$base_name$rpe_end

    # Parse the motion model
    # xxxxxDd0.4Dt1Cdxxxxx -> 0.4Dt1C -> 0.4
    # Split on d, take the 2:nd substring. Split on D take the first sting.
    Dd=$(echo $base_name | cut -d 'd' -f 2)
    #echo $Dd
    Dd=$(echo $Dd | cut -d 'D' -f 1)
    #echo $Dd

    # From the file name extract the resolution and the sensor cut off
    arr=$(echo $base_name | tr "_" "\n")
    num_value=0
    i=0
    # Get the second 'string' -> contains
    for x in $arr
    do

	# Parse res -> skip the 3 first chars
	if [ $i -eq 1 ]
	then 	    
	    res=${x:3}
	fi
	# Parse SC flag
	if [ $i -eq 2 ]
	then
	    SC=${x:2}
	fi
	# Parse the sensorcutoff
	if [ $i -eq 4 ]
	then
	    sensorcutoff=${x:12}
	fi
	i=$((i+1)) 
    done

    echo "Res $res SC $SC sensorcutoff $sensorcutoff"
    echo -n "$SC $res $sensorcutoff " >> data_rpe.txt

    # Need to open the files (rpe file) and extract mean and stddev...
    # ----- the file content ----
# compared_pose_pairs 1673 pairs
# translational_error.rmse 0.309811 m
# translational_error.mean 0.146038 m
# translational_error.median 0.039656 m
# translational_error.std 0.273233 m
# translational_error.min 0.000000 m
# translational_error.max 1.367542 m
# rotational_error.rmse 0.669293 deg
# rotational_error.mean 0.517586 deg
# rotational_error.median 0.404536 deg
# rotational_error.std 0.424332 deg
# rotational_error.min 0.000000 deg
# rotational_error.max 3.493946 deg
    while read p; do

	arr=$(echo $p | tr " " "\n")
	num_value=0
	i=0
	# Get the second 'string' -> contains
	for x in $arr
	do
	    if [ $i -eq 1 ]
		then 
		#echo "\"$x\""
		#echo $i
		num_value=$x
	    fi
	    i=$((i+1)) 
	done
	echo -n $num_value >> data_rpe.txt
	echo -n " " >> data_rpe.txt
	
	
    done < $rpe_txt
    echo -n $Dd >> data_rpe.txt
    echo "" >> data_rpe.txt

done
