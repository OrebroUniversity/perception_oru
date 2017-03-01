#!/bin/bash
FILES=./*.rpe
rpe_end='.rpe'

echo "# Parsed rpe data using parse_stat_d2d_eval.sh..." > data_rpe.txt

for gt in $FILES
do
  # Remove the ending _gt.txt
    base_name=${gt%_gt.txt}
    echo "Processing $base_name files..."

    rpe_txt=$base_name

    # Parse the motion model
    # xxxxxDd0.4Dt1Cdxxxxx -> 0.4Dt1C -> 0.4
    # Split on d, take the 2:nd substring. Split on D take the first sting.
    Dd=$(echo $base_name | cut -d 'd' -f 2)
#    echo $Dd
    Dd=$(echo $Dd | cut -d 'D' -f 1)
#    echo $Dd

    Dt=$(echo $base_name | cut -d 'T' -f 1)
    #echo $Dt
    Dt=$(echo $Dt | cut -d 't' -f 2)
    #echo $Dt
    # Remove final dot
    Dt=${Dt::-1}
    

    # Parse the resolution (assumes a fixed offset)
    res=${base_name:12}
#    echo $res
    res=$(echo $res | cut -d 'D' -f 1)
#    echo $res
    
    # Get the type of objective used (d2d, d2d_sc, filter)...
    # Get the type name (after Ts.) but before rpe (...Ts.XXXXX.rpe)
    type=$(echo $base_name | cut -d 'T' -f 2)
    type=${type:2}
    type=$(echo $type | cut -d '.' -f 1)
    
    echo "Res $res Dd $Dd Dt $Dt objective $type"
    echo -n "$res $Dd $Dt $type " >> data_rpe.txt
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
