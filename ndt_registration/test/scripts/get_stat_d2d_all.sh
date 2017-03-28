#!/bin/bash

BASEDIR=$(dirname "$0")
echo "$BASEDIR"

/bin/bash $BASEDIR/get_stat_d2d_eval.sh
/bin/bash $BASEDIR/get_stat_d2d_sc_eval.sh
/bin/bash $BASEDIR/get_stat_filter_eval.sh
