# This file contain a message definition for a single NDT Cell. 
# This file was based on ndt_cell.h
# by Tomasz Kucner

bool hasGaussian # informs if the cell contains Gaussian (I guess)
float64 cost # cost of some kind
uint8 isEmpty # based on the most recent observation, is the cell seen
              # empty (1), occupied (-1) or not at all (0)
float64 consistency_score 

float64[] cov # covariance of normal distribution
float64[] icov # precomputed inverted covariance of normal distribution
float64[] evecs # eigen vectors
float64[] mean # mean values
float64[] evals # eigen values

uint32 N # Number of points used for Normal distribution estimation so far
uint32 emptyval # The number of times a cell was observed empty (using ray casting)
float32 occ # Occupancy value stored as "Log odds" (if you wish)
