#In this file we define a sinngle ndt cell message
bool hasGaussian_    #Tell if the cell was only initialized and has no gaussian
float64 center_x       #x coordinate of center used for uninitilaised cells
float64 center_y       #y coordinate of center used for uninitilaised cells
float64 center_z       #z coordinate of center used for uninitilaised cells
float64 mean_x       #x coordinate of mean
float64 mean_y       #y coordinate of mean
float64 mean_z       #z coordinate of mean
float64[] cov_matrix #covariance matrix
float64 occupancy    #occupancy
int64 N              #point count