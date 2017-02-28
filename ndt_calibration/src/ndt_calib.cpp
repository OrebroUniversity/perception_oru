#include <ndt_calibration/ndt_calib.h>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_min.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>

//////////////////////////////////////////////////////////
double 
scoreICP(const NDTCalibScanPairs &pairs, const Eigen::Affine3d &Ts) {
    double e = 0;
    for(unsigned int i=0; i<pairs.size(); ++i){
	e += pairs[i].scoreICP(Ts);
    }
    return e;
}

double 
scoreEstSensorPose(const NDTCalibScanPairs &pairs, const Eigen::Affine3d &Ts){
    double e=0;
    for(unsigned int i=0; i<pairs.size(); ++i){
	e += pairs[i].first.scoreEstSensorPose(Ts);
	e += pairs[i].second.scoreEstSensorPose(Ts);
    }
    return e;
}

double 
scoreEstSensorPoseRel(const NDTCalibScanPairs &pairs, const Eigen::Affine3d &Ts){
    double e=0;
    for(unsigned int i=0; i<pairs.size(); ++i){
	e += pairs[i].scoreEstSensorPoseRel(Ts);
    }
    //  std::cout << " e : " << e << std::endl;
    return e;
}

double 
scoreNDT(const NDTCalibScanPairs &pairs, const Eigen::Affine3d &Ts) {
    double e = 0;
    for(unsigned int i=0; i<pairs.size(); ++i){
	e += pairs[i].scoreNDT(Ts);
    }
    return e;
}

/////////////////////////////////////////////////////////////////////
//! Optimization error functions                                                                   
double
score_f(const gsl_vector *v, void *params)
{
    NDTCalibOptimize *opt;
    opt = (NDTCalibOptimize*)params;
  
    Eigen::VectorXd x(opt->getNbParameters());
    for (int i = 0; i < x.size(); i++) {
	x[i] = gsl_vector_get(v, i);
    }
  
    return opt->getScore(x);
}


void
score_df(const gsl_vector *v, void *params, gsl_vector *df)
{
    const double EPSILON = 1e-04;
    NDTCalibOptimize *opt;
    opt = (NDTCalibOptimize*)params;

    Eigen::VectorXd x(opt->getNbParameters());
    for (int i = 0; i < x.size(); i++) {
	x[i] = gsl_vector_get(v, i);
    }
  
    for (int i = 0; i < x.size(); i++)
    {
	Eigen::VectorXd x1 = x; x1[i] += EPSILON; Eigen::VectorXd x2 = x; x2[i] -= EPSILON;
	double f1 = opt->getScore(x1);
	double f2 = opt->getScore(x2);
	double d = (f1 - f2)/ (2.*EPSILON);
	gsl_vector_set(df, i, d);
    }
}

void
score_fdf(const gsl_vector *x, void *params,
	  double *f, gsl_vector *df)
{
    *f = score_f(x, params);
    score_df(x, params, df);
}


/// This is the most fundamental score. We must end up here independent on how many parameters we try to optimize, note that the time offset has to be updated before this point (interpPairPoses).
double NDTCalibOptimize::getScore6d(const Eigen::Affine3d &T) const {
  switch (_scoreType) {
  case SCORE_ICP:
      return scoreICP(_pairs, T);
  case SCORE_EST_SENSOR_POSE:
      return scoreEstSensorPose(_pairs, T);
  case SCORE_EST_SENSOR_POSE_REL:
      return scoreEstSensorPoseRel(_pairs, T);
  case SCORE_NDT:
      return scoreNDT(_pairs, T);
  default:
      std::cerr << "Error: unknown score type : " << _scoreType << std::endl;
	return -1;
  }
}

        
void NDTCalibOptimize::extractActiveInitialParameters(Eigen::VectorXd &params) const {
    // Based on the selected objective compute the initial parameters used.
    _objectiveType.getInitialActiveParameters(_initParameters, params);
}

void NDTCalibOptimize::assignParameters(const Eigen::VectorXd &params, Eigen::Affine3d &Ts, double &t)
{
    // Given the current objective type compute the Affine params and time offset.
    
    // Convert the active params to 'all params'...
    Eigen::VectorXd all_params;
    _objectiveType.getAllParameters(_initParameters, params, all_params);
    
    t = all_params[6];
    Eigen::VectorXd pose_params(6);
    for (int i = 0; i < 6; i++) {
        pose_params[i] = all_params[i];
    }
    
    Ts = getAsAffine3dFromTranslationEulerAngles(pose_params);
}

void NDTCalibOptimize::setInitialParameters(const Eigen::Affine3d &Ts, const double &t) {
     _initParameters.resize(7);
     Eigen::VectorXd x = getTranslationEulerAnglesVectorFromAffine3d(Ts);
     for (size_t i = 0; i < 6; i++) {
         _initParameters[i] = x[i];
     }
     _initParameters[6] = t;
}
 

int NDTCalibOptimize::getNbParameters() const {
    return _objectiveType.getNbParameters();
}

double NDTCalibOptimize::getScore(const Eigen::VectorXd &x) {

    // Need to get all 7 parameters (ether all from x or partly from x).
    Eigen::Affine3d Ts;
    double t;
    assignParameters(x, Ts, t);

    if (_objectiveType.optimizeTime()) {
        // Need to update the time
        this->interpPairPoses(t);
    }
    return getScore6d(Ts);
}

void NDTCalibOptimize::interpPose(double time, Eigen::Affine3d &T) {
    ros::Time t(time);
    _poseInterp.getTransformationForTime(t, std::string("/EKF"), T); 
}

void NDTCalibOptimize::interpPairPoses(double sensorTimeOffset) {
    for (int i = 0; i < _pairs.size(); i++) {
	interpPose(_pairs[i].first.stamp - sensorTimeOffset, _pairs[i].first.pose);
	interpPose(_pairs[i].second.stamp - sensorTimeOffset, _pairs[i].second.pose);
    }
}

bool NDTCalibOptimize::calibrate(Eigen::Affine3d &Ts, double &sensorTimeOffset) {
    this->setInitialParameters(Ts, sensorTimeOffset);
    size_t iter = 0;
    int status;
    int nb_params = this->getNbParameters();
  
    const gsl_multimin_fdfminimizer_type *gsl_type;
    gsl_multimin_fdfminimizer *s;
  
    gsl_vector *x;
    gsl_multimin_function_fdf score;
  
    score.f = &score_f;
    score.df = &score_df;
    score.fdf = &score_fdf;
    score.n = nb_params;
    score.params = this;
  
    x = gsl_vector_alloc (nb_params);
  
    Eigen::VectorXd init_offset;
    this->extractActiveInitialParameters(init_offset);

    for (int i = 0; i < nb_params; i++)
    {
	gsl_vector_set (x, i, init_offset[i]);
    }
  
    gsl_type = gsl_multimin_fdfminimizer_conjugate_fr;
    s = gsl_multimin_fdfminimizer_alloc (gsl_type, nb_params);
  
    gsl_multimin_fdfminimizer_set (s, &score, x, 0.01, /*1e-4*/0.1);
  
    printf ("using %s method\n",
	    gsl_multimin_fdfminimizer_name (s));
  
    do
    {
	iter++;
	status = gsl_multimin_fdfminimizer_iterate (s);
    
	if (status) {
	    /// 
	    if (status == GSL_ENOPROG) {
		std::cout << " no progress - quitting" << std::endl;
	    }
	    else {
		std::cout << " status : " << status << " - quitting" << std::endl; 
	    }
	    break;
	}
	status = gsl_multimin_test_gradient (s->gradient, 1e-4);
    
	if (status == GSL_SUCCESS)
	    printf ("Minimum found at:\n");
    
	std::cout << iter << " [ " << std::flush;
	for (int i = 0; i < nb_params; i++) {
	    std::cout << gsl_vector_get (s->x, i) << " " << std::flush;
	}
	std::cout << "] " << s->f << std::endl;
    }
  
    while (status == GSL_CONTINUE && iter < 1000);
  
    Eigen::VectorXd x_vec(nb_params);
    for (int i = 0; i < nb_params; i++)
    {
	x_vec[i] = gsl_vector_get(s->x, i);
    }
  
    this->assignParameters(x_vec, Ts, sensorTimeOffset);
    
    gsl_multimin_fdfminimizer_free (s);
    gsl_vector_free (x);
  
    return (status == GSL_SUCCESS);
}


