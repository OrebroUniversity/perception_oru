#include <boost/thread/mutex.hpp>

#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/StdVector>
#include <time.h>

#define EIGEN_USE_NEW_STDVECTOR

#ifndef SDF_TRACKER
#define SDF_TRACKER

class SDF_Parameters
{
public:
  bool interactive_mode;
  int XSize;
  int YSize;
  int ZSize;
  int raycast_steps;
  int image_height;
  int image_width;
  double fx;
  double fy;
  double cx;
  double cy;
  double Wmax;
  double resolution;
  double Dmax; 
  double Dmin;
  Eigen::Matrix4d pose_offset;
  double robust_statistic_coefficient;
  double regularization;
  double min_parameter_update;
  double min_pose_change;
  std::string render_window;

  SDF_Parameters();
  virtual ~SDF_Parameters();
};

typedef Eigen::Matrix<double,6,1> Vector6d; 

class SDFTracker
{
  protected:
  // variables
  std::vector<Eigen::Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d> > transformations_;
  std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > interest_points_;
  std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > Points_;

  Eigen::Matrix4d Transformation_;
  Vector6d Pose_;
  Vector6d cumulative_pose_;
  cv::Mat *depthImage_;
  cv::Mat *depthImage_denoised_;

  boost::mutex transformation_mutex_;
  boost::mutex depth_mutex_;
  boost::mutex points_mutex_;
  boost::mutex depthDenoised_mutex_;
  std::string camera_name_;
  
  bool** validityMask_;
  float*** myGrid_; 
  bool first_frame_;
  bool quit_;
  SDF_Parameters parameters_;
  // functions 
  Eigen::Vector4d VertexInterp(double iso, Eigen::Vector4d &p1d, Eigen::Vector4d &p2d,double valp1, double valp2);
  void MarchingTetrahedrons(Eigen::Vector4d &Origin, int tetrahedron);
  virtual void Init(SDF_Parameters &parameters);
  virtual void DeleteGrids(void);

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<Eigen::Vector4d> triangles_;
  
  /// Returns the signed distance at the given location 
  virtual double SDF(const Eigen::Vector4d &location);

  /// Computes the gradient of the SDF at the location, along dimension dim, with central differences. stepSize chooses how far away from the central cell the samples should be taken before computing the difference
  virtual double SDFGradient(const Eigen::Vector4d &location, int dim, int stepSize);

  /// Saves the current volume as a VTK image.
  virtual void SaveSDF(const std::string &filename = std::string("sdf_volume.vti"));
  
  /// Loads a volume from a VTK image. The grid is resized to fit the loaded volume
  virtual void LoadSDF(const std::string &filename);

  /// Checks the validity of the gradient of the SDF at the current point   
  bool ValidGradient(const Eigen::Vector4d &location);

  /// Sets the current depth map
  virtual void UpdateDepth(const cv::Mat &depth);   
  
  /// Alternative to depth maps, sets the current point vector (x y z 1)
  virtual void UpdatePoints(const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d> > &Points);   

  
  /// Estimates the incremental pose change vector from the current pose, relative to the current depth map
  virtual Vector6d EstimatePoseFromDepth(void); 

  /// Estimates the incremental pose change vector from the current pose, relative to the current point vector
  virtual Vector6d EstimatePoseFromPoints(void); 

  /// Fuses the current depth map into the TSDF volume, the current depth map is set using UpdateDepth 
  virtual void FuseDepth(void);

  /// Fuses the current point vector into the TSDF volume, the current point vector is set using UpdatePoints 
  virtual void FusePoints(void);

  /// Estimates the pose, fuses the given depth map and renders a virtual depth map (all in one) 
  virtual void FuseDepth(const cv::Mat &depth);

  /// Render a virtual depth map. If interactive mode is true (see class SDF_Parameters) it will also display a preview window with estimated normal vectors
  virtual void Render(void);

  /// Finds the inverse projection of a 3D point to the image plane, given camera parameters
  cv::Point2d To2D(const Eigen::Vector4d &location, double fx, double fy, double cx, double cy);

  /// Projects a depth map pixel into a 3D point, given camera parameters.
  Eigen::Vector4d To3D(int row, int column, double depth, double fx, double fy, double cx, double cy);
  
  /// Computes an antisymmetric matrix based on the pose change vector. To get a transformation matrix from this, you have to call Twist(xi).exp().
  Eigen::Matrix4d Twist(const Vector6d &xi);

  /// Runs the Marching Tetrahedrons algorithm. The result is then available in triangles_. Each three consecutive entries in triangles_ represents one triangle.
  void MakeTriangles(void);

  /// Dumps the zero level set as triangles to an OBJ file. SaveTriangles must be preceded by a call to MakeTriangles.
  void SaveTriangles(const std::string filename = std::string("triangles.obj"));

  /// Dumps the zero level set as triangles to an STL file. This method also computes the triangles so there is no need to call MakeTriangles.
  void SaveTrianglesSTL(const std::string filename = std::string("triangles.stl"));
  /// gets the denoised image produced by Render.
  void GetDenoisedImage(cv::Mat &img); 

  /// gets the current transformation matrix
  Eigen::Matrix4d GetCurrentTransformation(void);

  /// sets the current transformation to the given matrix
  void SetCurrentTransformation(const Eigen::Matrix4d &T); 
  
  /// In interactive sessions, this function returns true at any point after a user has pressed "q" or <ESC> in the render window. 
  bool Quit(void);

  /// Constructor with default parameters
  SDFTracker();

  /// Constructor with custom parameters
  SDFTracker(SDF_Parameters &parameters);
  virtual ~SDFTracker();   

};

#endif
