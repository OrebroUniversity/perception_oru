
#include "graph_map_fuser.h"
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ndt_map/ndt_conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl/point_cloud.h"
#include <Eigen/Eigen>
#include "eigen_conversions/eigen_msg.h"
#include <tf_conversions/tf_eigen.h>


#include "sensor_msgs/PointCloud2.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include <tf/transform_broadcaster.h>

#include <boost/circular_buffer.hpp>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_srvs/Empty.h>

#include <boost/foreach.hpp>
#include <ndt_map/NDTMapMsg.h>
#include "gnuplot-iostream.h"
#include "lidarUtils/lidar_utilities.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "boost/serialization/serialization.hpp"
#include <boost/program_options.hpp>
#include "visualization/graph_plot.h"
namespace po = boost::program_options;
/** \brief A ROS node which implements an NDTFuser or NDTFuserHMT object
 * \author Daniel adolfsson based on code from Todor Stoyanov
 *
 */

using namespace libgraphMap;
class OpenGraphMap {

protected:
  // Our NodeHandle

public:
  // Constructor
  OpenGraphMap(ros::NodeHandle param_nh,const string &file_name)
  {
    LoadGraphMap(file_name);
    NDTMapPtr ptr=boost::dynamic_pointer_cast<NDTMapType>( graph_map_->GetCurrentNode()->GetMap());
    NDTMap * map=ptr->GetMap();
    cout<<"ndt map size="<<map->getAllCells().size()<<endl;
    for(int i=0;i<20;i++){
      if(!param_nh.ok())
        exit(0);
    cout<<"plot"<<endl;
    GraphPlot::PlotPoseGraph(graph_map_);
    NDTMapPtr curr_node = boost::dynamic_pointer_cast< NDTMapType >(graph_map_->GetCurrentNode()->GetMap());
    GraphPlot::SendGlobalMapToRviz(curr_node->GetMap(),1,graph_map_->GetCurrentNodePose());
    sleep(1);
    }
  }
  void LoadGraphMap(string file_name){
    std::ifstream ifs(file_name);
    boost::archive::text_iarchive ia(ifs);
    ia & graph_map_;
    cout<<graph_map_->ToString();
  }

private:
  GraphMapNavigatorPtr graph_map_;

  ros::Subscriber gt_sub;
  // Components for publishing
  ros::Publisher output_pub_;
  Eigen::Affine3d pose_;
  boost::mutex m, message_m;
  std::string gt_topic, bag_name;
  ros::Publisher map_publisher_;


};

int main(int argc, char **argv)
{
  string file_name;
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("file-name", po::value<std::string>(&file_name)->default_value(std::string("full_map_serialization.dat")), "name of file to load containing graphMap");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << "\n";
    exit(0);
  }
  ros::init(argc, argv, "show_map");
  ros::NodeHandle param("~");
  cout<<"Attempt to open map: "<<file_name<<endl<<"Continue? y/n"<<endl;
  char c=getchar();
  if(c!='y'&&c!='Y')
    exit(0);

  OpenGraphMap t(param, file_name);
  while(param.ok()){
    ros::spinOnce();
  }

  return 0;
}

