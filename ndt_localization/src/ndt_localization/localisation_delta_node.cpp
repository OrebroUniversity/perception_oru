//ros
#include "ros/ros.h"
#include "ros/rate.h"
#include <tf/transform_listener.h>
class delta {
	double odom_old_x, odom_old_y, odom_old_th;
	double mcl_old_x, mcl_old_y, mcl_old_th;
	bool first_load;
public:
	delta(ros::NodeHandle parameters){
		static tf::TransformListener tf_listener_odo;
		tf::StampedTransform transform_odo;
		static tf::TransformListener tf_listener_mcl;
		tf::StampedTransform transform_mcl;

		while(true){
			tf_listener_mcl.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.1));
			try{
				tf_listener_mcl.lookupTransform("/map", "/base_link", ros::Time(0), transform_mcl);
				mcl_th = tf::getYaw(transform.getRotation());
				mcl_x = transform.getOrigin().x();
				mcl_y = transform.getOrigin().y();
				tf_listener_odom.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.1));
				try{
					tf_listener_mcl.lookupTransform("/map", "/base_link", ros::Time(0), transform_mcl);
					mcl_th = tf::getYaw(transform.getRotation());
					mcl_x = transform.getOrigin().x();
					mcl_y = transform.getOrigin().y();
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s", ex.what());
					return;
				}
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s", ex.what());
				return;
			}





		}




	}





};



int main(int argc, char **argv){
	ros::init(argc, argv, "localisation_delta");
	ros::NodeHandle parameters("~");
	delta d(parameters);
}
