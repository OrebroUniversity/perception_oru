#include "PoseInterpolationNavMsgsOdo.h"

/**
* This reads all in once
*/
void PoseInterpolationNavMsgsOdo::readBagFile(tf::StampedTransform *sensor_link){
	rosbag::View view(bag, rosbag::TopicQuery(topics));
		BOOST_FOREACH(rosbag::MessageInstance const m, view)
		{
			//////////////////////////////////////////////////////////////////////////////////
			tf::tfMessage::ConstPtr transform = m.instantiate<tf::tfMessage>();
			if (transform != NULL){
				if(m.getTopic() == topics[0]){
					for (int i = 0; i < transform->transforms.size(); i++){
							//std::cout << "frame_id : " << transform->transforms[i].header.frame_id;
							//std::cout << " cframe_id : " << transform->transforms[i].child_frame_id << std::endl;
							tf::StampedTransform stf;
							transformStampedMsgToTF(transform->transforms[i], stf);
							transformer.setTransform(stf);
			      }
			      if(sensor_link != NULL){
							sensor_link->stamp_ = transform->transforms[0].header.stamp;
							//fprintf(stderr,"Setting sensor link %s -> %s\n", sensor_link->frame_id_.c_str(), sensor_link->child_frame_id_.c_str());
							transformer.setTransform(*sensor_link);
						}else{
							//fprintf(stderr,"NULL\n");
						}
						last_read_tf = transform->transforms[0].header.stamp;
				}
			}
			/////////////////////////////////////////////////////////////////////////////////////////////
		}
		fprintf(stderr,"Cache length %lf",transformer.getCacheLength().toSec());
		bag.close();
		
}

void PoseInterpolationNavMsgsOdo::readUntilTime(ros::Time t){
		if(I == viewer->end()){
			fprintf(stderr,"END:");
		}
		if(last_read_tf>t){
			//fprintf(stderr,"No need to read!\n");
			return; ///No need to read new ones
		}
		
		while(last_read_tf <= t && I != viewer->end()){
			rosbag::MessageInstance const m = *I;
			//fprintf(stderr,"READING\n");
			//////////////////////////////////////////////////////////////////////////////////
			tf::tfMessage::ConstPtr transform = m.instantiate<tf::tfMessage>();
			if (transform != NULL){
				if(m.getTopic() == topics[0]){
					
					if(transform->transforms.size()<=0) continue;
					
					for (int i = 0; i < transform->transforms.size(); i++){
							//std::cout << "frame_id : " << transform->transforms[i].header.frame_id;
							//std::cout << " cframe_id : " << transform->transforms[i].child_frame_id << std::endl;
							tf::StampedTransform stf;
							transformStampedMsgToTF(transform->transforms[i], stf);
							transformer.setTransform(stf);
			      }
			      
			      last_read_tf = transform->transforms[0].header.stamp;
						
			      if(sensor_link_ != NULL){
							sensor_link_->stamp_ = transform->transforms[0].header.stamp;
							//fprintf(stderr,"Setting sensor link %s -> %s\n", sensor_link->frame_id_.c_str(), sensor_link->child_frame_id_.c_str());
							transformer.setTransform(*sensor_link_);
						}else{
							//fprintf(stderr,"NULL\n");
						}
				}
			}
			I++;
			/////////////////////////////////////////////////////////////////////////////////////////////
		}
		//fprintf(stderr,"Cache length %lf",transformer.getCacheLength().toSec());
		//bag.close();
		
}




void PoseInterpolationNavMsgsOdo::TransformTFToEigen(const tf::Transform &t, Eigen::Affine3d &k)
 {
     for(int i=0; i<3; i++)
     {
       k.matrix()(i,3) = t.getOrigin()[i];
       for(int j=0; j<3; j++)
       {
         k.matrix()(i,j) = t.getBasis()[i][j];
       }
     }
     // Fill in identity in last row
     for (int col = 0 ; col < 3; col ++)
       k.matrix()(3, col) = 0;
     k.matrix()(3,3) = 1;
 
}
 


/**
* Returns the interpolated Affine transformation for a Time t
*/
bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id,Eigen::Affine3d &T){
	tf::StampedTransform transform;
	//fprintf(stderr,"DT: %lf ",t0.toSec()-t1.toSec());
	if(t1 > t0) readUntilTime(t1);
	else readUntilTime(t0);
	
	std::string schaiba;
	if(!transformer.canTransform 	(frame_id, t1, frame_id, t0, fixedframe, &schaiba)){
		
		return false;
	}

	transformer.lookupTransform(frame_id,t1, frame_id, t0, fixedframe, transform);
	TransformTFToEigen (transform, T);
	
	return true;
}


/**
* Returns the interpolated Affine transformation for a Time t
*/
bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0,ros::Time t1, std::string frame_id,tf::Transform &T){
	tf::StampedTransform transform;
	//fprintf(stderr,"DT: %lf ",t0.toSec()-t1.toSec());
	
	if(t1 > t0) readUntilTime(t1);
	else readUntilTime(t0);
	
	std::string schaiba;
	
	if(!transformer.canTransform 	(frame_id, t0, frame_id, t1, fixedframe, &schaiba)){
		fprintf(stderr,"FAIL\n");
		return false;
	} 

	//transformer.lookupTransform(frame_id,t0, frame_id, t1, fixedframe, transform);
	transformer.lookupTransform(frame_id,t0, frame_id, t1, fixedframe, transform);
	T = transform;
	
	Eigen::Affine3d Ta;
	TransformTFToEigen (transform, Ta);
	std::cout<<"trans: "<<Ta.translation().transpose();
	std::cout<<":: euler: "<<Ta.rotation().eulerAngles(0,1,2).transpose()<<std::endl;
	return true;
}


/**
* Returns the sensor pose for time t
*/
bool PoseInterpolationNavMsgsOdo::getTransformationForTime(ros::Time t0, std::string frame_id,tf::Transform &T){
	tf::StampedTransform transform;
	//fprintf(stderr,"DT: %lf ",t0.toSec()-t1.toSec());
	std::string schaiba;
	readUntilTime(t0+ros::Duration(10.0));
	if(!transformer.canTransform 	(fixedframe,frame_id,t0, &schaiba)){
		return false;
	}

	transformer.lookupTransform(fixedframe,frame_id,t0, transform);
	T = transform;
	
	return true;
}


