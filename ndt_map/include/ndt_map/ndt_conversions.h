#ifndef NDT_CONVERSIONS_HH
#define NDT_CONVERSIONS_HH

#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/NDTMap.h>
#include <ndt_map/NDTCell.h>
#include <string>
namespace lslgeneric
{
  template<typename PointT>
    bool toMessage(NDTMap<PointT> *map, ndt_map::NDTMap &msg,std::string frame_name){
    std::vector<lslgeneric::NDTCell<pcl::PointXYZ>*> map_vector=map->getAllInitializedCells();
    msg.header.stamp=ros::Time::now();
    msg.header.frame_id=frame_name;//is it in *map?    
    if(!map->getGridSizeInMeters(msg.x_size,msg.y_size,msg.z_size)){
      ROS_ERROR("NO GRID SIZE");
      return false;
    }
    if(!map->getCentroid(msg.x_cen,msg.y_cen,msg.z_cen)){
      ROS_ERROR("NO GRID CENTER");
      return false;
    }
    if(!map->getCellSizeInMeters(msg.x_cell_size,msg.y_cell_size,msg.z_cell_size)){
      ROS_ERROR("NO CELL SIZE");
      return false;
    }
    for (int cell_idx=0;cell_idx<map_vector.size();cell_idx++){
      if(map_vector[cell_idx]->hasGaussian_){ //????????????????????????
        ndt_map::NDTCell cell;
        Eigen::Vector3d means=map_vector[cell_idx]->getMean();
        cell.mean_x=means(0);
        cell.mean_y=means(1);
        cell.mean_z=means(2);
        cell.occupancy=map_vector[cell_idx]->getOccupancyRescaled();
        Eigen::Matrix3d cov=map_vector[cell_idx]->getCov();
        for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
            cell.cov_matrix.push_back(cov(i,j));
          }
        }
        cell.N=map_vector[cell_idx]->getN();
        msg.cells.push_back(cell);
      }
    }
    return true;
  }

  template<typename PointT>
    bool fromMessage(LazyGrid<PointT>* &idx, NDTMap<PointT>* &map, ndt_map::NDTMap msg, std::string &frame_name){
    if(!(msg.x_cell_size==msg.y_cell_size&&msg.y_cell_size==msg.z_cell_size)){ 
       ROS_ERROR("SOMETHING HAS GONE VERY WRONG YOUR VOXELL IS NOT A CUBE"); 
       return false;
    }
    idx=new LazyGrid<PointT>(msg.x_cell_size);
    /* LazyGrid<PointT> *idx; */
    /* if(msg.x_cell_size==msg.y_cell_size&&msg.y_cell_size==msg.z_cell_size){ */
    /*   idx=new LazyGrid<PointT>(msg.x_cell_size); */
    /* } */
    /* else{ */
    /*   ROS_ERROR("SOMETHING HAS GONE VERY WRONG YOUR VOXELL IS NOT A CUBE"); */
    /*   return false; */
    /* } */

    map = new NDTMap<PointT>(idx,msg.x_cen,msg.y_cen,msg.z_cen,msg.x_size,msg.y_size,msg.z_size);
    frame_name=msg.header.frame_id;
    ROS_INFO("number of cells %d",msg.cells.size());
    for(int itr=0;itr<msg.cells.size();itr++){
      Eigen::Vector3d mean;
      Eigen::Matrix3d cov;
      cov<<msg.cells[itr].mean_x,msg.cells[itr].mean_y,msg.cells[itr].mean_z;
      int m_itr=0;
      for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
            cov(i,j)=msg.cells[itr].cov_matrix[m_itr];
            m_itr++;
          }
        }
      map->addDistributionToCell(cov,mean,msg.cells[itr].N);
    }
    ROS_INFO("%d",map->getMyIndexInt());
    return true;
  }
}
#endif
