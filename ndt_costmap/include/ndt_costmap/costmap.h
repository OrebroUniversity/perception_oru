#ifndef NDT_COSTMAP_HH
#define NDT_COSTMAP_HH

#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/lazy_grid.h>
//#include <ndt_costmap/TraversabilityMapMsg.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/OccupancyGrid.h>


namespace lslgeneric {
  struct LevelSet {
	std::vector<double> clear_levels;
	std::vector<int> map_ids;
  };
  class NDTCostmap {
  private:
    LevelSet **levels;
    NDTMap *map_;
    LazyGrid *grid_;
    int sx, sy, sz;
    bool alloc;
  public:
    NDTCostmap(NDTMap *map) {
      //std::cerr<<"creating costmap\n";
      map_ = map;
      if(map_ != NULL) {
        grid_ = dynamic_cast<LazyGrid*>(map_->getMyIndex());
        if(grid_ != NULL) { 
          grid_->getGridSize(sx,sy,sz);
          //std::cerr<<"got a lazy grid, size: "<<sx<<" "<<sy<<" "<<sz<<"\n";
          levels = new LevelSet*[sx];
          for(int i=0; i<sx; ++i) {
            levels[i] = new LevelSet[sy];
          }
          alloc = true;
        } else {
          alloc = false;
        }
      } else {
        alloc = false;
      }
    };
    ~NDTCostmap() {
      if(alloc) {
        for(int i=0; i<sx; ++i) {
          delete[] levels[i];
        }
        delete[] levels;
      }
    }
    void processMap(double robot_height, double occ);
    //bool toMessage(ndt_costmap::TraversabilityMapMsg msg);
    void saveCostMap(std::string filename, double max_h);
    void saveCostMapIncr(std::string filename, double max_h);
  };

  void NDTCostmap::processMap(double robot_height, double occ) {
	if(!alloc) return;
    //	std::cerr<<"processing map\n";
	double cx,cy,cz;
	grid_->getCellSize(cx,cy,cz);
	int nheight_cells = ceil(robot_height/cz);
	//std::cerr<<"nh "<<nheight_cells<<" cz "<<cz<<std::endl;
	for(int ix=0; ix<sx; ++ix) {
      for(int iy=0; iy<sy; ++iy) {
		for(int iz=0; iz<sz-nheight_cells; ++iz) { //
          NDTCell *ndcell;
          grid_->getCellAt(ix,iy,iz,ndcell);
          if(ndcell == NULL) {  continue; }
          //cell is occupied and has a gaussian
          if(ndcell->getOccupancy() > occ && ndcell->hasGaussian_) {
			if(ndcell->getClass() == NDTCell::HORIZONTAL ||ndcell->getClass() == NDTCell::INCLINED ) {
              bool space = true;
              //check we have free space on top
              for(int j=1; j<nheight_cells; ++j) {
				NDTCell* ndcell_up;
				grid_->getCellAt(ix,iy,iz+j,ndcell_up);
				if(ndcell_up == NULL) {
                  space = false;
                  break;
				} else {
                  space = !(ndcell_up->getOccupancy() > occ || ndcell_up->hasGaussian_ );
                  if(!space) break;
				}
              }
              if(space) {
				//std::cerr<<"s";
				Eigen::Vector3d mean = ndcell->getMean();
				levels[ix][iy].clear_levels.push_back(mean(2));
              }
			}else {
              //std::cerr<<"n";
			}
          }
          else {
			//std::cerr<<"e";
          }
		}
		//std::cerr<<std::endl;
      }
      //std::cerr<<"////////////////////////"<<std::endl;
      //std::cerr<<std::endl;
	}
  }
	    
  struct MapPair {
	double mean_height;
	int n_samples;
	cv::Mat image;
  };

  struct MapPairMsg {
	double mean_height;
	int n_samples;
	cv::Mat image;
  };
  class PairComp {
  public: 
	bool operator() (const std::pair<int,int> &lhs, const std::pair<int,int> &rhs) {
      return (lhs.first > rhs.first || ( lhs.first == rhs.first && lhs.second > rhs.second)) ;
	}
  };

  void NDTCostmap::saveCostMapIncr(std::string filename, double max_h) {
	if(!alloc) return;
	//std::cerr<<"displaying map\n";
	std::vector<MapPair> maps, maps_final;
	std::set<std::pair<int,int>,PairComp >merge_maps;

	for(int ix=0; ix<sx; ++ix) {
      for(int iy=0; iy<sy; ++iy) {
		for(int j=0; j<levels[ix][iy].clear_levels.size(); ++j) {
          //check neighbours for connectivity
          double my_h = levels[ix][iy].clear_levels[j];
          std::vector<int> map_ids_neighbours;
          std::vector<int> level_index_neighbours;
          std::vector<std::pair<int,int> > xy_ids_neighbours;
		    
          //check if our level fits with one of the levels of the neighbours
          if(ix > 0 && iy > 0) {
			//check ix-1,iy-1
			for(int p1=0; p1<levels[ix-1][iy-1].clear_levels.size(); ++p1) {
              if(fabsf(my_h-levels[ix-1][iy-1].clear_levels[p1]) < max_h) {
				xy_ids_neighbours.push_back(std::pair<int,int> (ix-1,iy-1));
				map_ids_neighbours.push_back(levels[ix-1][iy-1].map_ids[p1]);
				level_index_neighbours.push_back(p1);
				break;
              }
			}	
          }
          if(ix > 0) {
			//check ix-1,iy
			for(int p1=0; p1<levels[ix-1][iy].clear_levels.size(); ++p1) {
              if(fabsf(my_h-levels[ix-1][iy].clear_levels[p1]) < max_h) {
				xy_ids_neighbours.push_back(std::pair<int,int> (ix-1,iy));
				map_ids_neighbours.push_back(levels[ix-1][iy].map_ids[p1]);
				level_index_neighbours.push_back(p1);
				break;
              }
			}	
          }
          if( iy >0 ) {
			//check ix,iy-1
			for(int p1=0; p1<levels[ix][iy-1].clear_levels.size(); ++p1) {
              if(fabsf(my_h-levels[ix][iy-1].clear_levels[p1]) < max_h) {
				xy_ids_neighbours.push_back(std::pair<int,int> (ix,iy-1));
				map_ids_neighbours.push_back(levels[ix][iy-1].map_ids[p1]);
				level_index_neighbours.push_back(p1);
				break;
              }
			}
          }

          //at least one neighbour with a level	    
          if(level_index_neighbours.size() > 0) {
			//add ourselves to the first map that fits our level.
			levels[ix][iy].map_ids.push_back(levels[xy_ids_neighbours[0].first][xy_ids_neighbours[0].second].map_ids[level_index_neighbours[0]]);
			maps[map_ids_neighbours[0]].image.at<float>(ix,iy) = 1;
			maps[map_ids_neighbours[0]].mean_height = (maps[map_ids_neighbours[0]].mean_height*maps[map_ids_neighbours[0]].n_samples + my_h) /
              (maps[map_ids_neighbours[0]].n_samples + 1); 
			maps[map_ids_neighbours[0]].n_samples ++;

			//check if we should merge neighbours maps
			if(level_index_neighbours.size() == 2) {
              //if ids are different
              if(map_ids_neighbours[0] > map_ids_neighbours[1]) {
				merge_maps.insert(std::pair<int,int> (map_ids_neighbours[0],map_ids_neighbours[1]));
              }
              if(map_ids_neighbours[0] < map_ids_neighbours[1]) {
				merge_maps.insert(std::pair<int,int> (map_ids_neighbours[1],map_ids_neighbours[0]));
              }
			}
			if(level_index_neighbours.size() == 3) {
              if(map_ids_neighbours[0] > map_ids_neighbours[1]) {
				merge_maps.insert(std::pair<int,int> (map_ids_neighbours[0],map_ids_neighbours[1]));
              }
              if(map_ids_neighbours[1] > map_ids_neighbours[2]) {
				merge_maps.insert(std::pair<int,int> (map_ids_neighbours[1],map_ids_neighbours[2]));
              }
              if(map_ids_neighbours[0] > map_ids_neighbours[2]) {
				merge_maps.insert(std::pair<int,int> (map_ids_neighbours[0],map_ids_neighbours[2]));
              }
              if(map_ids_neighbours[0] < map_ids_neighbours[1]) {
				merge_maps.insert(std::pair<int,int> (map_ids_neighbours[1],map_ids_neighbours[0]));
              }
              if(map_ids_neighbours[1] < map_ids_neighbours[2]) {
				merge_maps.insert(std::pair<int,int> (map_ids_neighbours[2],map_ids_neighbours[1]));
              }
              if(map_ids_neighbours[0] < map_ids_neighbours[2]) {
				merge_maps.insert(std::pair<int,int> (map_ids_neighbours[2],map_ids_neighbours[0]));
              }
			}
          } else {
			//allocate new map and assign ourselves to it
			MapPair mp;
			mp.mean_height = levels[ix][iy].clear_levels[j];
			mp.n_samples = 1;
			//std::cerr<<"create cv mat of size "<<sx<<" "<<sy<<std::endl;
			mp.image = cv::Mat::zeros(sx,sy,CV_32F);
			//std::cerr<<"writing pixel at "<<ix<<" "<<iy<<std::endl;
			mp.image.at<float>(ix,iy) = 1;
			levels[ix][iy].map_ids.push_back(maps.size());
			maps.push_back(mp);
          }
		}
      }
      //std::cerr<<std::endl;
	}

	std::set<std::pair<int,int>,PairComp >::reverse_iterator itr;
	std::vector<std::vector<int> > mapindex;
	std::vector<std::vector<int> >::iterator jtr, jtr_tmp, prev_insert, tmp;
	for(int q=0; q<maps.size(); ++q) {
      std::vector<int> tmp;
      tmp.push_back(q);
      mapindex.push_back(tmp);
	}

	jtr = mapindex.begin();
	jtr_tmp = jtr;
	//merge maps
	for(itr=merge_maps.rbegin(); itr!=merge_maps.rend(); itr++) {
      bool did_insert = false;
      //std::cerr<<"merge maps "<<itr->first<<" "<<itr->second<<std::endl;
      //search for jtr == itr->first to define search region
      //std::cerr<<"at jtr "<<(*jtr)[0]<<std::endl;
      while((*jtr)[0] < itr->first) jtr++;
      //std::cerr<<"move to jtr "<<(*jtr)[0]<<std::endl;
      for(jtr_tmp = mapindex.begin(); jtr_tmp!=jtr; jtr_tmp++) {
		for(int i=0; i<jtr_tmp->size(); i++) {
          if((*jtr_tmp)[i] == itr->second) {
			if(!did_insert) {
              jtr_tmp->push_back(itr->first);
              prev_insert = jtr_tmp;
              did_insert = true;
			} else {
              //we need to merge two buckets
              prev_insert->insert(prev_insert->end(), jtr_tmp->begin(), jtr_tmp->end());
              //delete jtr bucket
              tmp = jtr_tmp;
              jtr_tmp --;
              mapindex.erase(tmp);	    
			}
			break;
          }
          if(i > 0 && (*jtr_tmp)[i] == itr->first) {
			if(!did_insert) {
              jtr_tmp->push_back(itr->second);
              prev_insert = jtr_tmp;
              did_insert = true;
			} else {
              //we need to merge two buckets
              prev_insert->insert(prev_insert->end(), jtr_tmp->begin(), jtr_tmp->end());
              //delete jtr bucket
              tmp = jtr_tmp;
              jtr_tmp --;
              mapindex.erase(tmp);	    
			}
			break;
          }
		}
      }
      jtr_tmp = jtr;
      jtr--;
      //std::cerr<<"move back to jtr "<<(*jtr)[0]<<std::endl;
      mapindex.erase(jtr_tmp);	    
	}

	for(jtr = mapindex.begin(); jtr!=mapindex.end(); jtr++) {
      if(jtr->size() == 1) {
		//copy map
		if(maps[(*jtr)[0]].n_samples > 20) {
          maps_final.push_back(maps[(*jtr)[0]]);
          //std::cout<<(*jtr)[0]<<std::endl;
		}
      } else {
		MapPair mp;
		mp.image = cv::Mat::zeros(sx,sy,CV_32F);
		mp.mean_height = 0;
		mp.n_samples = 0;
		for(int i=0; i<jtr->size(); ++i) {
          //std::cout<<(*jtr)[i]<<" ";
          mp.mean_height += maps[(*jtr)[i]].mean_height;
          mp.image = mp.image | maps[(*jtr)[i]].image;
          mp.n_samples += maps[(*jtr)[i]].n_samples;
		}
		if(mp.n_samples > 20) {
          mp.mean_height /= mp.n_samples;
          maps_final.push_back(mp);
		}
		//std::cout<<std::endl;
      }
	}

	maps = maps_final;

	//std::cerr<<"Created "<<maps.size()<<" maps\n";
	for(int q=0; q<maps.size(); ++q) {
      char win_name[300];
      snprintf(win_name,299,"Map Level %lf",maps[q].mean_height);
      cv::namedWindow(win_name);
      maps[q].image *= 255;
      cv::imshow(win_name, maps[q].image);
      snprintf(win_name,299,"MapLevel%lf.jpg",maps[q].mean_height);
      cv::imwrite(win_name,maps[q].image);
	}
	

  }
  void NDTCostmap::saveCostMap(std::string filename, double max_h) {
	if(!alloc) return;
	//std::cerr<<"displaying map\n";
	std::vector<MapPair> maps;
	for(int ix=0; ix<sx; ++ix) {
      for(int iy=0; iy<sy; ++iy) {
		for(int j=0; j<levels[ix][iy].clear_levels.size(); ++j) {
          bool found = false;
          int idx = -1;
          for(int q=0; q<maps.size(); ++q) {
			if(fabsf(maps[q].mean_height - levels[ix][iy].clear_levels[j]) < max_h) {
              found = true;
              idx = q;
              break;
			}
          }
          if(found) {
			maps[idx].mean_height = maps[idx].mean_height*maps[idx].n_samples + levels[ix][iy].clear_levels[j];
			maps[idx].n_samples++;
			maps[idx].mean_height /= maps[idx].n_samples;
			//std::cerr<<"writing pixel at "<<ix<<" "<<iy<<std::endl;
			maps[idx].image.at<float>(ix,iy) = 1;
          } else {
			//std::cerr<<"new map\n";
			MapPair mp;
			mp.mean_height = levels[ix][iy].clear_levels[j];
			mp.n_samples = 1;
			//std::cerr<<"create cv mat of size "<<sx<<" "<<sy<<std::endl;
			mp.image = cv::Mat::zeros(sx,sy,CV_32F);
			//std::cerr<<"writing pixel at "<<ix<<" "<<iy<<std::endl;
			mp.image.at<float>(ix,iy) = 1;
			maps.push_back(mp);
          }
		}
		/*
          if(levels[ix][iy].clear_levels.size() == 0) {
          std::cerr<<"0";
          } else {
          std::cerr<<"1";
          }*/
      }
      //	    std::cerr<<std::endl;
	}
	//std::cerr<<"Created "<<maps.size()<<" maps\n";
	for(int q=0; q<maps.size(); ++q) {
      char win_name[300];
      snprintf(win_name,299,"Map Level %lf",maps[q].mean_height);
      cv::namedWindow(win_name);
      cv::imshow(win_name, maps[q].image);
      snprintf(win_name,299,"MapLevel%lf.png",maps[q].mean_height);
      cv::imwrite(win_name,maps[q].image);
	}
	

  }
  
  /* bool NDTCostmap::toMessage(ndt_costmap::TraversabilityMapMsg msg, double max_h, std::string frame_id){ */
  /*   if(!alloc) return; */
  /*   double size_x, size_y, size_z; */
  /*   int size_x_cell_count, size_y_cell_count; */
  /*   double cen_x, cen_y, cen_z; */
  /*   double orig_x, orig_y; */
  /*   ndt_map->getGridSizeInMeters(size_x,size_y,size_z); */
  /*   ndt_map->getCentroid(cen_x,cen_y,cen_z); */
  /*   orig_x=cen_x-size_x/2.0; */
  /*   orig_y=cen_y-size_y/2.0; */
  /*   size_x_cell_count=int(size_x/resolution); */
  /*   size_y_cell_count=int(size_y/resolution); */
  /*   ros::Time n=ros::Time::now(); */
    
  /*   std::vector<MapPairMsg> maps; */
  /*   for(int ix=0; ix<sx; ++ix) { */
  /*     for(int iy=0; iy<sy; ++iy) { */
  /*   	for(int j=0; j<levels[ix][iy].clear_levels.size(); ++j) { */
  /*         bool found = false; */
  /*         int idx = -1; */
  /*         for(int q=0; q<maps.size(); ++q) { */
  /*   		if(fabsf(maps[q].mean_height - levels[ix][iy].clear_levels[j]) < max_h) { */
  /*             found = true; */
  /*             idx = q; */
  /*             break; */
  /*   		} */
  /*         } */
  /*         if(found) { */
  /*   		maps[idx].mean_height = maps[idx].mean_height*maps[idx].n_samples + levels[ix][iy].clear_levels[j]; */
  /*   		maps[idx].n_samples++; */
  /*   		maps[idx].mean_height /= maps[idx].n_samples; */
  /*   		//std::cerr<<"writing pixel at "<<ix<<" "<<iy<<std::endl; */
  /*   		maps[idx].image.at<float>(ix,iy) = 1; */
  /*         } else { */
  /*   		//std::cerr<<"new map\n"; */
  /*   		MapPairMsg mp; */
  /*   		mp.mean_height = levels[ix][iy].clear_levels[j]; */
  /*   		mp.n_samples = 1; */
  /*   		//std::cerr<<"create cv mat of size "<<sx<<" "<<sy<<std::endl; */
  /*   		mp.image = cv::Mat::zeros(sx,sy,CV_32F); */
  /*   		//std::cerr<<"writing pixel at "<<ix<<" "<<iy<<std::endl; */
  /*   		mp.image.at<float>(ix,iy) = 1; */
  /*   		maps.push_back(mp); */
  /*         } */
  /*   	} */
  /*   	/\* */
  /*         if(levels[ix][iy].clear_levels.size() == 0) { */
  /*         std::cerr<<"0"; */
  /*         } else { */
  /*         std::cerr<<"1"; */
  /*         }*\/ */
  /*     } */
  /*     //	    std::cerr<<std::endl; */
  /*   } */
  /*   //std::cerr<<"Created "<<maps.size()<<" maps\n"; */
  /*   for(int q=0; q<maps.size(); ++q) { */
  /*     char win_name[300]; */
  /*     snprintf(win_name,299,"Map Level %lf",maps[q].mean_height); */
  /*     cv::namedWindow(win_name); */
  /*     cv::imshow(win_name, maps[q].image); */
  /*     snprintf(win_name,299,"MapLevel%lf.png",maps[q].mean_height); */
  /*     cv::imwrite(win_name,maps[q].image); */
  /*   } */
	





  /* } */


  
}
#endif
