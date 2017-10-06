#ifndef NDTDL_POINT_CURV_H
#define NDTDL_POINT_CURV_H

#include <pcl/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>
#include <angles/angles.h>
namespace perception_oru{
namespace libgraphMap{

typedef velodyne_pointcloud::PointXYZIR PointType;

void segmentPointCurvature(const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud,
                           pcl::PointCloud<pcl::PointXYZ> &cornerPointsSharp,
                           pcl::PointCloud<pcl::PointXYZ> &cornerPointsLessSharp,
                           pcl::PointCloud<pcl::PointXYZ> &surfPointsFlat,
                           pcl::PointCloud<pcl::PointXYZ> &surfPointsLessFlat) {

  float cloudCurvature[80000];
  int cloudSortInd[80000];
  int cloudNeighborPicked[80000];
  int cloudLabel[80000];

  // Sort based on the ring number
  // Split the data based on each diode (ring)
  std::vector<pcl::PointCloud<velodyne_pointcloud::PointXYZIR> > ring_clouds(64);
  ring_clouds.reserve(64);
  int N_SCANS = 0;
  for (int i = 0; i < cloud.size(); i++) {
    velodyne_pointcloud::PointXYZIR p = cloud.points[i];
    ring_clouds[p.ring].push_back(p);
    if (p.ring > N_SCANS)
      N_SCANS = p.ring;
  }


  // Code from LOAM.
  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);

  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  for (int i = 0; i < ring_clouds.size(); i++) {
    *laserCloud += ring_clouds[i];
  }

  int cloudSize = laserCloud->size();

  int scanCount = -1;
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x
        + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
        + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
        + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
        + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
        + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y
        + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
        + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
        + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
        + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
        + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z
        + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
        + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
        + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
        + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
        + laserCloud->points[i + 5].z;

    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    if (int(laserCloud->points[i].intensity) != scanCount) {
      scanCount = int(laserCloud->points[i].intensity);

      if (scanCount > 0 && scanCount < N_SCANS) {
        scanStartInd[scanCount] = i + 5;
        scanEndInd[scanCount - 1] = i - 5;
      }
    }
  }

  scanStartInd[0] = 5;
  scanEndInd.back() = cloudSize - 5;

  for (int i = 5; i < cloudSize - 6; i++) {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    if (diff > 0.1) {

      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                          laserCloud->points[i].y * laserCloud->points[i].y +
                          laserCloud->points[i].z * laserCloud->points[i].z);

      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
          laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
          laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }

    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x
        + laserCloud->points[i].y * laserCloud->points[i].y
        + laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }

  cornerPointsSharp.clear();
  cornerPointsLessSharp.clear();
  surfPointsFlat.clear();
  surfPointsLessFlat.clear();

  for (int i = 0; i < N_SCANS; i++) {

    /////////////////
    //ROS_INFO_STREAM("scanStartInd[i] " << scanStartInd[i] << " scanEndInd[i] " << scanEndInd[i]);

    for (int k = scanStartInd[i]+10; k < scanEndInd[i]-10; k++) {

      float timeoffset = laserCloud->points[i].intensity;
      //ROS_INFO_STREAM("timeoffset : " << timeoffset);

      float ori = atan2(laserCloud->points[k].x, laserCloud->points[k].z);
      float ori_prev = atan2(laserCloud->points[k-1].x, laserCloud->points[k-1].z);
      float ori_next = atan2(laserCloud->points[k+1].x, laserCloud->points[k+1].z);

      float diff1 = fabs(angles::normalize_angle(ori_prev-ori));
      float diff2 = fabs(angles::normalize_angle(ori-ori_next));

      if (diff1 > M_PI/2.)
        continue;
      if (diff2 > M_PI/2.)
        continue;
      if (diff1 < 0.001)
        continue;
      if (diff2 < 0.001)
        continue;

      //      if (diff1*10 < diff2) {
      //        pointsNaNToEdge.push_back(laserCloud->points[k]);
      //        ROS_INFO_STREAM("++ori_prev : " << ori_prev << " ori : " << ori << " ori_next : " << ori_next << " diff1 : " << diff1 << " diff2 : " << diff2 << " ring : " << int(laserCloud->points[k].intensity));

      //      }
      //      if (diff2*10 < diff1) {
      //        pointsEdgeToNaN.push_back(laserCloud->points[k]);
      //        ROS_INFO_STREAM("--ori_prev : " << ori_prev << " ori : " << ori << " ori_next : " << ori_next << " diff1 : " << diff1 << " diff2 : " << diff2 << " ring : " << int(laserCloud->points[k].intensity));

      //      }
    }


    /////////



    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++) {
      int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;
      int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;

      for (int k = sp + 1; k <= ep; k++) {
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > 0.1) {

          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp.push_back(pcl::PointXYZ(laserCloud->points[ind].x,
                                                      laserCloud->points[ind].y,
                                                      laserCloud->points[ind].z));
            cornerPointsLessSharp.push_back(pcl::PointXYZ(laserCloud->points[ind].x,
                                                          laserCloud->points[ind].y,
                                                          laserCloud->points[ind].z));
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp.push_back(pcl::PointXYZ(laserCloud->points[ind].x,
                                                          laserCloud->points[ind].y,
                                                          laserCloud->points[ind].z));
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x
                - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x
                - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y
                - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z
                - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];

        if (ind < 5) {
          // Henrik: this happens occationally -> TODO find out why. Commonly the idx == 0.
          break;
        }

        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < 0.1) {

          cloudLabel[ind] = -1;
          surfPointsFlat.push_back(pcl::PointXYZ(laserCloud->points[ind].x,
                                                 laserCloud->points[ind].y,
                                                 laserCloud->points[ind].z));

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x
                - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x
                - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y
                - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z
                - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlat.push_back(pcl::PointXYZ(laserCloud->points[k].x,
                                                     laserCloud->points[k].y,
                                                     laserCloud->points[k].z));
        }
      }
    }

    //    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    //    pcl::VoxelGrid<PointType> downSizeFilter;
    //    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    //    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    //    downSizeFilter.filter(surfPointsLessFlatScanDS);

    //    surfPointsLessFlat += surfPointsLessFlatScanDS;
    //surfPointsLessFlat += *surfPointsLessFlatScan;

  }

}

}
}

#endif
