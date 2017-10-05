#ifndef MOTIONMODELS_H
#define MOTIONMODELS_H
#include "ndt_generic/motion_model_3d.h"
//Until ported this to YAML file
namespace lslgeneric
{
bool GetMotionModel(const std::string &dataset, std::vector<double> &motion_model, std::vector<double> &motion_model_offset){

 if(dataset.compare("hx")==0){
   motion_model[0] = 0.01;
   motion_model[1] = 0.002;
   motion_model[2] = 0.001;
   motion_model[3] = 0.001;
   motion_model[5] = 0.005;
   motion_model[4] = 0.001;

   motion_model[6] = 0.002;
   motion_model[7] = 0.005;
   motion_model[8] = 0.001;
   motion_model[9] = 0.001;
   motion_model[10] = 0.001;
   motion_model[11] = 0.005;

   motion_model[12] = 0.005;
   motion_model[13] = 0.001;
   motion_model[14] = 0.01;
   motion_model[15] = 0.0001;
   motion_model[16] = 0.0001;
   motion_model[17] = 0.005;

   motion_model[18] = 0.002;
   motion_model[19] = 0.001;
   motion_model[20] = 0.001;
   motion_model[21] = 0.01;
   motion_model[22] = 0.001;
   motion_model[23] = 0.001;

   //parPtr->motion_model[24] = 0.002;
   //parPtr->motion_model[25] = 0.0001;
   //parPtr->motion_model[26] = 0.001;
   //parPtr->motion_model[27] = 0.001;
   //parPtr->motion_model[28] = 0.01;
   //parPtr->motion_model[29] = 0.001;
   motion_model[25] = 0.005;
   motion_model[26] = 0.002;
   motion_model[24] = 0.0001;
   motion_model[27] = 0.001;
   motion_model[28] = 0.04;
   motion_model[29] = 0.001;

   motion_model[30] = 0.005;
   motion_model[31] = 0.002;
   motion_model[32] = 0.0001;
   motion_model[33] = 0.001;
   motion_model[34] = 0.001;
   motion_model[35] = 0.01;

   motion_model_offset[0] = 0.02;
   motion_model_offset[1] = 0.00002;
   motion_model_offset[2] = 0.002;
   motion_model_offset[3] = 0.000002;
   motion_model_offset[4] = 0.02;
   motion_model_offset[5] = 0.000002;
 }
 else if(dataset.compare("arla-2012")==0){
     motion_model.push_back(0.05);
     motion_model.push_back(0.05);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.02);

     motion_model.push_back(0.05);
     motion_model.push_back(0.1);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.02);


     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);

     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);

     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);
     motion_model.push_back(0.00000);

     motion_model.push_back(0.01);
     motion_model.push_back(0.01);
     motion_model.push_back(0.00001);
     motion_model.push_back(0.00001);
     motion_model.push_back(0.00001);
     motion_model.push_back(0.01);

     /*motion_model_offset.push_back(0.00);
     motion_model_offset.push_back(0.002);
     motion_model_offset.push_back(0.0000001);//0.002
     motion_model_offset.push_back(0.0000001);//0.001
     motion_model_offset.push_back(0.0000001);//0.001
     motion_model_offset.push_back(0.001);*/

     motion_model_offset.push_back(0.005);
     motion_model_offset.push_back(0.005);
     motion_model_offset.push_back(0.000000001);//0.002
     motion_model_offset.push_back(0.0000001);//0.001
     motion_model_offset.push_back(0.0000001);//0.001
   motion_model_offset.push_back(0.003);
 /*  motion_model[0] = 0.01;
   motion_model[1] = 0.00001;
   motion_model[2] = 0.00001;
   motion_model[3] = 0.00001;
   motion_model[5] = 0.00001;
   motion_model[4] = 0.00001;

   motion_model[6] = 0.00001;
   motion_model[7] = 0.01;
   motion_model[8] = 0.00001;
   motion_model[9] = 0.00001;
   motion_model[10] = 0.00001;
   motion_model[11] = 0.00001;

   motion_model[12] = 0.00001;
   motion_model[13] = 0.00001;
   motion_model[14] = 0.00000;
   motion_model[15] = 0.00001;
   motion_model[16] = 0.00001;
   motion_model[17] = 0.00001;

   motion_model[18] = 0.00001;
   motion_model[19] = 0.00001;
   motion_model[20] = 0.00001;
   motion_model[21] = 0.00001;
   motion_model[22] = 0.00000;
   motion_model[23] = 0.00001;

   //parPtr->motion_model[24] = 0.002;
   //parPtr->motion_model[25] = 0.0001;
   //parPtr->motion_model[26] = 0.001;
   //parPtr->motion_model[27] = 0.001;
   //parPtr->motion_model[28] = 0.01;
   //parPtr->motion_model[29] = 0.001;
   motion_model[25] = 0.00001;
   motion_model[26] = 0.00001;
   motion_model[24] = 0.00001;
   motion_model[27] = 0.00001;
   motion_model[28] = 0.00001;
   motion_model[29] = 0.00000;

   motion_model[30] = 0.00001;
   motion_model[31] = 0.00001;
   motion_model[32] = 0.00001;
   motion_model[33] = 0.00001;
   motion_model[34] = 0.00001;
   motion_model[35] = 0.001;

   motion_model_offset[0] = 0.00002;
   motion_model_offset[1] = 0.00002;
   motion_model_offset[2] = 0.0;
   motion_model_offset[3] = 0.0;
   motion_model_offset[4] = 0.0;
   motion_model_offset[5] = 0.000002; */
 }


}


}
#endif // MOTIONMODELS_H
