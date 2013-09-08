#include <Eigen/Eigen>
#include <cstdio>
#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char ** argv)
{

    if(argc != 3)
    {
        cout<<"Usage:"<<argv[0]<<" pathToGTtrajectory pathToProposedTrajectory\n";
        return -1;
    }

    const char *fname1 = argv[1];
    const char *fname2 = argv[2];

    char foutname [500];

    snprintf(foutname,499,"%s_res.m",fname1);

    cout<<"saving to << "<<foutname<<endl;

    FILE *fin1 = fopen(fname1,"r");
    FILE *fin2 = fopen(fname2,"r");
    FILE *fout= fopen(foutname,"w");

    //per file
    char *line = NULL;
    size_t len;
    if(fin1 == NULL ||fin2 == NULL || fout==NULL)
    {
        cout<<"Error reading file "<<fname1<<endl;
        return -1;
    }

    bool first = true;
    double x,y,z, px,py,pz,pw, ts;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> refPose,Pose,dP;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> refPosePrev,PosePrev;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> refPoseRel,PoseRel;

    Eigen::Quaterniond id, errorQ;
    id.setIdentity();

    vector<double> eT, eR;

    while(getline(&line,&len,fin1) > 0)
    {

        int n = sscanf(line,"%lf %lf %lf %lf %lf %lf %lf %lf",
                       &ts,&x,&y,&z,&px,&py,&pz,&pw);
        if(n != 8)
        {
            cout<<"wrong format of pose at : "<<line<<endl;
            break;
        }

        refPose =Eigen::Translation<double,3>(x,y,z)*
                 Eigen::Quaternion<double>(pw,px,py,pz);


        if(getline(&line,&len,fin2) > 0)
        {

            int n2 = sscanf(line,"%lf %lf %lf %lf %lf %lf %lf %lf",
                            &ts,&x,&y,&z,&px,&py,&pz,&pw);
            if(n2 != 8)
            {
                cout<<"wrong format of pose at : "<<line<<endl;
                break;
            }
            Pose =Eigen::Translation<double,3>(x,y,z)*
                  Eigen::Quaternion<double>(pw,px,py,pz);

        }
        else
        {
            continue;
        }

        if(first)
        {
            refPosePrev = refPose;
            PosePrev = Pose;
            first = false;
            continue;
        }


        //find the two relative poses
        refPoseRel = refPosePrev.inverse()*refPose;
        PoseRel = PosePrev.inverse()*Pose;

        dP = PoseRel.inverse()*refPoseRel;
        errorQ = dP.rotation();
        double angle = acos(id.dot(errorQ))/2;

        eR.push_back(angle);
        eT.push_back(dP.translation().norm());

        refPosePrev = refPose;
        PosePrev = Pose;

    }

    fclose(fin1);
    fclose(fin2);

    fprintf(fout,"eT = [");
    for(int i=0; i<eT.size(); i++)
    {
        fprintf(fout,"%lf ",eT[i]);
    }
    fprintf(fout,"];\n");

    fprintf(fout,"eR = [");
    for(int i=0; i<eR.size(); i++)
    {
        fprintf(fout,"%lf ",eR[i]);
    }
    fprintf(fout,"];\n");

    fclose(fout);

}
