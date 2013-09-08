#include <Eigen/Eigen>
#include <cstdio>
#include <iostream>

using namespace std;

int main(int argc, char ** argv)
{

    if(argc != 2)
    {
        cout<<"Usage:"<<argv[0]<<" pathToLogFile\n";
        return -1;
    }

    const char *fname = argv[1];
    char foutname [500];

    snprintf(foutname,499,"%s.quat",fname);

    cout<<"saving to << "<<foutname<<endl;

    FILE *fin = fopen(fname,"r");
    FILE *fout= fopen(foutname,"w");

    //per file
    char *line = NULL;
    size_t len;
    if(fin == NULL || fout==NULL)
    {
        cout<<"Error reading file "<<fname<<endl;
        return -1;
    }

    bool first = true;
    double xd,yd,zd, r,p,ya, x,y,z;
    Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> currPose;

    while(getline(&line,&len,fin) > 0)
    {

        double ts;
        int n = sscanf(line,"%lf %lf %lf %lf %lf %lf %lf",
                       &ts,&x,&y,&z,&r,&p,&ya);
        if(n != 7)
        {
            cout<<"wrong format of pose at : "<<line<<endl;
            break;
        }

        x = x/1000;
        y = y/1000;
        z = z/1000;

        currPose =Eigen::Translation<double,3>(x,y,z)*
                  Eigen::AngleAxis<double>(r,Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxis<double>(p,Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxis<double>(ya,Eigen::Vector3d::UnitZ()) ;


        Eigen::Vector3d v = currPose.translation();
        Eigen::Quaterniond q;
        q = currPose.rotation();

        fprintf(fout,"0 %lf %lf %lf %lf %lf %lf %lf\n",
                v(0),v(1),v(2),q.x(),q.y(),q.z(),q.w());

    }

    fclose(fin);
    fclose(fout);

}
