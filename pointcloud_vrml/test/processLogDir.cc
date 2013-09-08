#include <pointcloud_utils.h>
#include <cstdio>
#include <iostream>

using namespace std;

int main(int argc, char ** argv)
{

    if(argc != 5)
    {
        cout<<"Usage: ./logProcessor [b|h|m] numberOfFiles logPrefix outputPrefix\n\t b -- bremen log format \n\t h -- hanover log format\n";
        return -1;
    }

    bool isBremen = (argv[1][0] == 'b');
    bool isHanover = (argv[1][0] == 'h');
    bool isMartin = (argv[1][0] == 'm');

    if(!(isBremen || isHanover || isMartin))
    {
        cout<<"error, must be bremen or hanover format! \n";
        cout<<"Usage: ./logProcessor [b|h] numberOfFiles logPrefix outputPrefix\n\t b -- bremen log format \n\t h -- hanover log format\n";
        return -1;
    }

    const char *inDirName = argv[3];
    const char *outName = argv[4];
    //assume 3 digit format...

    int nFiles = atoi(argv[2]);

    int off = 0;
    for(int fileno=0; fileno<nFiles; fileno++)
    {

        if(fileno == 600)
        {
            fileno++;
            off=1;
        }

        char fname[300];
        pcl::PointCloud<pcl::PointXYZ> cloud;
        if(isBremen)
        {
            snprintf(fname,300,"%s%03d.txt",inDirName,fileno);
        }
        if(isHanover)
        {
            snprintf(fname,300,"%s%d.3d",inDirName,fileno+1);
        }
        if(isMartin)
        {
            snprintf(fname,300,"%s%03d.wrl",inDirName,fileno);
            pcl::PointCloud<pcl::PointXYZ> cl = lslgeneric::readVRML<pcl::PointXYZ>(fname);
            pcl::PointXYZ pt;
            for(int i=0; i<cl.points.size(); i++)
            {
                pt = cl.points[i];
                double d = sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
                if(d>1)
                {
                    cloud.points.push_back(pt);
                }
            }
        }
        else
        {

            FILE *vrml = fopen(fname,"r");
            //per file
            char *line = NULL;
            size_t len;
            bool first=true;
            size_t length = 0;
            if(vrml == NULL)
            {
                cout<<"Error reading file "<<fname<<endl;
                continue;
            }

            double MAX_RANGE = 25.0;
            while(getline(&line,&len,vrml) >0 )
            {
                if(first && isBremen)
                {
                    first=false;
                    continue;
                }
                else
                {
                    //read everything until ]
                    char *token = strtok(line," ");
                    pcl::PointXYZ pt;
                    if(token == NULL) continue;
                    pt.x = atof(token)/1000.;
                    token = strtok(NULL," ");
                    if(token == NULL) continue;
                    pt.y = atof(token)/1000.;
                    token = strtok(NULL," ");
                    if(token == NULL) continue;
                    pt.z = 1 - atof(token)/1000.;
                    double len = sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
                    if(len < MAX_RANGE)
                    {
                        cloud.points.push_back(pt);
                    }
                }
            }
            length = cloud.points.size();
            cloud.width = length;
            cloud.height = 1;
        }

        snprintf(fname,300,"%s%03d.wrl",outName,fileno-off);
        FILE *fout = fopen(fname,"w");

        fprintf(fout,"#VRML V2.0 utf8\n");
        fprintf(fout,"Shape {\n geometry PointSet {\n coord Coordinate {\n point [\n");
        for(unsigned int pit=0; pit<cloud.points.size(); ++pit)
        {
            pcl::PointXYZ thisPoint = cloud.points[pit];
            if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
            fprintf(fout,"%.5lf %.5lf %.5lf\n", thisPoint.x, thisPoint.y, thisPoint.z);
        }
        fprintf(fout,"]\n }\n }\n }\n");
        fclose(fout);
    }
}
