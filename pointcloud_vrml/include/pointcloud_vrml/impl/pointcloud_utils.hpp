namespace lslgeneric
{

template< typename PointT>
pcl::PointCloud<PointT> readVRML(const char* fname)
{
    pcl::PointCloud<PointT> cloud;
    FILE *vrml = fopen(fname,"r");

    cloud = lslgeneric::readVRML<PointT>(vrml);

    if(vrml!= NULL) fclose(vrml);
    return cloud;
}

template< typename PointT>
pcl::PointCloud<PointT> readVRML(FILE* vrml)
{

    pcl::PointCloud<PointT> cloud;
    char *line = NULL;
    size_t len;
    bool first=true;
    size_t length = 0;
    if(vrml == NULL)
    {
        std::cout<<"couldn't process vrml file\n";
        return cloud;
    }

    while(getline(&line,&len,vrml) >0 )
    {
        if(first)
        {
            //look for 'point [' token
            char *token = strtok(line," ");
            while(token!=NULL)
            {
                if(strncmp("point",token,5)==0)
                {
                    first=false;
                    break;
                }
                else
                {
                    token=strtok(NULL," ");
                }
            }
        }
        else
        {
            //read everything until ]
            char *token = strtok(line," ");
            if(strncmp("]",token,1)==0)
            {
                first=true;
                continue;
            }
            PointT pt;
            if(token == NULL) continue;
            pt.x = atof(token);
            token = strtok(NULL," ");
            if(token == NULL) continue;
            pt.y = atof(token);
            token = strtok(NULL," ");
            if(token == NULL) continue;
            pt.z = atof(token);
            cloud.points.push_back(pt);
        }
    }
    length = cloud.points.size();
    cloud.width = length;
    cloud.height = 1;
    return cloud;
}

///with intensity info from the colors
template< typename PointT>
pcl::PointCloud<PointT> readVRMLIntensity(const char* fname)
{
    pcl::PointCloud<PointT> cloud;
    FILE *vrml = fopen(fname,"r");

    cloud = lslgeneric::readVRMLIntensity<PointT>(vrml);

    fclose(vrml);
    return cloud;
}

template< typename PointT>
pcl::PointCloud<PointT> readVRMLIntensity(FILE* vrml)
{

    pcl::PointCloud<PointT> cloud;
    char *line = NULL;
    size_t len;
    bool first=true;
    bool second=false;
    int ctr=0;

    size_t length = 0;
    if(vrml == NULL) return cloud;

    while(getline(&line,&len,vrml) >0 )
    {
        if(first)
        {
            //look for 'point [' token
            char *token = strtok(line," ");
            while(token!=NULL)
            {
                if(strncmp("point",token,5)==0)
                {
                    first=false;
                    second=false;
                    break;
                }
                else if(strncmp("color",token,5)==0)
                {
                    first=false;
                    second=true;
                    break;
                }
                else
                {
                    token=strtok(NULL," ");
                }
            }
        }
        else
        {
            if(!second)
            {
                //read everything until ]
                char *token = strtok(line," ");
                if(strncmp("]",token,1)==0)
                {
                    first=true;
                    continue;
                }
                PointT pt;
                if(token == NULL) continue;
                pt.x = atof(token);
                token = strtok(NULL," ");
                if(token == NULL) continue;
                pt.y = atof(token);
                token = strtok(NULL," ");
                if(token == NULL) continue;
                pt.z = atof(token);
                cloud.points.push_back(pt);
            }
            else
            {
                //we are at second pass, reading color info
                char *token = strtok(line," ");
                if(strncmp("]",token,1)==0)
                {
                    first=true;
                    second=false;
                    continue;
                }
                if(strncmp("color",token,5)==0)
                {
                    continue;
                }
                if(ctr<cloud.points.size())
                {
                    if(token == NULL) continue;
                    //red channel, skip
                    token = strtok(NULL," ");
                    if(token == NULL) continue;
                    //green channel = intensity
                    cloud.points[ctr].intensity = atof(token);
                    token = strtok(NULL," ");
                    //blue channel, skip
                    ctr++;
                }
                else
                {
                    //error occured, we are at the wrong place
                    first=true;
                    second=false;
                    continue;

                }
            }
        }
    }
    length = cloud.points.size();
    cloud.width = length;
    cloud.height = 1;
    return cloud;
}

template< typename PointT>
void writeToVRML(const char* fname, pcl::PointCloud<PointT> &pc, Eigen::Vector3d col)
{
    FILE *out = fopen(fname,"w");
    fprintf(out,"#VRML V2.0 utf8\n");
    writeToVRML<PointT>(out,pc,col);
    fclose(out);
}

template< typename PointT>
void writeToVRML(FILE* fout, pcl::PointCloud<PointT> &pc,
                 Eigen::Vector3d col)
{
    fprintf(fout,"Shape {\n geometry PointSet {\n coord Coordinate {\n point [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit)
    {
        PointT thisPoint = pc.points[pit];
        if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
        fprintf(fout,"%.5lf %.5lf %.5lf\n", thisPoint.x, thisPoint.y, thisPoint.z);
    }

    fprintf(fout,"]\n}\n color Color {\n color [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit)
    {
        PointT thisPoint = pc.points[pit];
        if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
        fprintf(fout,"%.2f,%.2f,%.2f\n",col(0),col(1),col(2));
    }
    fprintf(fout,"]\n }\n }\n }\n");

}

template< typename PointT>
void writeToVRMLIntensity(const char* fname, pcl::PointCloud<PointT> &pc, Eigen::Vector3d col)
{
    FILE *out = fopen(fname,"w");
    fprintf(out,"#VRML V2.0 utf8\n");
    writeToVRMLIntensity<PointT>(out,pc,col);
    fclose(out);
}

template< typename PointT>
void writeToVRMLIntensity(FILE* fout, pcl::PointCloud<PointT> &pc,
                          Eigen::Vector3d col)
{
    fprintf(fout,"Shape {\n geometry PointSet {\n coord Coordinate {\n point [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit)
    {
        PointT thisPoint = pc.points[pit];
        if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
        fprintf(fout,"%.5lf %.5lf %.5lf\n", thisPoint.x, thisPoint.y, thisPoint.z);
    }

    fprintf(fout,"]\n}\n color Color {\n color [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit)
    {
        PointT thisPoint = pc.points[pit];
        if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
        if(col == Eigen::Vector3d(1,1,1))
        {
            fprintf(fout,"%.5f,%.5f,%.5f\n",thisPoint.intensity, thisPoint.intensity, thisPoint.intensity);
        }
        else
        {
            fprintf(fout,"%.2f,%.2f,%.2f\n",col(0),col(1),col(2));
        }
    }
    fprintf(fout,"]\n }\n }\n }\n");

}

template< typename PointT>
void writeToVRMLColor(const char* fname, pcl::PointCloud<PointT> &pc)
{
    FILE *out = fopen(fname,"w");
    fprintf(out,"#VRML V2.0 utf8\n");
    writeToVRMLColor<PointT>(out,pc);
    fclose(out);
}

template< typename PointT>
void writeToVRMLColor(FILE* fout, pcl::PointCloud<PointT> &pc)
{
    fprintf(fout,"Shape {\n geometry PointSet {\n coord Coordinate {\n point [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit)
    {
        PointT thisPoint = pc.points[pit];
        if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
        fprintf(fout,"%.5lf %.5lf %.5lf\n", thisPoint.x, thisPoint.y, thisPoint.z);
    }

    fprintf(fout,"]\n}\n color Color {\n color [\n");
    for(unsigned int pit=0; pit<pc.points.size(); ++pit)
    {
        PointT thisPoint = pc.points[pit];
        if(std::isnan(thisPoint.x) || std::isnan(thisPoint.y) || std::isnan(thisPoint.z)) continue;
        uint32_t rgb = *reinterpret_cast<int*>(&thisPoint.rgb);
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)       & 0x0000ff;
        fprintf(fout,"%.5f,%.5f,%.5f\n",(double) r/255., (double) g/255., (double) b/255.);
    }
    fprintf(fout,"]\n }\n }\n }\n");

}

template< typename PointT>
pcl::PointCloud<PointT> transformPointCloud(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tr, const pcl::PointCloud<PointT> &pc)
{
    Eigen::Transform<float,3,Eigen::Affine,Eigen::ColMajor> T = Tr.cast<float>();
    pcl::PointCloud<PointT> cloud;
    for(unsigned int pit=0; pit<pc.points.size(); ++pit)
    {
        PointT thisPoint = pc.points[pit];
        Eigen::Map<Eigen::Vector3f> pt((float*)&thisPoint,3);
        pt = T*pt;
        cloud.points.push_back(thisPoint);
    }
    cloud.width = pc.width;
    cloud.height = pc.height;
    return cloud;
}

template< typename PointT>
void transformPointCloudInPlace(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &Tr, pcl::PointCloud<PointT> &pc)
{
    Eigen::Transform<float,3,Eigen::Affine,Eigen::ColMajor> T = Tr.cast<float>();
    for(unsigned int pit=0; pit<pc.points.size(); ++pit)
    {
        Eigen::Map<Eigen::Vector3f> pt((float*)&pc.points[pit],3);
        pt = T*pt;
    }
}

template< typename PointT>
double geomDist(PointT p1, PointT p2)
{
    Eigen::Vector3d v;
    v << p1.x-p2.x, p1.y-p2.y, p1.z-p2.z;
    return v.norm();
}

}
