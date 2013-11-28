#include <cstring>
#include <cstdio>

#define JFFERR(x) std::cerr << x << std::endl; return -1;

namespace lslgeneric
{

template <typename PointT>
LazyGrid<PointT>::LazyGrid(double cellSize) //:mp(new pcl::PointCloud<PointT>())
{
    initialized = false;
    centerIsSet = false;
    sizeIsSet = false;
    cellSizeX = cellSizeY = cellSizeZ = cellSize;

}

template <typename PointT>
LazyGrid<PointT>::LazyGrid(LazyGrid<PointT> *prot)
{

    sizeXmeters = prot->sizeXmeters;
    sizeYmeters = prot->sizeYmeters;
    sizeZmeters = prot->sizeZmeters;

    cellSizeX = prot->cellSizeX;
    cellSizeY = prot->cellSizeY;
    cellSizeZ = prot->cellSizeZ;

    sizeX = abs(ceil(sizeXmeters/cellSizeX));
    sizeY = abs(ceil(sizeYmeters/cellSizeY));
    sizeZ = abs(ceil(sizeZmeters/cellSizeZ));

    centerX = prot->centerX;
    centerY = prot->centerY;
    centerZ = prot->centerZ;

    protoType = prot->protoType->clone();
    initialize();

}

template <typename PointT>
LazyGrid<PointT>::LazyGrid(double _sizeXmeters, double _sizeYmeters, double _sizeZmeters,
                           double _cellSizeX, double _cellSizeY, double _cellSizeZ,
                           double _centerX, double _centerY, double _centerZ,
                           Cell<PointT> *cellPrototype ) //:mp(new pcl::PointCloud<PointT>())
{

    sizeXmeters = _sizeXmeters;
    sizeYmeters = _sizeYmeters;
    sizeZmeters = _sizeZmeters;

    cellSizeX = _cellSizeX;
    cellSizeY = _cellSizeY;
    cellSizeZ = _cellSizeZ;

    sizeX = abs(ceil(sizeXmeters/cellSizeX));
    sizeY = abs(ceil(sizeYmeters/cellSizeY));
    sizeZ = abs(ceil(sizeZmeters/cellSizeZ));

    centerX = _centerX;
    centerY = _centerY;
    centerZ = _centerZ;

    protoType = cellPrototype->clone();
    initialize();


}

template <typename PointT>
void LazyGrid<PointT>::setCenter(const double &cx, const double &cy, const double &cz)
{
    centerX = cx;
    centerY = cy;
    centerZ = cz;

    centerIsSet = true;
    if(sizeIsSet)
    {
        initialize();
    }
}

template <typename PointT>
void LazyGrid<PointT>::setSize(const double &sx, const double &sy, const double &sz)
{

    sizeXmeters = sx;
    sizeYmeters = sy;
    sizeZmeters = sz;

    sizeX = abs(ceil(sizeXmeters/cellSizeX));
    sizeY = abs(ceil(sizeYmeters/cellSizeY));
    sizeZ = abs(ceil(sizeZmeters/cellSizeZ));

    sizeIsSet = true;
    if(centerIsSet)
    {
        initialize();
    }
}

template <typename PointT>
void LazyGrid<PointT>::initializeAll()
{
    if(!initialized)
    {
        this->initialize();
    }
    PointT centerCell;
    for(int i=0; i<sizeX; i++)
    {
        for(int j=0; j<sizeY; j++)
        {
            for(int k=0; k<sizeZ; k++)
            {
                dataArray[i][j][k] = new NDTCell<PointT>();

                //dataArray[i][j][k] = protoType->clone(); ///FIXME WHO WHAT WHERE AND WHY??
                dataArray[i][j][k]->setDimensions(cellSizeX,cellSizeY,cellSizeZ);

                int idcX, idcY, idcZ;
                PointT center;
                center.x = centerX;
                center.y = centerY;
                center.z = centerZ;
                this->getIndexForPoint(center, idcX,idcY,idcZ);
                centerCell.x = centerX + (i-idcX)*cellSizeX;
                centerCell.y = centerY + (j-idcY)*cellSizeY;
                centerCell.z = centerZ + (k-idcZ)*cellSizeZ;
                dataArray[i][j][k]->setCenter(centerCell);
                activeCells.push_back(dataArray[i][j][k]);
            }
        }
    }
}

template <typename PointT>
void LazyGrid<PointT>::initialize()
{

    dataArray = new Cell<PointT>***[sizeX];
    //linkedCells = new bool**[sizeX];
    for(int i=0; i<sizeX; i++)
    {
        dataArray[i] = new Cell<PointT>**[sizeY];
        //linkedCells[i] = new bool*[sizeY];
        for(int j=0; j<sizeY; j++)
        {
            dataArray[i][j] = new Cell<PointT>*[sizeZ];
            //linkedCells[i][j] = new bool[sizeZ];
            //set all cells to NULL
            memset(dataArray[i][j],0,sizeZ*sizeof(Cell<PointT>*));
            //memset(linkedCells[i][j],0,sizeZ*sizeof(bool));
        }
    }
    initialized = true;
}

template <typename PointT>
LazyGrid<PointT>::~LazyGrid()
{
    if(initialized)
    {
				//fprintf(stderr,"LAZY GRID DESTRUCTION -- ");
				int cnt = 0;
        //go through all cells and delete the non-NULL ones
        for(unsigned int i=0; i<activeCells.size(); ++i)
        {
            if(activeCells[i])
            {
                delete activeCells[i];
								cnt++;
            }
        }
        for(int i=0; i<sizeX; i++)
        {
            for(int j=0; j<sizeY; j++)
            {
                delete[] dataArray[i][j];
                //delete[] linkedCells[i][j];
            }
            delete[] dataArray[i];
            //delete[] linkedCells[i];
        }
        delete[] dataArray;
        //delete[] linkedCells;
        if(protoType!=NULL)
        {
            delete protoType;
        }
        //fprintf(stderr,"Deleted %d cells and array of (%d x %d)!!!\n",cnt, sizeX, sizeY);
    }
}

template <typename PointT>
Cell<PointT>* LazyGrid<PointT>::getCellForPoint(const PointT &point)
{

    int indX,indY,indZ;
    this->getIndexForPoint(point,indX,indY,indZ);

    if(indX >= sizeX || indY >= sizeY || indZ >= sizeZ || indX<0 || indY<0 || indZ<0) return NULL;
    if(!initialized) return NULL;
    if(dataArray == NULL) return NULL;
    if(dataArray[indX] == NULL) return NULL;
    if(dataArray[indX][indY] == NULL) return NULL;

    //    cout<<"LZ: "<<indX<<" "<<indY<<" "<<indZ<<endl;
    return dataArray[indX][indY][indZ];
}

template <typename PointT>
Cell<PointT>* LazyGrid<PointT>::addPoint(const PointT &point_c)
{

    PointT point = point_c;
    if(std::isnan(point.x) ||std::isnan(point.y) ||std::isnan(point.z))
    {
        return NULL;
    }
    /*    int *idX,*idY,*idZ;
      idX = new int[4];
      idY = new int[4];
      idZ = new int[4];

      this->getIndexArrayForPoint(point,idX,idY,idZ);
     */
    // for(int i=0; i<4; i++) {
    int indX,indY,indZ;
    //indX = idX[i]; indZ = idZ[i]; indY = idY[i];
    this->getIndexForPoint(point,indX,indY,indZ);
    PointT centerCell;

    if(indX >= sizeX || indY >= sizeY || indZ >= sizeZ || indX<0 || indY<0 || indZ<0)
    {
        //continue;
        return NULL;
    }

    if(!initialized) return NULL;
    if(dataArray == NULL) return NULL;
    if(dataArray[indX] == NULL) return NULL;
    if(dataArray[indX][indY] == NULL) return NULL;

    if(dataArray[indX][indY][indZ]==NULL)
    {
        //initialize cell
        dataArray[indX][indY][indZ] = protoType->clone();
        dataArray[indX][indY][indZ]->setDimensions(cellSizeX,cellSizeY,cellSizeZ);

        int idcX, idcY, idcZ;
        PointT center;
        center.x = centerX;
        center.y = centerY;
        center.z = centerZ;
        this->getIndexForPoint(center, idcX,idcY,idcZ);
        centerCell.x = centerX + (indX-idcX)*cellSizeX;
        centerCell.y = centerY + (indY-idcY)*cellSizeY;
        centerCell.z = centerZ + (indZ-idcZ)*cellSizeZ;
        dataArray[indX][indY][indZ]->setCenter(centerCell);
        /*
           cout<<"center: "<<centerX<<" "<<centerY<<" "<<centerZ<<endl;
           cout<<"size  : "<<sizeX<<" "<<sizeY<<" "<<sizeZ<<endl;
           cout<<"p  : "<<point.x<<" "<<point.y<<" "<<point.z<<endl;
           cout<<"c  : "<<centerCell.x<<" "<<centerCell.y<<" "<<centerCell.z<<endl;
           cout<<"id : "<<indX<<" "<<indY<<" "<<indZ<<endl;
           cout<<"cs : "<<cellSizeX<<" "<<cellSizeY<<" "<<cellSizeZ<<endl;
         */
        activeCells.push_back(dataArray[indX][indY][indZ]);
    }
    dataArray[indX][indY][indZ]->addPoint(point);
    return dataArray[indX][indY][indZ];
    //}
    /*
       delete []idX;
       delete []idY;
       delete []idZ; */
}

template <typename PointT>
typename SpatialIndex<PointT>::CellVectorItr LazyGrid<PointT>::begin()
{
    return activeCells.begin();
}

template <typename PointT>
typename SpatialIndex<PointT>::CellVectorItr LazyGrid<PointT>::end()
{
    return activeCells.end();
}

template <typename PointT>
int LazyGrid<PointT>::size()
{
    return activeCells.size();
}

template <typename PointT>
SpatialIndex<PointT>* LazyGrid<PointT>::clone() const
{
    return new LazyGrid<PointT>(cellSizeX);
}

template <typename PointT>
SpatialIndex<PointT>* LazyGrid<PointT>::copy() const
{
    LazyGrid<PointT> *ret = new LazyGrid<PointT>(cellSizeX);
    typename std::vector<Cell<PointT>*>::const_iterator it;
    it = activeCells.begin();
    while(it!=activeCells.end())
    {
        NDTCell<PointT>* r = dynamic_cast<NDTCell<PointT>*> (*it);
        if(r == NULL) continue;
        for(unsigned int i=0; i<r->points_.size(); i++)
        {
            ret->addPoint(r->points_[i]);
        }
        it++;
    }
    return ret;
}

template <typename PointT>
void LazyGrid<PointT>::getNeighbors(const PointT &point, const double &radius, std::vector<Cell<PointT>*> &cells)
{

    int indX,indY,indZ;
    this->getIndexForPoint(point, indX,indY,indZ);
    if(indX >= sizeX || indY >= sizeY || indZ >= sizeZ)
    {
        cells.clear();
        return;
    }

    for(int x = indX - radius/cellSizeX; x<indX+radius/cellSizeX; x++)
    {
        if(x < 0 || x >= sizeX) continue;
        for(int y = indY - radius/cellSizeY; y<indY+radius/cellSizeY; y++)
        {
            if(y < 0 || y >= sizeY) continue;
            for(int z = indZ - radius/cellSizeZ; z<indZ+radius/cellSizeZ; z++)
            {
                if(z < 0 || z >= sizeZ) continue;
                if(dataArray[x][y][z]==NULL) continue;
                cells.push_back(dataArray[x][y][z]);
            }
        }
    }

}

template <typename PointT>
inline void LazyGrid<PointT>::getIndexForPoint(const PointT& point, int &indX, int &indY, int &indZ)
{
    indX = floor((point.x - centerX)/cellSizeX+0.5) + sizeX/2.0;
    indY = floor((point.y - centerY)/cellSizeY+0.5) + sizeY/2.0;
    indZ = floor((point.z - centerZ)/cellSizeZ+0.5) + sizeZ/2.0;
}

/*
   void LazyGrid<PointT>::getIndexArrayForPoint(const PointT& point, int* &indX, int* &indY, int *&indZ) {
   indX[0] = floor((point.x - centerX)/cellSizeX+0.5) + sizeX/2;
   indY[0] = floor((point.y - centerY)/cellSizeY+0.5) + sizeY/2;
   indZ[0] = floor((point.z - centerZ)/cellSizeZ+0.5) + sizeZ/2;

//get center of the cell
int idcX, idcY, idcZ;
PointT center;
center.x = centerX;
center.y = centerY;
center.z = centerZ;
this->getIndexForPoint(center, idcX,idcY,idcZ);
double centerCellx = centerX + (indX[0]-idcX)*cellSizeX;
double centerCelly = centerY + (indY[0]-idcY)*cellSizeY;
double centerCellz = centerZ + (indZ[0]-idcZ)*cellSizeZ;
int pX,pY,pZ;
pX = point.x - centerCellx > 0? 1 : -1;
pY = point.y - centerCelly > 0? 1 : -1;
pZ = point.z - centerCellz > 0? 1 : -1;
indX[1] = indX[0]+pX;
indY[1] = indY[0];
indZ[1] = indZ[0];
indX[2] = indX[0];
indY[2] = indY[0]+pY;
indZ[2] = indZ[0];
indX[3] = indX[0];
indY[3] = indY[0];
indZ[3] = indZ[0]+pZ;
}
 */
template <typename PointT>
std::vector<NDTCell<PointT>*> LazyGrid<PointT>::getClosestCells(const PointT &pt)
{
    int indXn,indYn,indZn;
    int indX,indY,indZ;
    this->getIndexForPoint(pt,indX,indY,indZ);
    NDTCell<PointT> *ret = NULL;
    std::vector<NDTCell<PointT>*> cells;

    int i =2; //how many cells on each side
    //the strange thing for the indeces is for convenience of writing
    //basicly, we go through 2* the number of cells and use parity to
    //decide if we subtract or add. should work nicely
    for(int x=1; x<2*i+2; x++)
    {
        indXn = (x%2 == 0) ? indX+x/2 : indX-x/2;
        for(int y=1; y<2*i+2; y++)
        {
            indYn = (y%2 == 0) ? indY+y/2 : indY-y/2;
            for(int z=1; z<2*i+2; z++)
            {
                indZn = (z%2 == 0) ? indZ+z/2 : indZ-z/2;
                if(checkCellforNDT(indXn,indYn,indZn,true))
                {
                    ret = dynamic_cast<NDTCell<PointT>*> (dataArray[indXn][indYn][indZn]);
                    cells.push_back(ret);
                }
            }
        }
    }
    return cells;
}

template <typename PointT>
std::vector<NDTCell<PointT>*> LazyGrid<PointT>::getClosestNDTCells(const PointT &point, int &n_neigh, bool checkForGaussian)
{

#if 0
    std::vector<int> id;
    std::vector<float> dist;
    int NCELLS = 4;
    id.reserve(NCELLS);
    dist.reserve(NCELLS);
    const PointT pt(point);
    std::vector<NDTCell<PointT>*> cells;

    //if(meansTree.input_.get()==NULL) { return cells;} ///FIXME::: This is needed
    //if(meansTree.input_->size() < 1) { return cells;} ///FIXME::: This is needed
    if(!meansTree.nearestKSearch(pt,NCELLS,id,dist))
    {
        return cells;
    }
    NDTCell<PointT> *ret = NULL;

    for(int i=0; i<NCELLS; i++)
    {
        //if(dist[i]>2*radius) continue;
        PointT close = mp->points[id[i]];
        int indX,indY,indZ;
        this->getIndexForPoint(close, indX,indY,indZ);
        if(checkCellforNDT(indX,indY,indZ))
        {
            ret = dynamic_cast<NDTCell<PointT>*> (dataArray[indX][indY][indZ]);
            cells.push_back(ret);
        }
    }
    return cells;
#endif
    int indXn,indYn,indZn;
    int indX,indY,indZ;
    this->getIndexForPoint(point,indX,indY,indZ);
    NDTCell<PointT> *ret = NULL;
    std::vector<NDTCell<PointT>*> cells;

    int i = n_neigh; //how many cells on each side

    //for(int i=1; i<= maxNumberOfCells; i++) {
    //the strange thing for the indeces is for convenience of writing
    //basicly, we go through 2* the number of cells and use parity to
    //decide if we subtract or add. should work nicely
    for(int x=1; x<2*i+2; x++)
    {
        indXn = (x%2 == 0) ? indX+x/2 : indX-x/2;
        for(int y=1; y<2*i+2; y++)
        {
            indYn = (y%2 == 0) ? indY+y/2 : indY-y/2;
            for(int z=1; z<2*i+2; z++)
            {
                indZn = (z%2 == 0) ? indZ+z/2 : indZ-z/2;
                if(checkCellforNDT(indXn,indYn,indZn,checkForGaussian))
                {
                    ret = dynamic_cast<NDTCell<PointT>*> (dataArray[indXn][indYn][indZn]);
                    cells.push_back(ret);
                }
            }
        }
    }

    return cells;
}

/*
template <typename PointT>
void LazyGrid<PointT>::initKDTree()
{

    NDTCell<PointT>* ndcell = NULL;
    PointT curr;
    Eigen::Vector3d m;
    pcl::PointCloud<PointT> mc;

    for(unsigned int i=0; i<activeCells.size(); i++)
    {
        ndcell = dynamic_cast<NDTCell<PointT>*> (activeCells[i]);
        if(ndcell == NULL) continue;
        if(!ndcell->hasGaussian_) continue;
        m = ndcell->getMean();
        curr.x = m(0);
        curr.y = m(1);
        curr.z = m(2);
        mc.push_back(curr);
    }

    if(mc.points.size() > 0)
    {
        *mp = mc;
        meansTree.setInputCloud(mp);
    }

}*/

template <typename PointT>
NDTCell<PointT>* LazyGrid<PointT>::getClosestNDTCell(const PointT &point, bool checkForGaussian)
{

#if 0
    //KD tree based old stuff
    std::vector<int> id;
    std::vector<float> dist;
    id.reserve(1);
    dist.reserve(1);
    const PointT pt(point);
    // if(meansTree.input_.get()==NULL) {return NULL;} ///FIXME::: This is needed
    // if(meansTree.input_->size() < 1) {return NULL;} ///FIXME::: This is needed
    if(!meansTree.nearestKSearch(pt,1,id,dist))
    {
        return NULL;
    }
    PointT close = mp->points[id[0]];

    NDTCell<PointT> *ret = NULL;
    int indX,indY,indZ;
    this->getIndexForPoint(close, indX,indY,indZ);
    if(checkCellforNDT(indX,indY,indZ))
    {
        ret = dynamic_cast<NDTCell<PointT>*> (dataArray[indX][indY][indZ]);
    }
    return ret;
#endif

    int indXn,indYn,indZn;
    int indX,indY,indZ;
    this->getIndexForPoint(point,indX,indY,indZ);
    NDTCell<PointT> *ret = NULL;
    std::vector<NDTCell<PointT>*> cells;

    if(!checkForGaussian)
    {
        //just give me whatever is in this cell
        if(checkCellforNDT(indX,indY,indZ,checkForGaussian))
        {
            ret = dynamic_cast<NDTCell<PointT>*> (dataArray[indX][indY][indZ]);
        }
        return ret;
    }

    int i =1; //how many cells on each side

    //for(int i=1; i<= maxNumberOfCells; i++) {
    //the strange thing for the indeces is for convenience of writing
    //basicly, we go through 2* the number of cells and use parity to
    //decide if we subtract or add. should work nicely
    for(int x=1; x<2*i+2; x++)
    {
        indXn = (x%2 == 0) ? indX+x/2 : indX-x/2;
        for(int y=1; y<2*i+2; y++)
        {
            indYn = (y%2 == 0) ? indY+y/2 : indY-y/2;
            for(int z=1; z<2*i+2; z++)
            {
                indZn = (z%2 == 0) ? indZ+z/2 : indZ-z/2;
                if(checkCellforNDT(indXn,indYn,indZn))
                {
                    ret = dynamic_cast<NDTCell<PointT>*> (dataArray[indXn][indYn][indZn]);
                    cells.push_back(ret);
                }
            }
        }
    }

    double minDist = INT_MAX;
    Eigen::Vector3d tmean;
    PointT pt = point;
    for(unsigned int i=0; i<cells.size(); i++)
    {
        tmean = cells[i]->getMean();
        tmean(0) -= pt.x;
        tmean(1) -= pt.y;
        tmean(2) -= pt.z;
        double d = tmean.norm();
        if(d<minDist)
        {
            minDist = d;
            ret = cells[i];
        }
    }
    cells.clear();
    return ret;
}

template <typename PointT>
bool LazyGrid<PointT>::checkCellforNDT(int indX, int indY, int indZ, bool checkForGaussian)
{

    if(indX < sizeX && indY < sizeY && indZ < sizeZ &&
            indX >=0 && indY >=0 && indZ >=0)
    {
        if(dataArray[indX][indY][indZ]!=NULL)
        {
            NDTCell<PointT>* ret = dynamic_cast<NDTCell<PointT>*> (dataArray[indX][indY][indZ]);
            if(ret!=NULL)
            {
                if(ret->hasGaussian_ || (!checkForGaussian))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

template <typename PointT>
void LazyGrid<PointT>::setCellType(Cell<PointT> *type)
{
    if(type!=NULL)
    {
        protoType = type->clone();
    }
}


#if 0
template <typename PointT>
bool LazyGrid<PointT>::getLinkedAt(int indX, int indY, int indZ)
{
    if(indX < sizeX && indY < sizeY && indZ < sizeZ &&
            indX >=0 && indY >=0 && indZ >=0)
    {
        return linkedCells[indX][indY][indZ];
    }
    return false;

}
#endif

template <typename PointT>
void LazyGrid<PointT>::getCellSize(double &cx, double &cy, double &cz)
{
    cx = cellSizeX;
    cy = cellSizeY;
    cz = cellSizeZ;
}

template <typename PointT>
void LazyGrid<PointT>::getCenter(double &cx, double &cy, double &cz)
{
    cx = centerX;
    cy = centerY;
    cz = centerZ;

}

template <typename PointT>
void LazyGrid<PointT>::getGridSize(int &cx, int &cy, int &cz)
{
    cx = sizeX;
    cy = sizeY;
    cz = sizeZ;
}

template <typename PointT>
void LazyGrid<PointT>::getGridSizeInMeters(double &cx, double &cy, double &cz)
{
    cx = sizeXmeters;
    cy = sizeYmeters;
    cz = sizeZmeters;
}

template <typename PointT>
int LazyGrid<PointT>::loadFromJFF(FILE * jffin)
{
    double lazyGridData[9]; // = { sizeXmeters, sizeYmeters, sizeZmeters,
    //     cellSizeX,   cellSizeY,   cellSizeZ,
    //     centerX,     centerY,     centerZ };
    NDTCell<PointT> prototype_;
    if(fread(&lazyGridData, sizeof(double), 9, jffin) <= 0)
    {
        JFFERR("reading lazyGridData failed");
    }
    if(fread(&prototype_, sizeof(Cell<PointT>), 1, jffin) <= 0)
    {
        JFFERR("reading prototype_ failed");
    }

    // just in case someone was messing around with the new NDTMap
    centerIsSet = false;
    sizeIsSet = false;

    protoType = prototype_.clone();

    this->setSize(lazyGridData[0], lazyGridData[1], lazyGridData[2]);

    cellSizeX = lazyGridData[3];
    cellSizeY = lazyGridData[4];
    cellSizeZ = lazyGridData[5];

    this->setCenter(lazyGridData[6], lazyGridData[7], lazyGridData[8]);

    this->initializeAll();
    int indX, indY, indZ;
    float r,g,b;
    double xs,ys,zs;
    PointT centerCell;
    float occ;
    unsigned int N;

    // load all cells
    while (1)
    {
        if(prototype_.loadFromJFF(jffin) < 0)
        {
            if(feof(jffin))
            {
                break;
            }
            else
            {
                JFFERR("loading cell failed");
            }
        }

        if(!feof(jffin))
        {
            // std::cout << prototype_.getOccupancy() << std::endl; /* for debugging */
        }
        else
        {
            break;
        }
	centerCell = prototype_.getCenter();
        this->getIndexForPoint(centerCell, indX, indY, indZ);
        if(indX < 0 || indX >= sizeX) continue; 
        if(indY < 0 || indY >= sizeY) continue; 
        if(indZ < 0 || indZ >= sizeZ) continue; 
	if(!initialized) return -1;
        if(dataArray == NULL) return -1;
        if(dataArray[indX] == NULL) return -1;
        if(dataArray[indX][indY] == NULL) return -1;

        if(dataArray[indX][indY][indZ] != NULL)
        {
            NDTCell<PointT>* ret = dynamic_cast<NDTCell<PointT>*>(dataArray[indX][indY][indZ]);
	    prototype_.getRGB(r,g,b);
	    prototype_.getDimensions(xs,ys,zs);

	    ret->setDimensions(xs,ys,zs);
	    ret->setCenter(centerCell);
	    ret->setMean(prototype_.getMean());
	    ret->setCov(prototype_.getCov());
	    ret->setRGB(r,g,b);
	    ret->setOccupancy(prototype_.getOccupancy());
	    ret->setEmptyval(prototype_.getEmptyval());
	    ret->setEventData(prototype_.getEventData());
	    ret->setN(prototype_.getN());
	    ret->isEmpty = prototype_.isEmpty;
	    ret->hasGaussian_ = prototype_.hasGaussian_;
	    ret->consistency_score = prototype_.consistency_score;

	} else {
	    //initialize cell
	    std::cerr<<"NEW CELL\n";
	    dataArray[indX][indY][indZ] = prototype_.copy();
	    activeCells.push_back(dataArray[indX][indY][indZ]);
	}
    }

    return 0;
}

template <typename PointT>
inline
bool LazyGrid<PointT>::traceLine(const Eigen::Vector3d &origin, const PointT &endpoint,const Eigen::Vector3d &diff_ ,
																	const double& maxz, std::vector<NDTCell<PointT>*> &cells)
{
	if(endpoint.z>maxz)
	{
		return false;
	}
	
	double min1 = std::min(cellSizeX,cellSizeY);
	double min2 = std::min(cellSizeZ,cellSizeY);
	double resolution = std::min(min1,min2); ///Select the smallest resolution
	
	if(resolution<0.01)
	{
		fprintf(stderr,"Resolution very very small (%lf) :( \n",resolution);
		return false;
	}
	double l = diff_.norm();
	int N = l / (resolution);
	//if(N <= 0)
	//{
	//	//fprintf(stderr,"N=%d (r=%lf l=%lf) :( ",N,resolution,l);
	//	return false;
	//}
	
	NDTCell<PointT>* ptCell = NULL;    
	PointT pt;
	PointT po;
	po.x = origin(0); po.y = origin(1); po.z = origin(2);
	Eigen::Vector3d diff = diff_/(float)N;
	
	int idxo=0, idyo=0,idzo=0;
	for(int i=0; i<N-2; i++)
	{
		pt.x = origin(0) + ((float)(i+1)) *diff(0);
		pt.y = origin(1) + ((float)(i+1)) *diff(1);
		pt.z = origin(2) + ((float)(i+1)) *diff(2);
		int idx,idy,idz;
		idx = floor((pt.x - centerX)/cellSizeX+0.5) + sizeX/2.0;
		idy = floor((pt.y - centerY)/cellSizeY+0.5) + sizeY/2.0;
		idz = floor((pt.z - centerZ)/cellSizeZ+0.5) + sizeZ/2.0;
		///We only want to check every cell once, so
		///increase the index if we are still in the same cell
		if(idx == idxo && idy==idyo && idz ==idzo)
		{
			continue;
		}
		else
		{
			idxo = idx;
			idyo = idy;
			idzo = idz;
		}
		
		if(idx < sizeX && idy < sizeY && idz < sizeZ && idx >=0 && idy >=0 && idz >=0){
			ptCell = dynamic_cast<NDTCell<PointT> *> (dataArray[idx][idy][idz]);
			if(ptCell !=NULL) {
				cells.push_back(ptCell);
			} else {
				this->addPoint(pt); ///Add fake point to initialize!
			}
		}
	}
	return true;
	
}

template <typename PointT>
inline
bool LazyGrid<PointT>::traceLineWithEndpoint(const Eigen::Vector3d &origin, const PointT &endpoint,const Eigen::Vector3d &diff_ ,const double& maxz, std::vector<NDTCell<PointT>*> &cells, Eigen::Vector3d &final_point)
{
    if(endpoint.z>maxz)
    {
	return false;
    }

    double min1 = std::min(cellSizeX,cellSizeY);
    double min2 = std::min(cellSizeZ,cellSizeY);
    double resolution = std::min(min1,min2); ///Select the smallest resolution

    if(resolution<0.01)
    {
	fprintf(stderr,"Resolution very very small (%lf) :( \n",resolution);
	return false;
    }
    double l = diff_.norm();
    int N = l / (resolution);
    NDTCell<PointT>* ptCell = NULL;    
    PointT pt;
    PointT po;
    po.x = origin(0); po.y = origin(1); po.z = origin(2);
    if(N == 0)
    {
	//fprintf(stderr,"N=%d (r=%lf l=%lf) :( ",N,resolution,l);
	//return false;
	this->getNDTCellAt(po,ptCell);
	if(ptCell!=NULL) {
	    cells.push_back(ptCell);
	}
	return true;
    }

    Eigen::Vector3d diff = diff_/(float)N;

    int idxo=0, idyo=0,idzo=0;
    bool complete = true;
    for(int i=0; i<N-2; i++)
    {
	pt.x = origin(0) + ((float)(i+1)) *diff(0);
	pt.y = origin(1) + ((float)(i+1)) *diff(1);
	pt.z = origin(2) + ((float)(i+1)) *diff(2);
	int idx,idy,idz;
	idx = floor((pt.x - centerX)/cellSizeX+0.5) + sizeX/2.0;
	idy = floor((pt.y - centerY)/cellSizeY+0.5) + sizeY/2.0;
	idz = floor((pt.z - centerZ)/cellSizeZ+0.5) + sizeZ/2.0;
	///We only want to check every cell once, so
	///increase the index if we are still in the same cell
	if(idx == idxo && idy==idyo && idz ==idzo)
	{
	    continue;
	}
	else
	{
	    idxo = idx;
	    idyo = idy;
	    idzo = idz;
	}

	if(idx < sizeX && idy < sizeY && idz < sizeZ && idx >=0 && idy >=0 && idz >=0){
	    ptCell = dynamic_cast<NDTCell<PointT> *> (dataArray[idx][idy][idz]);
	    if(ptCell !=NULL) {
		cells.push_back(ptCell);
	    } else {
		this->addPoint(pt); ///Add fake point to initialize!
	    }
	} else {
	    //out of the map, we won't be coming back any time soon
	    complete = false;
	    final_point = origin+(float(i))*diff;
	    break;
	}
    }
    if(complete) final_point = origin+diff_;
    return true;

}

} //end namespace
