#include <climits>
namespace lslgeneric
{
template <typename PointT>
CellVector<PointT>::CellVector():mp(new pcl::PointCloud<PointT>())
{
    protoType= new NDTCell<PointT>();
    treeUpdated = false;
}

template <typename PointT>
CellVector<PointT>::CellVector(Cell<PointT>* cellPrototype):mp(new pcl::PointCloud<PointT>())
{
    protoType = cellPrototype->clone();
    treeUpdated = false;
}

template <typename PointT>
CellVector<PointT>::~CellVector()
{
    //go through all cells and delete the non-NULL ones
    for(unsigned int i=0; i<activeCells.size(); ++i)
    {
        if(activeCells[i]!=NULL)
        {
            delete activeCells[i];
        }
    }
}

template <typename PointT>
Cell<PointT>* CellVector<PointT>::getCellForPoint(const PointT &point)
{
    Cell<PointT>* ret = NULL;
    if (treeUpdated)
    {
        std::vector<int> id;
        std::vector<float> dist;
        int NCELLS = 1;
        id.reserve(NCELLS);
        dist.reserve(NCELLS);
        const PointT pt(point);
        if(!meansTree.nearestKSearch(pt,NCELLS,id,dist)) return ret;

        Cell<PointT>* tmp = activeCells[id[0]];
        ret = static_cast<NDTCell<PointT>*>(tmp);
    }
    else
    {
        float min_dist = std::numeric_limits<float>::max( );
        typename SpatialIndex<PointT>::CellVectorItr it = this->begin();
        while(it!=this->end())
        {
            float tmp=pcl::squaredEuclideanDistance((*it)->getCenter(), point);
            if (tmp < min_dist)
            {
                min_dist = tmp;
                ret = (*it);
            }
            it++;
        }
    }
    return ret;
}

template <typename PointT>
Cell<PointT>* CellVector<PointT>::addPoint(const PointT &point)
{
    return NULL;
    // Do nothing...
}

template <typename PointT>
void CellVector<PointT>::addCellPoints(pcl::PointCloud<PointT> pc, const std::vector<size_t> &indices)
{
    activeCells.push_back(protoType->clone());
    for (size_t i = 0; i < indices.size(); i++)
        (activeCells.back())->addPoint(pc[indices[i]]); // Add the point to the cell.
    treeUpdated = false;
}


template <typename PointT>
void CellVector<PointT>::addCell(Cell<PointT>* cell)
{
    activeCells.push_back(cell);
}

template <typename PointT>
void CellVector<PointT>::addNDTCell(NDTCell<PointT>* cell)
{
    this->addCell(static_cast<Cell<PointT>*>(cell));
}

template <typename PointT>
typename SpatialIndex<PointT>::CellVectorItr CellVector<PointT>::begin()
{
    //cout<<"active cells "<<activeCells.size()<<endl;
    return activeCells.begin();
}

template <typename PointT>
typename SpatialIndex<PointT>::CellVectorItr CellVector<PointT>::end()
{
    return activeCells.end();
}

template <typename PointT>
int CellVector<PointT>::size()
{
    return activeCells.size();
}

template <typename PointT>
SpatialIndex<PointT>* CellVector<PointT>::clone() const
{
    return new CellVector<PointT>();
}

template <typename PointT>
SpatialIndex<PointT>* CellVector<PointT>::copy() const
{
    //std::cout<<"COPY CELL VECTOR\n";
    //assert(false); // This needs to be updated!
    CellVector<PointT> *ret = new CellVector<PointT>();
    for(unsigned int i =0; i< activeCells.size(); i++)
    {
        NDTCell<PointT>* r = dynamic_cast<NDTCell<PointT>*> (activeCells[i]->copy());
        if(r == NULL) continue;
        for(size_t i=0; i<r->points_.size(); i++)
        {
            ret->activeCells.push_back(r->copy());
        }
    }
    return ret;
}

template <typename PointT>
void CellVector<PointT>::getNeighbors(const PointT &point, const double &radius, std::vector<Cell<PointT>*> &cells)
{

    if (treeUpdated)
    {
        std::vector<int> id;
        std::vector<float> dist;
        int NCELLS = 4;
        id.reserve(NCELLS);
        dist.reserve(NCELLS);
        const PointT pt(point);

        if(!meansTree.nearestKSearch(pt,NCELLS,id,dist)) return;

        for(int i=0; i<NCELLS; i++)
        {
            Cell<PointT>* tmp = activeCells[id[i]];
            if (tmp != NULL)
                cells.push_back(tmp);
        }
    }
    else
    {
        float radius_sqr = radius*radius;
        typename SpatialIndex<PointT>::CellVectorItr it = this->begin();
        while(it!=this->end())
        {
            float tmp=pcl::squaredEuclideanDistance((*it)->getCenter(), point);
            if (tmp < radius_sqr)
            {
                cells.push_back(*it);
            }
        }
    }
}

template <typename PointT>
void CellVector<PointT>::initKDTree()
{

    NDTCell<PointT>* ndcell = NULL;
    PointT curr;
    Eigen::Vector3d m;
    pcl::PointCloud<PointT> mc;

    for(size_t i=0; i<activeCells.size(); i++)
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

    //++++++++++++++++++treeUpdated = true;
}

template <typename PointT>
void CellVector<PointT>::setCellType(Cell<PointT> *type)
{
    if(type!=NULL)
    {
        protoType = type->clone();
    }
}

template <typename PointT>
NDTCell<PointT>* CellVector<PointT>::getClosestNDTCell(const PointT &point)
{

    Cell<PointT>* tmp = getCellForPoint(point);
    NDTCell<PointT>* ret = dynamic_cast<NDTCell<PointT>*>(tmp);
    return ret;
}

template <typename PointT>
std::vector<NDTCell<PointT>*> CellVector<PointT>::getClosestNDTCells(const PointT &point, double &radius)
{

    std::vector<NDTCell<PointT>*> ret;
    std::vector<Cell<PointT>*> cells;
    getNeighbors(point, radius, cells);
    for (size_t i = 0; i < cells.size(); i++)
    {
        NDTCell<PointT>* tmp = dynamic_cast<NDTCell<PointT>*>(cells[i]);
        if (tmp != NULL)
        {
            ret.push_back(tmp);
        }
    }
    return ret;
}

template <typename PointT>
NDTCell<PointT>*
CellVector<PointT>::getCellIdx(unsigned int idx) const
{
    if (idx >= activeCells.size())
        return NULL;
    NDTCell<PointT>* tmp = dynamic_cast<NDTCell<PointT>*>(activeCells[idx]);
    if (tmp != NULL)
    {
        return tmp;
    }
    return NULL;
}

template <typename PointT>
void CellVector<PointT>::cleanCellsAboveSize(double size)
{
    //clean cells with variance more then x meters in any direction
    Eigen::Vector3d evals;
    lslgeneric::SpatialIndex<pcl::PointXYZ>::CellVectorItr it = this->begin();
    lslgeneric::SpatialIndex<pcl::PointXYZ>::CellVectorItr it_tmp;
    while(it!=this->end())
    {
        lslgeneric::NDTCell<pcl::PointXYZ> *ndcell = dynamic_cast<lslgeneric::NDTCell<pcl::PointXYZ>* >(*it);
        if(ndcell != NULL)
        {
            if(ndcell->hasGaussian_)
            {
                evals = ndcell->getEvals();
                if(sqrt(evals(2)) < size)
                {
                    it++;
                    continue;
                }
                //std::cout<<"rem cell at "<<ndcell->getMean().transpose()<<" evals are "<<evals.transpose()<<std::endl;
                ndcell->hasGaussian_ = false;
            }
            delete ndcell;
            ndcell = NULL;
        }
        it_tmp = it;
        it_tmp--;
        this->activeCells.erase(it);
        it = it_tmp;
        it++;
    }

}
template <typename PointT>
int CellVector<PointT>::loadFromJFF(FILE * jffin)
{
    NDTCell<PointT> prototype_;
    if(fread(&prototype_, sizeof(Cell<PointT>), 1, jffin) <= 0)
    {
        JFFERR("reading prototype_ failed");
    }
    protoType = prototype_.clone();
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
        //initialize cell
        this->addCell(prototype_.copy());
    }

    this->initKDTree();

    return 0;
}

}
