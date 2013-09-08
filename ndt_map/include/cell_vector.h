/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, AASS Research Center, Orebro University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of AASS Research Center nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef LSL_CELL_VECTOR_HH
#define LSL_CELL_VECTOR_HH

#include <spatial_index.h>
#include <ndt_cell.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace lslgeneric
{

/** \brief A spatial index represented as a grid map
    \details A grid map with delayed allocation of cells.
*/
template <typename PointT>
class CellVector : public SpatialIndex<PointT>
{
public:
    CellVector();
    CellVector(Cell<PointT>* cellPrototype);
    CellVector(const CellVector<PointT>& other)
    {
        //std::cout<<"CREATE COPY CELL VECTOR\n";
        this = other.copy();
    }
    virtual ~CellVector();

    virtual Cell<PointT>* getCellForPoint(const PointT &point);
    virtual Cell<PointT>* addPoint(const PointT &point);
    void addCellPoints(pcl::PointCloud<PointT> pc, const std::vector<size_t> &indices);
    void addCell(Cell<PointT>* cell);
    void addNDTCell(NDTCell<PointT>* cell);

    virtual typename SpatialIndex<PointT>::CellVectorItr begin();
    virtual typename SpatialIndex<PointT>::CellVectorItr end();
    virtual int size();

    ///clone - create an empty object with same type
    virtual SpatialIndex<PointT>* clone() const;
    ///copy - create the same object as a new instance
    virtual SpatialIndex<PointT>* copy() const;

    ///method to return all cells within a certain radius from a point
    virtual void getNeighbors(const PointT &point, const double &radius, std::vector<Cell<PointT>*> &cells);

    ///sets the cell factory type
    virtual void setCellType(Cell<PointT> *type);


    void initKDTree();

    NDTCell<PointT>* getClosestNDTCell(const PointT &pt);
    std::vector<NDTCell<PointT>*> getClosestNDTCells(const PointT &point, double &radius);
    NDTCell<PointT>* getCellIdx(unsigned int idx) const;

    void cleanCellsAboveSize(double size);
    int loadFromJFF(FILE * jffin);
private:
    std::vector<Cell<PointT>*> activeCells;
    Cell<PointT> *protoType;
    pcl::KdTreeFLANN<PointT> meansTree;
    typename pcl::KdTree<PointT>::PointCloudPtr mp;
    bool treeUpdated;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}; //end namespace
#include <impl/cell_vector.hpp>

#endif
