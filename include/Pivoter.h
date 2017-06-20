#ifndef PIVOTER_H
#define PIVOTER_H

#include <memory>

#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>

#include "Front.h"
#include "Edge.h"

class Pivoter
{
protected:
  pcl::KdTreeFLANN<pcl::PointNormal> _kdtree;
  pcl::PointCloud<pcl::PointNormal>::Ptr _cloud;
  std::vector<bool> _is_used;
  Front _front;
  double _radius;

  void prepare(const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
               const double radius);

//  std::pair<pcl::Vertices::Ptr, pcl::PointNormal> findSeed();

  bool findSeed(pcl::Vertices::Ptr &seed, pcl::PointNormal &center,
                bool &isBackBall);

  bool pivot(const Edge &edge, uint32_t &idExtended,
             pcl::PointNormal &centerJr, int &idInvolvmentFront,
             bool &isBackBool) const;

  size_t getNumUsedPoint() const;

public:
  Pivoter();
  ~Pivoter();

  pcl::PolygonMesh::Ptr proceed(const pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud,
                                const double radius, const bool isDirty = false);

  typedef boost::shared_ptr<Pivoter> Ptr;
  typedef boost::shared_ptr<Pivoter const> ConstPtr;
};

#endif
