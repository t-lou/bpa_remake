#ifndef FRONT_H
#define FRONT_H

#include <list>
#include <map>

#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

#include "Edge.h"

class Front
{
  typedef std::pair<uint32_t, uint32_t> Signature;
protected:
  std::map<Signature, Edge> _front;
//  std::list<Edge> _front;
  std::vector<int> _num_occurence;

public:
  Front(const size_t numPoints = 0);
  ~Front();

  Edge::Ptr getActiveEdge();

  void addTriangle(const pcl::Vertices::ConstPtr& seed,
                   const pcl::PointNormal &center,
                   const double radius, const bool isBackBall);

  void addPoint(const Edge &lastEdge, const uint32_t idExtended,
                const pcl::PointNormal &center, const bool isBackBall);

  void addEdge(const Edge &edge);

  bool isPointIn(const uint32_t &id) const;

  size_t getNumActiveFront() const;

  bool isEdgeIn(const Edge &edge) const;

  int getConditionEdgeIn(const Edge &edge, const uint32_t &idExtended) const;

  void removeEdge(const uint32_t id0, const uint32_t id1);

  typedef boost::shared_ptr<Front> Ptr;
  typedef boost::shared_ptr<Front const> ConstPtr;
};

#endif
