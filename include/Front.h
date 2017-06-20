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
  std::map<Signature, Edge> _boundary;
  Edge::Ptr _currentEdge;

public:
  Front();
  ~Front();

  Edge::Ptr getActiveEdge();

  void addTriangle(const pcl::Vertices::ConstPtr& seed,
                   const pcl::PointNormal &center,
                   const double radius, const bool isBackBall);

  void addPoint(const Edge &lastEdge, const uint32_t idExtended,
                const pcl::PointNormal &center, const bool isBackBall);

  void addEdge(const Edge &edge);

  size_t getNumActiveFront() const;

  bool isEdgeIn(const Edge &edge) const;

  int getConditionEdgeIn(const Edge &edge, const uint32_t &idExtended) const;

  void removeEdge(const uint32_t id0, const uint32_t id1);

  void setFeedback(const bool isBoundary);

  void clear();

  void prepareDirtyFix(std::vector<bool> &isUsed);

  typedef boost::shared_ptr<Front> Ptr;
  typedef boost::shared_ptr<Front const> ConstPtr;
};

#endif
