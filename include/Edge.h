#ifndef EDGE_H
#define EDGE_H

#include <vector>
#include <memory>

#include <pcl/point_types.h>

class Edge
{
protected:
  std::vector<uint32_t> _idVertices;
  uint32_t _idOpposite;
  pcl::PointNormal _center;
  double _radius;
  bool _isBackBall;

public:
  Edge();
  Edge(const uint32_t id0, const uint32_t id1);
  Edge(const std::vector<uint32_t>& edge, const uint32_t idOpposite,
       const double radius, const pcl::PointNormal &center,
       const bool isBackBall = false);
  ~Edge();

  double getRadius() const;

  pcl::PointNormal getCenter() const;

  uint32_t getIdVertice(const size_t id) const;

  uint32_t getIdOpposite() const;

  void setIdVertice(const size_t id, const uint32_t idVertice);

  bool isBackBall() const;

  std::pair<uint32_t, uint32_t> getSignature() const;

//  bool operator == (const Edge &another) const;

  typedef boost::shared_ptr<Edge> Ptr;
  typedef boost::shared_ptr<Edge const> ConstPtr;
};

#endif
