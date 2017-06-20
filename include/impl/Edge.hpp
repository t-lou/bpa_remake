#ifndef EDGE_HPP
#define EDGE_HPP

#include "Edge.h"

Edge::Edge()
{
}

Edge::Edge(const uint32_t id0, const uint32_t id1)
{
  _idVertices.resize(2);
  _idVertices.at(0) = id0;
  _idVertices.at(1) = id1;
}

Edge::Edge(const std::vector<uint32_t>& edge, const uint32_t idOpposite,
           const double radius, const pcl::PointNormal &center,
           const bool isBackBall) :
  _idVertices(edge),
  _idOpposite(idOpposite),
  _radius(radius),
  _center(center),
  _isBackBall(isBackBall)
{
}

Edge::~Edge()
{
}

double Edge::getRadius() const
{
  return _radius;
}

pcl::PointNormal Edge::getCenter() const
{
  return _center;
}

uint32_t Edge::getIdVertice(const size_t id) const
{
  assert(id < _idVertices.size());
  return _idVertices.at(id);
}

uint32_t Edge::getIdOpposite() const
{
  return _idOpposite;
}

void Edge::setIdVertice(const size_t id, const uint32_t idVertice)
{
  assert(id < _idVertices.size());
  _idVertices.at(id) = idVertice;
}

//bool Edge::operator ==(const Edge& another) const
//{
//  return (another._idVertices.at(0) == _idVertices.at(0) && another._idVertices.at(1) == _idVertices.at(1))
//      || (another._idVertices.at(1) == _idVertices.at(0) && another._idVertices.at(0) == _idVertices.at(1));
//}

bool Edge::isBackBall() const
{
  return _isBackBall;
}

std::pair<uint32_t, uint32_t> Edge::getSignature() const
{
  assert(_idVertices.size() == 2);
  return std::pair<uint32_t, uint32_t>(_idVertices.at(0), _idVertices.at(1));
}

#endif
