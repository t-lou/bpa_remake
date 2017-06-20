#ifndef FRONT_HPP
#define FRONT_HPP

#include <algorithm>
#include <boost/make_shared.hpp>

#include "Front.h"

Front::Front()
{
  clear();
}

Front::~Front()
{
}

Edge::Ptr Front::getActiveEdge()
{
  Edge::Ptr re;
  if(!_front.empty())
  {
    re = boost::make_shared<Edge>(_front.begin()->second);
    _currentEdge = re;
    _front.erase(_front.begin());
  }
  return re;
}

void Front::setFeedback(const bool isBoundary)
{
  if(isBoundary)
  {
    _boundary[_currentEdge->getSignature()] = *_currentEdge;
  }
}

void Front::addTriangle(const pcl::Vertices::ConstPtr& seed,
                        const pcl::PointNormal &center,
                        const double radius, const bool isBackBall)
{
  assert(seed->vertices.size() == 3);
  for(size_t idv = 0; idv < 3; ++idv)
  {
    std::vector<uint32_t> edge(2, 0);
    edge.at(0) = seed->vertices.at(idv);
    edge.at(1) = seed->vertices.at((idv + 2) % 3);

    addEdge(Edge(edge, seed->vertices.at((idv + 1) % 3),
                 radius, center, isBackBall));
  }
}

void Front::addPoint(const Edge &lastEdge, const uint32_t idExtended,
                     const pcl::PointNormal &center, const bool isBackBall)
{
  std::vector<uint32_t> edge(2, 0);

  edge.at(0) = lastEdge.getIdVertice(0);
  edge.at(1) = idExtended;
  addEdge(Edge(edge, lastEdge.getIdVertice(1),
               lastEdge.getRadius(), center, isBackBall));

  edge.at(0) = idExtended;
  edge.at(1) = lastEdge.getIdVertice(1);
  addEdge(Edge(edge, lastEdge.getIdVertice(0),
               lastEdge.getRadius(), center, isBackBall));
}

void Front::addEdge(const Edge& edge)
{
  if(!isEdgeIn(edge))
  {
    _front[Signature(edge.getIdVertice(0), edge.getIdVertice(1))] = edge;
  }
}

size_t Front::getNumActiveFront() const
{
  return _front.size();
}

bool Front::isEdgeIn(const Edge& edge) const
{
  // toggle comment to delete one-directionally
  return _front.find(Signature(edge.getIdVertice(0), edge.getIdVertice(1))) != _front.end()
      || _front.find(Signature(edge.getIdVertice(1), edge.getIdVertice(0))) != _front.end();
//  return _front.find(Signature(edge.getIdVertice(0), edge.getIdVertice(1))) != _front.end();
}

int Front::getConditionEdgeIn(const Edge& edge, const uint32_t& idExtended) const
{
  Edge ex0 = edge;
  Edge ex1 = edge;
  ex0.setIdVertice(0, idExtended);
  ex0.setIdVertice(1, edge.getIdVertice(0)); // (idExtend, edge[0])
  ex1.setIdVertice(0, edge.getIdVertice(1));
  ex1.setIdVertice(1, idExtended); // (idExtend, edge[1])
  bool is_ex0_in = isEdgeIn(ex0);
  bool is_ex1_in = isEdgeIn(ex1);
  if(is_ex0_in)
  {
    if(is_ex1_in)
    {
      // both (idExtend, edge[0]) and (idExtend, edge[1]) are in front
      return 2;
    }
    else
    {
      // (idExtend, edge[0]) is in front
      return 0;
    }
  }
  else
  {
    if(is_ex1_in)
    {
      // (idExtend, edge[1]) is in front
      return 1;
    }
    else
    {
      return -1; // not in front
    }
  }
}

void Front::removeEdge(const uint32_t id0, const uint32_t id1)
{
  if(!_front.empty())
  {
    std::map<Signature, Edge>::iterator loc
        = _front.find(Signature(id0, id1));
    if(loc != _front.end())
    {
      _front.erase(loc);
    }
    else // comment to delete one-directionally
    {
      loc = _front.find(Signature(id1, id0));
      if(loc != _front.end())
      {
        _front.erase(loc);
      }
    }
  }
}

void Front::clear()
{
  _front.clear();
  _boundary.clear();
  _currentEdge.reset();
}

void Front::prepareDirtyFix(std::vector<bool> &isUsed)
{
  std::fill(isUsed.begin(), isUsed.end(), true);
  for(std::map<Signature, Edge>::iterator it = _boundary.begin();
      it != _boundary.end(); ++it)
  {
    isUsed.at(it->first.first) = false;
    isUsed.at(it->first.second) = false;
  }

  _boundary.swap(_front);
  _boundary.clear();
}

#endif
