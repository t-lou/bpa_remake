#ifndef FRONT_HPP
#define FRONT_HPP

#include <algorithm>

#include "Front.h"

Front::Front(const size_t numPoints) :
  _num_occurence(std::vector<int>(numPoints, 0))
{
}

Front::~Front()
{
}

Edge::Ptr Front::getActiveEdge()
{
  // TODO maximal usage is 2 times
  Edge::Ptr re;
  if(!_front.empty())
  {
    --_num_occurence.at(_front.begin()->second.getIdVertice(0));
    --_num_occurence.at(_front.begin()->second.getIdVertice(1));

    re = Edge::Ptr(new Edge());
    *re = _front.begin()->second;
    _front.erase(_front.begin());
  }
  return re;
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

    ++_num_occurence.at(edge.getIdVertice(0));
    ++_num_occurence.at(edge.getIdVertice(1));
//    std::cout << "add " << edge.getIdVertice(0) << "  " << edge.getIdVertice(1) << "\n";
  }
//  else
//    std::cout<< edge.getIdVertice(0) << "  " << edge.getIdVertice(1) << " added before\n";
}

bool Front::isPointIn(const uint32_t& id) const
{
  assert(id < _num_occurence.size());
  return _num_occurence.at(id) > 0;
}

size_t Front::getNumActiveFront() const
{
  return _front.size();
}

bool Front::isEdgeIn(const Edge& edge) const
{
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
      --_num_occurence.at(id0);
      --_num_occurence.at(id1);
//      std::cout << "remove " << id0 << "  " << id1 << "\n";
    }
    else
    {
      loc = _front.find(Signature(id1, id0));
      if(loc != _front.end())
      {
        _front.erase(loc);
        --_num_occurence.at(id0);
        --_num_occurence.at(id1);
//        std::cout << "remove " << id1 << "  " << id0 << "\n";
      }
    }
  }
}

#endif
