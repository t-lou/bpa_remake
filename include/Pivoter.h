#ifndef PIVOTER_H
#define PIVOTER_H

#include <memory>
#include <vector>
#include <list>
#include <map>
#include <set>

#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>

template<typename PointNT>
class Pivoter
{
protected:
  class Edge
  {
  protected:
    std::vector<uint32_t> _id_vertices;
    uint32_t _id_opposite;
    PointNT _center;
    double _radius;
    bool _is_back_ball;

  public:
    Edge ();

    Edge (const uint32_t id0, const uint32_t id1);

    Edge (const std::vector<uint32_t> &edge, const uint32_t id_opposite, const double radius, const PointNT &center,
          const bool is_back_ball = false);

    ~Edge ();

    double
    getRadius () const;

    PointNT
    getCenter () const;

    uint32_t
    getIdVertice (const size_t id) const;

    uint32_t
    getIdOpposite () const;

    void
    setIdVertice (const size_t id, const uint32_t id_vertice);

    bool
    isBackBall () const;

    std::pair<uint32_t, uint32_t>
    getSignature () const;

    std::pair<uint32_t, uint32_t>
    getSignatureReverse () const;

    typedef boost::shared_ptr<Edge> Ptr;
    typedef boost::shared_ptr<Edge const> ConstPtr;
  };

  class Front
  {
    typedef std::pair<uint32_t, uint32_t> Signature;
  protected:
    std::map<Signature, Edge> _front;
    std::map<Signature, Edge> _boundary;
    std::set<Signature> _finished;
    typename Edge::Ptr _currentEdge;

  public:
    Front ();

    ~Front ();

    typename Edge::Ptr
    getActiveEdge ();

    void
    addTriangle (const pcl::Vertices::ConstPtr &seed, const PointNT &center, const double radius,
                 const bool is_back_ball);

    void
    addPoint (const Edge &lastEdge, const uint32_t id_extended, const PointNT &center, const bool is_back_ball);

    void
    addEdge (const Edge &edge);

    size_t
    getNumActiveFront () const;

    bool
    isEdgeIn (const Edge &edge) const;

    void
    removeEdge (const uint32_t id0, const uint32_t id1);

    void
    setFeedback (const bool is_boundary);

    void
    clear ();

    bool
    isEdgeFinished (const Edge &edge);

    typedef boost::shared_ptr<Front> Ptr;
    typedef boost::shared_ptr<Front const> ConstPtr;
  };

protected:
  typename pcl::KdTreeFLANN<PointNT> _kdtree;
  typename pcl::PointCloud<PointNT>::Ptr _cloud;
  std::vector<bool> _is_used;
  Front _front;
  double _radius;

  void
  prepare (const typename pcl::PointCloud<PointNT>::ConstPtr &cloud, const double radius);

  bool
  findSeed (pcl::Vertices::Ptr &seed, PointNT &center, bool &is_back_ball);

  bool
  pivot (const Edge &edge, uint32_t &id_extended, PointNT &center_new, bool &is_back_ball) const;

  void
  proceedFront (pcl::PolygonMesh::Ptr &mesh);

public:
  Pivoter ();

  ~Pivoter ();

  pcl::PolygonMesh::Ptr
  proceed (const typename pcl::PointCloud<PointNT>::ConstPtr &cloud, const double radius);

  typedef boost::shared_ptr<Pivoter<PointNT> > Ptr;
  typedef boost::shared_ptr<Pivoter<PointNT> const> ConstPtr;
};

#endif
