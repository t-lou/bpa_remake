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

class Pivoter
{
protected:
  class Edge
  {
  protected:
    std::vector<uint32_t> _idVertices;
    uint32_t _idOpposite;
    pcl::PointNormal _center;
    double _radius;
    bool _isBackBall;

  public:
    Edge ();

    Edge (const uint32_t id0, const uint32_t id1);

    Edge (const std::vector<uint32_t> &edge, const uint32_t idOpposite, const double radius,
          const pcl::PointNormal &center, const bool isBackBall = false);

    ~Edge ();

    double getRadius () const;

    pcl::PointNormal getCenter () const;

    uint32_t getIdVertice (const size_t id) const;

    uint32_t getIdOpposite () const;

    void setIdVertice (const size_t id, const uint32_t idVertice);

    bool isBackBall () const;

    std::pair<uint32_t, uint32_t> getSignature () const;

    std::pair<uint32_t, uint32_t> getSignatureReverse () const;

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
    Edge::Ptr _currentEdge;

  public:
    Front ();

    ~Front ();

    Edge::Ptr getActiveEdge ();

    void addTriangle (const pcl::Vertices::ConstPtr &seed, const pcl::PointNormal &center, const double radius,
                      const bool isBackBall);

    void
    addPoint (const Edge &lastEdge, const uint32_t idExtended, const pcl::PointNormal &center, const bool isBackBall);

    void addEdge (const Edge &edge);

    size_t getNumActiveFront () const;

    bool isEdgeIn (const Edge &edge) const;

    int getConditionEdgeIn (const Edge &edge, const uint32_t &idExtended) const;

    void removeEdge (const uint32_t id0, const uint32_t id1);

    void setFeedback (const bool isBoundary);

    void clear ();

    void prepareDirtyFix (std::vector<bool> &isUsed);

    bool isEdgeFinished (const Edge &edge);

    typedef boost::shared_ptr<Front> Ptr;
    typedef boost::shared_ptr<Front const> ConstPtr;
  };

protected:
  pcl::KdTreeFLANN<pcl::PointNormal> _kdtree;
  pcl::PointCloud<pcl::PointNormal>::Ptr _cloud;
  std::vector<bool> _is_used;
  Front _front;
  double _radius;

  void prepare (const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud, const double radius);

  bool findSeed (pcl::Vertices::Ptr &seed, pcl::PointNormal &center, bool &isBackBall);

  bool pivot (const Edge &edge, const bool isCountAll, uint32_t &idExtended, pcl::PointNormal &centerJr,
              bool &isBackBool) const;

  void proceedFront (const bool isCountAll, pcl::PolygonMesh::Ptr &mesh);

public:
  Pivoter ();

  ~Pivoter ();

  pcl::PolygonMesh::Ptr proceed (const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud, const double radius);

  typedef boost::shared_ptr<Pivoter> Ptr;
  typedef boost::shared_ptr<Pivoter const> ConstPtr;
};

#endif
