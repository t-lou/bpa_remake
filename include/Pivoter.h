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
  /**
   * This class describes the pivoting ball, which rolls on points and links the mesh.
   */
  class Edge
  {
  protected:
    /** list of vertice index, should have length 2 */
    std::vector<uint32_t> _id_vertices;
    /** index of the opposite vertice, it and _id_vertices form the last triangle */
    uint32_t _id_opposite;
    /** center point of the last ball, it may not be one in the cloud */
    PointNT _center;
    /** whether this ball it to the it rolled in the back face of surface (not normal direction) */
    bool _is_back_ball;

  public:
    Edge ();

    /**
     * a fake constructor with only vertice index on edge
     * @param id0 index of start point on edge
     * @param id1 index of end point on edge
     */
    Edge (const uint32_t id0, const uint32_t id1);

    /**
     * real constructor for edge with full information
     * @param edge list of vertice index on edge
     * @param id_opposite index of opposite vertice
     * @param center center point of the ball
     * @param is_back_ball whether the ball was rolling on the back surface
     */
    Edge (const std::vector<uint32_t> &edge, const uint32_t id_opposite, const PointNT &center,
          const bool is_back_ball = false);

    ~Edge ();

    /**
     * returns the center of ball
     * @return
     */
    PointNT
    getCenter () const;

    /**
     * get the index of id-th vertice on edge, id should be either 0 or 1
     * @param id
     * @return
     */
    uint32_t
    getIdVertice (const size_t id) const;

    /**
     * get the index of opposite vertice
     * @return
     */
    uint32_t
    getIdOpposite () const;

    /**
     * set the index of id-th vertice to be id_vertice
     * @param id index inside edge. Should be 0 or 1
     * @param id_vertice value of vertice index
     */
    void
    setIdVertice (const size_t id, const uint32_t id_vertice);

    /**
     * checks whether the ball was rolling on back surface
     * @return
     */
    bool
    isBackBall () const;

    /**
     * get the signature of this edge, [index start vertice, index end vertice]
     * @return
     */
    std::pair<uint32_t, uint32_t>
    getSignature () const;

    /**
     * get the reverse signature of this edge, [index end vertice, index start vertice]
     * @return
     */
    std::pair<uint32_t, uint32_t>
    getSignatureReverse () const;

    typedef std::shared_ptr<Edge> Ptr;
    typedef std::shared_ptr<Edge const> ConstPtr;
  };

  /**
   * Front manages the edges, which ones are to be pivoted, which ones are pivoted.
   */
  class Front
  {
    typedef std::pair<uint32_t, uint32_t> Signature;
  protected:
    /** The set of edges to be pivoted */
    std::map<Signature, Edge> _front;
    /** The set of edges which are pivoted, but cannot reach suitable points. Edges here should be open boundary */
    std::map<Signature, Edge> _boundary;
    /** The set of successfully pivoted edges */
    std::set<Signature> _finished;
    /** The edge that is being pivoted and waiting for pivoting result: boundary or pivoted? */
    typename Edge::Ptr _currentEdge;

  public:
    Front ();

    ~Front ();

    /**
     * get one of the edges to pivot
     * @return
     */
    typename Edge::Ptr
    getActiveEdge ();

    /**
     * add the three edges of one triangle to edges to pivot
     * @param seed index of three vertices
     * @param center the center of the ball
     * @param is_back_ball whether the ball was pivoted on the back surface
     */
    void
    addTriangle (const pcl::Vertices::ConstPtr &seed, const PointNT &center, const bool is_back_ball);

    /**
     * extend the edges with one pivoted edge and vertice on new ball. the vertice on new ball should
     * form two edges with the start vertice and end vertice of the old edge
     * @param last_edge the edge which was pivoted and contributes two vertices to new edges
     * @param id_vetice_extended index of the vertice to form new edges
     * @param center center of the new ball
     * @param is_back_ball whether ball is rolling on back surface
     */
    void
    addPoint (const Edge &last_edge, const uint32_t id_vetice_extended, const PointNT &center, const bool is_back_ball);

    /**
     * add one edge to edges to pivot
     * @param edge
     */
    void
    addEdge (const Edge &edge);

    /**
     * checks whether edge is one the edges to pivot
     * @param edge
     * @return
     */
    bool
    isEdgeOnFront (const Edge &edge) const;

    /**
     * removes the edge with signature [id0,id1] or [id1,id0] in edges to pivoted
     * @param id0
     * @param id1
     */
    void
    removeEdge (const uint32_t id0, const uint32_t id1);

    /**
     * set the edge being pivoted as boundary or pivoted edge
     * @param is_boundary
     */
    void
    setFeedback (const bool is_boundary);

    /**
     * reset the front
     */
    void
    clear ();

    /**
     * checks whether edge or the reversed edge is pivoted
     * @param edge
     * @return
     */
    bool
    isEdgeFinished (const Edge &edge) const;

    typedef std::shared_ptr<Front> Ptr;
    typedef std::shared_ptr<Front const> ConstPtr;
  };

protected:
  /** kdtree for neighbor search */
  typename pcl::KdTreeFLANN<PointNT> _kdtree;
  /** point cloud */
  typename pcl::PointCloud<PointNT>::Ptr _cloud;
  /** _is_used[id] indicates whether point _cloud[id] is used for meshing */
  std::vector<bool> _is_used;
  /** edge manager */
  Front _front;
  /** search radius. default value is -1, radius would be guessed if it is non-positive */
  double _radius;
  /** whether balls on the back surface would be considered */
  bool _is_allow_back_ball;
  /** whether the pivoting ball could flip from front face to back face or vice versa */
  bool _is_allow_flip;
  /** threshold for avoiding collinear points */
  double _threshold_collinear_cos;
  /** threshold for avoiding too near points */
  double _threshold_distance_near;

  /**
   * prepare the kdtree and other data for ball pivoting
   */
  void
  prepare ();

  /**
   * find one starting triangle for ball pivoting
   * @param seed the index of triangle vertices
   * @param center the center of ball
   * @param is_back_ball whether the found seed is on back surface
   * @return whether one triangle is found
   */
  bool
  findSeed (pcl::Vertices::Ptr &seed, PointNT &center, bool &is_back_ball);

  /**
   * pivoting around edge
   * @param edge edge for pivoting
   * @param id_extended index of the hitting vertice, which would bring more edges for pivoting
   * @param center_new
   * @param is_back_ball
   * @return whether pivoting is successful, if not, it is a bounday
   */
  bool
  pivot (const Edge &edge, uint32_t &id_extended, PointNT &center_new, bool &is_back_ball) const;

  /**
   * pivot until the front has no edge to pivot
   * @param mesh
   */
  void
  proceedFront (pcl::PolygonMesh::Ptr &mesh);

  /**
   * find the center of newly pivoted ball, if empty, then pivoting is not successful
   * @param is_back_first whether to consider assumed center on back surface first. it changes the result if
   *        flipping between front surface and back surface is enabled (setAllowFlip)
   * @param index index of triangle vertices, which are all one ball surface
   * @param is_back_ball whether the pivoted ball is on back surface
   * @return
   */
  std::shared_ptr<PointNT>
  getBallCenter (const bool is_back_first, std::vector<uint32_t> &index, bool &is_back_ball) const;

public:
  Pivoter ();

  ~Pivoter ();

  /**
   * set the radius of ball
   * @param radius
   */
  void
  setSearchRadius (const double radius)
  {
    _radius = radius;
  }

  /**
   * get the radius of ball
   * @return
   */
  double
  getSearchRadius () const
  {
    return _radius;
  }

  /**
   * set whether ball on the back surface is allowed. if this value is set to true, the ball only pivots on the
   * direction of the normal vectors
   * @param is_allow_back_ball
   */
  void
  setAllowBackBall (const bool is_allow_back_ball)
  {
    _is_allow_back_ball = is_allow_back_ball;
  }

  /**
   * get whether ball on the back surface is allowed
   * @return
   */
  bool
  getAllowBackBall () const
  {
    return _is_allow_back_ball;
  }

  /**
   * set whether flipping between front surface and back surface is allow. if not, the ball would only pivot
   * so it stays on one side of the surface, depending on where the center of seed ball was found
   * @param is_allow_flip
   */
  void
  setAllowFlip (const bool is_allow_flip)
  {
    _is_allow_flip = is_allow_flip;
  }

  /**
   * get whether flipping between front surface and back surface is allow
   * @return
   */
  bool
  getAllowFlip () const
  {
    return _is_allow_flip;
  }

  /**
   * this function starts the preparation, pivoting process and generates the final mesh
   * @param cloud
   * @return
   */
  pcl::PolygonMesh::Ptr
  proceed ();

  /**
   * set the cloud to reconstruct
   * @param cloud
   */
  void
  setInputCloud (const typename pcl::PointCloud<PointNT>::ConstPtr &cloud);

  /**
   * estimate one radius and set it as ball radius. num_sample_point points would be selected randomly in cloud,
   * as least ratio_success of the sample points must have at least num_point_in_radius neighbors within
   * the estimated radius.
   * @param num_sample_point
   * @param num_point_in_radius
   * @param ratio_success
   */
  void
  setEstimatedRadius (const int num_sample_point, const int num_point_in_radius, const float ratio_success);

  typedef std::shared_ptr<Pivoter<PointNT> > Ptr;
  typedef std::shared_ptr<Pivoter<PointNT> const> ConstPtr;
};

#endif
