#ifndef PIVOTER_HPP
#define PIVOTER_HPP

#include <algorithm>
#include <cmath>

#include <boost/make_shared.hpp>

#include "Pivoter.h"

Pivoter::Pivoter ()
{
}

Pivoter::~Pivoter ()
{
}

/**
 * @brief get_plane_between returns the plane between two points,
 *        it is perpendicular to v0-v1 and crosses their mid-point
 * @param v0
 * @param v1
 * @return
 */
Eigen::Vector4f get_plane_between (const Eigen::Vector3f &v0, const Eigen::Vector3f &v1)
{
  Eigen::Vector4f plane;
  Eigen::Vector3f normal = (v1 - v0);
  normal.normalize ();
  plane << normal, -(v1 + v0).dot (normal) * 0.5f;
  return plane;
}

/**
 * @brief is_positions_near checks whether any pair of three points are near
 * @param pos0
 * @param pos1
 * @param pos2
 * @param threshold threshold for whether two points are near
 * @return
 */
bool is_positions_near (const Eigen::Vector3f &pos0, const Eigen::Vector3f &pos1, const Eigen::Vector3f &pos2,
                        const float threshold)
{
  return (pos0 - pos1).norm () < threshold || (pos1 - pos2).norm () < threshold || (pos2 - pos0).norm () < threshold;
}

/**
 * @brief get_id_point_in_sphere returns the index of points which are in sphere
 *        with center and radius as given
 * @param kdtree kdtree containing the point cloud
 * @param center
 * @param radius
 * @return
 */
std::vector<int>
get_id_point_in_sphere (const pcl::KdTreeFLANN<pcl::PointNormal> &kdtree, const pcl::PointNormal &center,
                        const double radius)
{
  std::vector<int> indices;
  std::vector<float> sqr_distances;
  kdtree.radiusSearch (center, radius, indices, sqr_distances);
  return indices;
}

/**
 * @brief num_point_in_sphere returns the number of points in sphere with given center and radius
 * @param center
 * @param radius
 * @param kdtree
 * @param is_count_all whether to count all points or only unused points
 * @param is_used a list indicating whether points with same index are used
 * @return
 */
size_t num_point_in_sphere (const pcl::PointNormal &center, const float radius,
                            const pcl::KdTreeFLANN<pcl::PointNormal> &kdtree, const bool is_count_all,
                            const std::vector<bool> &is_used)
{
  std::vector<int> indices = get_id_point_in_sphere (kdtree, center, radius);
  if (is_count_all)
  {
    return indices.size ();
  } else
  {
    size_t count = 0;
    for (std::vector<int>::const_iterator it = indices.begin (); it != indices.end (); ++it)
    {
      if (!is_used.at (*it))
      {
        ++count;
      }
    }
    return count;
  }
}

/**
 * @brief vec2pointnormal returns a point with vector as coordinate
 * @param vector
 * @return
 */
boost::shared_ptr<pcl::PointNormal> vec2pointnormal (const Eigen::Vector3f &vector)
{
  boost::shared_ptr<pcl::PointNormal> pt = boost::shared_ptr<pcl::PointNormal> (new pcl::PointNormal ());
  pt->getVector3fMap () = vector;
  return pt;
}

bool is_normal_consistent (const Eigen::Vector3f &normal, std::vector<uint32_t> &index,
                           const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud)
{
  assert(index.size () == 3);
  int count_consistent = 0;
  for (size_t id = 0; id < 3; ++id)
  {
    if (normal.dot (cloud->at (index.at (id)).getNormalVector3fMap ()) > 0.0f)
    {
      ++count_consistent;
    }
  }
  return count_consistent >= 2;
}

Eigen::Vector3f
get_circle_center (const Eigen::Vector3f &point0, const Eigen::Vector3f &point1, const Eigen::Vector3f &point2)
{
  // https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_from_cross-_and_dot-products
  const Eigen::Vector3f vec2 = point0 - point1;
  const Eigen::Vector3f vec0 = point1 - point2;
  const Eigen::Vector3f vec1 = point2 - point0;
  const float area = vec0.cross (vec1).norm ();
  const float determinator = 2.0f * area * area;

  const float alpha = vec0.dot (vec0) * vec2.dot (-vec1) / determinator;
  const float beta = vec1.dot (vec1) * -vec2.dot (vec0) / determinator;
  const float gamma = vec2.dot (vec2) * vec1.dot (-vec0) / determinator;

  return alpha * point0 + beta * point1 + gamma * point2;
}

boost::shared_ptr<pcl::PointNormal> get_ball_center (const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
                                                     const pcl::KdTreeFLANN<pcl::PointNormal> &kdtree,
                                                     const double radius, const bool is_back_first,
                                                     const bool is_count_all, const std::vector<bool> &is_used,
                                                     std::vector<uint32_t> &index, bool &is_back_ball)
{
  boost::shared_ptr<pcl::PointNormal> center;
  const static float cos_10 = (float) cos (10.0 * M_PI / 180.0);
  const Eigen::Vector3f pos0 (cloud->at (index.at (0)).getVector3fMap ());
  const Eigen::Vector3f pos1 (cloud->at (index.at (1)).getVector3fMap ());
  const Eigen::Vector3f pos2 (cloud->at (index.at (2)).getVector3fMap ());
  const double search_radius = radius;
  const double thres_near = 1e-6;

  Eigen::Vector3f vec0 = pos1 - pos0;
  Eigen::Vector3f vec1 = pos2 - pos0;
  vec0.normalize ();
  vec1.normalize ();

  if (!is_positions_near (pos0, pos1, pos2, thres_near) && fabs (vec0.dot (vec1)) < cos_10)
  {
    Eigen::Vector3f center_circle = get_circle_center (pos0, pos1, pos2);

    // move to distance _radius
    float radius_planar = (center_circle - pos0).norm ();
    if (radius_planar < radius)
    {
      Eigen::Vector3f normal = vec0.cross (vec1);
      boost::shared_ptr<pcl::PointNormal> center_candidate;
      float dist_normal = sqrt (radius * radius - radius_planar * radius_planar);
      normal.normalize ();
      if (!is_normal_consistent (normal, index, cloud))
      {
        normal = -normal;
        std::swap (index.at (0), index.at (2));
      }
      normal *= dist_normal;

      if (is_back_first)
      {
        center_candidate = vec2pointnormal (Eigen::Vector3f (center_circle - normal));
        if (num_point_in_sphere (*center_candidate, search_radius, kdtree, is_count_all, is_used) <= 3)
        {
          center = center_candidate;
          is_back_ball = true;
        } else
        {
          center_candidate = vec2pointnormal (Eigen::Vector3f (center_circle + normal));
          if (num_point_in_sphere (*center_candidate, search_radius, kdtree, is_count_all, is_used) <= 3)
          {
            center = center_candidate;
            is_back_ball = false;
          }
        }
      } else
      {
        center_candidate = vec2pointnormal (Eigen::Vector3f (center_circle + normal));
        if (num_point_in_sphere (*center_candidate, search_radius, kdtree, is_count_all, is_used) <= 3)
        {
          center = center_candidate;
          is_back_ball = false;
        } else
        {
          center_candidate = vec2pointnormal (Eigen::Vector3f (center_circle - normal));
          if (num_point_in_sphere (*center_candidate, search_radius, kdtree, is_count_all, is_used) <= 3)
          {
            center = center_candidate;
            is_back_ball = true;
          }
        }
      }
    }
  }

  return center;
}

Eigen::Vector3f
get_normal_triangle (const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud, const std::vector<uint32_t> &index)
{
  assert(index.size () == 3);
  Eigen::Vector3f p0 = cloud->at (index.at (0)).getVector3fMap ();
  Eigen::Vector3f normal = (cloud->at (index.at (1)).getVector3fMap () - p0).cross (
    cloud->at (index.at (2)).getVector3fMap () - p0);
  normal.normalize ();
  return normal;
}

void reorder (const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud, pcl::Vertices &triangle)
{
  if (!is_normal_consistent (get_normal_triangle (cloud, triangle.vertices), triangle.vertices, cloud))
  {
    std::swap (triangle.vertices.at (1), triangle.vertices.at (2));
  }
}

float get_distance_point_place (const Eigen::Vector4f &plane, const pcl::PointNormal &point)
{
  return plane (3) + point.getVector3fMap ().dot (plane.segment (0, 3));
}

float get_angle_rotation (const Eigen::Vector3f &point0, const Eigen::Vector3f &point1, const Eigen::Vector3f &center,
                          const Eigen::Vector4f &plane)
{
  Eigen::Vector3f vc0 = point0 - center;
  Eigen::Vector3f vc1 = point1 - center;
  vc0.normalize ();
  vc1.normalize ();

  float sin_ = vc0.cross (-Eigen::Vector3f (plane.segment (0, 3))).dot (vc1);
  float cos_ = vc0.dot (vc1);
  float angle = atan2 (sin_, cos_);
  if (angle < 0.0f) // -pi~pi -> 0~2pi
  {
    angle += (float) (2.0 * M_PI);
  }
  return angle;
}

bool Pivoter::pivot (const Edge &edge, const bool isCountAll, uint32_t &idExtended, pcl::PointNormal &centerJr,
                     bool &isBackBool) const
{
  const uint32_t id0 = edge.getIdVertice (0);
  const uint32_t id1 = edge.getIdVertice (1);
  const uint32_t id_op = edge.getIdOpposite ();
  const Eigen::Vector3f center = edge.getCenter ().getVector3fMap ();
  const Eigen::Vector3f v0 = _cloud->at (id0).getVector3fMap ();
  const Eigen::Vector3f v1 = _cloud->at (id1).getVector3fMap ();
  const Eigen::Vector3f mid = (v0 + v1) * 0.5f;
  // pivot opposite to normal direction, change direction for angle
  const Eigen::Vector4f plane = edge.isBackBall () ? get_plane_between (v0, v1) : get_plane_between (v1, v0);
  pcl::PointCloud<pcl::PointNormal> center_candidates;
  std::vector<float> dot_candidates;
  std::vector<uint32_t> id_candidates;
  std::vector<bool> is_back_candidates;

  const double search_radius = sqrt (_radius * _radius - (v0 - mid).dot (Eigen::Vector3f (v0 - mid))) + _radius;
  std::vector<uint32_t> point3 (3, 0);
  std::vector<int> indices = get_id_point_in_sphere (_kdtree, *vec2pointnormal (mid), search_radius);

  center_candidates.reserve (indices.size ());
  dot_candidates.reserve (indices.size ());
  id_candidates.reserve (indices.size ());
  is_back_candidates.reserve (indices.size ());
  for (std::vector<int>::iterator it = indices.begin (); it != indices.end (); ++it)
  {
    point3.at (0) = id0;
    point3.at (1) = id1;
    point3.at (2) = (uint32_t) (*it);

    if (point3.at (2) == id0 || point3.at (2) == id1 || point3.at (2) == id_op ||
        !is_normal_consistent (get_normal_triangle (_cloud, point3), point3, _cloud) ||
        fabs (get_distance_point_place (plane, _cloud->at (*it))) > _radius)
    {
      continue;
    }

//    if(!_is_used.at(*it) || idInvolvmentFront >= 0)
    {
      bool is_back_bool;
      boost::shared_ptr<pcl::PointNormal> center_jr = get_ball_center (_cloud, _kdtree, _radius, edge.isBackBall (),
                                                                       isCountAll, _is_used, point3, is_back_bool);

      if (center_jr)
      {
        center_candidates.push_back (*center_jr);
        dot_candidates.push_back (get_angle_rotation (center, center_jr->getVector3fMap (), mid, plane));
        id_candidates.push_back ((uint32_t) *it);
        is_back_candidates.push_back (is_back_bool);
      }
    }
  }
  if (!center_candidates.empty ())
  {
    int id_min = std::distance (dot_candidates.begin (),
                                std::min_element (dot_candidates.begin (), dot_candidates.end ()));
    idExtended = id_candidates.at (id_min);
    centerJr = center_candidates.at (id_min);
    isBackBool = is_back_candidates.at (id_min);
    return true;
  } else
  {
    return false;
  }
}

void Pivoter::prepare (const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud, const double radius)
{
  _radius = radius;
  _cloud = cloud->makeShared ();

  _kdtree.setInputCloud (cloud);
  _is_used.clear ();
  _is_used.resize (cloud->size (), false);

  _front = Front ();
}

void Pivoter::proceedFront (const bool isCountAll, pcl::PolygonMesh::Ptr &mesh)
{
  Pivoter::Edge::Ptr edge;
  while (edge = _front.getActiveEdge ())
  {
    uint32_t id_ext;
    pcl::PointNormal center_new;
    bool is_back_ball;
    if (!_front.isEdgeFinished (*edge) && pivot (*edge, isCountAll, id_ext, center_new, is_back_ball))
    {
      const uint32_t id0 = edge->getIdVertice (0);
      const uint32_t id1 = edge->getIdVertice (1);
      pcl::Vertices triangle;
      // add to mesh
      triangle.vertices.reserve (3);
      triangle.vertices.push_back (id0);
      triangle.vertices.push_back (id1);
      triangle.vertices.push_back (id_ext);
      mesh->polygons.push_back (triangle);

      _is_used.at (id_ext) = true;
      _front.addPoint (*edge, id_ext, center_new, is_back_ball);

      // pivoting succeeds, not boundary
      _front.setFeedback (false);
    } // if(pivot(*edge, id_ext, center_new, is_back_ball))
    else
    {
      // not pivoted
      _front.setFeedback (true);
    }
  }
}

pcl::PolygonMesh::Ptr Pivoter::proceed (const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud, const double radius)
{
  assert(cloud);
  pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh ());

  prepare (cloud, radius);

  while (true)
  {
    proceedFront (true, mesh);

    pcl::Vertices::Ptr seed;
    pcl::PointNormal center;
    bool is_back_ball;
    if (findSeed (seed, center, is_back_ball))
    {
      // add to mesh
      mesh->polygons.push_back (*seed);
      // add for pivoting
      _front.addTriangle (seed, center, _radius, is_back_ball);
    } else
    {
      // cannot find proper seed
      break;
    }
  }

  pcl::toPCLPointCloud2 (*_cloud, mesh->cloud);

  return mesh;
}

bool Pivoter::findSeed (pcl::Vertices::Ptr &seed, pcl::PointNormal &center, bool &isBackBall)
{
  const double search_radius = _radius * 2.0;

  for (size_t id_search = 0; id_search < _is_used.size (); ++id_search)
  {
    if (_is_used.at (id_search))
    {
      continue;
    }

    std::vector<uint32_t> index3 (3, 0);
    std::vector<int> indices = get_id_point_in_sphere (_kdtree, _cloud->at (id_search), search_radius);
    if (indices.size () < 3)
    {
      continue;
    }

    for (size_t idn1 = 1; idn1 < indices.size (); ++idn1)
    {
      const uint32_t index1 = indices.at (idn1);
      if (_is_used.at (index1) || index1 == (uint32_t) id_search)
      {
        continue;
      }

      for (size_t idn2 = 0; idn2 < idn1; ++idn2)
      {
        const uint32_t index2 = indices.at (idn2);

        if (_is_used.at (index2) || index1 == index2 || index2 == (uint32_t) id_search)
        {
          continue;
        }

        index3.at (0) = (uint32_t) id_search;
        index3.at (1) = index1;
        index3.at (2) = index2;

        bool is_back_ball;
        boost::shared_ptr<pcl::PointNormal> center_ = get_ball_center (_cloud, _kdtree, _radius, false, true, _is_used,
                                                                       index3, is_back_ball);
        if (center_)
        {
          seed = pcl::Vertices::Ptr (new pcl::Vertices ());
          seed->vertices = index3;
          center = *center_;
          isBackBall = is_back_ball;
          _is_used.at (index3.at (0)) = true;
          _is_used.at (index3.at (1)) = true;
          _is_used.at (index3.at (2)) = true;

          return true;
        }
      }
    }

    _is_used.at (id_search) = true;
  }
  return false;
}

Pivoter::Edge::Edge ()
{
}

Pivoter::Edge::Edge (const uint32_t id0, const uint32_t id1)
{
  _idVertices.resize (2);
  _idVertices.at (0) = id0;
  _idVertices.at (1) = id1;
}

Pivoter::Edge::Edge (const std::vector<uint32_t> &edge, const uint32_t idOpposite, const double radius,
                     const pcl::PointNormal &center, const bool isBackBall) : _idVertices (edge),
                                                                              _idOpposite (idOpposite),
                                                                              _radius (radius), _center (center),
                                                                              _isBackBall (isBackBall)
{
}

Pivoter::Edge::~Edge ()
{
}

double Pivoter::Edge::getRadius () const
{
  return _radius;
}

pcl::PointNormal Pivoter::Edge::getCenter () const
{
  return _center;
}

uint32_t Pivoter::Edge::getIdVertice (const size_t id) const
{
  assert(id < _idVertices.size ());
  return _idVertices.at (id);
}

uint32_t Pivoter::Edge::getIdOpposite () const
{
  return _idOpposite;
}

void Pivoter::Edge::setIdVertice (const size_t id, const uint32_t idVertice)
{
  assert(id < _idVertices.size ());
  _idVertices.at (id) = idVertice;
}

bool Pivoter::Edge::isBackBall () const
{
  return _isBackBall;
}

std::pair<uint32_t, uint32_t> Pivoter::Edge::getSignature () const
{
  assert(_idVertices.size () == 2);
  return std::pair<uint32_t, uint32_t> (_idVertices.at (0), _idVertices.at (1));
}

std::pair<uint32_t, uint32_t> Pivoter::Edge::getSignatureReverse () const
{
  assert(_idVertices.size () == 2);
  return std::pair<uint32_t, uint32_t> (_idVertices.at (1), _idVertices.at (0));
}

Pivoter::Front::Front ()
{
  clear ();
}

Pivoter::Front::~Front ()
{
}

Pivoter::Edge::Ptr Pivoter::Front::getActiveEdge ()
{
  Edge::Ptr re;
  if (!_front.empty ())
  {
    re = boost::make_shared<Edge> (_front.begin ()->second);
    _currentEdge = re;
    _front.erase (_front.begin ());
  }
  return re;
}

void Pivoter::Front::setFeedback (const bool isBoundary)
{
  if (isBoundary)
  {
    _boundary[_currentEdge->getSignature ()] = *_currentEdge;
  }
  _finished.insert (_currentEdge->getSignature ());
}

void
Pivoter::Front::addTriangle (const pcl::Vertices::ConstPtr &seed, const pcl::PointNormal &center, const double radius,
                             const bool isBackBall)
{
  assert(seed->vertices.size () == 3);
  for (size_t idv = 0; idv < 3; ++idv)
  {
    std::vector<uint32_t> edge (2, 0);
    edge.at (0) = seed->vertices.at (idv);
    edge.at (1) = seed->vertices.at ((idv + 2) % 3);

    addEdge (Edge (edge, seed->vertices.at ((idv + 1) % 3), radius, center, isBackBall));
  }
}

void Pivoter::Front::addPoint (const Edge &lastEdge, const uint32_t idExtended, const pcl::PointNormal &center,
                               const bool isBackBall)
{
  std::vector<uint32_t> edge (2, 0);

  edge.at (0) = lastEdge.getIdVertice (0);
  edge.at (1) = idExtended;
  addEdge (Edge (edge, lastEdge.getIdVertice (1), lastEdge.getRadius (), center, isBackBall));

  edge.at (0) = idExtended;
  edge.at (1) = lastEdge.getIdVertice (1);
  addEdge (Edge (edge, lastEdge.getIdVertice (0), lastEdge.getRadius (), center, isBackBall));
}

void Pivoter::Front::addEdge (const Edge &edge)
{
  if (!isEdgeIn (edge))
  {
    _front[Signature (edge.getIdVertice (0), edge.getIdVertice (1))] = edge;
  }
}

size_t Pivoter::Front::getNumActiveFront () const
{
  return _front.size ();
}

bool Pivoter::Front::isEdgeIn (const Edge &edge) const
{
  // toggle comment to delete one-directionally
  return _front.find (edge.getSignature ()) != _front.end () ||
         _front.find (edge.getSignatureReverse ()) != _front.end ();
}

int Pivoter::Front::getConditionEdgeIn (const Edge &edge, const uint32_t &idExtended) const
{
  Edge ex0 = edge;
  Edge ex1 = edge;
  ex0.setIdVertice (0, idExtended);
  ex0.setIdVertice (1, edge.getIdVertice (0)); // (idExtend, edge[0])
  ex1.setIdVertice (0, edge.getIdVertice (1));
  ex1.setIdVertice (1, idExtended); // (idExtend, edge[1])
  bool is_ex0_in = isEdgeIn (ex0);
  bool is_ex1_in = isEdgeIn (ex1);
  if (is_ex0_in)
  {
    if (is_ex1_in)
    {
      // both (idExtend, edge[0]) and (idExtend, edge[1]) are in front
      return 2;
    } else
    {
      // (idExtend, edge[0]) is in front
      return 0;
    }
  } else
  {
    if (is_ex1_in)
    {
      // (idExtend, edge[1]) is in front
      return 1;
    } else
    {
      return -1; // not in front
    }
  }
}

void Pivoter::Front::removeEdge (const uint32_t id0, const uint32_t id1)
{
  if (!_front.empty ())
  {
    std::map<Signature, Edge>::iterator loc = _front.find (Signature (id0, id1));
    if (loc != _front.end ())
    {
      _front.erase (loc);
    } else // comment to delete one-directionally
    {
      loc = _front.find (Signature (id1, id0));
      if (loc != _front.end ())
      {
        _front.erase (loc);
      }
    }
  }
}

void Pivoter::Front::clear ()
{
  _front.clear ();
  _boundary.clear ();
  _currentEdge.reset ();
}

void Pivoter::Front::prepareDirtyFix (std::vector<bool> &isUsed)
{
  std::fill (isUsed.begin (), isUsed.end (), true);
  for (std::map<Signature, Edge>::iterator it = _boundary.begin (); it != _boundary.end (); ++it)
  {
    isUsed.at (it->first.first) = false;
    isUsed.at (it->first.second) = false;
  }

  _boundary.swap (_front);
  _boundary.clear ();
}

bool Pivoter::Front::isEdgeFinished (const Edge &edge)
{
  if (_finished.find (edge.getSignature ()) != _finished.end () ||
      _finished.find (edge.getSignatureReverse ()) != _finished.end ())
  {
    return true;
  } else
  {
    return false;
  }
}

#endif
