#ifndef PIVOTER_HPP
#define PIVOTER_HPP

#include <pcl/filters/random_sample.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "Pivoter.h"

/**
 * @brief get_plane_between returns the plane between two points,
 *        it is perpendicular to v0-v1 and crosses their mid-point
 * @param v0
 * @param v1
 * @return
 */
Eigen::Vector4f get_plane_between(const Eigen::Vector3f &v0,
                                  const Eigen::Vector3f &v1) {
  const Eigen::Vector3f normal = (v1 - v0).normalized();
  Eigen::Vector4f plane;
  // (v1+v0)/2 is on plane
  plane << normal, -(v1 + v0).dot(normal) * 0.5f;
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
bool is_positions_near(const Eigen::Vector3f &pos0, const Eigen::Vector3f &pos1,
                       const Eigen::Vector3f &pos2, const float threshold) {
  return (pos0 - pos1).norm() < threshold || (pos1 - pos2).norm() < threshold ||
         (pos2 - pos0).norm() < threshold;
}

/**
 * @brief get_id_point_in_sphere returns the index of points which are in sphere
 *        with center and radius as given
 * @param kdtree kdtree containing the point cloud
 * @param center
 * @param radius
 * @return
 */
template <typename PointNT>
std::vector<int> get_id_point_in_sphere(const pcl::KdTreeFLANN<PointNT> &kdtree,
                                        const PointNT &center,
                                        const double radius) {
  std::vector<int> indices;
  std::vector<float> sqr_distances;
  kdtree.radiusSearch(center, radius, indices, sqr_distances);
  return indices;
}

/**
 * @brief num_point_in_sphere returns the number of points in sphere with given
 * center and radius
 * @param center
 * @param radius
 * @param kdtree
 * @return
 */
template <typename PointNT>
size_t num_point_in_sphere(const PointNT &center, const float radius,
                           const pcl::KdTreeFLANN<PointNT> &kdtree) {
  return get_id_point_in_sphere(kdtree, center, radius).size();
}

/**
 * @brief vec2pointnormal returns a point with vector as coordinate
 * @param vector
 * @return
 */
template <typename PointNT>
std::shared_ptr<PointNT> vec2pointnormal(const Eigen::Vector3f &vector) {
  std::shared_ptr<PointNT> pt = std::shared_ptr<PointNT>(new PointNT());
  pt->getVector3fMap() = vector;
  return pt;
}

/**
 * checks whether normal is consistent with the normal vectors of points with
 * indexes
 * @tparam PointNT
 * @param normal
 * @param index
 * @param cloud
 * @return
 */
template <typename PointNT>
bool is_normal_consistent(
    const Eigen::Vector3f &normal, const std::vector<uint32_t> &indexes,
    const typename pcl::PointCloud<PointNT>::ConstPtr &cloud) {
  assert(indexes.size() == 3);
  int count_consistent = 0;
  for (size_t id = 0; id < 3; ++id) {
    if (normal.dot(cloud->at(indexes.at(id)).getNormalVector3fMap()) > 0.0f) {
      ++count_consistent;
    }
  }
  return count_consistent >= 2;
}

/**
 * get the center of circle where three point are on
 * @param point0
 * @param point1
 * @param point2
 * @return
 */
Eigen::Vector3f get_circle_center(const Eigen::Vector3f &point0,
                                  const Eigen::Vector3f &point1,
                                  const Eigen::Vector3f &point2) {
  // https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates_from_cross-_and_dot-products
  const Eigen::Vector3f vec2 = point0 - point1;
  const Eigen::Vector3f vec0 = point1 - point2;
  const Eigen::Vector3f vec1 = point2 - point0;
  const float area = vec0.cross(vec1).norm();
  const float determinator = 2.0f * area * area;

  const float alpha = vec0.dot(vec0) * vec2.dot(-vec1) / determinator;
  const float beta = vec1.dot(vec1) * -vec2.dot(vec0) / determinator;
  const float gamma = vec2.dot(vec2) * vec1.dot(-vec0) / determinator;

  return alpha * point0 + beta * point1 + gamma * point2;
}

/**
 * get the normal vector of a triangle, (p1-p0)x(p2-p0)
 * @tparam PointNT
 * @param cloud
 * @param index
 * @return
 */
template <typename PointNT>
Eigen::Vector3f get_normal_triangle(
    const typename pcl::PointCloud<PointNT>::ConstPtr &cloud,
    const std::vector<uint32_t> &indexes) {
  assert(indexes.size() == 3);
  const Eigen::Vector3f p0 = cloud->at(indexes.at(0)).getVector3fMap();
  return (cloud->at(indexes.at(1)).getVector3fMap() - p0)
      .cross(cloud->at(indexes.at(2)).getVector3fMap() - p0)
      .normalized();
}

/**
 * reorder the vertices of triangle so the normal vector of triangle is
 * consistent with normals of vertices
 * @tparam PointNT
 * @param cloud
 * @param triangle
 */
template <typename PointNT>
void reorder(const typename pcl::PointCloud<PointNT>::ConstPtr &cloud,
             pcl::Vertices &triangle) {
  if (!is_normal_consistent(
          get_normal_triangle<PointNT>(cloud, triangle.vertices),
          triangle.vertices, cloud)) {
    // if the order (0,1,2) is not consistent, (0,2,1) should be consistent
    std::swap(triangle.vertices.at(1), triangle.vertices.at(2));
  }
}

/**
 * get the distance from point to plane
 * @tparam PointNT
 * @param plane
 * @param point
 * @return
 */
template <typename PointNT>
float get_distance_point_plane(const Eigen::Vector4f &plane,
                               const PointNT &point) {
  // plane is (a,b,c,d), point is (x,y,z), then distance is ax+by+cz+d
  return point.getVector3fMap().dot(plane.segment(0, 3)) + plane(3);
}

/**
 * get the signed rotation angle from (point0-center) to (point1-center) on
 * plane
 * @param point0
 * @param point1
 * @param center
 * @param plane the rotation is along the normal vector of plane
 * @return
 */
float get_angle_rotation(const Eigen::Vector3f &point0,
                         const Eigen::Vector3f &point1,
                         const Eigen::Vector3f &center,
                         const Eigen::Vector4f &plane) {
  const Eigen::Vector3f vc0 = (point0 - center).normalized();
  const Eigen::Vector3f vc1 = (point1 - center).normalized();

  const float sin_val =
      vc0.cross(-Eigen::Vector3f(plane.segment(0, 3))).dot(vc1);
  const float cos_val = vc0.dot(vc1);
  float angle = atan2(sin_val, cos_val);
  if (angle < 0.0f)  // -pi~pi -> 0~2pi
  {
    angle += (float)(2.0 * M_PI);
  }
  return angle;
}

/**
 * estimate a radius for the ball pivoting algorithm. the returned estimation is
 * the minimal value, so that at least min_success_rate of the sample points
 * have at least num_in_radius neighbors
 * @tparam PointNT
 * @param kdtree
 * @param num_sample_point
 * @param num_in_radius
 * @param min_success_rate
 * @return
 */
template <typename PointNT>
double guess_radius(const pcl::KdTreeFLANN<PointNT> &kdtree,
                    const int num_sample_point = 500,
                    const int num_in_radius = 5,
                    const float min_success_rate = 0.95f) {
  std::vector<float> farthest_distances;
  const typename pcl::PointCloud<PointNT>::ConstPtr &cloud =
      kdtree.getInputCloud();

  // sample num_sample_point points in cloud
  pcl::PointCloud<PointNT> samples;
  pcl::RandomSample<PointNT> sampler;
  sampler.setInputCloud(cloud);
  sampler.setSample(num_sample_point);
  sampler.filter(samples);

  farthest_distances.reserve(num_sample_point);
  for (int id = 0; id < num_sample_point; ++id) {
    std::vector<int> indices;
    std::vector<float> sqr_distances;
    kdtree.nearestKSearch(samples.at(id), num_in_radius, indices,
                          sqr_distances);
    farthest_distances.push_back(sqr_distances.back());
  }

  // ascending summary of num_in_radius-th nearest neighbor
  std::sort(farthest_distances.begin(), farthest_distances.end());

  return sqrt(farthest_distances.at(
      (int)floor(min_success_rate * (float)num_sample_point)));
}

template <typename PointNT>
Pivoter<PointNT>::Pivoter()
    : _radius(-1.0),
      _is_allow_back_ball(false),
      _is_allow_flip(false),
      _threshold_collinear_cos(cos(10.0 * M_PI / 180.0)),
      _threshold_distance_near(1e-6) {}

template <typename PointNT>
Pivoter<PointNT>::~Pivoter() {}

template <typename PointNT>
std::shared_ptr<PointNT> Pivoter<PointNT>::getBallCenter(
    const bool is_back_first, std::vector<uint32_t> &index,
    bool &is_back_ball) const {
  std::shared_ptr<PointNT> center;
  // for checking whether three points are collinear
  const Eigen::Vector3f pos0(_cloud->at(index.at(0)).getVector3fMap());
  const Eigen::Vector3f pos1(_cloud->at(index.at(1)).getVector3fMap());
  const Eigen::Vector3f pos2(_cloud->at(index.at(2)).getVector3fMap());

  const Eigen::Vector3f vec0 = (pos1 - pos0).normalized();
  const Eigen::Vector3f vec1 = (pos2 - pos0).normalized();

  // the three points should not be too near or collinear
  if (!is_positions_near(pos0, pos1, pos2, _threshold_distance_near) &&
      fabs(vec0.dot(vec1)) < _threshold_collinear_cos) {
    Eigen::Vector3f center_circle = get_circle_center(pos0, pos1, pos2);

    // move to distance _radius along normal direction
    float radius_planar = (center_circle - pos0).norm();
    if (radius_planar < _radius) {
      Eigen::Vector3f normal = vec0.cross(vec1).normalized();
      std::shared_ptr<PointNT> center_candidate;
      const float dist_normal =
          sqrt((float)_radius * _radius - radius_planar * radius_planar);
      if (!is_normal_consistent<PointNT>(normal, index, _cloud)) {
        // reorder the vertices of triangle and reverse the normal vector
        normal = -normal;
        std::swap(index.at(0), index.at(2));
      }
      normal *= dist_normal;

      if (is_back_first) {
        center_candidate =
            vec2pointnormal<PointNT>(Eigen::Vector3f(center_circle - normal));
        if (num_point_in_sphere(*center_candidate, _radius, _kdtree) <= 3) {
          center = center_candidate;
          is_back_ball = true;
        } else if (_is_allow_flip) {
          center_candidate =
              vec2pointnormal<PointNT>(Eigen::Vector3f(center_circle + normal));
          if (num_point_in_sphere(*center_candidate, _radius, _kdtree) <= 3) {
            center = center_candidate;
            is_back_ball = false;
          }
        }
      } else {
        center_candidate =
            vec2pointnormal<PointNT>(Eigen::Vector3f(center_circle + normal));
        if (num_point_in_sphere(*center_candidate, _radius, _kdtree) <= 3) {
          center = center_candidate;
          is_back_ball = false;
        } else if (_is_allow_flip) {
          center_candidate =
              vec2pointnormal<PointNT>(Eigen::Vector3f(center_circle - normal));
          if (num_point_in_sphere(*center_candidate, _radius, _kdtree) <= 3) {
            center = center_candidate;
            is_back_ball = true;
          }
        }
      }
    }
  }

  return center;
}

template <typename PointNT>
bool Pivoter<PointNT>::pivot(const Edge &edge, uint32_t &id_extended,
                             PointNT &center_new, bool &is_back_ball) const {
  const uint32_t id0 = edge.getIdVertice(0);
  const uint32_t id1 = edge.getIdVertice(1);
  const uint32_t id_op = edge.getIdOpposite();
  const Eigen::Vector3f center = edge.getCenter().getVector3fMap();
  const Eigen::Vector3f v0 = _cloud->at(id0).getVector3fMap();
  const Eigen::Vector3f v1 = _cloud->at(id1).getVector3fMap();
  const Eigen::Vector3f mid = (v0 + v1) * 0.5f;
  // pivot opposite to normal direction, change direction for angle
  const Eigen::Vector4f plane =
      edge.isBackBall() ? get_plane_between(v0, v1) : get_plane_between(v1, v0);
  pcl::PointCloud<PointNT> center_candidates;
  std::vector<float> dot_candidates;
  std::vector<uint32_t> id_candidates;
  std::vector<bool> is_back_candidates;

  const double search_radius =
      sqrt(_radius * _radius - (v0 - mid).dot(Eigen::Vector3f(v0 - mid))) +
      _radius;
  std::vector<uint32_t> point3(3, 0);
  std::vector<int> indices = get_id_point_in_sphere<PointNT>(
      _kdtree, *vec2pointnormal<PointNT>(mid), search_radius);

  center_candidates.reserve(indices.size());
  dot_candidates.reserve(indices.size());
  id_candidates.reserve(indices.size());
  is_back_candidates.reserve(indices.size());
  for (std::vector<int>::iterator it = indices.begin(); it != indices.end();
       ++it) {
    point3.at(0) = id0;
    point3.at(1) = id1;
    point3.at(2) = (uint32_t)(*it);

    if (point3.at(2) == id0 || point3.at(2) == id1 || point3.at(2) == id_op ||
        !is_normal_consistent<PointNT>(
            get_normal_triangle<PointNT>(_cloud, point3), point3, _cloud) ||
        fabs(get_distance_point_plane<PointNT>(plane, _cloud->at(*it))) >
            _radius) {
      continue;
    }

    // the three points are different, the normal of triangle is consistent to
    // the normal vectors or vertices and the cloud[point3[2]] has distance to
    // plane smaller than radius
    bool is_back_bool;
    std::shared_ptr<PointNT> center_jr =
        getBallCenter(edge.isBackBall(), point3, is_back_bool);

    if (center_jr) {
      center_candidates.push_back(*center_jr);
      dot_candidates.push_back(
          get_angle_rotation(center, center_jr->getVector3fMap(), mid, plane));
      id_candidates.push_back((uint32_t)*it);
      is_back_candidates.push_back(is_back_bool);
    }
  }

  // get the first hit point
  if (!center_candidates.empty()) {
    int id_min = std::distance(
        dot_candidates.begin(),
        std::min_element(dot_candidates.begin(), dot_candidates.end()));
    id_extended = id_candidates.at(id_min);
    center_new = center_candidates.at(id_min);
    is_back_ball = is_back_candidates.at(id_min);
    return true;
  } else {
    return false;
  }
}

template <typename PointNT>
void Pivoter<PointNT>::setInputCloud(
    const typename pcl::PointCloud<PointNT>::ConstPtr &cloud) {
  _cloud = cloud->makeShared();
  prepare();
}

template <typename PointNT>
void Pivoter<PointNT>::prepare() {
  assert(_cloud);
  _kdtree.setInputCloud(_cloud);
  _is_used.clear();
  _is_used.resize(_cloud->size(), false);
  _front = Front();
}

template <typename PointNT>
void Pivoter<PointNT>::setEstimatedRadius(const int num_sample_point,
                                          const int num_point_in_radius,
                                          const float ratio_success) {
  assert(_cloud && _kdtree.getInputCloud());  // kdtree must be filled
  _radius = guess_radius(_kdtree, num_sample_point, num_point_in_radius,
                         ratio_success);
}

template <typename PointNT>
void Pivoter<PointNT>::proceedFront(pcl::PolygonMesh::Ptr &mesh) {
  typename Pivoter::Edge::Ptr edge;
  while (edge = _front.getActiveEdge()) {
    uint32_t id_ext;
    PointNT center_new;
    bool is_back_ball;
    if (!_front.isEdgeFinished(*edge) &&
        pivot(*edge, id_ext, center_new, is_back_ball)) {
      const uint32_t id0 = edge->getIdVertice(0);
      const uint32_t id1 = edge->getIdVertice(1);
      pcl::Vertices triangle;
      // add to mesh
      triangle.vertices.reserve(3);
      triangle.vertices.push_back(id0);
      triangle.vertices.push_back(id1);
      triangle.vertices.push_back(id_ext);
      mesh->polygons.push_back(triangle);

      _is_used.at(id_ext) = true;
      _front.addPoint(*edge, id_ext, center_new, is_back_ball);

      // pivoting succeeds, not boundary
      _front.setFeedback(false);
    } else {
      // not pivoted
      _front.setFeedback(true);
    }
  }
}

template <typename PointNT>
pcl::PolygonMesh::Ptr Pivoter<PointNT>::proceed() {
  assert(_cloud);  // preparation is done in setInputCloud
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());

  // if radius is not valid, guess with default configuration (see default
  // parameters of guess_radius)
  if (_radius < 0.0) {
    _radius = guess_radius(_kdtree);
  }

  // proceed until not seed can bt found
  while (true) {
    proceedFront(mesh);

    pcl::Vertices::Ptr seed;
    PointNT center;
    bool is_back_ball;
    if (findSeed(seed, center, is_back_ball)) {
      // add to mesh
      mesh->polygons.push_back(*seed);
      // add for pivoting
      _front.addTriangle(seed, center, is_back_ball);
    } else {
      // cannot find proper seed
      break;
    }
  }

  pcl::toPCLPointCloud2(*_cloud, mesh->cloud);

  return mesh;
}

template <typename PointNT>
bool Pivoter<PointNT>::findSeed(pcl::Vertices::Ptr &seed, PointNT &center,
                                bool &is_back_ball) {
  const double search_radius = _radius * 2.0;

  // search in all points
  for (size_t id_search = 0; id_search < _is_used.size(); ++id_search) {
    if (_is_used.at(id_search)) {
      // actually ignore used or obsolete points
      continue;
    }

    std::vector<uint32_t> index3(3, 0);
    std::vector<int> indices = get_id_point_in_sphere<PointNT>(
        _kdtree, _cloud->at(id_search), search_radius);
    if (indices.size() < 3) {
      continue;
    }

    for (size_t idn1 = 1; idn1 < indices.size(); ++idn1) {
      const uint32_t index1 = indices.at(idn1);
      if (_is_used.at(index1) || index1 == (uint32_t)id_search) {
        continue;
      }

      for (size_t idn2 = 0; idn2 < idn1; ++idn2) {
        const uint32_t index2 = indices.at(idn2);

        if (_is_used.at(index2) || index1 == index2 ||
            index2 == (uint32_t)id_search) {
          continue;
        }

        index3.at(0) = (uint32_t)id_search;
        index3.at(1) = index1;
        index3.at(2) = index2;

        bool is_back_ball;
        std::shared_ptr<PointNT> center_ =
            getBallCenter(false, index3, is_back_ball);
        if (!center_ && _is_allow_back_ball) {
          center_ = getBallCenter(true, index3, is_back_ball);
        }
        if (center_) {
          seed = pcl::Vertices::Ptr(new pcl::Vertices());
          seed->vertices = index3;
          center = *center_;
          is_back_ball = is_back_ball;
          _is_used.at(index3.at(0)) = true;
          _is_used.at(index3.at(1)) = true;
          _is_used.at(index3.at(2)) = true;

          return true;
        }
      }
    }

    _is_used.at(id_search) = true;
  }
  return false;
}

template <typename PointNT>
Pivoter<PointNT>::Edge::Edge() {}

template <typename PointNT>
Pivoter<PointNT>::Edge::Edge(const uint32_t id0, const uint32_t id1) {
  _id_vertices.resize(2);
  _id_vertices.at(0) = id0;
  _id_vertices.at(1) = id1;
}

template <typename PointNT>
Pivoter<PointNT>::Edge::Edge(const std::vector<uint32_t> &edge,
                             const uint32_t id_opposite, const PointNT &center,
                             const bool is_back_ball)
    : _id_vertices(edge),
      _id_opposite(id_opposite),
      _center(center),
      _is_back_ball(is_back_ball) {}

template <typename PointNT>
Pivoter<PointNT>::Edge::~Edge() {}

template <typename PointNT>
PointNT Pivoter<PointNT>::Edge::getCenter() const {
  return _center;
}

template <typename PointNT>
uint32_t Pivoter<PointNT>::Edge::getIdVertice(const size_t id) const {
  assert(id < _id_vertices.size());
  return _id_vertices.at(id);
}

template <typename PointNT>
uint32_t Pivoter<PointNT>::Edge::getIdOpposite() const {
  return _id_opposite;
}

template <typename PointNT>
void Pivoter<PointNT>::Edge::setIdVertice(const size_t id,
                                          const uint32_t id_vertice) {
  assert(id < _id_vertices.size());
  _id_vertices.at(id) = id_vertice;
}

template <typename PointNT>
bool Pivoter<PointNT>::Edge::isBackBall() const {
  return _is_back_ball;
}

template <typename PointNT>
std::pair<uint32_t, uint32_t> Pivoter<PointNT>::Edge::getSignature() const {
  assert(_id_vertices.size() == 2);
  return std::pair<uint32_t, uint32_t>(_id_vertices.at(0), _id_vertices.at(1));
}

template <typename PointNT>
std::pair<uint32_t, uint32_t> Pivoter<PointNT>::Edge::getSignatureReverse()
    const {
  assert(_id_vertices.size() == 2);
  return std::pair<uint32_t, uint32_t>(_id_vertices.at(1), _id_vertices.at(0));
}

template <typename PointNT>
Pivoter<PointNT>::Front::Front() {
  clear();
}

template <typename PointNT>
Pivoter<PointNT>::Front::~Front() {}

template <typename PointNT>
typename Pivoter<PointNT>::Edge::Ptr Pivoter<PointNT>::Front::getActiveEdge() {
  typename Edge::Ptr re;
  if (!_front.empty()) {
    re = std::make_shared<Edge>(_front.begin()->second);
    _currentEdge = re;
    _front.erase(_front.begin());
  }
  return re;
}

template <typename PointNT>
void Pivoter<PointNT>::Front::setFeedback(const bool is_boundary) {
  if (is_boundary) {
    _boundary[_currentEdge->getSignature()] = *_currentEdge;
  }
  _finished.insert(_currentEdge->getSignature());
}

template <typename PointNT>
void Pivoter<PointNT>::Front::addTriangle(const pcl::Vertices::ConstPtr &seed,
                                          const PointNT &center,
                                          const bool is_back_ball) {
  assert(seed->vertices.size() == 3);
  for (size_t idv = 0; idv < 3; ++idv) {
    std::vector<uint32_t> edge(2, 0);
    edge.at(0) = seed->vertices.at(idv);
    edge.at(1) = seed->vertices.at((idv + 2) % 3);

    addEdge(Edge(edge, seed->vertices.at((idv + 1) % 3), center, is_back_ball));
  }
}

template <typename PointNT>
void Pivoter<PointNT>::Front::addPoint(const Edge &last_edge,
                                       const uint32_t id_vertice_extended,
                                       const PointNT &center,
                                       const bool is_back_ball) {
  std::vector<uint32_t> edge(2, 0);

  edge.at(0) = last_edge.getIdVertice(0);
  edge.at(1) = id_vertice_extended;
  addEdge(Edge(edge, last_edge.getIdVertice(1), center, is_back_ball));

  edge.at(0) = id_vertice_extended;
  edge.at(1) = last_edge.getIdVertice(1);
  addEdge(Edge(edge, last_edge.getIdVertice(0), center, is_back_ball));
}

template <typename PointNT>
void Pivoter<PointNT>::Front::addEdge(const Edge &edge) {
  if (!isEdgeOnFront(edge)) {
    _front[Signature(edge.getIdVertice(0), edge.getIdVertice(1))] = edge;
  }
}

template <typename PointNT>
bool Pivoter<PointNT>::Front::isEdgeOnFront(const Edge &edge) const {
  // toggle comment to delete one-directionally
  return _front.find(edge.getSignature()) != _front.end() ||
         _front.find(edge.getSignatureReverse()) != _front.end();
}

template <typename PointNT>
void Pivoter<PointNT>::Front::removeEdge(const uint32_t id0,
                                         const uint32_t id1) {
  if (!_front.empty()) {
    typename std::map<Signature, Edge>::iterator loc =
        _front.find(Signature(id0, id1));
    if (loc != _front.end()) {
      _front.erase(loc);
    } else  // comment to delete one-directionally
    {
      loc = _front.find(Signature(id1, id0));
      if (loc != _front.end()) {
        _front.erase(loc);
      }
    }
  }
}

template <typename PointNT>
void Pivoter<PointNT>::Front::clear() {
  _front.clear();
  _boundary.clear();
  _currentEdge.reset();
}

template <typename PointNT>
bool Pivoter<PointNT>::Front::isEdgeFinished(const Edge &edge) const {
  return _finished.find(edge.getSignature()) != _finished.end() ||
         _finished.find(edge.getSignatureReverse()) != _finished.end();
}

#endif
