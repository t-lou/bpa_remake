#ifndef PIVOTER_HPP
#define PIVOTER_HPP

#include <algorithm>
#include <cmath>

#include "Pivoter.h"

Pivoter::Pivoter()
{
}

Pivoter::~Pivoter()
{
}

Eigen::Vector4f get_plane_between(const Eigen::Vector3f &v0, const Eigen::Vector3f &v1)
{
  Eigen::Vector4f plane;
  Eigen::Vector3f normal = (v1 - v0);
  normal.normalize();
  plane << normal, -(v1 + v0).dot(normal) * 0.5f;
  return plane;
}

bool is_positions_near(const Eigen::Vector3f &pos0, const Eigen::Vector3f &pos1,
                       const Eigen::Vector3f &pos2, const float threshold)
{
  return (pos0 - pos1).norm() < threshold || (pos1 - pos2).norm() < threshold
      || (pos2 - pos0).norm() < threshold;
}

std::vector<int> get_id_point_in_sphere(const pcl::KdTreeFLANN<pcl::PointNormal> &kdtree,
                                        const pcl::PointNormal &center,
                                        const double radius)
{
  std::vector<int> indices;
  std::vector<float> sqr_distances;
  kdtree.radiusSearch(center, radius, indices, sqr_distances);
  return indices;
}

//size_t num_point_in_sphere(const pcl::PointNormal &center, const float radius,
//                           const pcl::KdTreeFLANN<pcl::PointNormal> &kdtree)
//{
//  std::vector<int> indices;
//  std::vector<float> sqr_distances;
//  kdtree.radiusSearch(center, radius, indices, sqr_distances);
//  return indices.size();
//}

size_t num_point_in_sphere(const pcl::PointNormal &center, const float radius,
                           const pcl::KdTreeFLANN<pcl::PointNormal> &kdtree,
                           const bool is_count_all, const std::vector<bool> &is_used)
{
  std::vector<int> indices = get_id_point_in_sphere(kdtree, center, radius);
  if(is_count_all)
  {
    return indices.size();
  }
  else
  {
    size_t count = 0;
    for(std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it)
    {
      if(!is_used.at(*it))
      {
        ++count;
      }
    }
    return count;
  }
}

boost::shared_ptr<pcl::PointNormal> vec2pointnormal(const Eigen::Vector3f &vector)
{
  boost::shared_ptr<pcl::PointNormal> pt
      = boost::shared_ptr<pcl::PointNormal>(new pcl::PointNormal());
  pt->x = vector(0);
  pt->y = vector(1);
  pt->z = vector(2);
  return pt;
}

bool is_normal_consistent(const Eigen::Vector3f &normal, std::vector<uint32_t> &index,
                          const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud)
{
  assert(index.size() == 3);
  int count_consistent = 0;
  for(size_t id = 0; id < 3; ++id)
  {
    if(normal.dot(cloud->at(index.at(id)).getNormalVector3fMap()) > 0.0f)
    {
      ++count_consistent;
    }
  }
  return count_consistent >= 2;
}

Eigen::Vector3f get_circle_center(const Eigen::Vector3f &pt0, const Eigen::Vector3f &pt1,
                                  const Eigen::Vector3f &pt2)
{
  // code from https://github.com/rodschulz/BPA, get the center
  const Eigen::Vector3f d10 = pt1 - pt0;
  const Eigen::Vector3f d20 = pt2 - pt0;
  const Eigen::Vector3f d01 = pt0 - pt1;
  const Eigen::Vector3f d12 = pt1 - pt2;
  const Eigen::Vector3f d21 = pt2 - pt1;
  const Eigen::Vector3f d02 = pt0 - pt2;

  float norm01 = d01.norm();
  float norm12 = d12.norm();
  float norm02 = d02.norm();

  float norm01C12 = d01.cross(d12).norm();

  float alpha = (norm12 * norm12 * d01.dot(d02)) / (2.0f * norm01C12 * norm01C12);
  float beta = (norm02 * norm02 * d10.dot(d12)) / (2.0f * norm01C12 * norm01C12);
  float gamma = (norm01 * norm01 * d20.dot(d21)) / (2.0f * norm01C12 * norm01C12);

  return alpha * pt0 + beta * pt1 + gamma * pt2;
}

boost::shared_ptr<pcl::PointNormal> get_ball_center(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
                                                    const pcl::KdTreeFLANN<pcl::PointNormal> &kdtree,
                                                    const double radius,
                                                    const bool is_back_first,
                                                    const bool is_count_all,
                                                    const std::vector<bool> &is_used,
                                                    std::vector<uint32_t> &index,
                                                    bool &is_back_ball)
{
  boost::shared_ptr<pcl::PointNormal> center;
  const static float cos_10 = (float)cos(10.0 * M_PI / 180.0);
  const Eigen::Vector3f pos0(cloud->at(index.at(0)).getVector3fMap());
  const Eigen::Vector3f pos1(cloud->at(index.at(1)).getVector3fMap());
  const Eigen::Vector3f pos2(cloud->at(index.at(2)).getVector3fMap());
  const double search_radius = radius;
  const double thres_near = 1e-6;

  Eigen::Vector3f vec0 = pos1 - pos0;
  Eigen::Vector3f vec1 = pos2 - pos0;
  vec0.normalize();
  vec1.normalize();

  if(!is_positions_near(pos0, pos1, pos2, thres_near) && fabs(vec0.dot(vec1)) < cos_10)
  {
    Eigen::Vector3f center_circle = get_circle_center(pos0, pos1, pos2);

    // move to distance _radius
    float radius_planar = (center_circle - pos0).norm();
    if(radius_planar < radius)
    {
      Eigen::Vector3f normal = vec0.cross(vec1);
      boost::shared_ptr<pcl::PointNormal> center_candidate;
      float dist_normal = sqrt(radius * radius - radius_planar * radius_planar);
      normal.normalize();
      if(!is_normal_consistent(normal, index, cloud))
      {
        normal = -normal;
        std::swap(index.at(0), index.at(2));
      }
      normal *= dist_normal;

      if(is_back_first)
      {
        center_candidate = vec2pointnormal(Eigen::Vector3f(center_circle - normal));
        if(num_point_in_sphere(*center_candidate, search_radius,
                               kdtree, is_count_all, is_used) <= 3)
        {
          center = center_candidate;
          is_back_ball = true;
        }
        else
        {
          center_candidate = vec2pointnormal(Eigen::Vector3f(center_circle + normal));
          if(num_point_in_sphere(*center_candidate, search_radius,
                                 kdtree, is_count_all, is_used) <= 3)
          {
            center = center_candidate;
            is_back_ball = false;
          }
        }
      }
      else
      {
        center_candidate = vec2pointnormal(Eigen::Vector3f(center_circle + normal));
        if(num_point_in_sphere(*center_candidate, search_radius,
                               kdtree, is_count_all, is_used) <= 3)
        {
          center = center_candidate;
          is_back_ball = false;
        }
        else
        {
          center_candidate = vec2pointnormal(Eigen::Vector3f(center_circle - normal));
          if(num_point_in_sphere(*center_candidate, search_radius,
                                 kdtree, is_count_all, is_used) <= 3)
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

Eigen::Vector3f get_normal_triangle(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
                                    const std::vector<uint32_t> &index)
{
  assert(index.size() == 3);
  Eigen::Vector3f p0 = cloud->at(index.at(0)).getVector3fMap();
  Eigen::Vector3f normal = (cloud->at(index.at(1)).getVector3fMap() - p0)
      .cross(cloud->at(index.at(2)).getVector3fMap() - p0);
  normal.normalize();
  return normal;
}

void reorder(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
             pcl::Vertices &triangle)
{
  if(!is_normal_consistent(get_normal_triangle(cloud, triangle.vertices),
                           triangle.vertices, cloud))
  {
    std::swap(triangle.vertices.at(1), triangle.vertices.at(2));
  }
}

float get_distance_point_place(const Eigen::Vector4f &plane,
                               const pcl::PointNormal &point)
{
  return plane(3) + point.getVector3fMap().dot(plane.segment(0, 3));
}

float get_angle_rotation(const Eigen::Vector3f &point0, const Eigen::Vector3f &point1,
                         const Eigen::Vector3f &center, const Eigen::Vector4f &plane)
{
  Eigen::Vector3f vc0 = point0 - center;
  Eigen::Vector3f vc1 = point1 - center;
  vc0.normalize();
  vc1.normalize();

  float sin_ = vc0.cross(-Eigen::Vector3f(plane.segment(0, 3))).dot(vc1);
  float cos_ = vc0.dot(vc1);
  float angle = atan2(sin_, cos_);
  if(angle < 0.0f) // -pi~pi -> 0~2pi
  {
    angle += (float)(2.0 * M_PI);
  }
  return angle;
}

bool Pivoter::pivot(const Edge &edge, const bool isCountAll, uint32_t &idExtended,
                    pcl::PointNormal &centerJr, bool &isBackBool) const
{
  const uint32_t id0 = edge.getIdVertice(0);
  const uint32_t id1 = edge.getIdVertice(1);
  const uint32_t id_op = edge.getIdOpposite();
  const Eigen::Vector3f center = edge.getCenter().getVector3fMap();
  const Eigen::Vector3f v0 = _cloud->at(id0).getVector3fMap();
  const Eigen::Vector3f v1 = _cloud->at(id1).getVector3fMap();
  const Eigen::Vector3f mid = (v0 + v1) * 0.5f;
  // pivot opposite to normal direction, change direction for angle
  const Eigen::Vector4f plane = edge.isBackBall() ?
        get_plane_between(v0, v1) : get_plane_between(v1, v0);
  pcl::PointCloud<pcl::PointNormal> center_candidates;
  std::vector<float> dot_candidates;
  std::vector<uint32_t> id_candidates;
  std::vector<bool> is_back_candidates;

  const double search_radius = sqrt(_radius*_radius - (v0-mid).dot(Eigen::Vector3f(v0-mid)))
                               + _radius;
  std::vector<uint32_t> point3(3, 0);
  std::vector<int> indices = get_id_point_in_sphere(_kdtree, *vec2pointnormal(mid), search_radius);

  center_candidates.reserve(indices.size());
  dot_candidates.reserve(indices.size());
  id_candidates.reserve(indices.size());
  is_back_candidates.reserve(indices.size());
  for(std::vector<int>::iterator it = indices.begin(); it != indices.end(); ++it)
  {
    point3.at(0) = id0;
    point3.at(1) = id1;
    point3.at(2) = (uint32_t)(*it);

    if(point3.at(2) == id0 || point3.at(2) == id1 || point3.at(2) == id_op
       || !is_normal_consistent(get_normal_triangle(_cloud, point3), point3, _cloud)
       || fabs(get_distance_point_place(plane, _cloud->at(*it))) > _radius)
    {
      continue;
    }

//    if(!_is_used.at(*it) || idInvolvmentFront >= 0)
    {
      bool is_back_bool;
      boost::shared_ptr<pcl::PointNormal> center_jr
          = get_ball_center(_cloud, _kdtree, _radius, edge.isBackBall(),
                            isCountAll, _is_used, point3, is_back_bool);

      if(center_jr)
      {
        center_candidates.push_back(*center_jr);
        dot_candidates.push_back(get_angle_rotation(center, center_jr->getVector3fMap(),
                                                    mid, plane));
        id_candidates.push_back((uint32_t)*it);
        is_back_candidates.push_back(is_back_bool);
      }
    }
  }
  if(!center_candidates.empty())
  {
    int id_min = std::distance(dot_candidates.begin(),
                               std::min_element(dot_candidates.begin(), dot_candidates.end()));
    idExtended = id_candidates.at(id_min);
    centerJr = center_candidates.at(id_min);
    isBackBool = is_back_candidates.at(id_min);
    return true;
  }
  else
  {
    return false;
  }
}

void Pivoter::prepare(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
                      const double radius)
{
  _radius = radius;
  _cloud = cloud->makeShared();

  _kdtree.setInputCloud(cloud);
  _is_used.clear();
  _is_used.resize(cloud->size(), false);

  _front = Front();
}

void Pivoter::proceedFront(const bool isCountAll, pcl::PolygonMesh::Ptr &mesh)
{
  Edge::Ptr edge;
  while(edge = _front.getActiveEdge())
  {
    uint32_t id_ext;
    pcl::PointNormal center_new;
    bool is_back_ball;
    if(!_front.isEdgeFinished(*edge)
       && pivot(*edge, isCountAll, id_ext, center_new, is_back_ball))
    {
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
    } // if(pivot(*edge, id_ext, center_new, is_back_ball))
    else
    {
      // not pivoted
      _front.setFeedback(true);
    }
  }
}

pcl::PolygonMesh::Ptr Pivoter::proceed(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud,
                                       const double radius)
{
  assert(cloud);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh());

  prepare(cloud, radius);

  while(true)
  {
    proceedFront(true, mesh);

    pcl::Vertices::Ptr seed;
    pcl::PointNormal center;
    bool is_back_ball;
    if(findSeed(seed, center, is_back_ball))
    {
      // add to mesh
      mesh->polygons.push_back(*seed);
      // add for pivoting
      _front.addTriangle(seed, center, _radius, is_back_ball);
    }
    else
    {
      // cannot find proper seed
      break;
    }
  }

  pcl::toPCLPointCloud2(*_cloud, mesh->cloud);

  return mesh;
}

bool Pivoter::findSeed(pcl::Vertices::Ptr &seed, pcl::PointNormal &center,
                       bool &isBackBall)
{
  const double search_radius = _radius * 2.0;

  for(size_t id_search = 0; id_search < _is_used.size(); ++id_search)
  {
    if(_is_used.at(id_search))
    {
      continue;
    }

    std::vector<uint32_t> index3(3, 0);
    std::vector<int> indices = get_id_point_in_sphere(_kdtree, _cloud->at(id_search), search_radius);
    if(indices.size() < 3)
    {
      continue;
    }

    for(size_t idn1 = 1; idn1 < indices.size(); ++idn1)
    {
      const uint32_t index1 = indices.at(idn1);
      if(_is_used.at(index1) || index1 == (uint32_t)id_search)
      {
        continue;
      }

      for(size_t idn2 = 0; idn2 < idn1; ++idn2)
      {
        const uint32_t index2 = indices.at(idn2);

        if(_is_used.at(index2) || index1 == index2 || index2 == (uint32_t)id_search)
        {
          continue;
        }

        index3.at(0) = (uint32_t)id_search;
        index3.at(1) = index1;
        index3.at(2) = index2;

        bool is_back_ball;
        boost::shared_ptr<pcl::PointNormal> center_
            = get_ball_center(_cloud, _kdtree, _radius, false,
                              true, _is_used, index3, is_back_ball);
        if(center_)
        {
          seed = pcl::Vertices::Ptr(new pcl::Vertices());
          seed->vertices = index3;
          center = *center_;
          isBackBall = is_back_ball;
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

size_t Pivoter::getNumUsedPoint() const
{
  size_t count = 0;
  for(std::vector<bool>::const_iterator it = _is_used.begin(); it != _is_used.end(); ++it)
  {
    if(*it)
    {
      ++count;
    }
  }
  return count;
}

#endif
