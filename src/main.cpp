#include <pcl/PolygonMesh.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "impl/Pivoter.hpp"

/**
 * checks whether filename refers to a valid file
 * @param filename
 * @return
 */
bool is_file_exist(const std::string &filename) {
  struct stat buffer;
  return (stat(filename.c_str(), &buffer) == 0);
}

/**
 * Smooths the point cloud and compute the normal of points.
 * @tparam T
 * @param cloud input point cloud
 * @param radius radius for searching neighboring points
 * @param is_compute_normal whether to computer the normal vectors
 * @return
 */
template <typename T, typename NT>
typename pcl::PointCloud<NT>::Ptr smooth(
    const typename pcl::PointCloud<T>::ConstPtr &cloud, const double radius,
    const bool is_compute_normal) {
  typename pcl::PointCloud<NT>::Ptr cloud_out(new pcl::PointCloud<NT>());
  typename pcl::MovingLeastSquares<T, NT> mls;
  typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());

  mls.setInputCloud(cloud);
  mls.setComputeNormals(is_compute_normal);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(radius);
  mls.process(*cloud_out);
  return cloud_out;
}

/**
 * checks whether normal vector is contained in point cloud 2
 * @param cloud2
 * @return
 */
bool is_normal_in_field(const pcl::PCLPointCloud2::ConstPtr &cloud2) {
  // properties for normal vectors should contain substring "normal_"
  const std::string target = "normal";
  for (size_t id = 0; id < cloud2->fields.size(); ++id) {
    if (cloud2->fields.at(id).name.find(target) != std::string::npos) {
      return true;
    }
  }
  return false;
}

/**
 * returns a point cloud containing only non-nan points in in
 * @tparam T
 * @param in
 * @return
 */
template <class T>
typename pcl::PointCloud<T>::Ptr filter_nan(
    const typename pcl::PointCloud<T>::ConstPtr &in) {
  typename pcl::PointCloud<T>::Ptr out(new typename pcl::PointCloud<T>());
  out->reserve(in->size());
  for (size_t id = 0; id < in->size(); ++id) {
    const T &pt = in->at(id);
    if (!isnan(pt.x) && !isnan(pt.y) && !isnan(pt.z)) {
      out->push_back(pt);
    }
  }
  return out;
}

/**
 * Read the point cloud in file as pcl::PCLPointCloud2
 * @param filename
 * @return
 */
pcl::PCLPointCloud2::Ptr load_pointcloud2(const std::string &filename) {
  pcl::PCLPointCloud2::Ptr cloud;
  if (filename.find(".ply") != std::string::npos) {
    cloud = std::make_shared<pcl::PCLPointCloud2>(pcl::PCLPointCloud2());
    pcl::PLYReader reader;
    reader.read(filename, *cloud);
  } else if (filename.find(".pcd") != std::string::npos) {
    cloud = std::make_shared<pcl::PCLPointCloud2>(pcl::PCLPointCloud2());
    pcl::PCDReader reader;
    reader.read(filename, *cloud);
  }
  return cloud;
}

/**
 * Load the point cloud in filename.
 * @param filename
 * @param radius_mls
 * @return
 */
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr load_cloud(
    const std::string &filename, const double radius_mls) {
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr re;
  pcl::PCLPointCloud2::Ptr cloud2 = load_pointcloud2(filename);
  if (is_normal_in_field(cloud2)) {
    re = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::fromPCLPointCloud2<pcl::PointXYZRGBNormal>(*cloud2, *re);
    re = filter_nan<pcl::PointXYZRGBNormal>(re);
  } else {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromPCLPointCloud2<pcl::PointXYZRGB>(*cloud2, *cloud_xyz);
    cloud_xyz = filter_nan<pcl::PointXYZRGB>(cloud_xyz);
    re = smooth<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(cloud_xyz, radius_mls,
                                                          true);
  }
  return re;
}

int main(int argc, char **argv) {
  if (argc == 1) {
    std::cout << "./bpa_remake filename_input filename_output radius_for_BPA "
                 "(radius_for_MLS)"
              << std::endl;
    return 1;
  }

  const std::string fn_in = argv[1];
  const std::string fn_out = argv[2];
  const double radius = atof(argv[3]);
  const double radius_mls = argc >= 5 ? atof(argv[4]) : radius * 2.0f;

  if (!is_file_exist(fn_in)) {
    std::cout << fn_in << " does not exist" << std::endl;
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_in =
      load_cloud(fn_in, radius_mls);
  Pivoter<pcl::PointXYZRGBNormal> pivoter;

  pivoter.setSearchRadius(radius);
  pivoter.setInputCloud(cloud_in);
  // pivoter.setEstimatedRadius (500, 5, 0.99f);
  pcl::PolygonMesh::Ptr mesh = pivoter.proceed();
  pcl::io::savePLYFileBinary(fn_out, *mesh);
  return 0;
}
