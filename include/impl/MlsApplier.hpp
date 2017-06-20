#ifndef MLS_APPLIER_HPP
#define MLS_APPLIER_HPP

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "MlsApplier.h"

template<class T>
pcl::PointCloud<pcl::PointNormal>::Ptr
MlsApplier::smooth(const typename pcl::PointCloud<T>::ConstPtr& cloud,
                   const double radius, const bool isComputeNormal)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_out(
        new pcl::PointCloud<pcl::PointNormal>());
  typename pcl::MovingLeastSquares<T, pcl::PointNormal> mls;
  typename pcl::search::KdTree<T>::Ptr tree(
        new pcl::search::KdTree<T>());

  mls.setInputCloud(cloud);
  mls.setComputeNormals(isComputeNormal);
  mls.setPolynomialFit(true);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(radius);
  mls.process(*cloud_out);
  return cloud_out;
}

#endif
