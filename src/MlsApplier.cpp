#include "impl/MlsApplier.hpp"

template
pcl::PointCloud<pcl::PointNormal>::Ptr
MlsApplier::smooth<pcl::PointNormal>(const typename pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud,
                                     const double radius, const bool isComputeNormal);
template
pcl::PointCloud<pcl::PointNormal>::Ptr
MlsApplier::smooth<pcl::PointXYZ>(const typename pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                  const double radius, const bool isComputeNormal);