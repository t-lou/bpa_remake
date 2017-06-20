#ifndef MLS_APPLIER_H
#define MLS_APPLIER_H

#include <pcl/common/common.h>
#include <pcl/point_types.h>

class MlsApplier
{
protected:

public:
  MlsApplier();
  ~MlsApplier();

  template<class T>
  static
  pcl::PointCloud<pcl::PointNormal>::Ptr
  smooth(const typename pcl::PointCloud<T>::ConstPtr& cloud,
         const double radius, const bool isComputeNormal);
};

#endif

