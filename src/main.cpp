#include <iostream>
#include <string>
#include <cmath>

#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>

#include "BpaRemake"

bool is_normal_in_field(const pcl::PCLPointCloud2::ConstPtr &cloud2)
{
  const std::string target = "normal";
  for(size_t id = 0; id < cloud2->fields.size(); ++id)
  {
    if(cloud2->fields.at(id).name.find(target) != std::string::npos)
    {
      return true;
    }
  }
  return false;
}

template<class T>
typename pcl::PointCloud<T>::Ptr
filter_nan(const typename pcl::PointCloud<T>::ConstPtr &in)
{
  typename pcl::PointCloud<T>::Ptr out(new typename pcl::PointCloud<T>());
  out->reserve(in->size());
  for(size_t id = 0; id < in->size(); ++id)
  {
    const T &pt = in->at(id);
    if(!isnan(pt.x) && !isnan(pt.y) && !isnan(pt.z))
    {
      out->push_back(pt);
    }
  }
  return out;
}

pcl::PCLPointCloud2::Ptr load_pointcloud2(const std::string &filename)
{
  pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
  pcl::PLYReader reader;
  reader.read(filename, *cloud);
  return cloud;
}

pcl::PointCloud<pcl::PointNormal>::Ptr load_cloud(const std::string &filename,
                                                  const double radius_mls)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr re;
  pcl::PCLPointCloud2::Ptr cloud2 = load_pointcloud2(filename);
  if(is_normal_in_field(cloud2))
  {
    re = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    pcl::fromPCLPointCloud2<pcl::PointNormal>(*cloud2, *re);
    re = filter_nan<pcl::PointNormal>(re);
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2<pcl::PointXYZ>(*cloud2, *cloud_xyz);
    cloud_xyz = filter_nan<pcl::PointXYZ>(cloud_xyz);
    re = MlsApplier::smooth<pcl::PointXYZ>(cloud_xyz, radius_mls, true);
  }
  return re;
}

int main(int argc, char **argv)
{
  const std::string fn_in = argv[1];
  const std::string fn_out = argv[2];
  const double radius = atof(argv[3]);
  const double radius_mls = argc >= 5 ? atof(argv[4]) : radius;

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in = load_cloud(fn_in, radius_mls);
  Pivoter pivoter;

  pcl::PolygonMesh::Ptr mesh = pivoter.proceed(cloud_in, radius);
  pcl::io::savePLYFileBinary(fn_out, *mesh);
  return 0;
}
