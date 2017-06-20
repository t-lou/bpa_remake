#include <iostream>
#include <string>

#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>

#include "BpaRemake"

template <class T>
typename pcl::PointCloud<T>::Ptr load_ply(const std::string &filename)
{
  pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2());
  typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>());
  pcl::PLYReader reader;
  reader.read(filename, *cloud_in);
  pcl::fromPCLPointCloud2<T>(*cloud_in, *cloud);
  return cloud;
}

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in
      = load_ply<pcl::PointNormal>("/home/tlou/12.2622_-14.3964_1.ply");
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_o
  //    = load_ply<pcl::PointXYZ>("/home/tlou/bun_zipper.ply");
  //pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in
  //    = MlsApplier::smooth<pcl::PointXYZ>(cloud_in_o, 0.01, true);
  //Pivoter pivoter(cloud_in, 0.02);
  Pivoter pivoter;

  pcl::PolygonMesh::Ptr mesh = pivoter.proceed(cloud_in, 0.03, false);
  pcl::io::savePLYFileBinary("/home/tlou/mesh1.ply", *mesh);
  return 0;
}
