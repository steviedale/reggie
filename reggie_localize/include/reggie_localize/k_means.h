#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <vector>


struct XYZRGB
{
  float x;
  float y;
  float z;
  float r;
  float g;
  float b;
};


struct Boundary
{
  XYZRGB min;
  XYZRGB max;  
};


Boundary get_boundary(pcl::PointCloud<pcl::PointXYZRGB> cloud_ptr);

float get_l2_distance(Boundary boundary, XYZRGB p1, XYZRGB p2);

XYZRGB get_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, pcl::PointIndices indices);

std::vector<Boundary> k_means_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, int k);
