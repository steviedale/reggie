#include <cstdlib>
#include <math.h>
#include <reggie_locator/k_means.h>
#include <iostream>


pcl::PointXYZRGB toPCL(XYZRGB p)
{
  pcl::PointXYZRGB pcl_p;

  pcl_p.x = p.x;
  pcl_p.y = p.y;
  pcl_p.z = p.z;
  pcl_p.r = u_char(p.r);
  pcl_p.g = u_char(p.g);
  pcl_p.b = u_char(p.b);

  return pcl_p;
}

XYZRGB fromPCL(pcl::PointXYZRGB pcl_p)
{
  XYZRGB p;

  p.x = pcl_p.x;
  p.y = pcl_p.y;
  p.z = pcl_p.z;
  p.r = (float)pcl_p.r;
  p.g = (float)pcl_p.g;
  p.b = (float)pcl_p.b;

  return p;
}

Boundary get_boundary(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  Boundary boundary;
  boundary.min = fromPCL(cloud_ptr->points.at(0));
  boundary.max = boundary.min;

  for(int i = 0; i < cloud_ptr->size(); ++i)
  {
     XYZRGB p = fromPCL(cloud_ptr->points.at(i));

     if(p.x < boundary.min.x) boundary.min.x = p.x;
     if(p.y < boundary.min.y) boundary.min.y = p.y;
     if(p.z < boundary.min.z) boundary.min.z = p.z;

     if(p.r < boundary.min.r) boundary.min.r = p.r;
     if(p.g < boundary.min.g) boundary.min.g = p.g;
     if(p.b < boundary.min.b) boundary.min.b = p.b;

     if(p.x > boundary.max.x) boundary.max.x = p.x;
     if(p.y > boundary.max.y) boundary.max.y = p.y;
     if(p.z > boundary.max.z) boundary.max.z = p.z;

     if(p.r > boundary.max.r) boundary.max.r = p.r;
     if(p.g > boundary.max.g) boundary.max.g = p.g;
     if(p.b > boundary.max.b) boundary.max.b = p.b;
  }

  return boundary;
}

float get_l2_distance(Boundary boundary, XYZRGB p1, XYZRGB p2)
{
  float dx = (p1.x - p2.x) / (boundary.max.x - boundary.min.x);
  float dy = (p1.y - p2.y) / (boundary.max.y - boundary.min.y);
  float dz = (p1.z - p2.z) / (boundary.max.z - boundary.min.z);
  float dr = (p1.r - p2.r) / (boundary.max.r - boundary.min.r);
  float dg = (p1.g - p2.g) / (boundary.max.g - boundary.min.g);
  float db = (p1.b - p2.b) / (boundary.max.b - boundary.min.b);

  //return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2) + pow(dr, 2) + pow(dg, 2) + pow(db, 2));
  return sqrt(pow(dr, 2) + pow(dg, 2) + pow(db, 2));
}

XYZRGB get_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, pcl::PointIndices indices)
{
  float x_mean = 0.0;
  float y_mean = 0.0;
  float z_mean = 0.0;
  float r_mean = 0.0;
  float g_mean = 0.0;
  float b_mean = 0.0;
  int n = 1;

  for(int i = 0; i < indices.indices.size(); ++i)
  {
    XYZRGB p = fromPCL(cloud_ptr->points.at(indices.indices.at(i))); 
    x_mean += (p.x - x_mean) / n;
    y_mean += (p.y - y_mean) / n;
    z_mean += (p.z - z_mean) / n;
    r_mean += (p.r - r_mean) / n;
    g_mean += (p.g - g_mean) / n;
    b_mean += (p.b - b_mean) / n;
    ++n;
  }

  XYZRGB centroid;
  centroid.x = x_mean;
  centroid.y = y_mean;
  centroid.z = z_mean;
  centroid.r = r_mean;
  centroid.g = g_mean;
  centroid.b = b_mean;

  return centroid;
}

std::vector<Boundary> k_means_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, int k)
{
  // get min and max values
  Boundary boundary = get_boundary(cloud_ptr);

  std::vector<XYZRGB> centroids;
  std::vector<pcl::PointIndices> centroid_indices;
  centroid_indices.resize(k);
  // create random centroids within min and max values
  for(int i = 0; i < k; ++i)
  {
    // use random point from cloud
    centroids.push_back(fromPCL(cloud_ptr->points.at(rand() % cloud_ptr->size())));
  }
  while(true)
  {
    std::vector<pcl::PointIndices> next_centroid_indices;
    next_centroid_indices.resize(k);
    
    // sort each point into the closest centroid group
    for(int i = 0; i < cloud_ptr->points.size(); ++i)
    {
      XYZRGB p = fromPCL(cloud_ptr->points.at(i));
      // find closest centroid to p
      int closest_group_index = 0;
      float closest_distance = get_l2_distance(boundary, p, centroids.at(0));
      for(int j = 1; j < k; ++j)
      {
         float dist = get_l2_distance(boundary, p, centroids.at(j));
         if(dist < closest_distance)
         {
           closest_distance = dist;
           closest_group_index = j;
         }
      }
      // insert point into closest centroid's group
      next_centroid_indices.at(closest_group_index).indices.push_back(i);
    }

    //////// DEBUG /////////
    std::cout << "THE COUNT:" << std::endl;
    for(int i = 0; i < k; ++i)
    {
      std::cout << next_centroid_indices.at(i).indices.size() << std::endl;
    }
    //////// DEBUG /////////

    // if all points have stayed in the same centroid groups, then exit loop
    bool indices_equal = true;
    for(int i = 0; i < k; ++i)
    {
      if(centroid_indices.at(i).indices.size() != next_centroid_indices.at(i).indices.size())
      {
        indices_equal = false;
	break;
      }
      for(int j = 0; j < centroid_indices.at(i).indices.size(); ++j)
      {
        if(centroid_indices.at(i).indices.at(j) != next_centroid_indices.at(i).indices.at(j))
	{
          indices_equal = false;
	  break;
	}
      }
    }
    if(indices_equal) break;

    // recalculate centroids
    for(int i = 0; i < k; ++i)
    {
      centroids[i] = get_centroid(cloud_ptr, next_centroid_indices.at(i));
    }     
    
    centroid_indices = next_centroid_indices;
  }

  // create a boundary box for each cluster
  std::vector<Boundary> cluster_boundaries; 
  for(int i = 0; i < k; ++i)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setIndices(boost::make_shared<const pcl::PointIndices>(centroid_indices.at(i)));
    extract_indices.setInputCloud(cloud_ptr);
    extract_indices.filter(*cluster_cloud_ptr);

    cluster_boundaries.push_back(get_boundary(cluster_cloud_ptr));
  }

    XYZRGB min_p = boundary.min;
    XYZRGB max_p = boundary.max;

    std::cout << "X = [" << min_p.x << ", " << max_p.x << "]" << std::endl;
    std::cout << "Y = [" << min_p.y << ", " << max_p.y << "]" << std::endl;
    std::cout << "Z = [" << min_p.z << ", " << max_p.z << "]" << std::endl;
    std::cout << "R = [" << min_p.r << ", " << max_p.r << "]" << std::endl;
    std::cout << "G = [" << min_p.y << ", " << max_p.g << "]" << std::endl;
    std::cout << "B = [" << min_p.z << ", " << max_p.b << "]" << std::endl << std::endl;
  return cluster_boundaries;
}
