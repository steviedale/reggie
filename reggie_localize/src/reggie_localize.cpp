#include <reggie_localize/reggie_localize.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/ply_io.h>

#include <tf2_eigen/tf2_eigen.h>

#include <iostream>


float percent_red(pcl::PointXYZRGB p)
{
  return (float)p.r / ((float)p.r + (float)p.g + (float)p.b);
}

float percent_green(pcl::PointXYZRGB p)
{
  return (float)p.g / ((float)p.r + (float)p.g + (float)p.b);
}

float alpha(pcl::PointXYZRGB p)
{
  return ((float)p.r + (float)p.g + (float)p.b) / (255.0 * 3.0);
}

ReggieLocalize::ReggieLocalize()
: nh_()
, camera_topic_("/camera/depth_registered/points")
, cleaned_map_pub_(nh_.advertise<sensor_msgs::PointCloud2>("cleaned_map", 1))
, raw_map_pub_(nh_.advertise<sensor_msgs::PointCloud2>("raw_map", 1))
, green_map_marker_pub_(nh_.advertise<sensor_msgs::PointCloud2>("green_map_marker", 1))
, yellow_map_marker_pub_(nh_.advertise<sensor_msgs::PointCloud2>("yellow_map_marker", 1))
, blue_map_marker_pub_(nh_.advertise<sensor_msgs::PointCloud2>("blue_map_marker", 1))
, blue_robot_marker_pub_(nh_.advertise<sensor_msgs::PointCloud2>("blue_robot_marker", 1))
, green_robot_marker_pub_(nh_.advertise<sensor_msgs::PointCloud2>("green_robot_marker", 1))
, no_map_markers_pub_(nh_.advertise<sensor_msgs::PointCloud2>("no_map_markers", 1))
{
  update_robot_pose_srv_ = nh_.advertiseService("update_robot_pose", &ReggieLocalize::update_robot_pose, this);
  ros::Duration(5.0).sleep();
  init_plane_normal();
  init_map_frame();

  /*
  while (true)
  {
    update_robot_pose();
    ros::Duration(0.1).sleep();
    std::cout << "location updated" << std::endl;
  }
  */
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocalize::get_point_cloud()
{
  sensor_msgs::PointCloud2::ConstPtr cloud_msg_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(camera_topic_, nh_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg_ptr, *cloud_ptr);
  std::cout << "num points: " << cloud_ptr->size() << std::endl;
  return remove_nan_points(cloud_ptr);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocalize::remove_nan_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  // remove all points with x, y, or z = NaN
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_ptr, *new_cloud_ptr, indices);
  return new_cloud_ptr;
}

void ReggieLocalize::init_plane_normal()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr = get_point_cloud();
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(50);
  seg.setDistanceThreshold(0.05);
  seg.setInputCloud(cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    // TODO: throw and exception here instead
    ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.") ;
    exit(1);
  }

  plane_normal_ = Eigen::Vector3d(coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2));
  ROS_DEBUG_STREAM("Plane Coefficients: " << coefficients->values.at(0) << ", " << coefficients->values.at(1) << ", " << coefficients->values.at(2));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocalize::segment_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, bool set_negative, float distance_thresh)
{
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(50);
  seg.setDistanceThreshold(distance_thresh);
  seg.setInputCloud(cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    // TODO: throw and exception here instead
    ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.") ;
    exit(1);
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud_ptr);
  extract_indices.setIndices(inliers);
  extract_indices.setNegative(set_negative);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract_indices.filter(*new_cloud_ptr);

  return new_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocalize::segment_boundary(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, Boundary boundary, bool set_negative)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud_ptr, *new_cloud_ptr);

   pcl::CropBox<pcl::PointXYZRGB> box_filter;
   box_filter.setMin(Eigen::Vector4f(boundary.min_x, boundary.min_y, boundary.min_z, 0));
   box_filter.setMax(Eigen::Vector4f(boundary.max_x, boundary.max_y, boundary.max_z, 0));
   box_filter.setInputCloud(new_cloud_ptr);
   box_filter.setNegative(set_negative);
   box_filter.filter(*new_cloud_ptr);

  return new_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocalize::filter_color_range(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr,
  float min_pr, float max_pr, float min_pg, float max_pg, float min_pa, float max_pa)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointIndices::Ptr keep_indices(new pcl::PointIndices);

  for (int i = 0; i < cloud_ptr->points.size(); ++i)
  {
    pcl::PointXYZRGB p = cloud_ptr->points.at(i);

    float pr = percent_red(p);
    float pg = percent_green(p);
    float pa = alpha(p);

    if (pr >= min_pr && pr <= max_pr && pg >= min_pg && pg <= max_pg && pa >= min_pa && pa <= max_pa)
    {
      keep_indices->indices.push_back(i);
    }
  }
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud_ptr);
  extract_indices.setIndices(keep_indices);
  extract_indices.setNegative(false);
  extract_indices.filter(*new_cloud_ptr);

  return new_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ReggieLocalize::filter_biggest_cluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  ROS_DEBUG_STREAM("points in cloud: " << cloud_ptr->size());
  // get groups of clusters
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclidean_cluster;
  euclidean_cluster.setInputCloud(cloud_ptr);
  euclidean_cluster.setClusterTolerance(0.005);
  euclidean_cluster.setMinClusterSize(6);
  euclidean_cluster.setMaxClusterSize(1000);
  std::vector<pcl::PointIndices> cluster_groups;
  euclidean_cluster.extract(cluster_groups);

  ROS_DEBUG_STREAM("num_clusters: " << cluster_groups.size());

  // if there are no cluster, throw exception
  if (cluster_groups.size() == 0)
  {
    throw ClusterNotFoundException();
  }

  // find biggest cluster
  pcl::PointIndices::Ptr biggest_cluster_indices;
  int biggest_cluster_size = 0;
  for (int i = 0; i < cluster_groups.size(); ++i)
  {
    int cluster_size = cluster_groups.at(i).indices.size();
    ROS_DEBUG_STREAM("cluster_size: " << cluster_size);
    if (cluster_size > biggest_cluster_size)
    {
      biggest_cluster_size = cluster_size;
      biggest_cluster_indices = boost::make_shared<pcl::PointIndices>(cluster_groups.at(i));
    }
  }

  // filter points from biggest cluster
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
  extract_indices.setInputCloud(cloud_ptr);
  extract_indices.setIndices(biggest_cluster_indices);
  extract_indices.setNegative(false);
  extract_indices.filter(*new_cloud_ptr);

  return new_cloud_ptr;
}

Eigen::Vector3d ReggieLocalize::get_centroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  Eigen::Vector3d centroid;
  for (int i = 0; i < cloud_ptr->size(); ++i)
  {
    pcl::PointXYZRGB p = cloud_ptr->points.at(i);
    centroid[0] += (p.x - centroid[0]) / (i+1);
    centroid[1] += (p.y - centroid[1]) / (i+1);
    centroid[2] += (p.z - centroid[2]) / (i+1);
  }
  return centroid;
}

Boundary ReggieLocalize::create_boundary(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr)
{
  Boundary boundary;

  pcl::PointXYZRGB p0 = cloud_ptr->points.at(0);
  boundary.min_x = p0.x;
  boundary.min_y = p0.y;
  boundary.min_z = p0.z;
  boundary.max_x = p0.x;
  boundary.max_y = p0.y;
  boundary.max_z = p0.z;

  for (int i = 0; i < cloud_ptr->size(); ++i)
  {
     pcl::PointXYZRGB p = cloud_ptr->points.at(i);
     if (p.x < boundary.min_x) boundary.min_x = p.x;
     if (p.y < boundary.min_y) boundary.min_y = p.y;
     if (p.z < boundary.min_z) boundary.min_z = p.z;
     if (p.x > boundary.max_x) boundary.max_x = p.x;
     if (p.y > boundary.max_y) boundary.max_y = p.y;
     if (p.z > boundary.max_z) boundary.max_z = p.z;
  }

  boundary.min_x -= EXCLUSION_BOUNDARY_PADDING;
  boundary.min_y -= EXCLUSION_BOUNDARY_PADDING;
  boundary.min_z -= EXCLUSION_BOUNDARY_PADDING;
  boundary.max_x += EXCLUSION_BOUNDARY_PADDING;
  boundary.max_y += EXCLUSION_BOUNDARY_PADDING;
  boundary.max_z += EXCLUSION_BOUNDARY_PADDING;

  return boundary;
}

void ReggieLocalize::init_map_frame()
{
  float yellow_r_avg, yellow_g_avg, yellow_a_avg, green_r_avg, green_g_avg, green_a_avg, blue_r_avg,
    blue_g_avg, blue_a_avg;
  nh_.getParam("/marker_color_ranges/yellow/r_avg", yellow_r_avg);
  nh_.getParam("/marker_color_ranges/yellow/g_avg", yellow_g_avg);
  nh_.getParam("/marker_color_ranges/yellow/a_avg", yellow_a_avg);
  nh_.getParam("/marker_color_ranges/green/r_avg", green_r_avg);
  nh_.getParam("/marker_color_ranges/green/g_avg", green_g_avg);
  nh_.getParam("/marker_color_ranges/green/a_avg", green_a_avg);
  nh_.getParam("/marker_color_ranges/blue/r_avg", blue_r_avg);
  nh_.getParam("/marker_color_ranges/blue/g_avg", blue_g_avg);
  nh_.getParam("/marker_color_ranges/blue/a_avg", blue_a_avg);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, yellow_marker_cloud_ptr, green_marker_cloud_ptr, blue_marker_cloud_ptr;

  int attempts_left = 5;
  while(true) {
    try {
      cloud_ptr = get_point_cloud();

      cloud_ptr = segment_plane(cloud_ptr, false, 0.02);

      // get yellow cloud patch
      yellow_marker_cloud_ptr = filter_color_range(
        cloud_ptr,
        yellow_r_avg - COLOR_TOLERANCE,
        yellow_r_avg + COLOR_TOLERANCE,
        yellow_g_avg - COLOR_TOLERANCE,
        yellow_g_avg + COLOR_TOLERANCE,
        yellow_a_avg - ALPHA_TOLERANCE,
        yellow_a_avg + ALPHA_TOLERANCE
      );
      yellow_marker_cloud_ptr = filter_biggest_cluster(yellow_marker_cloud_ptr);
      if (yellow_marker_cloud_ptr->size() == 0) { throw EmptyCloudException(); }

      // get green cloud patch
      green_marker_cloud_ptr = filter_color_range(
        cloud_ptr,
        green_r_avg - COLOR_TOLERANCE,
        green_r_avg + COLOR_TOLERANCE,
        green_g_avg - COLOR_TOLERANCE,
        green_g_avg + COLOR_TOLERANCE,
        green_a_avg - ALPHA_TOLERANCE,
        green_a_avg + ALPHA_TOLERANCE
      );
      green_marker_cloud_ptr = filter_biggest_cluster(green_marker_cloud_ptr);
      if (green_marker_cloud_ptr->size() == 0) { throw EmptyCloudException(); }

      // get blue cloud patch
      blue_marker_cloud_ptr = filter_color_range(
        cloud_ptr,
        blue_r_avg - COLOR_TOLERANCE,
        blue_r_avg + COLOR_TOLERANCE,
        blue_g_avg - COLOR_TOLERANCE,
        blue_g_avg + COLOR_TOLERANCE,
        blue_a_avg - ALPHA_TOLERANCE,
        blue_a_avg + ALPHA_TOLERANCE
      );
      blue_marker_cloud_ptr = filter_biggest_cluster(blue_marker_cloud_ptr);
      if (blue_marker_cloud_ptr->size() == 0) { throw EmptyCloudException(); }

      // exit loop
      break;
    } catch (ClusterNotFoundException& e) {
      attempts_left -= 1;
      ROS_WARN_STREAM("Clustering corner markers failed (attempt " << 5 - attempts_left << " of 5)");
      if (attempts_left == 0) {
        ROS_ERROR_STREAM("Could not determine position of corner markers.");
        exit(1);
      }
    } catch (EmptyCloudException& e) {
      attempts_left -= 1;
      ROS_WARN_STREAM("Cloud empty after filtering. (attempt " << 5 - attempts_left << " of 5)");
      if (attempts_left == 0) {
        ROS_ERROR_STREAM("Could not determine position of corner markers.");
        exit(1);
      }
    }
  }
  // add marker boundaries to list
  marker_boundaries_.push_back(create_boundary(yellow_marker_cloud_ptr));
  marker_boundaries_.push_back(create_boundary(green_marker_cloud_ptr));
  marker_boundaries_.push_back(create_boundary(blue_marker_cloud_ptr));
  // calculate marker centroids
  Eigen::Vector3d yellow_marker_centroid = get_centroid(yellow_marker_cloud_ptr);
  Eigen::Vector3d green_marker_centroid = get_centroid(green_marker_cloud_ptr);
  Eigen::Vector3d blue_marker_centroid = get_centroid(blue_marker_cloud_ptr);

  // publish filtered marker clouds
  sensor_msgs::PointCloud2 yellow_cloud_msg;
  pcl::toROSMsg(*yellow_marker_cloud_ptr, yellow_cloud_msg);
  yellow_cloud_msg.header.frame_id = CAMERA_FRAME;
  yellow_map_marker_pub_.publish(yellow_cloud_msg);

  sensor_msgs::PointCloud2 green_cloud_msg;
  pcl::toROSMsg(*green_marker_cloud_ptr, green_cloud_msg);
  green_cloud_msg.header.frame_id = CAMERA_FRAME;
  green_map_marker_pub_.publish(green_cloud_msg);

  sensor_msgs::PointCloud2 blue_cloud_msg;
  pcl::toROSMsg(*blue_marker_cloud_ptr, blue_cloud_msg);
  blue_cloud_msg.header.frame_id = CAMERA_FRAME;
  blue_map_marker_pub_.publish(blue_cloud_msg);

  Eigen::Vector3d y_vector = (yellow_marker_centroid - green_marker_centroid);
  y_vector = -1 * y_vector / y_vector.norm();

  Eigen::Vector3d z_vector = plane_normal_;
  z_vector = -1 * z_vector / z_vector.norm();

  Eigen::Vector3d x_vector = z_vector.cross(y_vector);
  x_vector = -1 * x_vector / x_vector.norm();

  ROS_DEBUG_STREAM("x_vector: " << x_vector[0] << ", " << x_vector[1] << ", " << x_vector[2]);
  ROS_DEBUG_STREAM("y_vector: " << y_vector[0] << ", " << y_vector[1] << ", " << y_vector[2]);
  ROS_DEBUG_STREAM("z_vector: " << z_vector[0] << ", " << z_vector[1] << ", " << z_vector[2]);

  Eigen::Matrix3d rotation;
  rotation(0,0) = x_vector[0]; rotation(1,0) = x_vector[1]; rotation(2,0) = x_vector[2];
  rotation(0,1) = y_vector[0]; rotation(1,1) = y_vector[1]; rotation(2,1) = y_vector[2];
  rotation(0,2) = z_vector[0]; rotation(1,2) = z_vector[1]; rotation(2,2) = z_vector[2];

  camera_to_map_tf_ = Eigen::Isometry3d::Identity();
  camera_to_map_tf_.linear() = rotation;
  camera_to_map_tf_.translation() = yellow_marker_centroid;
  geometry_msgs::TransformStamped transform_msg = tf2::eigenToTransform(camera_to_map_tf_);
  transform_msg.header.frame_id = CAMERA_FRAME;
  transform_msg.child_frame_id = MAP_FRAME;
  static_broadcaster_.sendTransform(transform_msg);

  Eigen::Isometry3d green_tf = Eigen::Isometry3d::Identity();
  green_tf.linear() = rotation;
  green_tf.translation() = green_marker_centroid;
  geometry_msgs::TransformStamped green_tf_msg = tf2::eigenToTransform(green_tf);
  green_tf_msg.header.frame_id = CAMERA_FRAME;
  green_tf_msg.child_frame_id = "green_marker";
  static_broadcaster_.sendTransform(green_tf_msg);

  Eigen::Isometry3d blue_tf = Eigen::Isometry3d::Identity();
  blue_tf.linear() = rotation;
  blue_tf.translation() = blue_marker_centroid;
  geometry_msgs::TransformStamped blue_tf_msg = tf2::eigenToTransform(blue_tf);
  blue_tf_msg.header.frame_id = CAMERA_FRAME;
  blue_tf_msg.child_frame_id = "blue_marker";
  static_broadcaster_.sendTransform(blue_tf_msg);

  //map_x_length_ = 0.635485
  map_x_length_ = (yellow_marker_centroid - blue_marker_centroid).norm();
  map_y_length_ = (yellow_marker_centroid - green_marker_centroid).norm();
}

bool ReggieLocalize::update_robot_pose(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  float blue_r_avg, blue_g_avg, blue_a_avg, green_r_avg, green_g_avg, green_a_avg;
  nh_.getParam("/marker_color_ranges/blue/r_avg", blue_r_avg);
  nh_.getParam("/marker_color_ranges/blue/g_avg", blue_g_avg);
  nh_.getParam("/marker_color_ranges/blue/a_avg", blue_a_avg);
  nh_.getParam("/marker_color_ranges/green/r_avg", green_r_avg);
  nh_.getParam("/marker_color_ranges/green/g_avg", green_g_avg);
  nh_.getParam("/marker_color_ranges/green/a_avg", green_a_avg);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, blue_marker_cloud_ptr, green_marker_cloud_ptr;
  // attempt to get a cloud with valid position markers
  int attempts_left = 5;
  while (true) {
    try {
      cloud_ptr = get_point_cloud();

      // remove map markers
      for (int i = 0; i < marker_boundaries_.size(); ++i) {
        cloud_ptr = segment_boundary(cloud_ptr, marker_boundaries_.at(i), true);
      }

      sensor_msgs::PointCloud2 no_map_markers_cloud_msg;
      pcl::toROSMsg(*cloud_ptr, no_map_markers_cloud_msg);
      no_map_markers_cloud_msg.header.frame_id = CAMERA_FRAME;
      no_map_markers_pub_.publish(no_map_markers_cloud_msg);

      // get blue robot marker cloud
      blue_marker_cloud_ptr = filter_color_range(cloud_ptr,
        blue_r_avg - COLOR_TOLERANCE,
        blue_r_avg + COLOR_TOLERANCE,
        blue_g_avg - COLOR_TOLERANCE,
        blue_g_avg + COLOR_TOLERANCE,
        blue_a_avg - ALPHA_TOLERANCE,
        blue_a_avg + ALPHA_TOLERANCE
      );
      blue_marker_cloud_ptr = filter_biggest_cluster(blue_marker_cloud_ptr);
      if (blue_marker_cloud_ptr->size() == 0) { throw EmptyCloudException(); }

      // get green robot marker cloud
      green_marker_cloud_ptr = filter_color_range(
        cloud_ptr,
        green_r_avg - COLOR_TOLERANCE,
        green_r_avg + COLOR_TOLERANCE,
        green_g_avg - COLOR_TOLERANCE,
        green_g_avg + COLOR_TOLERANCE,
        green_a_avg - ALPHA_TOLERANCE,
        green_a_avg + ALPHA_TOLERANCE
      );
      green_marker_cloud_ptr = filter_biggest_cluster(green_marker_cloud_ptr);
      if (green_marker_cloud_ptr->size() == 0) { throw EmptyCloudException(); }

      break;
    } catch (ClusterNotFoundException& e) {
      attempts_left -= 1;
      ROS_WARN_STREAM("Clustering robot position markers failed (attempt " << 5 - attempts_left << " of 5)");
      if (attempts_left == 0)
      {
        ROS_WARN_STREAM("Could not find robot position markers.");
        response.success = false;
      }
    } catch (EmptyCloudException& e) {
      attempts_left -= 1;
      ROS_WARN_STREAM("Cloud empty after filtering. (attempt " << 5 - attempts_left << " of 5)");
      if (attempts_left == 0) {
        ROS_WARN_STREAM("Could not determine position of robot.");
        response.success = false;
        return false;
      }
    }
  }

  sensor_msgs::PointCloud2 blue_cloud_msg;
  pcl::toROSMsg(*blue_marker_cloud_ptr, blue_cloud_msg);
  blue_cloud_msg.header.frame_id = CAMERA_FRAME;
  blue_robot_marker_pub_.publish(blue_cloud_msg);

  sensor_msgs::PointCloud2 green_cloud_msg;
  pcl::toROSMsg(*green_marker_cloud_ptr, green_cloud_msg);
  green_cloud_msg.header.frame_id = CAMERA_FRAME;
  green_robot_marker_pub_.publish(green_cloud_msg);

  Eigen::Vector3d blue_marker_centroid = get_centroid(blue_marker_cloud_ptr);
  Eigen::Vector3d green_marker_centroid = get_centroid(green_marker_cloud_ptr);

  Eigen::Vector3d position = (green_marker_centroid + blue_marker_centroid) / 2;

  Eigen::Vector3d y_vector = (green_marker_centroid - blue_marker_centroid);
  y_vector = y_vector / y_vector.norm();
  Eigen::Vector3d z_vector = plane_normal_;
  z_vector = -1 * z_vector / z_vector.norm();
  Eigen::Vector3d x_vector = y_vector.cross(z_vector);
  x_vector = x_vector / x_vector.norm();

  ROS_DEBUG_STREAM("x_vector: " << x_vector[0] << ", " << x_vector[1] << ", " << x_vector[2]);
  ROS_DEBUG_STREAM("y_vector: " << y_vector[0] << ", " << y_vector[1] << ", " << y_vector[2]);
  ROS_DEBUG_STREAM("z_vector: " << z_vector[0] << ", " << z_vector[1] << ", " << z_vector[2]);

  Eigen::Matrix3d rotation;
  rotation(0,0) = x_vector[0];
  rotation(1,0) = x_vector[1];
  rotation(2,0) = x_vector[2];
  rotation(0,1) = y_vector[0];
  rotation(1,1) = y_vector[1];
  rotation(2,1) = y_vector[2];
  rotation(0,2) = z_vector[0];
  rotation(1,2) = z_vector[1];
  rotation(2,2) = z_vector[2];

  Eigen::Isometry3d camera_to_robot_tf = Eigen::Isometry3d::Identity();
  camera_to_robot_tf.linear() = rotation;
  camera_to_robot_tf.translation() = position;

  Eigen::Isometry3d map_to_robot_tf = camera_to_map_tf_.inverse()  * camera_to_robot_tf;

  // fix robot tf 6cm off of ground
  map_to_robot_tf.translation().z() = 0.06;

  geometry_msgs::TransformStamped transform_msg = tf2::eigenToTransform(map_to_robot_tf);
  transform_msg.header.frame_id = MAP_FRAME;
  transform_msg.child_frame_id = ROBOT_FRAME;
  dynamic_broadcaster_.sendTransform(transform_msg);

  response.success = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reggie_localize_node");
  ReggieLocalize reggie_localize;
  ros::spin();
}
