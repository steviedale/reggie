#include <ros/ros.h>
#include <reggie_support/EpisodeElement.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <yaml-cpp/yaml.h>

class ReggieEpisode {
public:
  ReggieEpisode();
  bool add_episode_element_cb(reggie_support::EpisodeElement::Request& request, reggie_support::EpisodeElement::Response& response);
  bool write_episode_cb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

private:
  ros::NodeHandle nh_, pnh_;
  ros::ServiceServer add_episode_element_srv_, write_episode_srv_;
  std::string episode_directory_, cloud_directory_;
  YAML::Node yaml_node_;
};