#include <reggie_episode/reggie_episode.h>

#include <ctime>
#include <fstream>
#include <experimental/filesystem>


namespace fs = std::experimental::filesystem;

std::string get_date_time_str()
{
  time_t raw_time;
  struct tm* time_info;
  char buffer[80];

  time(&raw_time);
  time_info = localtime(&raw_time);

  strftime(buffer, sizeof(buffer), "%d-%m-%Y_%H:%M:%S", time_info);
  std::string str(buffer);

  return str;
}


ReggieEpisode::ReggieEpisode()
: nh_()
, pnh_("~")
, add_episode_element_srv_(nh_.advertiseService("add_episode_element_srv", &ReggieEpisode::add_episode_element_cb, this))
, write_episode_srv_(nh_.advertiseService("write_episode_srv", &ReggieEpisode::write_episode_cb, this))
{
  // get root episode directory
  pnh_.getParam("episode_directory", episode_directory_);
  ROS_ERROR_STREAM("episode dir: " << episode_directory_);
  std::string datetime = get_date_time_str();
  episode_directory_ = episode_directory_ + "/" + datetime;
  fs::path episode_path(episode_directory_);
  // create folder for this specific episode
  if (!fs::create_directory(episode_path)) {
    ROS_ERROR_STREAM("Could not create episode directory at " << episode_path);
    exit(1);
  }
  // create cloud directory
  cloud_directory_ = episode_directory_ + "/clouds";
  fs::path cloud_path(cloud_directory_);
  if (!fs::create_directory(cloud_path)) {
    ROS_ERROR_STREAM("Could not create cloud directory at " << cloud_path);
    exit(1);
  }
  yaml_node_["boo"] = "yah"; 
}

bool ReggieEpisode::add_episode_element_cb(reggie_support::EpisodeElement::Request& request, reggie_support::EpisodeElement::Response& response) 
{
  return true;
}

bool ReggieEpisode::write_episode_cb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response){
  std::string yaml_file = episode_directory_ + "/episode.yaml";
  std::ofstream fout;
  fout.open(yaml_file);
  fout << yaml_node_;
  fout.close();

  ROS_INFO_STREAM("Episode file written to " << yaml_file);
  ROS_INFO_STREAM("yaml: " << yaml_node_);

  response.success = true;

  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "reggie_episode_node");
  ReggieEpisode reggie_episode;
  ros::spin();
  return 0;
}