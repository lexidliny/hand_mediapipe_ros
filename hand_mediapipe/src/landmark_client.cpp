#include "ros/ros.h"
#include "hand_mediapipe/LandmarkPoint.h"
#include <vector>
#include <geometry_msgs/Point.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "landmark_client_node");
  ros::NodeHandle n;


  ros::ServiceClient client = n.serviceClient<hand_mediapipe::LandmarkPoint>("landmark_server");
  hand_mediapipe::LandmarkPoint srv;

  std::vector<geometry_msgs::Point> landmark_points;

  if (client.call(srv))
  {
    landmark_points = srv.response.LandmarkPoints;
  }
  else
  {
    ROS_ERROR("Failed to call service landmark_server");
    return 1;
  }

    std::cout << landmark_points.size() << std::endl;
  for(auto landmark_point : landmark_points)
  {
    std::cout << landmark_point << std::endl;
  }

  return 0;
}