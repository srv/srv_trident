
#include <matchmaker/client.h>

#include <vision_msgs/TrainDetector.h>

namespace srv_trident_services
{

class ObjectCharacterization
{

public:

  ObjectCharacterization() : nh_priv_("~")
  {
    std::string soa_service_name;
    nh_priv_.param<std::string>("soa_service_name", soa_service_name, "Object Characterization");

    std::string topic = nh_.resolveName("train");
    mm_service_ad_ = mm_client_.advertiseService(soa_service_name, topic);

    ROS_INFO("Looking for ROS service on topic %s", topic.c_str());
    service_client_ = nh_.serviceClient<vision_msgs::TrainDetector>("train");

    timer_ = nh_.createTimer(ros::Duration(1.0), &ObjectCharacterization::checkService, this);
  }

  void checkService(const ros::TimerEvent&)
  {
    if (!service_client_.exists())
    {
      ROS_INFO("Waiting for ROS service...");
      mm_service_ad_.setStatus(matchmaker::ServiceStatus::REMOVED);
    }
    else
    {
      mm_service_ad_.setStatus(matchmaker::ServiceStatus::AVAILABLE);
      ROS_INFO("Service \"Object Characterization\" is now available.");
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::ServiceClient service_client_;
  matchmaker::Client mm_client_;
  matchmaker::ServiceAdvertiser mm_service_ad_;
  ros::Timer timer_;
};


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_characterization");
  srv_trident_services::ObjectCharacterization node;
  ros::spin();
  return 0;
}

