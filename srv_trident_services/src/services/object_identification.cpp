
#include <matchmaker/client.h>

#include <vision_msgs/DetectionArray.h>

namespace srv_trident_services
{

class ObjectIdentification
{

public:

  ObjectIdentification() : nh_priv_("~")
  {
    std::string soa_service_name;
    nh_priv_.param<std::string>("soa_service_name", soa_service_name, "Object Identification");

    detections_topic_ = nh_.resolveName("detections");
    mm_service_ad_ = mm_client_.advertiseService(soa_service_name, detections_topic_);

    ROS_INFO("Looking for ROS messages on topic %s", detections_topic_.c_str());

    timer_ = nh_.createTimer(ros::Duration(1.0), &ObjectIdentification::checkTopic, this);
  }

  void checkTopic(const ros::TimerEvent&)
  {
    vision_msgs::DetectionArrayConstPtr detections =
      ros::topic::waitForMessage<vision_msgs::DetectionArray>(detections_topic_,
          nh_, ros::Duration(0.5));
    if (!detections)
    {
      ROS_INFO("Waiting for ROS messages...");
      mm_service_ad_.setStatus(matchmaker::ServiceStatus::REMOVED);
    }
    else
    {
      mm_service_ad_.setStatus(matchmaker::ServiceStatus::AVAILABLE);
      ROS_INFO("Service \"Object Identification\" is now available.");
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  matchmaker::Client mm_client_;
  matchmaker::ServiceAdvertiser mm_service_ad_;
  ros::Timer timer_;
  std::string detections_topic_;
};


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_identification");
  srv_trident_services::ObjectIdentification node;
  ros::spin();
  return 0;
}

