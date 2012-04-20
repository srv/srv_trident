#include <actionlib/server/simple_action_server.h>

#include <matchmaker/client.h>
#include <auv_msgs/CharacterizeObjectAction.h>

namespace srv_trident_services
{

class ObjectCharacterization
{

public:

  ObjectCharacterization()
  {
    std::string action_name;
    nh_.param<std::string>("action_name", action_name, "characterize_object");

    // create the action server
    bool auto_start(false);
    action_server_.reset(
        new actionlib::SimpleActionServer<auv_msgs::CharacterizeObjectAction>(
          nh_, action_name, 
          boost::bind(&ObjectCharacterization::actionCallback, this, _1), 
          auto_start));

    // advertize the service
    mm_service_ad_ = mm_client_.advertiseService("Object Characterization", "characterize_object");

    // connect to training action server and image server
    // on success, set
    mm_service_ad_.setStatus(matchmaker::ServiceStatus::AVAILABLE);
    ROS_INFO_STREAM("Service \"Object Characterization\" is now available.");
  }

  void actionCallback(const auv_msgs::CharacterizeObjectGoalConstPtr& goal)
  {
    // unpack goal

    // perform training and give feedback
    // 1. retrieve images
    // 2. create masks
    // 3. send training data

    // return training result

    auv_msgs::CharacterizeObjectResult result;
    action_server_->setSucceeded(result);
  }


private:
  ros::NodeHandle nh_;
  matchmaker::Client mm_client_;
  matchmaker::ServiceAdvertiser mm_service_ad_;

  boost::shared_ptr<actionlib::SimpleActionServer<auv_msgs::CharacterizeObjectAction> > action_server_;

};


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_characterization");
  srv_trident_services::ObjectCharacterization node;
  ros::spin();
  return 0;
}

