#include <actionlib/server/simple_action_server.h>

#include <matchmaker/client.h>
#include <auv_msgs/IdentifyObjectAction.h>

#include <vision_msgs/DetectionArray.h>

namespace srv_trident_services
{

/**
 * \brief Service for object identification.
 * \author Stephan Wirth
 * This service offers identification of an object.
 * The ROS interface is an action server that offers the execution of an
 * auv_msgs::IdentifyObjectAction.
 * When the execution of an action is requested, this process subscribes
 * to messages from object detection. 
 * The action can be executed in two modes: single shot and continuous.
 * In single shot mode, the action is stopped after the first object
 * detection. The detection result is given to clients via an action
 * result message.
 * In continuous mode, the action will not stop until preempted by the
 * client. Detection results are given to clients via action feedback
 * messages.
 */

class ObjectIdentification
{

public:

  ObjectIdentification() : nh_(), nh_priv_("~")
  {
    std::string action_name;
    nh_priv_.param<std::string>("action_name", action_name, "identify_object");
    std::string soa_service_name;
    nh_priv_.param<std::string>("soa_service_name", soa_service_name, "Object Identification");

    // create the action server
    bool auto_start(false);
    action_server_.reset(
        new actionlib::SimpleActionServer<auv_msgs::IdentifyObjectAction>(
          nh_, action_name, auto_start));
    action_server_->registerGoalCallback(
        boost::bind(&ObjectIdentification::goalCallback, this));
    action_server_->registerPreemptCallback(
        boost::bind(&ObjectIdentification::preemptCallback, this));
    action_server_->start();

    waitForPublisher();

    // advertise the service
    mm_service_ad_ = 
      mm_client_.advertiseService(soa_service_name, action_name);

    mm_service_ad_.setStatus(matchmaker::ServiceStatus::AVAILABLE);
    ROS_INFO_STREAM("SOA service \"" << soa_service_name << "\" advertised.");
  }

  void waitForPublisher()
  {
    subscribe();
    ROS_INFO_STREAM("Waiting for publisher of " << nh_.resolveName("detections") << "...");
    while(sub_detections_.getNumPublishers() == 0)
    {
      ros::Duration(1.0).sleep();
    }
    ROS_INFO_STREAM("Publisher found!");
    unsubscribe();
  }

  void subscribe()
  {
    ROS_INFO_STREAM("Subscribing to " << nh_.resolveName("detections") << ".");
    sub_detections_ = 
      nh_.subscribe("detections", 1, 
        &ObjectIdentification::detectionCallback, this);
  }

  void unsubscribe()
  {
    ROS_INFO_STREAM("Unsubscribing from " << nh_.resolveName("detections") << ".");
    sub_detections_.shutdown();
  }

  void goalCallback()
  {
    ROS_INFO("New goal received.");
    subscribe();
    single_shot_ = action_server_->acceptNewGoal()->single_shot;
  }

  void preemptCallback()
  {
    ROS_INFO("Action preempted.");
    unsubscribe();
    action_server_->setPreempted();
  }

  void detectionCallback(const vision_msgs::DetectionArrayConstPtr& detections_msg)
  {
    if (action_server_->isPreemptRequested())
    {
      preemptCallback();
      return;
    }

    if (single_shot_)
    {
      unsubscribe();
      auv_msgs::IdentifyObjectResult result_msg;
      result_msg.header = detections_msg->header;
      getObjectPose(*detections_msg, result_msg.object_relative_to_camera,
          result_msg.confidence);
      action_server_->setSucceeded(result_msg);
    }
    else
    {
      auv_msgs::IdentifyObjectFeedback feedback_msg;
      feedback_msg.header = detections_msg->header;
      getObjectPose(*detections_msg, feedback_msg.object_relative_to_camera,
          feedback_msg.confidence);
      action_server_->publishFeedback(feedback_msg);
    }
  }

  bool getObjectPose(const vision_msgs::DetectionArray& detections_msg, 
      geometry_msgs::PoseWithCovariance& object_pose_msg, float& confidence)
  {
    confidence = 0.0;
    if (detections_msg.detections.size() == 0)
    {
      return false;
    }
    // find the object of interest with highest confidence,
    bool confident_detection_found = false;
    for (size_t i = 0; i < detections_msg.detections.size(); ++i)
    {
      if (detections_msg.detections[i].score > confidence)
      {
        confidence = detections_msg.detections[i].score;
        object_pose_msg.pose.position.x = detections_msg.detections[i].image_pose.x;
        object_pose_msg.pose.position.y = detections_msg.detections[i].image_pose.y;
        object_pose_msg.pose.position.z = detections_msg.detections[i].scale;
        confident_detection_found = true;
      }
    }
    return confident_detection_found;
  }

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  matchmaker::Client mm_client_;
  matchmaker::ServiceAdvertiser mm_service_ad_;
  boost::shared_ptr<actionlib::SimpleActionServer<auv_msgs::IdentifyObjectAction> > action_server_;
  ros::Subscriber sub_detections_;
  bool single_shot_;
};


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_identification");
  srv_trident_services::ObjectIdentification node;
  ros::spin();
  return 0;
}

