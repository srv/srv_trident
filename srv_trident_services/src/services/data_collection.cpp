#include <signal.h>
#include <actionlib/server/simple_action_server.h>

#include <auv_msgs/CollectDataAction.h>

#include <matchmaker/client.h>


pid_t fork_process(const char *command)
{
  pid_t pid;

  pid = fork();

  if (pid < 0)
    return pid;
  else if (pid == 0)
  {
    execlp("roslaunch", "roslaunch", command, NULL);
    perror("execl");
    exit(1);
  }
  return pid;
}


namespace srv_trident_services
{

/**
 * \brief Service for data collection
 * \author Stephan Wirth
 * This service offers data collection.
 * It is basically a matchmaker interface for rosbag record.
 */
class DataCollection
{

public:

  DataCollection() : nh_(), nh_priv_("~"), rosbag_process_id_(-1)
  {
    // read params
    std::string action_name;
    nh_priv_.param<std::string>("action_name", action_name, "collect_data");
    std::string soa_service_name;
    nh_priv_.param<std::string>("soa_service_name", soa_service_name, "Data Collection");
    nh_priv_.param<std::string>("rosbag_launch", rosbag_launch_, "");

    // create the action server
    bool auto_start(false);
    action_server_.reset(
        new actionlib::SimpleActionServer<auv_msgs::CollectDataAction>(
          nh_, action_name, auto_start));
    action_server_->registerGoalCallback(
        boost::bind(&DataCollection::goalCallback, this));
    action_server_->registerPreemptCallback(
        boost::bind(&DataCollection::preemptCallback, this));
    action_server_->start();

    // advertise the service
    mm_service_ad_ = 
      mm_client_.advertiseService(soa_service_name, action_name);

    ROS_INFO_STREAM("SOA service \"" << soa_service_name << "\" advertised.");
    mm_service_ad_.setStatus(matchmaker::ServiceStatus::AVAILABLE);
  }

  void goalCallback()
  {
    ROS_INFO("New goal received.");
    action_server_->acceptNewGoal();
    if (!action_server_->isPreemptRequested())
    {
      startRecord();
    }
  }

  void preemptCallback()
  {
    ROS_INFO("Action preempted.");
    stopRecord();
    action_server_->setPreempted();
  }

  bool startRecord()
  {
    if (rosbag_process_id_ > 0)
    {
      std::string msg("cannot start record, rosbag process already running!");
      ROS_ERROR("%s", msg.c_str());
      mm_service_ad_.setStatus(matchmaker::ServiceStatus::ERROR, msg);
      action_server_->setAborted(auv_msgs::CollectDataResult(), msg);
      return false;
    }
    std::string command = rosbag_launch_;
    rosbag_process_id_ = fork_process(command.c_str());
    if (rosbag_process_id_ < 0)
    {
      std::string msg("forking rosbag launch failed!");
      ROS_ERROR("%s", msg.c_str());
      mm_service_ad_.setStatus(matchmaker::ServiceStatus::ERROR, msg);
      action_server_->setAborted(auv_msgs::CollectDataResult(), msg);
      return false;
    }
    ROS_INFO("Rosbag record forked with pid %i", rosbag_process_id_);
    ROS_INFO("Setting service status to busy.");
    mm_service_ad_.setStatus(matchmaker::ServiceStatus::ACTIVE_BUSY, "recording");
    ROS_INFO("My service status is %s", mm_service_ad_.getStatus().toString().c_str());
    return true;
  }

  bool stopRecord()
  {
    if (rosbag_process_id_ <= 0)
    {
      std::string msg("cannot stop record, rosbag process is not running!");
      ROS_ERROR("%s", msg.c_str());
      mm_service_ad_.setStatus(matchmaker::ServiceStatus::ERROR, msg);
      return false;
    }
    int result = kill(rosbag_process_id_, SIGINT);
    if (result == -1)
    {
      std::string msg("error closing rosbag process: kill returned -1!");
      ROS_ERROR("%s", msg.c_str());
      mm_service_ad_.setStatus(matchmaker::ServiceStatus::ERROR, msg);
      return false;
    }
    ROS_INFO("Waiting for child process to finish...");
    wait();
    rosbag_process_id_ = -1;
    ROS_INFO("Setting service status to available.");
    mm_service_ad_.setStatus(matchmaker::ServiceStatus::AVAILABLE);
    return true;
  }

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  matchmaker::Client mm_client_;
  matchmaker::ServiceAdvertiser mm_service_ad_;
  boost::shared_ptr<actionlib::SimpleActionServer<auv_msgs::CollectDataAction> > action_server_;
  std::string rosbag_launch_;
  pid_t rosbag_process_id_;

};


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_collection");
  srv_trident_services::DataCollection node;
  ros::spin();
  return 0;
}

