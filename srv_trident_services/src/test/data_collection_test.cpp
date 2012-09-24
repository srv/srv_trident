#include <gtest/gtest.h>

#include <matchmaker/client.h>
#include <actionlib/client/simple_action_client.h>

#include <auv_msgs/CollectDataAction.h>

TEST(CollectDataActionTest, runTest)
{
  // remove temporary bagfile if exists
  int res = system("rm -f /tmp/collect_data_test.bag");
  ASSERT_EQ(res, 0);

  matchmaker::Client mm_client;
  matchmaker::ServiceDetailsPtr service;
  do
  {
    ROS_INFO("Looking for service \"Data Collection\"");
    service = mm_client.findService("Data Collection");
    sleep(1);
  } while (service.get() == 0);
  EXPECT_TRUE(service->status.hasValue(matchmaker::ServiceStatus::AVAILABLE));
  std::cout << "Service status: " << service->status.toString() << std::endl;

  ROS_INFO_STREAM("Found usable service.");

  bool spin_thread = true;
  std::string action_server_name = service->topic;
  actionlib::SimpleActionClient<auv_msgs::CollectDataAction> action_client(action_server_name, spin_thread);

  ROS_INFO_STREAM("Waiting for action server \"" << action_server_name 
      << "\" for 5 sec.");

  bool action_server_found = action_client.waitForServer(ros::Duration(5));
  ASSERT_TRUE(action_server_found);

  ROS_INFO("Action server found. Sending goal.");
  auv_msgs::CollectDataGoal goal;
  action_client.sendGoal(goal);
  std::cout << "Service status: " << service->status.toString() << std::endl;
  ROS_INFO("Goal sent. Sleeping for 5 sec.");
  ros::Time start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < 5)
  {
    ros::spinOnce();
    std::cout << "Service status: " << service->status.toString() << std::endl;
    std::cout << "getServiceStatus: " << mm_client.getServiceStatus(service->id).toString();
  }
  EXPECT_TRUE(service->status.hasValue(matchmaker::ServiceStatus::ACTIVE_BUSY));
  ROS_INFO("Stopping action.");
  std::cout << "Service status: " << service->status.toString() << std::endl;

  action_client.cancelAllGoals();
  // action should be preempted in less than 3 secs.
  start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < 3)
  {
    ros::spinOnce();
  }
  EXPECT_TRUE(service->status.hasValue(matchmaker::ServiceStatus::AVAILABLE));

  int status = system("test -s /tmp/collect_data_test.bag");
  EXPECT_EQ(status, 0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "data_collection_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}

