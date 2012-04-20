#include <gtest/gtest.h>

#include <matchmaker/client.h>
#include <actionlib/client/simple_action_client.h>

#include <auv_msgs/IdentifyObjectAction.h>

TEST(ObjectIdentificationTest, singleShot)
{
  matchmaker::Client mm_client;
  matchmaker::ServiceDetailsPtr service;
  do
  {
    ROS_INFO("Looking for service \"Object Identification\"");
    service = mm_client.findService("Object Identification");
    sleep(1);
  } while (service.get() == 0);
  ASSERT_TRUE(service->status.isUsable());

  ROS_INFO_STREAM("Found usable service.");

  bool spin_thread = true;
  std::string action_server_name = service->topic;
  actionlib::SimpleActionClient<auv_msgs::IdentifyObjectAction> action_client(action_server_name, spin_thread);

  ROS_INFO_STREAM("Waiting for action server \"" << action_server_name 
      << "\" for 5 sec.");

  bool action_server_found = action_client.waitForServer(ros::Duration(5));
  ASSERT_TRUE(action_server_found);

  ROS_INFO("Action server found. Sending goal.");

  auv_msgs::IdentifyObjectGoal goal;
  goal.single_shot = true;

  action_client.sendGoal(goal);

  ROS_INFO("Goal sent. Waiting for result.");

  bool finished_before_timeout = action_client.waitForResult(ros::Duration(5));
  EXPECT_TRUE(finished_before_timeout);

  if (finished_before_timeout)
  {
    ROS_INFO_STREAM("Action finished before timeout. State: " << 
        action_client.getState().toString());
  }
}


// some globals for the next test
int g_active_cb_called;
int g_done_cb_called;
int g_feedback_cb_called;

void activeCb()
{
  ROS_INFO("Goal just went active.");
  g_active_cb_called++;
}

void doneCb(const actionlib::SimpleClientGoalState&,
    const auv_msgs::IdentifyObjectResultConstPtr&)
{
  ROS_INFO("Goal finished.");
  g_done_cb_called++;
}

void feedbackCb(const auv_msgs::IdentifyObjectFeedbackConstPtr& feedback)
{
  ROS_INFO("Feedback received.");
  g_feedback_cb_called++;
}

TEST(ObjectIdentificationTest, continuous)
{
  matchmaker::Client mm_client;
  matchmaker::ServiceDetailsPtr service;
  do
  {
    ROS_INFO("Looking for service \"Object Identification\"");
    service = mm_client.findService("Object Identification");
    sleep(1);
  } while (service.get() == 0);
  ASSERT_TRUE(service->status.isUsable());

  ROS_INFO_STREAM("Found usable service.");

  bool spin_thread = true;
  std::string action_server_name = service->topic;
  actionlib::SimpleActionClient<auv_msgs::IdentifyObjectAction> action_client(action_server_name, spin_thread);

  ROS_INFO_STREAM("Waiting for action server \"" << action_server_name 
      << "\" for 5 sec.");

  bool action_server_found = action_client.waitForServer(ros::Duration(5));
  ASSERT_TRUE(action_server_found);

  ROS_INFO("Action server found. Sending goal.");

  auv_msgs::IdentifyObjectGoal goal;
  goal.single_shot = false;

  g_active_cb_called = 0;
  g_done_cb_called = 0;
  g_feedback_cb_called = 0;
  action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  ROS_INFO("Goal sent. Waiting for 5 sec.");
  ros::Time start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < 5)
  {
    ros::spinOnce();
  }

  EXPECT_EQ(g_active_cb_called, 1);
  EXPECT_EQ(g_done_cb_called, 0);
  EXPECT_GT(g_feedback_cb_called, 50);

  action_client.cancelAllGoals();

  // action should be preempted in less than 3 secs.
  start_time = ros::Time::now();
  while ((ros::Time::now() - start_time).toSec() < 3)
  {
    ros::spinOnce();
  }

  EXPECT_EQ(g_done_cb_called, 1);
}
  
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "object_identification_test", ros::init_options::AnonymousName);
  return RUN_ALL_TESTS();
}

