/**
 * @file talker.cpp
 * @brief This tutorial demonstrates simple sending of messages over the ROS system.
 * @author Vasista Ayyagari
 * @copyright Vasista Ayyagari, 2020
 */
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/AddNums.h"


std::string stringMsg = "This is the default messsage before service is called";

/** 
 * @brief A service function that handles AddNums service
 * @params req: the service request 
 * @params res: the service response
 * @return bool
 */
bool adder(beginner_tutorials::AddNums::Request  &req,
         beginner_tutorials::AddNums::Response &res) {
  res.Sum = req.A + req.B;
  if (req.A > 100000 || req.B >100000) {
    ROS_WARN_STREAM("Input numbers for Addition are high");
  }
  if (req.A > 2147483640 || req.B >2147483640) {
    ROS_ERROR_STREAM("Input numbers are close to limits of Int");
  }
  stringMsg = "After service call: " + std::to_string(res.Sum);
  return true;
}


int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  tf::TransformBroadcaster bc;
  tf::Transform trans;

  int freq = std::atoi(argv[1]);

  ROS_DEBUG_STREAM(std::to_string(freq));
  if (freq < 10) {
    ROS_WARN_STREAM("frequency is too low for real-time");
  }
  if (freq <= 0) {
    ROS_FATAL_STREAM(std::to_string(freq));
  }
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 10000);
  auto server = n.advertiseService("AddNums", adder);
  ROS_INFO_STREAM("Node has started");
  ros::Rate loop_rate(freq);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  std::vector<int> prevCounts(3, 0);
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    if (freq < 10) {
      ROS_WARN_STREAM("frequency is too low for real-time");
    }
    if (freq <= 0) {
      ROS_FATAL_STREAM(std::to_string(freq));
    }

    trans.setOrigin(tf::Vector3(0.0, 2.0, 4.0));
    trans.setRotation(
      tf::Quaternion(0.0640713, 0.0911575, 0.1534393, 0.9818562));
    bc.sendTransform(
      tf::StampedTransform(trans, ros::Time::now(), "world", "talk"));

    std_msgs::String msg;

    msg.data = stringMsg;

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
    prevCounts[0] = prevCounts[1];
    prevCounts[1] = prevCounts[2];
    prevCounts[2] = count;
  }


  return 0;
}
