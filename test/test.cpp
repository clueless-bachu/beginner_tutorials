/**
 * @file test.cpp
 * @brief This file has test cases for unittesting.
 * @author Vasista Ayyagari
 * @copyright Vasista Ayyagari, 2020
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <std_msgs/String.h>
#include "beginner_tutorials/AddNums.h"

/**
 * @brief The function tests the AddNums service with some base cases
 * @param TestSuite, testCase
 * @return void
 */
TEST(serviceTest, serviceVerification) {
  ros::NodeHandle n;
  ros::ServiceClient client =
  n.serviceClient<beginner_tutorials::AddNums>("AddNums");
  beginner_tutorials::AddNums srv;
  srv.request.A = 3;
  srv.request.B = 1009;
  if (client.call(srv)) {
    EXPECT_EQ(srv.response.Sum, 1012);
  } else {
    EXPECT_EQ(false, true);
  }
}

// Main file to run the gtest cases
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle n;
    return RUN_ALL_TESTS();
}
