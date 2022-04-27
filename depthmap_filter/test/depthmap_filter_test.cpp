/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file depthmap_filter_test.cpp
 * @author W. Nicholas Greene
 * @date 2017-11-15 12:40:45 (Wed)
 */

#include <gtest/gtest.h>
#include <ros/init.h>
#include "depthmap_filter.hpp"

TEST(DepthmapFilter, Constructor) {
  depthmap_filter::DepthmapFilter blob;
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "depthmap_filter_test");
  ros::Time::init();
  return RUN_ALL_TESTS();
}
