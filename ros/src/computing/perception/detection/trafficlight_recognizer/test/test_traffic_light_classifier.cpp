/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <gtest/gtest.h>
#include "trafficlight_recognizer/traffic_light_classifier.h"

class TrafficLightClassifierNodeTestSuite : public ::testing::Test
{
protected:

  typedef std::shared_ptr<trafficlight_recognizer::TrafficLightClassifierNode> ClassifierPtr;

  typedef std::shared_ptr<trafficlight_recognizer::ColorClassifier> ColorClassifierPtr;

  typedef std::shared_ptr<trafficlight_recognizer::ArrowClassifier> ArrowClassifierPtr;

  ColorClassifierPtr color_classifier_;

  ArrowClassifierPtr arrow_classifier_;

  ClassifierPtr classifier_;

  virtual void SetUp()
  {
    generateTestData();
  }

  virtual void generateTestData()
  {
  }

};

TEST_F(TrafficLightClassifierNodeTestSuite, runClassifier)
{
  // std::cerr << "\n" << __func__ << ":runClassifier" << std::endl;

  // classifier_ = std::make_shared<trafficlight_recognizer::TrafficLightClassifierNode>();

  // cv::Mat croped_image = cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0,0,255));
  // cv::Mat debug_image;
  // autoware_msgs::LampStateArray lamp_state_array;
  // bool is_succeeded = classifier_->classify(croped_image, lamp_state_array, debug_image);
  // ASSERT_EQ(true, is_succeeded) << "failed classify image" ;

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TrafficLightClassifierNodeTestSuite");

  return RUN_ALL_TESTS();
}
