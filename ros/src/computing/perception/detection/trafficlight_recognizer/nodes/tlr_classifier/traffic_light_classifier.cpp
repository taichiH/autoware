#include "trafficlight_recognizer/traffic_light_classifier.h"

namespace trafficlight_recognizer
{

  ColorClassifier::ColorClassifier()
  {
    utils_ = std::make_shared<Utils>();
  }

  bool ColorClassifier::hsv_filter(const cv::Mat &input_image,
                                   std::vector<cv::Mat> &output_images)
  {
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);

    for (int i=0; i<lower_ranges_.size(); ++i)
      {
        cv::Mat dst_image;
        cv::inRange(hsv_image, lower_ranges_.at(i), upper_ranges_.at(i), dst_image);
        output_images.push_back(dst_image);
      }

    return true;
  }


  bool ColorClassifier::get_color_ratios(const std::vector<cv::Mat> &input_images,
                                         std::vector<float> &ratios)
  {
    for (int i=0; i<input_images.size(); ++i)
      {
        cv::Mat image = input_images.at(i).clone();
        int image_size = image.cols * image.rows;
        int area = utils_->get_area(image);
        float ratio = static_cast<float>(area) / static_cast<float>(image_size);
        ratios.push_back(ratio);
      }

    moving_average_filter(ratios);
    return true;
  }


  float ColorClassifier::moving_average_filter(std::vector<float> ratios)
  {
    if (ratios_buffer_.size() > buffer_size_)
      {
        ratios_buffer_.erase(ratios_buffer_.begin());
      }
    ratios_buffer_.push_back(ratios);

    int ratios_size = ratios.size();
    ratios.clear();

    for (int i=0; i<ratios_size; ++i)
      {
        float average = 0.0;
        for (int j=0; j<ratios_buffer_.size(); ++j)
          {
            average += ratios_buffer_.at(i).at(j);
          }
        average = average / ratios_buffer_.size();
        ratios.push_back(average);
      }

    if (ratios.size() != ratios_size)
      {
        ROS_ERROR("ratios.size is not 3");
        return false;
      }

    return true;
  }


  bool ColorClassifier::get_lamp_states(autoware_msgs::LampStateArray& states,
                                        std::vector<float> ratios)
  {
    if (ratios.at(static_cast<int>(ColorClassifier::Lamp::GREEN)) > ratios_thresh_)
      {
        autoware_msgs::LampState state;
        state.type = autoware_msgs::LampState::GREEN;
        states.states.push_back(state);
      }
    if (ratios.at(static_cast<int>(ColorClassifier::Lamp::YELLOW)) > ratios_thresh_)
      {
        autoware_msgs::LampState state;
        state.type = autoware_msgs::LampState::YELLOW;
        states.states.push_back(state);
      }
    if (ratios.at(static_cast<int>(ColorClassifier::Lamp::RED)) > ratios_thresh_)
      {
        autoware_msgs::LampState state;
        state.type = autoware_msgs::LampState::RED;
        states.states.push_back(state);
      }

    return true;
  }

  bool ColorClassifier::get_params(const ros::NodeHandle& pnh)
  {
    pnh.getParam("buffer_size", buffer_size_);
    pnh.getParam("ratios_thresh", ratios_thresh_);

    std::vector<int> green_max_hsv;
    std::vector<int> green_min_hsv;
    std::vector<int> red_max_hsv;
    std::vector<int> red_min_hsv;
    std::vector<int> yellow_max_hsv;
    std::vector<int> yellow_min_hsv;

    pnh.getParam("green_max", green_max_hsv);
    pnh.getParam("green_min", green_min_hsv);
    pnh.getParam("yellow_max", yellow_max_hsv);
    pnh.getParam("yellow_min", yellow_min_hsv);
    pnh.getParam("red_max", red_max_hsv);
    pnh.getParam("red_min", red_min_hsv);

    lower_ranges_.push_back
      (cv::Scalar(green_min_hsv.at(ColorClassifier::HSV::Hue),
                  green_min_hsv.at(ColorClassifier::HSV::Sat),
                  green_min_hsv.at(ColorClassifier::HSV::Val)));
    upper_ranges_.push_back
      (cv::Scalar(green_max_hsv.at(ColorClassifier::HSV::Hue),
                  green_max_hsv.at(ColorClassifier::HSV::Sat),
                  green_max_hsv.at(ColorClassifier::HSV::Val)));

    lower_ranges_.push_back
      (cv::Scalar(yellow_min_hsv.at(ColorClassifier::HSV::Hue),
                  yellow_min_hsv.at(ColorClassifier::HSV::Sat),
                  yellow_min_hsv.at(ColorClassifier::HSV::Val)));
    upper_ranges_.push_back
      (cv::Scalar(yellow_max_hsv.at(ColorClassifier::HSV::Hue),
                  yellow_max_hsv.at(ColorClassifier::HSV::Sat),
                  yellow_max_hsv.at(ColorClassifier::HSV::Val)));

    lower_ranges_.push_back
      (cv::Scalar(red_min_hsv.at(ColorClassifier::HSV::Hue),
                  red_min_hsv.at(ColorClassifier::HSV::Sat),
                  red_min_hsv.at(ColorClassifier::HSV::Val)));
    upper_ranges_.push_back
      (cv::Scalar(red_max_hsv.at(ColorClassifier::HSV::Hue),
                  red_max_hsv.at(ColorClassifier::HSV::Sat),
                  red_max_hsv.at(ColorClassifier::HSV::Val)));


    if (lower_ranges_.size() != 3 || upper_ranges_.size() != 3)
      {
        ROS_ERROR("lower_ranges_ and upper_ranges_ size is not suitable");
        return false;
      }

    return true;
  }


  ArrowClassifier::ArrowClassifier()
  {
    utils_ = std::make_shared<Utils>();
  }

  bool ArrowClassifier::get_shape(cv::Mat& image, std::vector<std::vector<cv::Point> >& contours)
  {
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2*dilation_size_+1, 2*dilation_size_+1),
                                                cv::Point(dilation_size_, dilation_size_));
    cv::dilate(image, image, element);

    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy,
                     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
  }

  bool ArrowClassifier::calc_arrow_direction(const cv::Mat& image,
                                             const cv::Rect rect,
                                             autoware_msgs::LampState& state)
  {
    cv::Mat lt_cell = image(cv::Rect(rect.x,
                                     rect.y,
                                     static_cast<int>(rect.x + rect.width * 0.5),
                                     static_cast<int>(rect.y + rect.height * 0.5)));
    cv::Mat lb_cell = image(cv::Rect(rect.x,
                                     static_cast<int>(rect.y + rect.height * 0.5),
                                     static_cast<int>(rect.x + rect.width * 0.5),
                                     rect.y + rect.height));
    cv::Mat rt_cell = image(cv::Rect(static_cast<int>(rect.x + rect.width * 0.5),
                                     rect.y,
                                     rect.x + rect.width,
                                     static_cast<int>(rect.y + rect.height * 0.5)));
    cv::Mat rb_cell = image(cv::Rect(static_cast<int>(rect.x + rect.width * 0.5),
                                     static_cast<int>(rect.y + rect.height * 0.5),
                                     rect.x + rect.width,
                                     rect.y + rect.height));

    std::pair<Utils::Position, float> lt_area =
      std::make_pair(Utils::Position::LEFTTOP, utils_->get_area(lt_cell));
    std::pair<Utils::Position, float> lb_area =
      std::make_pair(Utils::Position::LEFTBOTTOM, utils_->get_area(lb_cell));
    std::pair<Utils::Position, float> rt_area =
      std::make_pair(Utils::Position::RIGHTTOP, utils_->get_area(rt_cell));
    std::pair<Utils::Position, float> rb_area =
      std::make_pair(Utils::Position::RIGHTBOTTOM, utils_->get_area(rb_cell));

    std::vector<std::pair<Utils::Position, float>> area_list{lt_area, lb_area, rt_area, rb_area};

    std::sort(area_list.begin(), area_list.end(),
              [](const std::pair<Utils::Position, float> &left,
                 const std::pair<Utils::Position, float> &right)
              {return left.second < right.second;});

    std::vector<Utils::Position> cells{area_list.at(0).first, area_list.at(1).first};

    if (utils_->in_vector(cells, Utils::Position::LEFTTOP) &&
        utils_->in_vector(cells, Utils::Position::RIGHTBOTTOM))
      {
        state.type = autoware_msgs::LampState::UP;
      }
    else if (utils_->in_vector(cells, Utils::Position::LEFTBOTTOM) &&
             utils_->in_vector(cells, Utils::Position::RIGHTBOTTOM))
      {
        state.type = autoware_msgs::LampState::DOWN;
      }
    else if (utils_->in_vector(cells, Utils::Position::LEFTTOP) &&
             utils_->in_vector(cells, Utils::Position::LEFTBOTTOM))
      {
        state.type = autoware_msgs::LampState::LEFT;
      }
    else if (utils_->in_vector(cells, Utils::Position::RIGHTTOP) &&
             utils_->in_vector(cells, Utils::Position::RIGHTBOTTOM))
      {
        state.type = autoware_msgs::LampState::RIGHT;
      }

    return true;
  }


  bool ArrowClassifier::load_template(const std::string path)
  {
    template_image_ = cv::imread(path);
    if (! template_image_.data)
      {
        ROS_ERROR("failed load template image");
        return false;
      }
    try
      {
        cv::cvtColor(template_image_, template_image_, CV_BGR2GRAY);
        cv::threshold(template_image_, template_image_, 127, 255, 0);
      }
    catch (cv::Exception& e)
      {
        ROS_ERROR("failed to convert template image : %s", e.what());
        return false;
      }

    return true;
  }

  bool ArrowClassifier::get_params(const ros::NodeHandle& pnh)
  {
    std::string template_path;
    pnh.getParam("template_path", template_path);


    if (! load_template(template_path))
      return false;

    return true;
  }


  bool ArrowClassifier::get_index_pairs
  (const std::vector< std::vector<cv::Point> >& image_contours,
   const std::vector< std::vector<cv::Point> >& template_contours,
   std::vector<std::pair<int, double>>& index_pairs)
  {
    for (int i=0; i<image_contours.size(); ++i)
      {
        for (int j=0; j<template_contours.size(); ++j)
          {
            double likelihood =
              cv::matchShapes(image_contours.at(i), template_contours.at(j), 1, 0.0);
            double area = cv::contourArea(image_contours);
            if (likelihood < pair_thresh_ && area > area_thresh_)
              {
                std::pair<int, double> index_pair = std::make_pair(i, likelihood);
                index_pairs.push_back(index_pair);
              }
          }
      }

    std::sort(index_pairs.begin(), index_pairs.end(),
              [](const std::pair<int, double> &left, const std::pair<int, double> &right)
              {return left.second < right.second;});

    return true;
  }

  bool ArrowClassifier::get_arrow_rects
  (const std::vector< std::vector<cv::Point> >& image_contours,
   const std::vector< std::vector<cv::Point> >& template_contours,
   const cv::Mat& image,
   std::vector<cv::Rect>& rects,
   cv::Mat& debug_image)
  {
    std::vector<std::pair<int, double>> index_pairs;
    get_index_pairs(image_contours, template_contours, index_pairs);

    debug_image = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
    for (int i=0; i<index_pairs.size(); ++i)
      {
        int idx = index_pairs.at(i).first;
        cv::drawContours(debug_image, image_contours, idx, cv::Scalar(0,0,255), 2);
        cv::Rect rect = cv::boundingRect(image_contours.at(idx));
        rects.push_back(rect);
      }

    return true;
  }

  bool TrafficLightClassifierNode::classify
  (const cv::Mat& image,
   autoware_msgs::LampStateArray& lamp_state_array,
   cv::Mat& debug_image)
  {
    ///// color classification /////

    std::vector<cv::Mat> filtered_images;
    color_classifier_->hsv_filter(image, filtered_images);
    std::vector<float> ratios;
    color_classifier_->get_color_ratios(filtered_images, ratios);
    autoware_msgs::LampStateArray color_lamp_states;
    color_classifier_->get_lamp_states(color_lamp_states, ratios);


    ////// arrow classification /////

    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, CV_BGR2GRAY);
    std::vector< std::vector<cv::Point> > image_contours;
    arrow_classifier_->get_shape(gray_image, image_contours);
    std::vector< std::vector<cv::Point> > template_contours;
    arrow_classifier_->get_shape(arrow_classifier_->template_image_, template_contours);
    std::vector<cv::Rect> rects;
    arrow_classifier_->get_arrow_rects
      (image_contours, template_contours, image, rects, debug_image);

    autoware_msgs::LampStateArray arrow_lamp_states;
    for (int i=0; i<rects.size(); ++i)
      {
        autoware_msgs::LampState state;
        arrow_classifier_->calc_arrow_direction(gray_image, rects.at(i), state);
        arrow_lamp_states.states.push_back(state);
      }


    // publish topics

    utils_->merge_msg(color_lamp_states, arrow_lamp_states, lamp_state_array);


    return true;
  }

  void TrafficLightClassifierNode::callback
  (const sensor_msgs::Image::ConstPtr& image_msg,
   const autoware_msgs::StampedRoi::ConstPtr& stamped_roi_msg)
  {
    autoware_msgs::TrafficLightState traffilight_state_msg;

    cv::Mat image;

    try {
      cv_bridge::CvImagePtr cv_image =
        cv_bridge::toCvCopy(image_msg, "bgr8");
      image = cv_image->image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("failed convert image from sensor_msgs::Image to cv::Mat");
      return;
    }

    autoware_msgs::TrafficLightStateArray trafficlight_state_array;
    cv::Mat concatenated_debug_image;
    for (int i=0; i<stamped_roi_msg->roi_array.size(); ++i)
      {
        autoware_msgs::TrafficLightState trafficlight_state;
        trafficlight_state.roi = stamped_roi_msg->roi_array.at(i);

        sensor_msgs::RegionOfInterest region_of_interest = stamped_roi_msg->roi_array.at(i);
        cv::Rect roi = cv::Rect(region_of_interest.x_offset,
                                region_of_interest.y_offset,
                                region_of_interest.width,
                                region_of_interest.height);

        cv::Mat croped_image = image(roi);
        cv::Mat debug_image;
        autoware_msgs::LampStateArray lamp_state_array;
        classify(croped_image, lamp_state_array, debug_image);

        lamp_state_array.header = stamped_roi_msg->header;
        trafficlight_state.states = lamp_state_array;
        trafficlight_state.header = stamped_roi_msg->header;
        trafficlight_state_array.states.push_back(trafficlight_state);
      }

    trafficlight_state_array.header = stamped_roi_msg->header;
    trafficlight_state_array_pub_.publish(trafficlight_state_array);
    image_pub_.publish(cv_bridge::CvImage
                       (stamped_roi_msg->header,
                        sensor_msgs::image_encodings::BGR8,
                        concatenated_debug_image).toImageMsg());
  }


  void TrafficLightClassifierNode::run()
  {
    ros::NodeHandle pnh("~");

    color_classifier_ = std::make_shared<ColorClassifier>();
    arrow_classifier_ = std::make_shared<ArrowClassifier>();
    utils_ = std::make_shared<Utils>();

    std::string image_topic_str;
    if ( !pnh.getParam("image_topic", image_topic_str) )
      image_topic_str = "/image_raw";

    if( !color_classifier_->get_params(pnh) )
      {
        ROS_ERROR("failed get color calssfier params");
        return;
      }

    if( !arrow_classifier_->get_params(pnh) )
      {
        ROS_ERROR("failed get arrow calssfier params");
        return;
      }

    image_pub_ =
      pnh.advertise<sensor_msgs::Image>("output_image", 1);
    trafficlight_state_array_pub_ =
      pnh.advertise<autoware_msgs::TrafficLightState>("output_light_states", 1);

    image_sub_.subscribe(pnh, "input_image", 1);
    stamped_roi_sub_.subscribe(pnh, "input_stamped_roi", 1);

    if (is_approximate_sync_)
      {
        approximate_sync_ =
          boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
        approximate_sync_->connectInput(image_sub_, stamped_roi_sub_);
        approximate_sync_->registerCallback
          (boost::bind(&TrafficLightClassifierNode::callback, this, _1, _2));
      }
    else
      {
        sync_  =
          boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        approximate_sync_->connectInput(image_sub_, stamped_roi_sub_);
        sync_->registerCallback
          (boost::bind(&TrafficLightClassifierNode::callback, this, _1, _2));
      }

    ros::spin();
  }

}
