#include "traffic_light_classifier.h"

namespace tlr_classifier
{

    int Utils::get_area(const cv::Mat& image)
    {
        // int cnt = 0;
        // for (int y=0; y<image.rows(); ++y) {
        //     for (int x=0; x<image.cols(); ++x) {
        //         if (image.at<uchar>(y, x) > 127)
        //             cnt++;
        //     }
        // }
        // return cnt;
    }

    template<typename T>
    bool Utils::in_vector(const T& c, const typename T::value_type& v)
    {
        // return(c.end() != std::find(c.begin(), c.end(), v));
    }

    bool Utils::merge_msg(const autoware_msgs::LampStateArray::ConstPtr& color_lamp_states,
                          const autoware_msgs::LampStateArray::ConstPtr& arrow_lamp_states,
                          autoware_msgs::LampStateArray::ConstPtr& lamp_states)
    {
        //     try
        //         {
        //             std::copy(color_lamp_states.states.begin(),
        //                       color_lamp_states.states.end(),
        //                       lamp_states.states.begin());
        //             lamp_states.insert(lamp_states.states.end(),
        //                                arrow_lamp_states.states.begin(),
        //                                arrow_lamp_states.states.end());
        //         }
        //     catch (...)
        //         {
        //             return false;
        //         }

        //     return true;
    }



    bool ColorClassifier::hsv_filter(const cv::Mat &input_image,
                                     std::vector<cv::Mat> &output_images)
    {
        // cv::Mat hsv_image;
        // cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);

        // output_images
        // for (int i=0; i<lower_ranges_.size(); ++i)
        //     {
        //         cv::Mat dst_image;
        //         cv::inRange(hsv_image, lower_ranges_.at(i), upper_ranges_.at(i), dst_image);
        //         output_images.push_back(dst_image);
        //     }

        // return true;
    }


    bool ColorClassifier::get_color_ratios(const cv::Mat &input_images,
                                           std::vector<float> &ratios)
    {
        // for (int i=0; i<input_images.size(); ++i)
        //     {
        //         cv::Mat image = input_images.at(i).clone();
        //         int image_size = image.cols * image.rows;
        //         int area = utils_.get_area(image);
        //         float ratio = static_cast<float>(area) / static_cast<float>(image_size);
        //         ratios.push_back(ratio);
        //     }

        // color_classifier_.moving_average_filter(ratios);
        // return true;
    }


    float ColorClassifier::moving_average_filter(std::vector<float> ratios)
    {
        // if (ratios_buffer_.size() > buffer_size_)
        //     {
        //         ratios_buffer_.erase(ratios_buffer_.begin());
        //     }
        // ratios_buffer_.push_back(ratios);

        // int ratios_size = ratios.size();
        // ratios.clear();

        // for (int i=0; i<ratios_size; ++i)
        //     {
        //         float average = 0.0;
        //         for (int j=0 j<ratios_buffer_.size(); ++j)
        //             {
        //                 average += ratios_buffer_.at(i).at(j);
        //             }
        //         average = average / ratios_buffer_.size();
        //         ratios.push_back(average);
        //     }

        // if (ratios.size() != ratios_size)
        //     {
        //         ROS_ERROR("ratios.size is not 3");
        //         return false;
        //     }

        // return true;
    }


    bool ColorClassifier::get_lamp_states(autoware_msgs::LampStateArray::ConstPtr& states,
                                          std::vector<float> ratios)
    {
        // autoware_msgs::LampStateArray states;
        // if (ratios.at(static_cast<int>(ColorClassifier::Lamp::GREEN)) > ratios_thresh_)
        //     {
        //         autoware_msgs::LampState state;
        //         state.type = autoware_msgs::LampState::GREEN;
        //         states.states.push_back(state);
        //     }
        // if (ratios.at(static_cast<int>(ColorClassifier::Lamp::YELLOW)) > ratios_thresh_)
        //     {
        //         autoware_msgs::LampState state;
        //         state.type = autoware_msgs::LampState::YELLOW;
        //         states.states.push_back(state);
        //     }
        // if (ratios.at(static_cast<int>(ColorClassfier::Lamp::RED)) > ratios_thresh_)
        //     {
        //         autoware_msgs::LampState state;
        //         state.type = autoware_msgs::LampState::RED;
        //         states.states.push_back(state);
        //     }

        // return true;
    }

    bool ColorClassifier::get_params(const ros::NodeHandle& pnh)
    {
        // pnh.getParam("buffer_size", buffer_size_, 5);
        // pnh.getParam("ratios_thresh", ratio_thresh_, 0.35);

        // pnh.getParam("green_h_max", green_h_max);
        // pnh.getParam("green_s_max", green_s_max);
        // pnh.getParam("green_v_max", green_v_max);
        // pnh.getParam("green_h_min", green_h_min);
        // pnh.getParam("green_s_min", green_s_min);
        // pnh.getParam("green_v_min", green_v_min);

        // pnh.getParam("yellow_h_max", yellow_h_max);
        // pnh.getParam("yellow_s_max", yellow_s_max);
        // pnh.getParam("yellow_v_max", yellow_v_max);
        // pnh.getParam("yellow_h_min", yellow_h_min);
        // pnh.getParam("yellow_s_min", yellow_s_min);
        // pnh.getParam("yellow_v_min", yellow_v_min);

        // pnh.getParam("red_h_max", red_h_max);
        // pnh.getParam("red_s_max", red_s_max);
        // pnh.getParam("red_v_max", red_v_max);
        // pnh.getParam("red_h_min", red_h_min);
        // pnh.getParam("red_s_min", red_s_min);
        // pnh.getParam("red_v_min", red_v_min);

        // color_classifier_.lower_ranges_.push_back(cv::Scalar(green_h_min, green_s_min, green_v_min));
        // color_classifier_.upper_ranges_.push_back(cv::Scalar(green_h_max, green_s_max, green_v_max));

        // color_classifier_.lower_ranges_.push_back(cv::Scalar(yellow_h_min, yellow_s_min, yellow_v_min));
        // color_classifier_.upper_ranges_.push_back(cv::Scalar(yellow_h_max, yellow_s_max, yellow_v_max));

        // color_classifier_.lower_ranges_.push_back(cv::Scalar(red_h_min, red_s_min, red_v_min));
        // color_classifier_.upper_ranges_.push_back(cv::Scalar(red_h_max, red_s_max, red_v_max));

        // if (color_classifier_.lower_ranges_.size() != 3 || color_classifier_.upper_ranges_.size() != 3)
        //     {
        //         ROS_ERROR("lower_ranges_ and upper_ranges_ size is not suitable");
        //         return false;
        //     }

        // return true;
    }


    bool ArrowClassifier::get_shape(cv::Mat& image, std::vector<std::vector<cv::Point> >& contours)
    {
        // cv::Mat element = cv::getStructuringElement(CV_MORPH_RECT,
        //                                         cv::Size(2*dilation_size_+1, 2*dilation_size_+1),
        //                                         cv::Point(dilation_size_, dilation_size_));
        // cv::dilate(image, image, element);

        // std::vector<Vec4i> hierarchy;
        // cv::findContours(image, contours, hierarchy,
        //                  CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    }

    bool ArrowClassifier::calc_arrow_direction(const cv::Mat& image, const cv::Rect rect, autoware_msgs::LampState::ConstPtr& state)
    {
        // cv::Mat lt_cell = image(cv::Rect(rect.x,
        //                                  rect.y,
        //                                  static_cast<int>(rect.x + rect.width * 0.5),
        //                                  static_cast<int>(rect.y + rect.height * 0.5)));
        // cv::Mat lb_cell = image(cv::Rect(rect.x,
        //                                  static_cast<int>(rect.y + rect.height * 0.5),
        //                                  static_cast<int>(rect.x + rect.width * 0.5),
        //                                  rect.y + rect.height));
        // cv::Mat rt_cell = image(cv::Rect(static_cast<int>(rect.x + rect.width * 0.5),
        //                                  rect.y,
        //                                  rect.x + rect.width,
        //                                  static_cast<int>(rect.y + rect.height * 0.5)));
        // cv::Mat rb_cell = image(cv::Rect(static_cast<int>(rect.x + rect.width * 0.5),
        //                                  static_cast<int>(rect.y + rect.height * 0.5),
        //                                  rect.x + rect.width,
        //                                  rect.y + rect.height));

        // std::pair lt_area<ArrowClassifier::Position, float> =
        //     std::make_pair(ArrowClasifier::Position::LEFTTOP, utils_.get_area(lt_cell));
        // std::pair lb_area<ArrowClassifier::Position, float> =
        //     std::make_pair(ArrowClasifier::Position::LEFTBOTTOM, utils_.get_area(lb_cell));
        // std::pair rt_area<ArrowClassifier::Position, float> =
        //     std::make_pair(ArrowClasifier::Position::RIGHTTOP, utils_.get_area(rt_cell));
        // std::pair rb_area<ArrowClassifier::Position, float> =
        //     std::make_pair(ArrowClasifier::Position::RIGHTBOTTOM, utils_.get_area(rb_cell));

        // std::vector<std::pair<ArrowClassifier::Position, float>> area_list{lt_area, lb_area, rt_area, rb_area};
        // std::sort(area_list.begin(), area_list.end(),
        //           [](const std::pair<ArrowClassifier::Position, float> &left,
        //              const std::pair<ArrowClassifier::Position, float> &right)
        //           {return left.second < right.second});

        // std::vector<ArrowClassifier::Position> cells{area_list.at(0).first, area_list.at(1).first};

        // if (utils_.in_vector(cells, ArrowClassifier::Position::LEFTTOP) &&
        //     utils_.in_vector(cells, ArrowClassifier::Position::RIGHTBOTTOM))
        //     {
        //         state.type = autoware_msgs::LampState::TOP;
        //     }
        // else if (utils_.in_vector(cells, ArrowClassifier::Position::LEFTBOTTOM) &&
        //          utils_.in_vector(cells, ArrowClassifier::Position::RIGHTBOTTOM))
        //     {
        //         state.type = autoware_msgs::LampState::BOTTOM;
        //     }
        // else if (utils_.in_vector(cells, ArrowClassifier::Position::LEFTTOP) &&
        //          utils_.in_vector(cells, ArrowClassifier::Position::LEFTBOTTOM))
        //     {
        //         state.type = autoware_msgs::LampState::LEFT;
        //     }
        // else if (utils_.in_vector(cells, ArrowClassifier::Position::RIGHTTOP) &&
        //          utils_.in_vector(cells, ArrowClassifier::Position::RIGHTBOTTOM))
        //     {
        //         state.type = autoware_msgs::LampState::RIGHT;
        //     }

        // return true;
    }


    bool ArrowClassifier::load_template(const std::string path)
    {
        // template_image_ = cv::imread(path);
        // if (! template_image_.data)
        //     {
        //         ROS_ERROR("failed load template image");
        //         return false;
        //     }
        // try
        //     {
        //         cv::cvtColor(template_image_, template_image_, CV_BGR2GRAY);
        //         cv::threshold(template_image_, template_image_, 127, 255, 0);
        //     }
        // catch (cv::Exception& e)
        //     {
        //         ROS_ERROR("failed to convert template image : %s", e.what());
        //         return false;
        //     }

        // return true;
    }

    bool ArrowClassifier::get_params(const ros::NodeHandle& pnh)
    {
        // std::string template_path;
        // pnh.getParam("template_path", template_path, "");


        // if (! arrow_classifier_.load_template(template_path))
        //     return false;

        // return true;
    }


    void TrafficLightClassifierNode::callback(const sensor_msgs::Image::ConstPtr& msg)
    {
        // autoware_msgs::TrafficLightState traffilight_state_msg;

        // cv::Mat image;

        // try {
        //     cv_bridge::CvImagePtr cv_image =
        //         cv_bridge::toCvCopy(msg, "bgr8");
        //     image = cv_image->image;
        // } catch (cv_bridge::Exception& e) {
        //     ROS_ERROR("failed convert image from sensor_msgs::Image to cv::Mat");
        //     return;
        // }


        // ///// color classification /////

        // std::vector<cv::Mat> filtered_images;
        // color_classifier_.hsv_filter(image, filtered_images);
        // std::vector<float> ratios;
        // color_classifier_.get_color_ratios(filtered_images, ratios);
        // autoware_msgs::LampStateArray color_lamp_states;
        // color_classifier_.get_lamp_states(color_lamp_states, ratios);


        // ////// arrow classification /////

        // cv::Mat gray_image;
        // cv::cvtColor(image, gray_image, CV_BGR2GRAY);

        // std::vector< std::vector<cv::Point> > image_contours;
        // arrow_classifier_.get_shape(gray_image, image_contours);
        // std::vector< std::vector<cv::Point> > template_contours;
        // arrow_classifier_.get_shape(template_image_, template_contours);

        // std::vector<std::pair<int, double>> index_pairs;
        // for (int i=0; i<image_contours.size(); ++i)
        //     {
        //         for (int j=0; j<template_contours.size(); ++j)
        //             {
        //                 double likelihood =
        //                     cv::matchShapes(image_contours.at(i), template_countors.at(j), 1, 0.0);
        //                 double area = cv::contourArea(image_contours);
        //                 if (likelihood < pair_thresh_ && area > area_thresh_)
        //                     {
        //                         std::pair<int, double> index_pair = std::make_pair(i, likelihood);
        //                         index_pairs.push_back(index_pair);
        //                     }
        //             }
        //     }

        // std::sort(index_pairs.begin(), index_pairs.end(),
        //           [](const std::pair<int, double> &left, const std::pair<int, double> &right)
        //           {return left.second < right.second});

        // cv::Mat debug_image = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
        // std::vector<cv::Rect> rects;
        // for (int i=0; i<index_pairs.size(); ++i)
        //     {
        //         int idx = index_pairs.at(i).first
        //         cv::drawContours(debug_image, image_contours, idx, cv::Scalar(0,0,255), 2);
        //         cv::Rect rect = cv::boundingRect(image_contours.at(idx));
        //         rects.push_back(rect);
        //     }

        // autoware_msgs::LampStateArray arrow_lamp_states;
        // for (int i=0; i<rects.size(); ++i)
        //     {
        //         autoware_msgs::LampState state;
        //         ArrowClassifier::Lamp direction = arrow_classifier_.calc_arrow_direction(gray_image, rects.at(i), state);
        //         arrow_lamp_states.states.push_back(state);
        //     }


        // // publish topics

        // autoware_msgs::LampStateArray trafficlight_states_msg;
        // utils_.merge_msg(color_lamp_states, arrow_lamp_states, lamp_states);
        // trafficlight_states_msg.header = msg.header;

        // trafficlight_state_pub_.publish(traffilight_states_msg);
        // image_pub_.publish(cv_bridge::CvImage(msg.header,
        //                                       sensor_msgs::image_encodings::BGR8,
        //                                       debug_image).toImageMsg());
    }


    void TrafficLightClassifierNode::run()
    {
        // ros::NodeHandle pnh("~");

        // std::string image_topic_str;
        // if (!pnh.getParam("image_topic", image_topic_str))
        //     image_topic_str = "/image_raw";

        // if(!color_classifier_.get_params(pnh))
        //     {
        //         ROS_ERROR("failed get color calssfier params");
        //         return;
        //     }

        // if(!arrow_classifier_.get_params(pnh))
        //     {
        //         ROS_ERROR("failed get arrow calssfier params");
        //         return;
        //     }


        // // color classifier
        // image_pub_ = pnh.advertise<sensor_msgs::Image>("output_image", 1);
        // trafficlight_states_pub_ = pnh.advertise<autoware_msgs::TrafficLightState>("output_light_states", 1);


        // image_sub_ = pnh_.subscribe("input_image", 1, &TrafficLightClassifierNode::callback, this);

        // ros::spin();
    }
}
