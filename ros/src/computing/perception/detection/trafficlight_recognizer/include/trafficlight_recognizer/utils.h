#ifndef TRAFFICLIGHT_RECOGNIZER_UTILS_H
#define TRAFFICLIGHT_RECOGNIZER_UTILS_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/Image.h>
#include <autoware_msgs/TrafficLightStateArray.h>
#include <autoware_msgs/TrafficLightState.h>
#include <autoware_msgs/LampStateArray.h>
#include <autoware_msgs/LampState.h>
#include <autoware_msgs/StampedRoi.h>

namespace trafficlight_recognizer
{

    class Utils
    {
    public:

        enum Position {
            LEFTTOP,
            LEFTBOTTOM,
            RIGHTTOP,
            RIGHTBOTTOM,
        };

        int get_area(const cv::Mat& image);

        bool merge_msg(const autoware_msgs::LampStateArray& color_lamp_states,
                       const autoware_msgs::LampStateArray& arrow_lamp_states,
                       autoware_msgs::LampStateArray& lamp_states);

        bool rosmsg2cvmat(const sensor_msgs::Image::ConstPtr& image_msg, cv::Mat& image);

        bool fit_in_frame(cv::Point& lt, cv::Point& rb, const cv::Size& size);

        template<typename T>
          inline bool in_vector(const T& c, const typename T::value_type& v)
          {
            return(c.end() != std::find(c.begin(), c.end(), v));
          }

    };

}

#endif
