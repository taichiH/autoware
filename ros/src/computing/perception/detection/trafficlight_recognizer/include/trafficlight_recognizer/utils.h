#ifndef TRAFFICLIGHT_RECOGNIZER_UTILS_H
#define TRAFFICLIGHT_RECOGNIZER_UTILS_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <autoware_msgs/TrafficLightStateArray.h>
#include <autoware_msgs/TrafficLightState.h>
#include <autoware_msgs/LampStateArray.h>
#include <autoware_msgs/LampState.h>
#include <autoware_msgs/StampedRoiArray.h>
#include <autoware_msgs/StampedRoi.h>

#include <eigen_conversions/eigen_msg.h>

namespace trafficlight_recognizer
{

  class GaussianDistribution{
  public:
    int d = 2;
    Eigen::Vector2d mean = Eigen::Vector2d(0,0);
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(d, d);
    Eigen::MatrixXd cov_inverse = cov;
    double cov_det_sqrt = std::sqrt(cov.determinant());
  };


  class ImageInfo {
  public:
    double stamp = 0.0;
    int signal = 0;
    cv::Mat image;
    cv::Rect rect;

    bool initialized = false;

    ImageInfo(const cv::Mat& _image = cv::Mat(3,3,CV_8UC3),
              const cv::Rect& _rect= cv::Rect(0,0,0,0),
              const int _signal = 0,
              const double _stamp = 0.0) {
      stamp = _stamp;
      signal = _signal;
      image = _image;
      rect = _rect;
    }

    ~ImageInfo() {};

  };


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


    inline float sigmoid(float x, float a=200, float movement=0.02)
    {
      return(1 - (1.0 / (1.0 + exp(a * (-x + movement)))));
    }

    template<typename T>
      inline bool in_vector(const T& c, const typename T::value_type& v)
      {
        return(c.end() != std::find(c.begin(), c.end(), v));
      }

  };

}

#endif
