#include "trafficlight_recognizer/utils.h"

namespace trafficlight_recognizer
{

  int Utils::get_area(const cv::Mat& image)
  {
    int cnt = 0;
    for (int y=0; y<image.rows; ++y) {
      for (int x=0; x<image.cols; ++x) {
        if (image.at<uchar>(y, x) > 127)
          cnt++;
      }
    }
    return cnt;
  }


  bool Utils::merge_msg(const autoware_msgs::LampStateArray& color_lamp_states,
                        const autoware_msgs::LampStateArray& arrow_lamp_states,
                        autoware_msgs::LampStateArray& lamp_states)
  {
    try
      {
        for (auto state : color_lamp_states.states)
          {
            lamp_states.states.push_back(state);
          }

        for (auto state : arrow_lamp_states.states)
          {
            lamp_states.states.push_back(state);
          }
      }
    catch (...)
      {
        return false;
      }

    return true;
  }

  bool Utils::rosmsg2cvmat(const sensor_msgs::Image::ConstPtr& image_msg, cv::Mat& image)
  {
    try
      {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
        image = cv_image->image;
      }
    catch (cv_bridge::Exception& e)
      {
        return false;
      }

    return true;
  }

} // trafficlight_recognizer
