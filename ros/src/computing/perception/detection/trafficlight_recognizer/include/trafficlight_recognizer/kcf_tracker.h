#ifndef _TRAFFICLIGHT_RECOGNIZER_KCF_TRACKER_
#define _TRAFFICLIGHT_RECOGNIZER_KCF_TRACKER_

#include <iostream>
#include <memory>
#include <Eigen/Core>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include "kcf.h"

namespace trafficlight_recognizer
{

  class GaussianDistribution
  {
  public:
    int d = 2;
    Eigen::Vector2d mean = Eigen::Vector2d(0,0);
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(d, d);
    Eigen::MatrixXd cov_inverse = cov;
    double cov_det_sqrt = std::sqrt(cov.determinant());
  };


  class ImageInfo
  {
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


  class KcfTracker : public KCF_Tracker
  {
  public:
    typedef std::shared_ptr<ImageInfo> ImageInfoPtr;

    ImageInfoPtr image_info_;

    double pi = 3.141592653589793;

    int frames = 0;

    int callback_count_ = 0;

    bool tracker_initialized_ = false;

    bool track_flag_ = false;

    bool signal_changed_ = false;

    int signal_ = 0;

    int prev_signal_ = 0;

    int queue_size_ = 2;

    float box_movement_thresh_ = 0.02;

    int cnt_ = 0;

    int buffer_size_ = 100;

    std::vector<double> image_stamps;

    std::vector<cv::Mat> image_buffer;

    std::vector<cv::Rect> rect_buffer;

    std::vector<cv::Rect> tracker_results_buffer_;

    bool boxesToBox(const std::vector<cv::Rect>& boxes,
                    const cv::Rect& roi_rect,
                    cv::Rect& output_box,
                    float& score);

    bool get_min_index(int& min_index);

    bool box_interpolation(const int min_index, int idx);

    bool create_buffer(const ImageInfoPtr& image_info);

    bool clear_buffer();

    bool create_tracker_results_buffer(const cv::Rect& bb);

    float check_confidence(const std::vector<cv::Rect> results,
                           float& box_movement_ratio);

    bool update_tracker(cv::Mat& image, cv::Rect& output_rect, const cv::Rect& roi_rect,
                        float& box_movement_ratio, float& tracker_conf, float& tracking_time);

    double calc_detection_score(const cv::Mat& box,
                                const cv::Point2f& nearest_roi_image_center);

    bool run(const ImageInfoPtr& image_info,
             cv::Rect& tracked_rect,
             int idx);

    void increment_cnt();

    int calc_offset(int x);

  private:

  }; // class KcfTracker

} // namespace trafficlight_recognizer

#endif
