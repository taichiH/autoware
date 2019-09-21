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

    bool initialized_ = false;

    KcfTracker(const int signal,
               const cv::Rect& projected_roi,
               const cv::Rect& init_box,
               const cv::Mat& original_image,
               const bool initialized);

    bool run(cv::Rect& output_box);


  private:

  }; // class KcfTracker


  class MultiKcfTracker
  {
  public:
    typedef std::shared_ptr<KcfTracker> KcfTrackerPtr;

    KcfTrackerPtr kcf_tracker_;

    std::map<int, KcfTrackerPtr> tracker_map_;

    bool run(const cv::Mat& original_image,
             const std::vector<cv::Rect>& projected_rois,
             const std::vector<int>& signals,
             const std::vector<cv::Rect>& init_boxes,
             std::vector<cv::Rect>& output_boxes);

  private:

  }; // MultiKcfTracker

} // namespace trafficlight_recognizer

#endif
