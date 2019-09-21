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

  class KcfTracker : public KCF_Tracker
  {
  public:

    bool initialize_ = false;

    int signal_ = 0;

    cv::Rect projected_roi_;

    cv::Rect init_box_;

    std::list<cv::Mat > interpolation_images_;


    KcfTracker(const cv::Rect& projected_roi,
               const cv::Rect& init_box,
               const std::list<cv::Mat>& interpolation_images,
               const bool init_box_stamp_changed);

    bool update_tracker(std::list<cv::Mat>& interpolation_images,
                        cv::Rect& output_box);

    bool run(cv::Rect& output_box);


  private:

  }; // class KcfTracker



  class MultiKcfTracker
  {
  public:

    typedef std::shared_ptr<KcfTracker> KcfTrackerPtr;

    KcfTrackerPtr kcf_tracker_;

    std::map<int, KcfTrackerPtr> tracker_map_;

    double init_box_stamp_ = 0;

    double prev_init_box_stamp_ = 0;

    std::vector<cv::Mat> image_buffer_;

    std::vector<double> image_stamp_buffer_;

    int buffer_size_ = 100;

    int interpolation_frequency_ = 1;

    MultiKcfTracker(int buffer_size = 100,
                    int interpolation_frequency = 1);

    bool push_to_tracker_map(std::map<int, KcfTrackerPtr>& tracker_map,
                             const int signal,
                             const cv::Rect& projected_roi,
                             const cv::Rect& init_box,
                             const std::list<cv::Mat>& interpolation_images,
                             const bool init_box_stamp_changed);

    bool pop_from_tracker_map(std::map<int, KcfTrackerPtr>& tracker_map,
                              const std::vector<int>& signals);


    bool create_buffer(const cv::Mat& image,
                       const double image_stamp);

    bool get_init_box_stamped_index(const std::vector<double>& image_stamp_buffer,
                                    int& stamped_index,
                                    double init_box_stamp);

    bool get_interpolation_images(const bool init_box_stamp_changed,
                                  const double init_box_stamp,
                                  const std::vector<cv::Mat>& image_buffer,
                                  const std::vector<double>& image_stamp_buffer,
                                  std::list<cv::Mat> interpolation_images);

    bool run(const std::vector<int>& signals,
             const cv::Mat& original_image,
             const std::vector<cv::Rect>& projected_rois,
             const std::vector<cv::Rect>& init_boxes,
             const double image_stamp,
             const double init_box_stamp,
             std::vector<cv::Rect>& output_boxes,
             std::vector<int>& output_signals);

  private:

  }; // MultiKcfTracker

} // namespace trafficlight_recognizer

#endif
