#include "trafficlight_recognizer/kcf_tracker.h"

namespace trafficlight_recognizer
{

  KcfTracker::KcfTracker(const int signal,
                         const cv::Rect& projected_roi,
                         const cv::Rect& init_box,
                         const std::vector<cv::Mat>& interpolation_images)
  {
    signal_ = signal;
    projected_roi_ = projected_roi;
    init_box_ = init_box;
    interpolation_images_ = interpolation_images;
  }

  bool KcfTracker::run(cv::Rect& output_box)
  {

    if (initialized_)
      {
        init(init_box_, original_image_);
        initialized_ = true;
        output_box = init_box_;
      }
    else
      {
        for (auto image : interpolation_images)
          {
            track(image);
          }
        BBox_c bb = getBBox();
        output_box = cv::Rect
          (bb.cx - bb.w * 0.5, bb.cy - bb.h * 0.5, bb.w, bb.h);
      }

    return true;

  }

  bool MultiKcfTracker::push_to_tracker_map(std::map<int, KcfTracker>& tracker_map,
                                            const int signal,
                                            const cv::Rect& projected_roi,
                                            const cv::Rect& init_box,
                                            const cv::Mat& original_image)
  {
    // push tracker
    if ( tracker_map.find(signal) == tracker_map.end() )
      {
        KcfTrackerPtr tracker = std::make_shared<KcfTracker>
          (signal, projected_roi, init_box, original_image);
        tracker_map.emplace(signal, tracker);
      }

    return true;
  }


  bool MultiKcfTracker::pop_from_tracker_map(std::map<int, KcfTracker>& tracker_map,
                                             const std::vector<int>& signals)
  {
    for (auto [k, v] : tracker_map)
      {
        if ( !utils_->in_vector(signals, k) )
          tracker_map.erase(k);
      }

    return true;
  }

  bool MultiKcfTracker::create_buffer(const cv::Mat& image,
                                      const double image_stamp)
  {
    if (image_buffer_.size() >= buffer_size_)
      {
        image_buffer_.pop_front();
        image_stamp_buffer_.pop_front();
      }
    image_buffer_.push_back(image);
    image_stamp_buffer_.push_back(image_stamp);

    return true;
  }

  bool MultiKcfTracker::get_init_box_stampd_index
  (int& stamped_index, double init_box_stamp){
    stamped_index = 0;
    bool found_stamped_index = false;

    if (image_stamp_buffer_.empty()) {
      ROS_ERROR("image_stamps is empty");
      return false;
    }

    for (int i=image_stamp_buffer_.size() - 1; i>=0; i--) {
      if (image_stamp_buffer_.at(i) - init_box_stamp < 0) {
        found_stamped_index = true;
        if (std::abs(image_stamp_buffer_.at(i+1) - init_box_stamp) <
            std::abs(image_stamp_buffer_.at(i) - init_box_stamp)){
          stamped_index = i+1;
        } else {
          stamped_index = i;
        }
        break;
      }
    }
    return found_stamped_index;
  }

  bool MultiKcfTracker::get_interpolation_images(const bool init_box_stamp_changed,
                                                 const double init_box_stamp,
                                                 std::vector<cv::Mat> interpolation_images)
  {
    if (init_box_stamp_changed)
      {
        int stamped_index = 0;
        get_init_box_stamped_index(stamped_index, init_box_stamp);

        for (int i=stamped_index; i<image_buffer_.size(); i+=interpolation_frequency_)
          {
            interpolation_images.push_back(image_buffer_.at(i));
          }
      }
    else
      {
        // a latest image
        interpolation_images.push_back(image_buffer_.back());
      }

    return true;
  }


  bool MultiKcfTracker::run(const std::vector<int>& signals,
                            const cv::Mat& original_image,
                            const std::vector<cv::Rect>& projected_rois,
                            const std::vector<cv::Rect>& init_boxes,
                            const double image_stamp,
                            const double init_box_stamp,
                            std::vector<cv::Rect>& output_boxes)
  {

    // images buffer for interpolation
    create_buffer(original_image, image_stamp);

    if ( init_box_stamp_ != 0 )
      {
        // create interpolation images caused by detection latency
        std::vector<cv::Mat> interpolation_images;
        bool init_box_stamp_changed = init_box_stamp_ != prev_init_box_stamp_;
        get_interpolation_images
          (init_box_stamp_changed, init_box_stamp, interpolation_images);

        // remove unnecessary tracker map
        pop_from_tracker_map(tracker_map_, signals);

        for (int i=0; i<signals.size(); ++i)

          {
            auto signal = signals.at(i);
            auto projected_roi = projected_rois.at(i);
            auto init_box = init_boxes.at(i);

            push_to_tracker_map
              (tracker_map_, signal, projected_roi, init_box, interpolation_images);

            cv::Rect output_box;
            auto kcf_tracker = tracker_map_.at(signal);
            kcf_tracker->run(output_box);

            output_boxes.push_back(output_box);

          }

      }

    prev_init_box_stamp_ = init_box_stamp_;

    return true;

  }

} // namespace trafficlight_recognizer
