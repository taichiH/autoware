#include "trafficlight_recognizer/kcf_tracker_node.h"

namespace trafficlight_recognizer
{
  void KcfTrackerROS::onInit()
  {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    pnh_.getParam("debug_log", debug_log_);
    pnh_.getParam("kernel_sigma", kernel_sigma_);
    pnh_.getParam("cell_size", cell_size_);
    pnh_.getParam("num_scales", num_scales_);
    pnh_.getParam("approximate_sync", is_approximate_sync_);
    pnh_.getParam("interpolation_frequency", interpolation_frequency_);
    pnh_.getParam("buffer_size", buffer_size_);

    utils_ = std::make_shared<Utils>();
    multi_kcf_tracker_ = std::make_shared<MultiKcfTracker>(buffer_size_, interpolation_frequency_);

    output_rois_pub_ = pnh_.advertise<autoware_msgs::StampedRoi>("output_rois", 1);

    boxes_sub = pnh_.subscribe("input_yolo_detected_boxes", 1, &KcfTrackerROS::boxes_callback, this);
    image_sub_.subscribe(pnh_, "input_raw_image", 1);
    stamped_roi_sub_.subscribe(pnh_, "input_nearest_roi_rect", 1);

    if (is_approximate_sync_){
      approximate_sync_ =
        boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(1000);
      approximate_sync_->connectInput(image_sub_, stamped_roi_sub_);
      approximate_sync_->registerCallback(boost::bind(&KcfTrackerROS::callback, this, _1, _2));
    } else {
      sync_  =
        boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
      sync_->connectInput(image_sub_, stamped_roi_sub_);
      sync_->registerCallback(boost::bind(&KcfTrackerROS::callback, this, _1, _2));
    }

  }


  void KcfTrackerROS::boxes_callback(const autoware_msgs::StampedRoi::ConstPtr& boxes)
  {
    boost::mutex::scoped_lock lock(mutex_);
    utils_->roismsg2cvrects(boxes->roi_array, detected_boxes_);
    detected_boxes_stamp_ = boxes->header.stamp.toSec();
  }


  void KcfTrackerROS::callback(const sensor_msgs::Image::ConstPtr& image_msg,
                               const autoware_msgs::StampedRoi::ConstPtr& projected_roi_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    cv::Mat original_image;
    std::vector<cv::Rect> projected_rois;
    std::vector<cv::Rect> init_boxes;
    utils_->rosmsg2cvmat(image_msg, original_image);
    utils_->roismsg2cvrects(projected_roi_msg->roi_array, projected_rois);

    std::vector<cv::Rect> output_boxes;
    std::vector<int> output_signals;
    multi_kcf_tracker_->run(projected_roi_msg->signals,
                            original_image,
                            projected_rois,
                            init_boxes,
                            image_msg->header.stamp.toSec(),
                            detected_boxes_stamp_,
                            output_boxes,
                            output_signals);

    autoware_msgs::StampedRoi output_rois_msg;
    utils_->cvrects2roismsg(output_boxes, output_rois_msg.roi_array);

    std::copy(output_signals.begin(),
              output_signals.end(),
              std::back_inserter(output_rois_msg.signals));

    output_rois_msg.header = projected_roi_msg->header;
    output_rois_pub_.publish(output_rois_msg);

  }

} // namespace trafficlight_recognizer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trafficlight_recognizer::KcfTrackerROS, nodelet::Nodelet)
