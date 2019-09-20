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
    pnh_.getParam("offset", offset_);

    utils_ = std::make_shared<Utils>();

    // publisher
    debug_image_pub_ =  pnh_.advertise<sensor_msgs::Image>("output_image", 1);
    output_rects_pub_ = pnh_.advertise<autoware_msgs::StampedRoi>("output_rect", 1);

    // subscriber for single callback
    boxes_sub = pnh_.subscribe
      ("input_yolo_detected_boxes", 1, &KcfTrackerROS::boxes_callback, this);

    // subscribers for message filter callback
    image_sub_.subscribe
      (pnh_, "input_raw_image", 1);
    stamped_roi_sub_.subscribe
      (pnh_, "input_nearest_roi_rect", 1);

    // register callback
    if (is_approximate_sync_){
      approximate_sync_ =
        boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(1000);
      approximate_sync_->connectInput(image_sub_, stamped_roi_sub_);
      approximate_sync_->registerCallback
        (boost::bind(&KcfTrackerROS::callback, this, _1, _2));
    } else {
      sync_  =
        boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
      sync_->connectInput(image_sub_, stamped_roi_sub_);
      sync_->registerCallback
        (boost::bind(&KcfTrackerROS::callback, this, _1, _2));
    }
  }


  void KcfTrackerROS::boxes_callback(const autoware_msgs::StampedRoiArray::ConstPtr& boxes_array)
  {
    boost::mutex::scoped_lock lock(mutex_);
    boxes_array_ = boxes_array;
    boxes_array_stamp_ = boxes_array_->header.stamp.toSec();
    boxes_callback_cnt_++;
  }


  void KcfTrackerROS::callback(const sensor_msgs::Image::ConstPtr& image_msg,
                               const autoware_msgs::StampedRoi::ConstPtr& projected_roi_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    cv::Mat image;
    utils_->rosmsg2cvmat(image_msg, image);

    original_image_size_ = image.size();

    autoware_msgs::StampedRoi output_rects;
    for (int i=0; i<projected_roi_msg->roi_array.size(); ++i)
      {
        sensor_msgs::RegionOfInterest projected_roi = projected_roi_msg->roi_array.at(i);
        cv::Rect roi = cv::Rect(projected_roi.x_offset, projected_roi.y_offset,
                                projected_roi.width, projected_roi.height);
        cv::Mat croped_image = image(roi);

        image_info_ = std::make_shared<ImageInfo>(croped_image,
                                                  roi,
                                                  projected_roi_msg->signals.at(i),
                                                  projected_roi_msg->header.stamp.toSec());
        sensor_msgs::RegionOfInterest tracked_rect;
        // kcf_tracker_->run(image_info_, tracked_rect, i);

        output_rects.roi_array.push_back(tracked_rect);
      }


    output_rects.header = header_;
    output_rects_pub_.publish(output_rects);
    debug_image_pub_.publish(cv_bridge::CvImage(header_,
                                                sensor_msgs::image_encodings::BGR8,
                                                image).toImageMsg());
  }

} // namespace trafficlight_recognizer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trafficlight_recognizer::KcfTrackerROS, nodelet::Nodelet)
