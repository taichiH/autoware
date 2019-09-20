#include "trafficlight_recognizer/feat_proj_bbox.h"

namespace trafficlight_recognizer
{
  void FeatProjBBox::onInit()
  {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    pnh_.getParam("z_max", z_max);
    pnh_.getParam("z_min", z_min);
    pnh_.getParam("w_max", w_max);
    pnh_.getParam("w_min", w_min);
    pnh_.getParam("h_max", h_max);
    pnh_.getParam("h_min", h_min);
    pnh_.getParam("approximate_sync", is_approximate_sync_);

    projected_rois_pub_ = pnh_.advertise<autoware_msgs::StampedRoi>("output", 1);

    sub_image_.subscribe(pnh_, "input_image", 1);
    sub_signal_.subscribe(pnh_, "input_signal", 1);

    if (is_approximate_sync_){
      approximate_sync_ =
        boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(1000);
      approximate_sync_->connectInput(sub_image_, sub_signal_);
      approximate_sync_->registerCallback(boost::bind(&FeatProjBBox::callback, this, _1, _2));
    } else {
      sync_  = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
      sync_->connectInput(sub_image_, sub_signal_);
      sync_->registerCallback(boost::bind(&FeatProjBBox::callback, this, _1, _2));
    }
  }

  void FeatProjBBox::callback(const sensor_msgs::Image::ConstPtr& in_image_msg,
                              const autoware_msgs::Signals::ConstPtr& signal_msg)
  {
    cv::Mat image;
    try {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
      image = cv_image->image;
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", in_image_msg->encoding.c_str());
      return;
    }

    float prev_z = 256 *256;

    autoware_msgs::StampedRoi projected_rois;

    for (auto signal : signal_msg->Signals) {
      if (int(signal.u) < 0 || int(signal.u) > image.cols ||
          int(signal.v) < 0 || int(signal.v) > image.rows) {
        continue;
      }

      float z_diff = z_max - z_min;
      int w_diff = w_max - w_min;
      int h_diff = h_max - h_min;
      float z = float(signal.z);
      if (z > z_max) z = z_max;
      if (z < z_min) z = z_min;

      int width_ = w_min + (z_max - z) * w_diff / z_diff;
      int height_ = h_min + (z_max - z) * h_diff / z_diff;

      cv::Point lt = cv::Point(int(signal.u) - width_ * 0.5, int(signal.v) - height_ * 0.5);
      cv::Point rb = cv::Point(int(signal.u) + width_ * 0.5, int(signal.v) + height_ * 0.5);
      if (rb.x > image.cols)  width_ = image.cols - lt.x;
      if (rb.y > image.rows)  height_ = image.rows - lt.y;
      if (lt.x < 0)  lt.x = 0;
      if (lt.y < 0) lt.y = 0;

      sensor_msgs::RegionOfInterest roi_rect;
      roi_rect.x_offset = lt.x;
      roi_rect.y_offset = lt.y;
      roi_rect.width = rb.x - lt.x;
      roi_rect.height = rb.y - lt.y;
      projected_rois.signals.push_back(static_cast<int>(signal.signalId));
      projected_rois.roi_array.push_back(roi_rect);

    }

    projected_rois_pub_.publish(projected_rois);
  }
} // namespace trafficlight_recognizer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trafficlight_recognizer::FeatProjBBox, nodelet::Nodelet)
