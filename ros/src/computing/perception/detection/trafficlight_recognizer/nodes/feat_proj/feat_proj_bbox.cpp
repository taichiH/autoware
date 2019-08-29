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

    mask_image_pub = pnh_.advertise<sensor_msgs::Image>("output/mask_image", 1);
    nearest_roi_image_pub = pnh_.advertise<sensor_msgs::Image>("nearest_roi_image", 1);
    nearest_roi_rect_pub = pnh_.advertise<kcf_ros::Rect>("nearest_roi_rect", 1);
    nearest_bbox_pub = pnh_.advertise<jsk_recognition_msgs::BoundingBox>("nearest_bbox", 1);
    visualization_line_pub = pnh_.advertise<visualization_msgs::Marker>("visualization_line", 1);

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

    cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

    int index = 0;
    int nearest_index = 0;
    float prev_z = 256 *256;

    autoware_msgs::DetectedObjectArray roi_image_array;
    kcf_ros::Rect nearest_roi_rect;

    for (auto signal : signal_msg->Signals) {
      if (int(signal.u) < 0 || int(signal.u) > image.cols ||
          int(signal.v) < 0 || int(signal.v) > image.rows) {
        // ROS_WARN("outside of image");
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
      cv::rectangle(mask, lt, rb, cv::Scalar(255, 255, 255), -1, CV_AA);

      if (rb.x > image.cols)  width_ = image.cols - lt.x;
      if (rb.y > image.rows)  height_ = image.rows - lt.y;
      if (lt.x < 0)  lt.x = 0;
      if (lt.y < 0) lt.y = 0;

      cv::Rect roi{lt.x, lt.y, width_, height_};
      cv::Mat roi_image = image(roi);

      sensor_msgs::ImagePtr roi_msg =
        cv_bridge::CvImage(in_image_msg->header, "rgb8", roi_image).toImageMsg();
      autoware_msgs::DetectedObject roi_image_msg;

      roi_image_msg.roi_image = *roi_msg;
      roi_image_msg.image_frame = in_image_msg->header.frame_id;
      roi_image_msg.x = lt.x;
      roi_image_msg.y = lt.y;
      roi_image_msg.width = rb.x - lt.x;
      roi_image_msg.height = rb.y - lt.y;

      roi_image_msg.pose.position.x = float(signal.x);
      roi_image_msg.pose.position.y = float(signal.y);
      roi_image_msg.pose.position.z = float(signal.z);
      roi_image_array.objects.push_back(roi_image_msg);

      if(z < prev_z){
        nearest_index = index;

        nearest_roi_rect.x = lt.x;
        nearest_roi_rect.y = lt.y;
        nearest_roi_rect.width = rb.x - lt.x;
        nearest_roi_rect.height = rb.y - lt.y;
        nearest_roi_rect.signal = signal.signalId;
        if (prev_signal != signal.signalId){
          nearest_roi_rect.changed = true;
        } else {
          nearest_roi_rect.changed = false;
        }
        nearest_roi_rect.header = signal_msg->header;

        prev_signal = signal.signalId;
        prev_z = z;
      }
      index++;
    }

    mask_image_pub.publish(cv_bridge::CvImage(in_image_msg->header,
                                              "mono8",
                                              mask).toImageMsg());

    if(roi_image_array.objects.size() > 0){
      nearest_roi_image_pub.publish(roi_image_array.objects.at(nearest_index).roi_image);
      nearest_roi_rect_pub.publish(nearest_roi_rect);

      jsk_recognition_msgs::BoundingBox nearest_bbox;
      nearest_bbox.pose.position.x = roi_image_array.objects.at(nearest_index).pose.position.x;
      nearest_bbox.pose.position.y = roi_image_array.objects.at(nearest_index).pose.position.y;
      nearest_bbox.pose.position.z = roi_image_array.objects.at(nearest_index).pose.position.z;
      nearest_bbox.dimensions.x = 2;
      nearest_bbox.dimensions.y = 2;
      nearest_bbox.dimensions.z = 0.7;
      nearest_bbox.header = in_image_msg->header;
      nearest_bbox_pub.publish(nearest_bbox);

      visualization_msgs::Marker line_strip;
      line_strip.header = in_image_msg->header;
      line_strip.ns = "visualization_lines";
      line_strip.action = visualization_msgs::Marker::ADD;
      line_strip.pose.orientation.w = 1.0;
      line_strip.id = 2;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_strip.scale.x = 0.1;
      line_strip.color.g = 1.0;
      line_strip.color.a = 1.0;

      int split = 100;
      for (uint32_t i = 0; i < split; ++i) {
          geometry_msgs::Point p;
          p.x = roi_image_array.objects.at(nearest_index).pose.position.x * i / split;
          p.y = roi_image_array.objects.at(nearest_index).pose.position.y * i / split;
          p.z = roi_image_array.objects.at(nearest_index).pose.position.z * i / split;

          line_strip.points.push_back(p);
      }
      visualization_line_pub.publish(line_strip);

    }
  }
} // namespace trafficlight_recognizer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trafficlight_recognizer::FeatProjBBox, nodelet::Nodelet)
