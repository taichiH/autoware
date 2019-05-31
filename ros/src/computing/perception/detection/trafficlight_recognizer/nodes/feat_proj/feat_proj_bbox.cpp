#include <iostream>
#include <ros/ros.h>
#include <autoware_msgs/Signals.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

ros::Publisher maskImagePublisher;
ros::Publisher nearestRoiImagePublisher;

ros::Publisher roiImageArrayPublisher;

float z_max = 100;
float z_min = 20;
int w_max = 350;
int w_min = 50;
int h_max = 200;
int h_min = 50;

void callback(const sensor_msgs::Image::ConstPtr& inImageMsg,
              const autoware_msgs::Signals::ConstPtr& signalMsg)
{
  cv::Mat image;
  try
    {
      cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(inImageMsg, "bgr8");
      image = cv_image->image;
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", inImageMsg->encoding.c_str());
      return;
    }
  cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

  int index = 0;
  int nearest_index = 0;
  float prev_z = 256 *256;
  autoware_msgs::DetectedObjectArray roiImageArray;

  for (auto signal : signalMsg->Signals) {
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

    sensor_msgs::ImagePtr roiMsg =
      cv_bridge::CvImage(inImageMsg->header, "rgb8", roi_image).toImageMsg();
    autoware_msgs::DetectedObject roiImage;

    roiImage.roi_image = *roiMsg;
    roiImage.image_frame = inImageMsg->header.frame_id;
    roiImage.x = lt.x;
    roiImage.y = lt.y;
    roiImage.width = rb.x;
    roiImage.height = rb.y;
    roiImage.pose.position.z = float(signal.z);
    roiImageArray.objects.push_back(roiImage);

    if(z < prev_z){
      nearest_index = index;
      prev_z = z;
    }
    index++;
  }

  sensor_msgs::ImagePtr outImageMsg =
    cv_bridge::CvImage(inImageMsg->header, "mono8", mask).toImageMsg();
  maskImagePublisher.publish(outImageMsg);

  if(roiImageArray.objects.size() > 0)
    nearestRoiImagePublisher.publish(roiImageArray.objects.at(nearest_index).roi_image);

  roiImageArrayPublisher.publish(roiImageArray);
}


typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image,
  autoware_msgs::Signals
  > SyncPolicy;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "feat_proj_bbox", ros::init_options::NoSigintHandler);
  ros::NodeHandle rosnode;
  ros::NodeHandle private_nh("~");

  std::string inputSignalTopic;
  std::string inputImageTopic;

  // some bboxes are overlaped
  private_nh.param<std::string>("input_signal", inputSignalTopic, "/roi_signal");
  private_nh.param<std::string>("input_image", inputImageTopic, "/image_raw");
  private_nh.param<float>("z_max", z_max);
  private_nh.param<float>("z_min", z_min);
  private_nh.param<float>("w_max", w_max);
  private_nh.param<float>("w_min", w_min);
  private_nh.param<float>("h_max", h_max);
  private_nh.param<float>("h_min", h_min);


  maskImagePublisher = rosnode.advertise<sensor_msgs::Image>("/feat_proj_bbox/output", 100);
  nearestRoiImagePublisher = rosnode.advertise<sensor_msgs::Image>("/feat_proj_bbox/nearest_box", 100);
  roiImageArrayPublisher = rosnode.advertise<autoware_msgs::DetectedObjectArray>("/feat_proj_bbox/output/roi_image_array", 100);

  message_filters::Subscriber<sensor_msgs::Image> sub_input_image;
  message_filters::Subscriber<autoware_msgs::Signals> sub_input_signal;
  sub_input_image.subscribe(rosnode, inputImageTopic, 1);
  sub_input_signal.subscribe(rosnode, inputSignalTopic, 1);

  boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_
    = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
  sync_->connectInput(sub_input_image, sub_input_signal);
  sync_->registerCallback(callback);

  ros::spin();
}
