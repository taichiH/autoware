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
ros::Publisher roiImageArrayPublisher;

int width_ = 200;
int height_ = 100;

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

  autoware_msgs::DetectedObjectArray roiImageArray;
  for (auto signal : signalMsg->Signals) {
    if (int(signal.u) < 0 || int(signal.u) > image.cols ||
        int(signal.v) < 0 || int(signal.v) > image.rows) {
      // ROS_WARN("outside of image");
      continue;
    }

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
    roiImageArray.objects.push_back(roiImage);
  }

  sensor_msgs::ImagePtr outImageMsg =
    cv_bridge::CvImage(inImageMsg->header, "mono8", mask).toImageMsg();
  maskImagePublisher.publish(outImageMsg);
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
  private_nh.param<std::string>("input_signal", inputSignalTopic, "/roi_singal");
  private_nh.param<std::string>("input_image", inputImageTopic, "/image_raw");

  maskImagePublisher = rosnode.advertise<sensor_msgs::Image>("/feat_proj_bbox/output", 100);
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
