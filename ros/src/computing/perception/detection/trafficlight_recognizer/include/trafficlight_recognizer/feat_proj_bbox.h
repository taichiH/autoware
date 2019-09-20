#ifndef _TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_BBOX_
#define _TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_BBOX_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <autoware_msgs/Signals.h>
#include <autoware_msgs/StampedRoi.h>
#include <sensor_msgs/RegionOfInterest.h>

namespace trafficlight_recognizer
{
  class FeatProjBBox : public nodelet::Nodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      autoware_msgs::Signals
      > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      autoware_msgs::Signals
      > ApproximateSyncPolicy;
  protected:
    float z_max = 100;
    float z_min = 20;
    int w_max = 350;
    int w_min = 50;
    int h_max = 200;
    int h_min = 50;
    int prev_signal = 0;

    bool debug_ = true;
    bool is_approximate_sync_ = true;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher projected_rois_pub_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<autoware_msgs::Signals> sub_signal_;

    virtual void onInit();
    virtual void callback(const sensor_msgs::Image::ConstPtr& in_image_msg,
                          const autoware_msgs::Signals::ConstPtr& signal_msg);

  private:
  }; // class FeatProjBBox
} // namespace trafficlight_recognizer

#endif
