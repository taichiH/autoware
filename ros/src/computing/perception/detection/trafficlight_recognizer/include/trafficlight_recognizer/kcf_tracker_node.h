#ifndef _TRAFFICLIGHT_RECOGNIZERR_KCF_TRACKER_NODE_
#define _TRAFFICLIGHT_RECOGNIZERR_KCF_TRACKER_NODE_

#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "trafficlight_recognizer/kcf_tracker.h"
#include "trafficlight_recognizer/utils.h"

namespace trafficlight_recognizer
{

  class KcfTrackerROS : public nodelet::Nodelet
  {
  public:

    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      autoware_msgs::StampedRoi
      > SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      autoware_msgs::StampedRoi
      > ApproximateSyncPolicy;

    typedef std::shared_ptr<Utils> UtilsPtr;

    typedef std::shared_ptr<MultiKcfTracker> MultiKcfTrackerPtr;

    bool debug_log_ = false;

    double kernel_sigma_ = 0.5;

    double cell_size_ = 4;

    double num_scales_ = 7;

    bool is_approximate_sync_ = true;

    int buffer_size_ = 100;

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    ros::Publisher output_rects_pub_;

    ros::Subscriber boxes_sub;

    boost::mutex mutex_;

    UtilsPtr utils_;

    MultiKcfTrackerPtr multi_kcf_tracker_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;

    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_;

    message_filters::Subscriber<autoware_msgs::StampedRoi> stamped_roi_sub_;

    void onInit();

    void boxes_callback(const autoware_msgs::StampedRoi::ConstPtr& boxes);

    void callback(const sensor_msgs::Image::ConstPtr& image_msg,
                  const autoware_msgs::StampedRoi::ConstPtr& stamped_roi_msg);

  }; // KcfTrackerNode


} // namespace trafficlight_recognizer

#endif
