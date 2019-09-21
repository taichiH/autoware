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

    typedef std::shared_ptr<ImageInfo> ImageInfoPtr;

    typedef std::shared_ptr<Utils> UtilsPtr;

    typedef std::shared_ptr<KcfTracker> KcfTrackerPtr;


    bool debug_log_ = false;

    double kernel_sigma_ = 0.5;

    double cell_size_ = 4;

    double num_scales_ = 7;

    int interpolation_frequency_ = 1;

    int offset_ = 0;

    bool is_approximate_sync_ = true;

    int boxes_callback_cnt_ = 0;

    int prev_boxes_callback_cnt_ = 0;

    double boxes_array_stamp_ = 0.0;

    cv::Size original_image_size_;

    autoware_msgs::StampedRoiArray::ConstPtr boxes_array_;

    std_msgs::Header header_;

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    ros::Publisher debug_image_pub_;

    ros::Publisher output_rects_pub_;

    ros::Subscriber boxes_sub;

    boost::mutex mutex_;

    ImageInfoPtr image_info_;

    UtilsPtr utils_;

    KcfTrackerPtr kcf_tracker_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;

    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_;

    message_filters::Subscriber<autoware_msgs::StampedRoi> stamped_roi_sub_;

    virtual void onInit();

    virtual void boxes_callback(const autoware_msgs::StampedRoiArray::ConstPtr& stamped_roi_array);

    virtual void callback(const sensor_msgs::Image::ConstPtr& image_msg,
                          const autoware_msgs::StampedRoi::ConstPtr& stamped_roi_msg);
  }; // KcfTrackerNode


} // namespace trafficlight_recognizer

#endif
