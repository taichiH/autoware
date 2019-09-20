#ifndef _TLR_TRACKER_KCF_TRACKER_
#define _TLR_TRACKER_KCF_TRACKER_

#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "trafficlight_recognizer/utils.h"

#include "kcf.h"

namespace trafficlight_recognizer
{

  class KcfTracker : public KCF_Tracker
  {
  protected:
    typedef std::shared_ptr<ImageInfo> ImageInfoPtr;

    typedef std::shared_ptr<Utils> UtilsPtr;

    ImageInfoPtr image_info_;

    UtilsPtr utils_;

    double pi = 3.141592653589793;

    int frames = 0;

    int callback_count_ = 0;

    bool tracker_initialized_ = false;

    bool track_flag_ = false;

    bool signal_changed_ = false;

    int signal_ = 0;

    int prev_signal_ = 0;

    int queue_size_ = 2;

    float box_movement_thresh_ = 0.02;

    int cnt_ = 0;

    int buffer_size_ = 100;

    std::vector<double> image_stamps;

    std::vector<cv::Mat> image_buffer;

    std::vector<cv::Rect> rect_buffer;

    std::vector<cv::Rect> tracker_results_buffer_;

    virtual bool boxesToBox(const autoware_msgs::StampedRoi& boxes,
                            const cv::Rect& roi_rect,
                            cv::Rect& output_box,
                            float& score);

    virtual bool get_min_index(int& min_index);

    virtual bool box_interpolation(const int min_index, int idx);

    virtual bool create_buffer(const ImageInfoPtr& image_info);

    virtual bool clear_buffer();

    virtual bool create_tracker_results_buffer(const cv::Rect& bb);

    virtual float check_confidence(const std::vector<cv::Rect> results,
                                   float& box_movement_ratio);

    virtual bool update_tracker(cv::Mat& image, cv::Rect& output_rect, const cv::Rect& roi_rect,
                                float& box_movement_ratio, float& tracker_conf, float& tracking_time);

    virtual double calc_detection_score(const sensor_msgs::RegionOfInterest& box,
                                        const cv::Point2f& nearest_roi_image_center);


    virtual bool run(const ImageInfoPtr& image_info,
                     sensor_msgs::RegionOfInterest& tracked_rect,
                     int idx);

    virtual void increment_cnt();

    virtual int calc_offset(int x);

  private:

  }; // class KcfTracker



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

    /* typedef std::shared_ptr<KcfTracker> KcfTrackerPtr; */


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

    /* KcfTrackerPtr kcf_tracker_; */

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
