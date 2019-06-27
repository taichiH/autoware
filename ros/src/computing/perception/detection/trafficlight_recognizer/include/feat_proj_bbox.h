#ifndef _TRAFFICLIGHT_RECOGNIXER_FEAT_PROJ_BBOX_
#define _TRAFFICLIGHT_RECOGNIXER_FEAT_PROJ_BBOX_

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
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <kcf_ros/Rect.h>

namespace trafficlight_recognizer
{
    class FeatProjBBox : public nodelet::Nodelet
    {
    public:
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image,
            autoware_msgs::Signals
            > SyncPolicy;
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image,
            kcf_ros::Rect
            > ApproximateSyncPolicy;
    protected:
        float z_max = 100;
        float z_min = 20;
        int w_max = 350;
        int w_min = 50;
        int h_max = 200;
        int h_min = 50;

        bool debug_ = true;

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Publisher mask_image_pub;
        ros::Publisher nearest_roi_image_pub;
        ros::Publisher vis_image_pub;

        message_filters::Subscriber<sensor_msgs::Image> sub_input_image;
        message_filters::Subscriber<autoware_msgs::Signals> sub_input_signal;
        sub_input_image.subscribe(pnh_, "input_signal", 1);
        sub_input_signal.subscribe(pnh_, "input_image", 1);

        boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_
            = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1);
        sync_->connectInput(sub_input_image, sub_input_signal);
        sync_->registerCallback(callback);

    private:
    } // class FeatProjBBox
} // namespace trafficlight_recognizer

#endif
