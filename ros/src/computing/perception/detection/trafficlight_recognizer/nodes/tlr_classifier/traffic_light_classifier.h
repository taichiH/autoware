#ifndef TRAFFIC_LIGHT_CLASSIFIER_H
#define TRAFFIC_LIGHT_CLASSIFIER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>


#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>

#include <autoware_msgs/TrafficLightStateArray.h>
#include <autoware_msgs/TrafficLightState.h>
#include <autoware_msgs/LampStateArray.h>
#include <autoware_msgs/LampState.h>
#include <autoware_msgs/StampedRoi.h>

namespace tlr_classifier
{

    class Utils
    {
    public:

        int get_area(const cv::Mat& image);

        template<typename T>
            bool in_vector(const T& c, const typename T::value_type& v);

        bool merge_msg(const autoware_msgs::LampStateArray& color_lamp_states,
                       const autoware_msgs::LampStateArray& arrow_lamp_states,
                       autoware_msgs::LampStateArray& lamp_states);

    };

    class ColorClassifier
    {
    public:

        enum HSV {
            Hue = 0,
            Sat = 1,
            Val = 2,
        };

        enum Lamp {
            GREEN = 0,
            YELLOW = 1,
            RED = 2,
        };

        // variables

        int buffer_size_ = 5;

        std::vector< std::vector<float> > ratios_buffer_;

        float ratios_thresh_ = 0.35;

        Utils utils_;

        // funtions

        ColorClassifier() {}

        ~ColorClassifier() {};

        bool get_params(const ros::NodeHandle& pnh);

        bool hsv_filter(const cv::Mat &input_image,
                        std::vector<cv::Mat> &output_images);

        bool get_color_ratios(const std::vector<cv::Mat> &input_images,
                              std::vector<float> &ratios);

        float moving_average_filter(std::vector<float> ratios);

        bool get_lamp_states(autoware_msgs::LampStateArray& states,
                             std::vector<float> ratios);


    private:

        std::vector<cv::Scalar> lower_ranges_;

        std::vector<cv::Scalar> upper_ranges_;

    };

    class ArrowClassifier
    {
    public:
        enum Position {
            LEFTTOP,
            LEFTBOTTOM,
            RIGHTTOP,
            RIGHTBOTTOM,
        };

        enum Lamp {
            LEFT = 0,
            RIGHT = 1,
            UP = 2,
            DOWN = 3,
        };

        // variables

        cv::Mat template_image_;

        float pair_thresh_ = 3;

        float area_thresh_ = 400;

        int dilation_size_ = 3;

        Utils utils_;

        // functions

        ArrowClassifier() {}

        ~ArrowClassifier() {};

        bool get_shape(cv::Mat& image, std::vector<std::vector<cv::Point> >& contours);

        bool calc_arrow_direction(const cv::Mat& image, const cv::Rect rect, autoware_msgs::LampState& state);

        bool load_template(const std::string path);

        bool get_params(const ros::NodeHandle& pnh);

        bool get_arrow_rects
            (const std::vector< std::vector<cv::Point> >& image_contours,
             const std::vector< std::vector<cv::Point> >& template_contours,
             const cv::Mat& image,
             std::vector<cv::Rect>& rects,
             cv::Mat& debug_image);

        bool get_index_pairs
            (const std::vector< std::vector<cv::Point> >& image_contours,
             const std::vector< std::vector<cv::Point> >& template_contours,
             std::vector<std::pair<int, double>>& index_pairs);

    };


    class TrafficLightClassifierNode
    {
    public:

        // variables

        ros::NodeHandle nh_;

        typedef message_filters::sync_policies::ExactTime<
            sensor_msgs::Image,
            autoware_msgs::StampedRoi
            > SyncPolicy;
        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image,
            autoware_msgs::StampedRoi
            > ApproximateSyncPolicy;

        boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
        boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;
        message_filters::Subscriber<sensor_msgs::Image> image_sub_;
        message_filters::Subscriber<autoware_msgs::StampedRoi> stamped_roi_sub_;

        bool is_approximate_sync_ = false;

        ColorClassifier color_classifier_;

        ArrowClassifier arrow_classifier_;

        Utils utils_;

        ros::Publisher image_pub_;

        ros::Publisher trafficlight_state_array_pub_;

        // functions

        void callback
            (const sensor_msgs::Image::ConstPtr& image_msg,
             const autoware_msgs::StampedRoi::ConstPtr& stamped_roi_msg);

        bool classify
            (const cv::Mat& image,
             autoware_msgs::LampStateArray& lamp_state_array,
             cv::Mat& debug_image);

        void run();


    }; // TrafficLightClassifier

} // tlr_classifier


#endif
