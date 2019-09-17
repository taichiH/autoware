#ifndef TRAFFIC_LIGHT_CLASSIFIER_H
#define TRAFFIC_LIGHT_CLASSIFIER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>

#include <autoware_msgs/TrafficLightStates.h>
#include <autoware_msgs/TrafficLightState.h>
#include <autoware_msgs/LampState.h>

namespace tlr_classifier {

    class Utils {
    public:

        int get_area(const cv::Mat& image);

        template<typename T>
            bool in_vector(const T& c, const typename T::value_type& v);

        bool merge_msg(autoware_msgs::LampStateArray::ConstPtr& color_lamp_states,
                       autoware_msgs::LampStateArray::ConstPtr& arrow_lamp_states,
                       autoware_msgs::LampStateArray::ConstPtr& lamp_states);

    };

    class ColorClassifier {
    public:

        enum Lamp {
            GREEN = 0,
            YELLOW = 1,
            RED = 2,
        };

        // variables

        int buffer_size_ = 5;

        std::vector< std::vector<float> > ratios_buffer_;

        float ratios_thresh_ = 0.35;


        // funtions

        ColorClassifier() {}

        ~ColorClassifier();

        bool hsv_filter(const cv::Mat &input_image,
                        std::vector<cv::Mat> &output_images);

        bool get_color_ratios(const cv::Mat &input_images,
                              std::vector<float> &ratios);

        bool get_lamp_states(autoware_msgs::LampStateArray::ConstPtr>& states
                             std::vector<float> ratios);


    };

    class ArrowClassifier {
    public:
        enum Position {
            LEFTTOP,
            LEFTBOTTOM,
            RIGHTTOP,
            RIGHTBOTOM,
        }

        enum Lamp {
            LEFT = 0,
            RIGHT = 1,
            UP = 2,
            DOWN = 3,
        };

        // variables

        cv::Mat template_image_;

        int dilation_size_ = 3;

        // functions

        ArrowClassifier() {}

        ~ArrowClassifier();

    };



    class TrafficLightClassifierNode {
    public:

        // variables

        ros::NodeHandle nh_;

        /* typedef std::shared_ptr<ColorClassifier> ColorClassifierPtr; */

        /* typedef std::shared_ptr<ArrowClassifier> ArrowClassifierPtr; */

        ColorClassifier color_classifier_;

        ArrowClassifier arrow_classifier_;

        Utils utils_;

        ros::Subscriber image_sub_;

        ros::Publisher image_pub_;

        ros::Publisher trafficlight_states_pub_;


        // functions

        void run();


    private:

        std::vector<cv::Scalar> lower_ranges_;

        std::vector<cv::Scalar> upper_ranges_;

    }; // TrafficLightClassifier

} // tlr_classifier


#endif
