#include "traffic_light_recognizer/feat_proj_bbox.h"

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

        mask_image_pub = pnh_.advertise<sensor_msgs::Image>("output", 1);
        nearest_roi_image_pub = pnh_.advertise<sensor_msgs::Image>("nearest_box", 1);
        vis_image_pub = pnh_.advertise<sensor_msgs::Image>("vis_image", 1);

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
        cv::Mat vis_image;
        float prev_z = 256 *256;

        autoware_msgs::DetectedObjectArray roi_image_array;
        kcf_ros::Rect croped_image_rect;
        kcf_ros::Rect info_msg;

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
            autoware_msgs::DetectedObject roi_image;

            roi_image.roi_image = *roi_msg;
            roi_image.image_frame = in_image_msg->header.frame_id;
            roi_image.x = lt.x;
            roi_image.y = lt.y;
            roi_image.width = rb.x;
            roi_image.height = rb.y;
            roi_image.pose.position.z = float(signal.z);
            roi_imageArray.objects.push_back(roi_image);

            if(z < prev_z){
                nearest_index = index;

                info_msg.x = lt.x;
                info_msg.y = lt.y;
                info_msg.width = rb.x;
                info_msg.height = rb.y;
                if (prev_signal != signal.signalId){
                    info_msg.changed = true;
                } else {
                    info_msg.changed = false;
                }

                if(debug_){
                    vis_image = roi_image.clone();
                    cv::putText(vis_image, std::to_string(signal.signalId),
                                cv::Point2i(20, 20),
                                cv::FONT_HERSHEY_DUPLEX, 0.8,
                                cv::Scalar(100,200, 0), 1);
                }
                prev_z = z;
            }
            index++;
        }

        mask_image_pub.publish(cv_bridge::CvImage(in_image_msg->header,
                                                  "mono8",
                                                  mask).toImageMsg());

        if(roi_image_array.objects.size() > 0){
            nearest_roi_image_pub.publish(roi_image_array.objects.at(nearest_index).roi_image);

            if(debug_){
                visImagePublisher.publish(cv_bridge::CvImage(in_image_msg->header,
                                                             "rgb8",
                                                             vis_image).toImageMsg());
            }
        }
    }
} // namespace trafficlight_recognizer
