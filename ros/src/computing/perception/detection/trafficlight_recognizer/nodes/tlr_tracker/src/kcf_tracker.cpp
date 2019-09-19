#include "tlr_tracker/kcf_tracker.h"
#include <time.h>

namespace trafficlight_recognizer
{
    void KcfTrackerROS::onInit()
    {
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();

        pnh_.getParam("debug_log", debug_log_);
        pnh_.getParam("kernel_sigma", kernel_sigma_);
        pnh_.getParam("cell_size", cell_size_);
        pnh_.getParam("num_scales", num_scales_);
        pnh_.getParam("approximate_sync", is_approximate_sync_);
        pnh_.getParam("interpolation_frequency", interpolation_frequency_);
        pnh_.getParam("offset", offset_);

        // publisher
        debug_image_pub_ =  pnh_.advertise<sensor_msgs::Image>("output_image", 1);
        output_rects_pub_ = pnh_.advertise<autoware_msgs::StampedRoi>("output_rect", 1);

        // subscriber for single callback
        boxes_sub = pnh_.subscribe
            ("input_yolo_detected_boxes", 1, &KcfTrackerROS::boxes_callback, this);

        // subscribers for message filter callback
        image_sub_.subscribe
            (pnh_, "input_raw_image", 1);
        stamped_roi_sub_.subscribe
            (pnh_, "input_nearest_roi_rect", 1);

        // register callback
        if (is_approximate_sync_){
            approximate_sync_ =
                boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(1000);
            approximate_sync_->connectInput(image_sub_, stamped_roi_sub_);
            approximate_sync_->registerCallback
                (boost::bind(&KcfTrackerROS::callback, this, _1, _2));
        } else {
            sync_  =
                boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
            sync_->connectInput(image_sub_, stamped_roi_sub_);
            sync_->registerCallback
                (boost::bind(&KcfTrackerROS::callback, this, _1, _2));
        }
    }

    void KcfTrackerROS::load_image(cv::Mat& image, const sensor_msgs::Image::ConstPtr& image_msg)
    {
        try {
            cv_bridge::CvImagePtr cv_image =
                cv_bridge::toCvCopy(image_msg, "bgr8");
            image = cv_image->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("failed convert image from sensor_msgs::Image to cv::Mat");
            return;
        }
    }

    void KcfTrackerROS::publish_messages(const cv::Mat& image, const cv::Rect& rect)
    {
    }

    bool KcfTrackerROS::calc_gaussian(double& likelihood,
                                      const Eigen::Vector2d& input_vec,
                                      const GaussianDistribution& distribution)
    {
        likelihood = 0;
        try {
            double tmp1 = 1 / (std::pow((2 * pi), distribution.d) * distribution.cov_det_sqrt);
            auto v_t = (input_vec - distribution.mean).transpose();
            auto v = (distribution.cov_inverse * (input_vec - distribution.mean));
            double tmp2 = std::exp(-0.5 * v_t * v);

            likelihood = tmp1 * tmp2;
            return true;
        } catch(...) {
            ROS_ERROR("error in %s", __func__);
            return false;
        }
    }


    double KcfTrackerROS::calc_detection_score(const sensor_msgs::RegionOfInterest& box,
                                               const cv::Point2f& nearest_roi_image_center)
    {
        double score = 0;
        double w_score = 0.8;
        double w_distance = 0;

        bool gaussian_distance_ = false;
        Eigen::Vector2d input_vec(box.x_offset + box.width * 0.5,
                                  box.y_offset + box.height * 0.5);
        Eigen::Vector2d center_vec(nearest_roi_image_center.x,
                                   nearest_roi_image_center.y);

        double w =(nearest_roi_image_center.x * 2);
        double h =(nearest_roi_image_center.y * 2);

        // calc gaussian distance
        if (gaussian_distance_) {
            double likelihood;
            GaussianDistribution distribution;
            distribution.mean = center_vec;
            calc_gaussian(likelihood, input_vec, distribution);
            std::cerr << "likelihood: " << likelihood << std::endl;
            w_distance = likelihood;
        } else {
            double diagonal = std::sqrt(w * w + h * h);
            w_distance = 1 - (input_vec - center_vec).norm() / diagonal;
        }

        score = (w_score + w_distance) / 2;

        return score;
    }

    bool KcfTrackerROS::boxesToBox(const autoware_msgs::StampedRoi& boxes,
                                   const cv::Rect& roi_rect,
                                   cv::Rect& output_box,
                                   float& score)
    {
        if (boxes.roi_array.size() == 0) {
            return false;
        }

        cv::Point2f nearest_roi_image_center(roi_rect.width * 0.5,
                                             roi_rect.height * 0.5);

        float max_score = 0;
        float min_distance = std::pow(24, 24);
        cv::Rect box_on_nearest_roi_image;

        for (sensor_msgs::RegionOfInterest box : boxes.roi_array) {

            float tmp_score = calc_detection_score(box, nearest_roi_image_center);
            if (tmp_score > max_score) {
                output_box = cv::Rect(box.x_offset, box.y_offset, box.width, box.height);
                score = tmp_score;
                max_score = tmp_score;
            }

        }

        return true;
    }

    float KcfTrackerROS::check_confidence(const std::vector<cv::Rect> results,
                                          float& box_movement_ratio)
    {

        box_movement_ratio = 0;

        auto current_result = results.at(results.size() - 1);
        auto prev_result = results.at(results.size() - 2);

        float distance =
            cv::norm(cv::Point2f(current_result.x + current_result.width * 0.5,
                                 current_result.y + current_result.height * 0.5) -
                     cv::Point2f(prev_result.x + prev_result.width * 0.5,
                                 prev_result.y + prev_result.height * 0.5));

        float raw_image_diagonal = Eigen::Vector2d(raw_image_width_, raw_image_height_).norm();
        box_movement_ratio = distance / raw_image_diagonal;
        return sigmoid(box_movement_ratio);
    }

    float KcfTrackerROS::sigmoid(float x, float a) {
        return(1 - (1.0 / (1.0 + exp(a * (-x + box_movement_thresh_)))));
    }

    bool KcfTrackerROS::create_tracker_results_buffer(const cv::Rect& bb)
    {
        try {
            if (tracker_results_buffer_.size() >= queue_size_)
                tracker_results_buffer_.erase(tracker_results_buffer_.begin());
            tracker_results_buffer_.push_back(bb);
            return true;
        } catch (...) {
            ROS_ERROR("failed create tracker results buffer");
            return false;
        }
    }

    bool KcfTrackerROS::get_min_index(int& min_index){
        min_index = 0;
        bool found_min_index = false;

        if (image_stamps.empty()) {
            ROS_ERROR("image_stamps is empty");
            return false;
        }

        for (int i=image_stamps.size() - 1; i>=0; i--) {
            if (image_stamps.at(i) - boxes_array_stamp_ < 0) {
                found_min_index = true;
                if (std::abs(image_stamps.at(i+1) - boxes_array_stamp_) <
                    std::abs(image_stamps.at(i) - boxes_array_stamp_)){
                    min_index = i+1;
                } else {
                    min_index = i;
                }
                break;
            }
        }
        return found_min_index;
    }

    bool KcfTrackerROS::update_tracker(cv::Mat& image, cv::Rect& output_rect, const cv::Rect& roi_rect,
                                       float& box_movement_ratio, float& tracker_conf, float& tracking_time) {
        try {
            double start_time = ros::Time::now().toSec();
            tracker.track(image);
            tracking_time = ros::Time::now().toSec() - start_time;

            if (debug_log_)
                ROS_INFO("tracking time: %.2lf [ms]", tracking_time * 1000);

            BBox_c bb = tracker.getBBox();

            cv::Point lt(bb.cx - bb.w * 0.5, bb.cy - bb.h * 0.5);
            cv::Point rb(bb.cx + bb.w * 0.5, bb.cy + bb.h * 0.5);
            if (rb.x > image.cols) rb.x = image.cols;
            if (rb.y > image.rows) rb.y = image.rows;
            if (lt.x < 0)  lt.x = 0;
            if (lt.y < 0) lt.y = 0;
            int width = rb.x - lt.x;
            int height = rb.y - lt.y;
            output_rect = cv::Rect(lt.x, lt.y, width, height);

            // tracked rect is outside of roi_rect
            if (bb.cx < roi_rect.x ||
                bb.cy < roi_rect.y ||
                bb.cx > roi_rect.x + roi_rect.width ||
                bb.cy > roi_rect.y + roi_rect.height) {
                return false;
            }

            if(!create_tracker_results_buffer(output_rect))
                return false;

            box_movement_ratio = 0.0;
            tracker_conf = 0.0;
            if (tracker_results_buffer_.size() >= queue_size_) {
                tracker_conf = check_confidence(tracker_results_buffer_, box_movement_ratio);
            }

        } catch (...) {
            ROS_ERROR("failed tracker update ");
            return false;
        }

        return true;
    }


    bool KcfTrackerROS::box_interpolation(const int min_index,
                                          const sensor_msgs::RegionOfInterest& projected_roi,
                                          int idx){
        if (debug_log_)
            ROS_INFO("buffer_size: %d, freq: %d, calc size: %d",
                     image_buffer.size() - min_index,
                     interpolation_frequency_,
                     int((image_buffer.size() - min_index - 1) / interpolation_frequency_));

        for (int i=min_index; i<image_buffer.size(); i+=interpolation_frequency_) {
            if (i == min_index) {
                cv::Mat debug_image = image_buffer.at(i).clone();
                cv::Rect box_on_nearest_roi_image;
                float detection_score;

                if (boxesToBox(boxes_array_->stamped_rois.at(idx),
                               rect_buffer.at(i),
                               box_on_nearest_roi_image,
                               detection_score)) {
                    cv::Rect init_box_on_raw_image(box_on_nearest_roi_image.x + rect_buffer.at(i).x - offset_,
                                                   box_on_nearest_roi_image.y + rect_buffer.at(i).y - offset_,
                                                   box_on_nearest_roi_image.width + offset_ * 2,
                                                   box_on_nearest_roi_image.height + offset_ * 2);

                    cv::Point lt(init_box_on_raw_image.x, init_box_on_raw_image.y);
                    cv::Point rb(init_box_on_raw_image.x + init_box_on_raw_image.width,
                                 init_box_on_raw_image.y + init_box_on_raw_image.height);
                    if (rb.x > image_buffer.at(i).cols) rb.x = image_buffer.at(i).cols;
                    if (rb.y > image_buffer.at(i).rows) rb.y = image_buffer.at(i).rows;
                    if (lt.x < 0)  lt.x = 0;
                    if (lt.y < 0) lt.y = 0;
                    int width = rb.x - lt.x;
                    int height = rb.y - lt.y;
                    init_box_on_raw_image = cv::Rect(lt.x, lt.y, width, height);

                    if (image_buffer.size() - min_index == 1) {
                        cv::Mat croped_image = image_buffer.at(i)(init_box_on_raw_image).clone();
                    }

                    try {
                        tracker.init(image_buffer.at(i), init_box_on_raw_image);
                    } catch (...) {
                        ROS_ERROR("failed tracker init ");
                        return false;
                    }
                } else {
                    ROS_ERROR("failed convert boxes to box");
                    return false;
                }
            } else {
                cv::Rect output_rect;
                float box_movement_ratio, tracker_conf, tracking_time;
                if (!update_tracker(image_buffer.at(i), output_rect, rect_buffer.at(i),
                                    box_movement_ratio, tracker_conf, tracking_time))
                    return false;
            }
        }
        return true;
    }

    bool KcfTrackerROS::create_buffer(const ImageInfoPtr& image_info){
        try {
            if (image_stamps.size() >= buffer_size_) {
                image_stamps.erase(image_stamps.begin());
                image_buffer.erase(image_buffer.begin());
                rect_buffer.erase(rect_buffer.begin());
            }
            image_stamps.push_back(image_info->stamp);
            image_buffer.push_back(image_info->image);
            rect_buffer.push_back(image_info->rect);
            return true;
        } catch (...) {
            ROS_ERROR("failed stack image_info buffer");
            return false;
        }
    }

    bool KcfTrackerROS::clear_buffer() {
        try {
            image_stamps.clear();
            image_buffer.clear();
            rect_buffer.clear();
            return true;
        } catch (...) {
            ROS_ERROR("failed clear image_info buffer");
            return false;
        }
    }

    void KcfTrackerROS::increment_cnt() {
        prev_signal_ = signal_;
        prev_boxes_callback_cnt_ = boxes_callback_cnt_;
        cnt_++;
    }

    int KcfTrackerROS::calc_offset(int x) {
        int offset = int((9.0 / 110.0) * x - (340.0 / 110.0));
        if (offset < 0) offset = 0;
        return offset;
    }


    bool KcfTrackerROS::track(const ImageInfoPtr& image_info,
                              const sensor_msgs::RegionOfInterest& projected_roi,
                              sensor_msgs::RegionOfInterest& tracked_rect,
                              int idx)
    {
        signal_ = image_info->signal;
        signal_changed_ = signal_ != prev_signal_;
        offset_ = calc_offset(tracked_rect.height);

        if (signal_changed_) {
            clear_buffer();
            increment_cnt();
            track_flag_ = false;
            tracker_initialized_ = false;
            return false;
        } else {
            create_buffer(image_info);
            track_flag_ = true;
        }

        float nearest_stamp = 0;
        if (boxes_callback_cnt_ != prev_boxes_callback_cnt_) {
            double start_time = ros::Time::now().toSec();
            int min_index = 0;
            if (!get_min_index(min_index)) {
                ROS_WARN("cannot find correspond index ...");
                track_flag_ = false;
                increment_cnt();
                return false;
            }

            if (!box_interpolation(min_index, projected_roi, idx)) {
                increment_cnt();
                return false;
            }
            tracker_initialized_ = true;

            double total_time = ros::Time::now().toSec() - start_time;
            if (debug_log_)
                ROS_INFO("interpolation total time: %.2lf [ms]", total_time * 1000);
        } else if (prev_boxes_callback_cnt_ == 0) {
            tracker_initialized_ = false;
            track_flag_ = false;
            increment_cnt();
            return false;
        } else {
            cv::Rect output_rect;
            float box_movement_ratio, tracker_conf, tracking_time;
            if (track_flag_ && tracker_initialized_) {

                if (!update_tracker(image_info->image, output_rect, image_info->rect,
                                    box_movement_ratio, tracker_conf, tracking_time)) {
                    increment_cnt();
                    track_flag_ = false;
                    tracker_initialized_ = false;
                    return false;
                }
                if (tracker_conf < 0.8) {
                    increment_cnt();
                    track_flag_ = false;
                    tracker_initialized_ = false;
                    return false;
                }

            } else {
                increment_cnt();
                return false;
            }

            tracked_rect.x_offset = output_rect.x;
            tracked_rect.y_offset = output_rect.y;
            tracked_rect.width = output_rect.width;
            tracked_rect.height = output_rect.height;

        }
        increment_cnt();

        return true;
    }

    void KcfTrackerROS::boxes_callback(const autoware_msgs::StampedRoiArray::ConstPtr& boxes_array){
        boost::mutex::scoped_lock lock(mutex_);
        boxes_array_ = boxes_array;
        boxes_array_stamp_ = boxes_array_->header.stamp.toSec();
        boxes_callback_cnt_++;
    }


    void KcfTrackerROS::callback(const sensor_msgs::Image::ConstPtr& image_msg,
                                 const autoware_msgs::StampedRoi::ConstPtr& projected_roi_msg)
    {
        boost::mutex::scoped_lock lock(mutex_);

        cv::Mat image;
        load_image(image, image_msg);
        raw_image_width_, raw_image_height_ = image.cols, image.rows;

        autoware_msgs::StampedRoi output_rects;
        for (int i=0; i<projected_roi_msg->roi_array.size(); ++i)
            {
                sensor_msgs::RegionOfInterest projected_roi =
                    projected_roi_msg->roi_array.at(i);
                cv::Rect roi = cv::Rect(projected_roi.x_offset,
                                        projected_roi.y_offset,
                                        projected_roi.width,
                                        projected_roi.height);
                cv::Mat croped_image = image(roi);
                ImageInfoPtr image_info(new ImageInfo(croped_image,
                                                      roi,
                                                      projected_roi_msg->signal_id,
                                                      image_msg->header.stamp.toSec()));
                sensor_msgs::RegionOfInterest tracked_rect;
                track(image_info, projected_roi, tracked_rect, i);
                output_rects.roi_array.push_back(tracked_rect);
            }


        output_rects.header = header_;
        output_rects_pub_.publish(output_rects);
        debug_image_pub_.publish(cv_bridge::CvImage(header_,
                                                    sensor_msgs::image_encodings::BGR8,
                                                    image).toImageMsg());
    }
} // namespace trafficlight_recognizer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(trafficlight_recognizer::KcfTrackerROS, nodelet::Nodelet)
