#include "trafficlight_recognizer/kcf_tracker.h"

namespace trafficlight_recognizer
{

  double KcfTracker::calc_detection_score(const sensor_msgs::RegionOfInterest& box,
                                          const cv::Point2f& nearest_roi_image_center)
  {
    // double score = 0;
    // double w_score = 0.8;
    // double w_distance = 0;

    // bool gaussian_distance_ = false;
    // Eigen::Vector2d input_vec(box.x_offset + box.width * 0.5,
    //                           box.y_offset + box.height * 0.5);
    // Eigen::Vector2d center_vec(nearest_roi_image_center.x,
    //                            nearest_roi_image_center.y);

    // double w =(nearest_roi_image_center.x * 2);
    // double h =(nearest_roi_image_center.y * 2);

    // // calc gaussian distance
    // if (gaussian_distance_) {
    //   double likelihood;
    //   GaussianDistribution distribution;
    //   distribution.mean = center_vec;
    //   calc_gaussian(likelihood, input_vec, distribution);
    //   std::cerr << "likelihood: " << likelihood << std::endl;
    //   w_distance = likelihood;
    // } else {
    //   double diagonal = std::sqrt(w * w + h * h);
    //   w_distance = 1 - (input_vec - center_vec).norm() / diagonal;
    // }

    // score = (w_score + w_distance) / 2;

    // return score;
  }


  bool KcfTracker::boxesToBox(const autoware_msgs::StampedRoi& boxes,
                              const cv::Rect& roi_rect,
                              cv::Rect& output_box,
                              float& score)
  {
    // if (boxes.roi_array.size() == 0) {
    //   return false;
    // }

    // cv::Point2f nearest_roi_image_center(roi_rect.width * 0.5,
    //                                      roi_rect.height * 0.5);

    // float max_score = 0;
    // float min_distance = std::pow(24, 24);
    // cv::Rect box_on_nearest_roi_image;

    // for (sensor_msgs::RegionOfInterest box : boxes.roi_array) {

    //   float tmp_score = calc_detection_score(box, nearest_roi_image_center);
    //   if (tmp_score > max_score) {
    //     output_box = cv::Rect(box.x_offset, box.y_offset, box.width, box.height);
    //     score = tmp_score;
    //     max_score = tmp_score;
    //   }

    // }

    // return true;
  }


  float KcfTracker::check_confidence(const std::vector<cv::Rect> results,
                                     float& box_movement_ratio)
  {
    // box_movement_ratio = 0;

    // auto current_result = results.at(results.size() - 1);
    // auto prev_result = results.at(results.size() - 2);

    // float distance =
    //   cv::norm(cv::Point2f(current_result.x + current_result.width * 0.5,
    //                        current_result.y + current_result.height * 0.5) -
    //            cv::Point2f(prev_result.x + prev_result.width * 0.5,
    //                        prev_result.y + prev_result.height * 0.5));

    // float raw_image_diagonal = Eigen::Vector2d(original_image_size_.width, original_image_size_.height).norm();
    // box_movement_ratio = distance / raw_image_diagonal;
    // return utils_->sigmoid(box_movement_ratio);
  }


  bool KcfTracker::create_tracker_results_buffer(const cv::Rect& bb)
  {
    // try {
    //   if (tracker_results_buffer_.size() >= queue_size_)
    //     tracker_results_buffer_.erase(tracker_results_buffer_.begin());
    //   tracker_results_buffer_.push_back(bb);
    //   return true;
    // } catch (...) {
    //   ROS_ERROR("failed create tracker results buffer");
    //   return false;
    // }
  }


  bool KcfTracker::get_min_index(int& min_index)
  {
    // min_index = 0;
    // bool found_min_index = false;

    // if (image_stamps.empty()) {
    //   ROS_ERROR("image_stamps is empty");
    //   return false;
    // }

    // for (int i=image_stamps.size() - 1; i>=0; i--) {
    //   if (image_stamps.at(i) - boxes_array_stamp_ < 0) {
    //     found_min_index = true;
    //     if (std::abs(image_stamps.at(i+1) - boxes_array_stamp_) <
    //         std::abs(image_stamps.at(i) - boxes_array_stamp_)){
    //       min_index = i+1;
    //     } else {
    //       min_index = i;
    //     }
    //     break;
    //   }
    // }
    // return found_min_index;
  }


  bool KcfTracker::update_tracker(cv::Mat& image, cv::Rect& output_rect, const cv::Rect& roi_rect,
                                  float& box_movement_ratio, float& tracker_conf, float& tracking_time)
  {
    // try
    //   {
    //     double start_time = ros::Time::now().toSec();
    //     track(image);
    //     tracking_time = ros::Time::now().toSec() - start_time;

    //     BBox_c bb = getBBox();

    //     cv::Point lt(bb.cx - bb.w * 0.5, bb.cy - bb.h * 0.5);
    //     cv::Point rb(bb.cx + bb.w * 0.5, bb.cy + bb.h * 0.5);
    //     if (rb.x > image.cols) rb.x = image.cols;
    //     if (rb.y > image.rows) rb.y = image.rows;
    //     if (lt.x < 0)  lt.x = 0;
    //     if (lt.y < 0) lt.y = 0;
    //     int width = rb.x - lt.x;
    //     int height = rb.y - lt.y;
    //     output_rect = cv::Rect(lt.x, lt.y, width, height);

    //     // tracked rect is outside of roi_rect
    //     if (bb.cx < roi_rect.x ||
    //         bb.cy < roi_rect.y ||
    //         bb.cx > roi_rect.x + roi_rect.width ||
    //         bb.cy > roi_rect.y + roi_rect.height) {
    //       return false;
    //     }

    //     if(!create_tracker_results_buffer(output_rect))
    //       return false;

    //     box_movement_ratio = 0.0;
    //     tracker_conf = 0.0;
    //     if (tracker_results_buffer_.size() >= queue_size_) {
    //       tracker_conf = check_confidence(tracker_results_buffer_, box_movement_ratio);
    //     }

    //   }
    // catch (...)
    //   {
    //     ROS_ERROR("failed tracker update ");
    //     return false;
    //   }

    // return true;
  }


  bool KcfTracker::box_interpolation(const int min_index, int idx)
  {
    // for (int i=min_index; i<image_buffer.size(); i+=interpolation_frequency_)
    //   {
    //     if (i == min_index) {
    //       cv::Mat debug_image = image_buffer.at(i).clone();
    //       cv::Rect box_on_nearest_roi_image;
    //       float detection_score;

    //       if (boxesToBox(boxes_array_->stamped_rois.at(idx),
    //                      rect_buffer.at(i),
    //                      box_on_nearest_roi_image,
    //                      detection_score)) {
    //         cv::Rect init_box_on_raw_image(box_on_nearest_roi_image.x + rect_buffer.at(i).x - offset_,
    //                                        box_on_nearest_roi_image.y + rect_buffer.at(i).y - offset_,
    //                                        box_on_nearest_roi_image.width + offset_ * 2,
    //                                        box_on_nearest_roi_image.height + offset_ * 2);

    //         cv::Point lt(init_box_on_raw_image.x, init_box_on_raw_image.y);
    //         cv::Point rb(init_box_on_raw_image.x + init_box_on_raw_image.width,
    //                      init_box_on_raw_image.y + init_box_on_raw_image.height);
    //         if (rb.x > image_buffer.at(i).cols) rb.x = image_buffer.at(i).cols;
    //         if (rb.y > image_buffer.at(i).rows) rb.y = image_buffer.at(i).rows;
    //         if (lt.x < 0)  lt.x = 0;
    //         if (lt.y < 0) lt.y = 0;
    //         int width = rb.x - lt.x;
    //         int height = rb.y - lt.y;
    //         init_box_on_raw_image = cv::Rect(lt.x, lt.y, width, height);

    //         if (image_buffer.size() - min_index == 1) {
    //           cv::Mat croped_image = image_buffer.at(i)(init_box_on_raw_image).clone();
    //         }

    //         try {
    //           init(image_buffer.at(i), init_box_on_raw_image);
    //         } catch (...) {
    //           ROS_ERROR("failed tracker init ");
    //           return false;
    //         }
    //       } else {
    //         ROS_ERROR("failed convert boxes to box");
    //         return false;
    //       }
    //     } else {
    //       cv::Rect output_rect;
    //       float box_movement_ratio, tracker_conf, tracking_time;
    //       if (!update_tracker(image_buffer.at(i), output_rect, rect_buffer.at(i),
    //                           box_movement_ratio, tracker_conf, tracking_time))
    //         return false;
    //     }
    //   }

    // return true;

  }


  bool KcfTracker::create_buffer(const ImageInfoPtr& image_info)
  {
    // try {
    //   if (image_stamps.size() >= buffer_size_) {
    //     image_stamps.erase(image_stamps.begin());
    //     image_buffer.erase(image_buffer.begin());
    //     rect_buffer.erase(rect_buffer.begin());
    //   }
    //   image_stamps.push_back(image_info->stamp);
    //   image_buffer.push_back(image_info->image);
    //   rect_buffer.push_back(image_info->rect);
    //   return true;
    // } catch (...) {
    //   ROS_ERROR("failed stack image_info buffer");
    //   return false;
    // }
  }


  bool KcfTracker::clear_buffer()
  {
    // try
    //   {
    //     image_stamps.clear();
    //     image_buffer.clear();
    //     rect_buffer.clear();
    //     return true;
    //   }
    // catch (...)
    //   {
    //     ROS_ERROR("failed clear image_info buffer");
    //     return false;
    //   }
  }


  void KcfTracker::increment_cnt()
  {
    // prev_signal_ = signal_;
    // prev_boxes_callback_cnt_ = boxes_callback_cnt_;
    // cnt_++;
  }


  int KcfTracker::calc_offset(int x)
  {
    // int offset = int((9.0 / 110.0) * x - (340.0 / 110.0));
    // if (offset < 0) offset = 0;
    // return offset;
  }


  bool KcfTracker::track(const ImageInfoPtr& image_info,
                         sensor_msgs::RegionOfInterest& tracked_rect,
                         int idx)
  {
    // signal_ = image_info->signal;
    // signal_changed_ = signal_ != prev_signal_;
    // offset_ = calc_offset(tracked_rect.height);

    // if (signal_changed_) {
    //   clear_buffer();
    //   increment_cnt();
    //   track_flag_ = false;
    //   tracker_initialized_ = false;
    //   return false;
    // } else {
    //   create_buffer(image_info);
    //   track_flag_ = true;
    // }

    // float nearest_stamp = 0;
    // if (boxes_callback_cnt_ != prev_boxes_callback_cnt_) {
    //   double start_time = ros::Time::now().toSec();
    //   int min_index = 0;
    //   if (!get_min_index(min_index)) {
    //     ROS_WARN("cannot find correspond index ...");
    //     track_flag_ = false;
    //     increment_cnt();
    //     return false;
    //   }

    //   if (!box_interpolation(min_index, idx)) {
    //     increment_cnt();
    //     return false;
    //   }
    //   tracker_initialized_ = true;

    //   double total_time = ros::Time::now().toSec() - start_time;
    // } else if (prev_boxes_callback_cnt_ == 0) {
    //   tracker_initialized_ = false;
    //   track_flag_ = false;
    //   increment_cnt();
    //   return false;
    // } else {
    //   cv::Rect output_rect;
    //   float box_movement_ratio, tracker_conf, tracking_time;
    //   if (track_flag_ && tracker_initialized_) {

    //     if (!update_tracker(image_info->image, output_rect, image_info->rect,
    //                         box_movement_ratio, tracker_conf, tracking_time)) {
    //       increment_cnt();
    //       track_flag_ = false;
    //       tracker_initialized_ = false;
    //       return false;
    //     }
    //     if (tracker_conf < 0.8) {
    //       increment_cnt();
    //       track_flag_ = false;
    //       tracker_initialized_ = false;
    //       return false;
    //     }

    //   } else {
    //     increment_cnt();
    //     return false;
    //   }

    //   tracked_rect.x_offset = output_rect.x;
    //   tracked_rect.y_offset = output_rect.y;
    //   tracked_rect.width = output_rect.width;
    //   tracked_rect.height = output_rect.height;

    // }
    // increment_cnt();

    // return true;

  }

} // namespace trafficlight_recognizer
