/**
 * @file body_pose_node.cpp
 * @brief Body pose estimation node: subscribes to image, runs pose model,
 *        publishes BodyPoseArray and debug image.
 *
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "body_pose/msg/body_pose_array.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "body_pose/image_utils.h"
#include "vision_service.h"

namespace {

constexpr int kNumKeypoints = 17;  // COCO pose

}  // namespace

class BodyPoseNode : public rclcpp::Node {
    public:
    BodyPoseNode() : Node("body_pose_node") {
        std::string config_path = declare_parameter<std::string>("config_path", "");
        const bool lazy_load = declare_parameter<bool>("lazy_load", true);
        score_threshold_ = declare_parameter<double>("score_threshold", 0.25);
        image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
        body_poses_topic_ =
            declare_parameter<std::string>("body_poses_topic", "/perception/body_poses");
        debug_image_topic_ =
            declare_parameter<std::string>("debug_image_topic", "/body_pose/debug_image");
        use_camera_ = declare_parameter<bool>("use_camera", true);
        camera_id_ = declare_parameter<int>("camera_id", 0);
        camera_fps_ = declare_parameter<double>("camera_fps", 30.0);

        if (config_path.empty()) {
            config_path = GetDefaultConfigPath();
        }
        if (config_path.empty()) {
            throw std::runtime_error(
                "config_path is empty. Set config_path to vision model yaml "
                "(e.g. share/body_pose/config/yolov8_pose.yaml)");
        }

        service_ = VisionService::Create(config_path, "", lazy_load);
        if (!service_) {
            throw std::runtime_error("VisionService::Create failed: " +
                                        VisionService::LastCreateError());
        }

        body_poses_pub_ = create_publisher<body_pose::msg::BodyPoseArray>(body_poses_topic_, 10);
        debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_,
                                                                rclcpp::SensorDataQoS());

        if (use_camera_) {
            image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_,
                                                                    rclcpp::SensorDataQoS());
            cap_.open(camera_id_);
            if (!cap_.isOpened()) {
                throw std::runtime_error("body_pose: cannot open camera id=" +
                                            std::to_string(camera_id_));
            }
            const int period_ms =
                (camera_fps_ > 1.0) ? static_cast<int>(1000.0 / camera_fps_) : 33;
            camera_timer_ = create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&BodyPoseNode::OnCameraTimer, this));
            RCLCPP_INFO(get_logger(),
                        "body_pose_node: use_camera=true, publishing to %s, camera_id=%d",
                        image_topic_.c_str(), camera_id_);
        } else {
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_, rclcpp::SensorDataQoS(),
                std::bind(&BodyPoseNode::OnImage, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(),
                        "body_pose_node: use_camera=false, subscribing to %s",
                        image_topic_.c_str());
        }

        RCLCPP_INFO(get_logger(),
                    "body_pose_node started, config=%s, body_poses_topic=%s",
                    config_path.c_str(), body_poses_topic_.c_str());
    }

    private:
    static std::string GetDefaultConfigPath() {
        try {
            return ament_index_cpp::get_package_share_directory("body_pose") +
                    "/config/yolov8_pose.yaml";
        } catch (...) {
            return "";
        }
    }

    void OnCameraTimer() {
        std_msgs::msg::Header header;
        header.stamp = now();
        header.frame_id = "camera";

        sensor_msgs::msg::Image img_msg;
        if (!body_pose::CaptureCameraFrame(cap_, img_msg, header)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "camera read empty frame");
            return;
        }
        cv::Mat bgr = body_pose::ImageMsgToBgr(img_msg);
        if (bgr.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                    "camera frame conversion failed");
            return;
        }

        image_pub_->publish(img_msg);

        std::vector<VisionServiceResult> results;
        if (service_->InferImage(bgr, &results) != VISION_SERVICE_OK) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "infer failed: %s",
                                    service_->LastError().c_str());
            return;
        }
        PublishBodyPoses(results, header);
        PublishDebugImage(header, bgr);
    }

    void OnImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat bgr = body_pose::ImageMsgToBgr(*msg);
        if (bgr.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                    "invalid or unsupported image (encoding=%s)",
                                    msg->encoding.c_str());
            return;
        }

        std::vector<VisionServiceResult> results;
        if (service_->InferImage(bgr, &results) != VISION_SERVICE_OK) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "infer failed: %s",
                                    service_->LastError().c_str());
            return;
        }

        PublishBodyPoses(results, msg->header);
        PublishDebugImage(msg, bgr);
    }

    void PublishBodyPoses(const std::vector<VisionServiceResult>& results,
                            const std_msgs::msg::Header& image_header) {
        body_pose::msg::BodyPoseArray out;
        out.header = image_header;
        out.poses.clear();

        for (size_t i = 0; i < results.size(); ++i) {
            const auto& r = results[i];
            if (r.score < static_cast<float>(score_threshold_)) continue;

            body_pose::msg::BodyPose pose;
            pose.bbox.x_offset = static_cast<uint32_t>(std::max(0.0f, r.x1));
            pose.bbox.y_offset = static_cast<uint32_t>(std::max(0.0f, r.y1));
            pose.bbox.width = static_cast<uint32_t>(std::max(0.0f, r.x2 - r.x1));
            pose.bbox.height = static_cast<uint32_t>(std::max(0.0f, r.y2 - r.y1));
            pose.confidence = r.score;
            pose.track_id = r.track_id;

            pose.keypoints.resize(static_cast<size_t>(kNumKeypoints));
            for (int k = 0; k < kNumKeypoints; ++k) {
                auto& kp = pose.keypoints[static_cast<size_t>(k)];
                kp.id = static_cast<uint8_t>(k);
                if (k < static_cast<int>(r.keypoints.size())) {
                    kp.position.x = r.keypoints[static_cast<size_t>(k)].x;
                    kp.position.y = r.keypoints[static_cast<size_t>(k)].y;
                    kp.position.z = 0.0;
                    kp.confidence =
                        r.keypoints[static_cast<size_t>(k)].visibility;
                } else {
                    kp.position.x = kp.position.y = kp.position.z = 0.0f;
                    kp.confidence = 0.0f;
                }
            }
            out.poses.push_back(pose);
        }

        body_poses_pub_->publish(out);
    }

    void PublishDebugImage(const std_msgs::msg::Header& header, const cv::Mat& bgr) {
        cv::Mat out_image;
        if (service_->Draw(bgr, &out_image) != VISION_SERVICE_OK) return;
        if (out_image.empty() || out_image.rows != bgr.rows || out_image.cols != bgr.cols)
            return;
        debug_pub_->publish(body_pose::BgrToImageMsg(out_image, header, "bgr8"));
    }

    void PublishDebugImage(const sensor_msgs::msg::Image::SharedPtr& src, const cv::Mat& bgr) {
        PublishDebugImage(src->header, bgr);
    }

    std::unique_ptr<VisionService> service_;
    double score_threshold_{0.25};
    std::string image_topic_;
    std::string body_poses_topic_;
    std::string debug_image_topic_;
    bool use_camera_{true};
    int camera_id_{0};
    double camera_fps_{30.0};
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr camera_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<body_pose::msg::BodyPoseArray>::SharedPtr body_poses_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<BodyPoseNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("body_pose_node"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
