/**
 * @file image_utils.h
 * @brief ROS Image <-> OpenCV BGR conversion and camera frame capture helpers.
 *
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace body_pose {

cv::Mat ImageMsgToBgr(const sensor_msgs::msg::Image& msg);

sensor_msgs::msg::Image BgrToImageMsg(
    const cv::Mat& bgr,
    const std_msgs::msg::Header& header,
    const std::string& encoding = "bgr8");

bool CaptureCameraFrame(
    cv::VideoCapture& cap,
    sensor_msgs::msg::Image& out_msg,
    const std_msgs::msg::Header& header);

}  // namespace body_pose

#endif  // YOLO_GENERAL_IMAGE_UTILS_H
