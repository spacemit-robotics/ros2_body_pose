/**
 * @file image_utils.cpp
 * @brief Implementation of ROS Image <-> OpenCV BGR and camera capture.
 *
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "body_pose/image_utils.h"

#include <cstring>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace body_pose {

cv::Mat ImageMsgToBgr(const sensor_msgs::msg::Image& msg) {
    if (msg.width <= 0 || msg.height <= 0 || msg.data.empty()) {
        return cv::Mat();
    }

    if (msg.encoding == "bgr8") {
        cv::Mat bgr(msg.height, msg.width, CV_8UC3,
                    const_cast<unsigned char*>(msg.data.data()),
                    static_cast<size_t>(msg.step));
        return bgr.clone();
    } else if (msg.encoding == "rgb8") {
        cv::Mat rgb(msg.height, msg.width, CV_8UC3,
                    const_cast<unsigned char*>(msg.data.data()),
                    static_cast<size_t>(msg.step));
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }

    return cv::Mat();
}

sensor_msgs::msg::Image BgrToImageMsg(
    const cv::Mat& bgr,
    const std_msgs::msg::Header& header,
    const std::string& encoding) {
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = static_cast<uint32_t>(bgr.rows);
    msg.width = static_cast<uint32_t>(bgr.cols);
    msg.encoding = encoding;
    msg.step = static_cast<uint32_t>(bgr.step);
    msg.is_bigendian = 0;

    msg.data.resize(static_cast<size_t>(bgr.rows) * bgr.step);
    if (bgr.isContinuous()) {
        std::memcpy(msg.data.data(), bgr.ptr(), msg.data.size());
    } else {
        for (int r = 0; r < bgr.rows; ++r) {
            std::memcpy(msg.data.data() + static_cast<size_t>(r) * bgr.step,
                        bgr.ptr(r), static_cast<size_t>(bgr.step));
        }
    }

    return msg;
}

bool CaptureCameraFrame(
    cv::VideoCapture& cap,
    sensor_msgs::msg::Image& out_msg,
    const std_msgs::msg::Header& header) {
    cv::Mat bgr;
    if (!cap.read(bgr) || bgr.empty()) {
        return false;
    }

    out_msg = BgrToImageMsg(bgr, header, "bgr8");
    return true;
}

}  // namespace body_pose
