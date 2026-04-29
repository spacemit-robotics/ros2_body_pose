/**
 * @file detection_utils.h
 * @brief Detection box encoding/decoding helpers.
 *
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DETECTION_UTILS_H
#define DETECTION_UTILS_H

#include <string>
#include <vector>

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/header.hpp"

#ifdef HAVE_VISION_MSGS
#include "vision_msgs/msg/detection2_d_array.hpp"
#endif

namespace body_pose {

struct DetectionBox {
    float x1 = 0;
    float y1 = 0;
    float x2 = 0;
    float y2 = 0;
    float score = 0;
    int label = 0;
    int track_id = 0;
    std::string class_name;
};

std_msgs::msg::Float32MultiArray EncodeBoxes(
    const std::vector<DetectionBox>& boxes,
    const std::string& dim0_label = "num_detections");

#ifdef HAVE_VISION_MSGS
vision_msgs::msg::Detection2DArray EncodeDetection2DArray(
    const std::vector<DetectionBox>& boxes,
    const std_msgs::msg::Header& header);
#endif

}  // namespace body_pose

#endif  // YOLO_GENERAL_DETECTION_UTILS_H
