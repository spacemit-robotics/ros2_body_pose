/**
 * @file detection_utils.cpp
 * @brief Implementation of detection box encode/decode helpers.
 *
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "body_pose/detection_utils.h"

#include <string>
#include <vector>

namespace body_pose {

std_msgs::msg::Float32MultiArray EncodeBoxes(
    const std::vector<DetectionBox>& boxes,
    const std::string& dim0_label) {
    constexpr int kFieldsPerBox = 7;

    std_msgs::msg::Float32MultiArray out;
    out.layout.dim.resize(2);
    out.layout.dim[0].label = dim0_label;
    out.layout.dim[1].label = "fields(x1,y1,x2,y2,score,label,track_id)";
    out.layout.dim[1].size = kFieldsPerBox;
    out.layout.dim[1].stride = kFieldsPerBox;
    out.layout.data_offset = 0;

    out.data.reserve(boxes.size() * kFieldsPerBox);
    for (const auto& b : boxes) {
        out.data.push_back(b.x1);
        out.data.push_back(b.y1);
        out.data.push_back(b.x2);
        out.data.push_back(b.y2);
        out.data.push_back(b.score);
        out.data.push_back(static_cast<float>(b.label));
        out.data.push_back(static_cast<float>(b.track_id));
    }

    auto n = static_cast<uint32_t>(boxes.size());
    out.layout.dim[0].size = n;
    out.layout.dim[0].stride = n ? n * kFieldsPerBox : 0;
    return out;
}

#ifdef HAVE_VISION_MSGS
vision_msgs::msg::Detection2DArray EncodeDetection2DArray(
    const std::vector<DetectionBox>& boxes,
    const std_msgs::msg::Header& header) {
    vision_msgs::msg::Detection2DArray msg;
    msg.header = header;

    for (const auto& b : boxes) {
        vision_msgs::msg::Detection2D det;
        det.header = header;

        det.bbox.center.position.x = static_cast<double>(b.x1 + b.x2) / 2.0;
        det.bbox.center.position.y = static_cast<double>(b.y1 + b.y2) / 2.0;
        det.bbox.size_x = static_cast<double>(b.x2 - b.x1);
        det.bbox.size_y = static_cast<double>(b.y2 - b.y1);

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = b.class_name.empty()
            ? ("class_" + std::to_string(b.label))
            : b.class_name;
        hyp.hypothesis.score = static_cast<double>(b.score);
        det.results.push_back(hyp);

        msg.detections.push_back(det);
    }
    return msg;
}
#endif

}  // namespace body_pose
