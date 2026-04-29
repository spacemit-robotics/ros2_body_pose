# body_pose

## 项目简介

ROS2 人体姿态估计节点，使用 YOLOv8-Pose 模型进行实时人体关键点检测。基于 `vision_service.h`（`model_zoo/vision`）实现高效的姿态估计功能。

## 功能特性

- 支持 COCO 17 关键点检测
- 实时多人姿态估计
- 可视化调试图像输出
- 支持直连摄像头或订阅图像话题
- 不支持：3D 姿态估计

## 快速开始

### 环境准备

- ROS2 Humble 或更高版本
- 已编译的 `components/model_zoo/vision` 组件
- YOLOv8-Pose 模型文件（如 yolov8n-pose.q.onnx）

### 构建编译

```bash
colcon build --packages-select body_pose
source install/setup.bash
```

### 运行示例

```bash
ros2 launch body_pose body_pose.launch.py
```

默认配置为 `config/yolov8_pose.yaml`，需先准备 yolov8n-pose 模型（如 `~/.cache/models/vision/yolov8_pose/yolov8n-pose.q.onnx`）。

## 详细使用

### 依赖

- `components/model_zoo/vision`：提供 `libvision.so` 与 `vision_service.h`
- YOLOv8-Pose 模型文件：默认由 `config/yolov8_pose.yaml` 指定

### 话题

| 类型 | 话题（默认） | 说明 |
|------|--------------|------|
| 订阅 | `/camera/image_raw` | 输入图像 |
| 发布 | `/perception/body_poses` | Float32MultiArray，每人 56 个数：x1,y1,x2,y2, score, kp0_x,kp0_y,kp0_vis, …, kp16_x,kp16_y,kp16_vis（COCO 17 关键点） |
| 发布 | `/body_pose/debug_image` | 带骨架的可视化图 |

### 配置

主要配置文件：`config/body_pose.yaml`

- `config_path`：留空则使用包内 `config/yolov8_pose.yaml`
- `score_threshold`、`image_topic`、`body_poses_topic`、`debug_image_topic`
- `use_camera`：`true` 时直连摄像头并发布到 `image_topic`，`false` 时仅订阅 `image_topic`
- `camera_id`、`camera_fps`：摄像头设备号与帧率

## 常见问题

### 崩溃排查（exit -11 / SIGSEGV）

若节点报 `process has died [exit code -11]`，多为推理库（如 SpaceMIT EP/ONNX）在首帧推理或 Draw 时崩溃。可先：

1. **关闭直连摄像头**：在 `config/body_pose.yaml` 中设 `use_camera: false`，用其他节点或脚本向 `/camera/image_raw` 发图，确认是否仅直连摄像头时崩溃。
2. **用 gdb 抓 backtrace**：
   ```bash
   gdb --args ros2 run body_pose body_pose_node --ros-args -p use_camera:=true --params-file install/body_pose/share/body_pose/config/body_pose.yaml
   (gdb) run
   # 崩溃后
   (gdb) bt
   ```
   根据 backtrace 判断是 vision_service/ONNX 内部问题还是摄像头驱动问题。

## 版本与发布

当前版本：1.0.0

变更记录：
- 初始版本发布

## 贡献方式

欢迎提交 Issue 和 Pull Request。

贡献者与维护者名单见：`CONTRIBUTORS.md`（如有）

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
