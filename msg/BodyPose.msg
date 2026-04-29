# 单人体姿态
sensor_msgs/RegionOfInterest bbox   # 人体边界框
KeyPoint[] keypoints                # 关键点（17 点 COCO 或 25 点）
float32 confidence                  # 整体置信度
int32 track_id -1                   # 上游追踪 ID（-1 表示无追踪）
