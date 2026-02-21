# 地图加载模块 (Map Server)

## 1. 模块概述

地图服务器 (`navigo_map_server`) 负责加载预建的占用栅格地图并发布为 ROS2 消息供导航系统使用。

## 2. 源码位置

```
src/navigation/src/navigo_map_server/
├── include/navigo_map_server/
│   ├── map_server.hpp          # 地图服务器头文件
│   └── map_io.hpp              # 地图IO操作头文件
├── src/
│   ├── map_server.cpp          # 地图服务器实现
│   ├── map_io.cpp              # 地图读写实现
│   └── main.cpp                # 入口点
└── CMakeLists.txt
```

## 3. 核心实现

### 3.1 MapServer 类 (map_server.cpp)

地图服务器是一个 **Lifecycle Node**，实现了标准的生命周期管理：

```cpp
// src/navigo_map_server/src/map_server.cpp

namespace navigo_map_server
{

MapServer::MapServer(const rclcpp::NodeOptions & options)
: navigo_util::LifecycleNode("map_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");
  
  // 声明参数
  declare_parameter("yaml_filename", rclcpp::ParameterValue(std::string("")));
  declare_parameter("topic_name", rclcpp::ParameterValue(std::string("map")));
  declare_parameter("frame_id", rclcpp::ParameterValue(std::string("map")));
}
```

**关键功能实现:**

#### 配置阶段 (on_configure)
```cpp
navigo_util::CallbackReturn
MapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // 获取地图文件路径
  std::string yaml_filename = get_parameter("yaml_filename").as_string();
  std::string topic_name = get_parameter("topic_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();

  // 使用 map_io 加载地图
  NAVIGO_MAP_IO::loadMapFromYaml(yaml_filename, msg_);

  // 如果没有设置 frame_id，使用 YAML 文件中的
  if (!frame_id_.empty()) {
    msg_.header.frame_id = frame_id_;
  }

  // 创建地图发布器
  occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    topic_name, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // 创建服务
  occ_service_ = create_service<nav_msgs::srv::GetMap>(
    "map",
    std::bind(&MapServer::getMapCallback, this, _1, _2, _3));

  return navigo_util::CallbackReturn::SUCCESS;
}
```

#### 激活阶段 (on_activate)
```cpp
navigo_util::CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // 发布地图 (transient_local QoS 确保后订阅者也能收到)
  occ_pub_->publish(msg_);

  // 创建 bond 连接用于生命周期监控
  createBond();

  return navigo_util::CallbackReturn::SUCCESS;
}
```

### 3.2 地图IO模块 (map_io.cpp)

地图IO模块负责解析YAML配置文件和加载图像数据：

#### YAML文件格式
```yaml
# map.yaml 示例
image: map.pgm           # 地图图像文件
resolution: 0.05         # 分辨率 (米/像素)
origin: [-10.0, -10.0, 0.0]  # 地图原点 [x, y, yaw]
negate: 0                # 是否取反
occupied_thresh: 0.65    # 占用阈值
free_thresh: 0.196       # 空闲阈值
mode: trinary            # 模式: trinary/scale/raw
```

#### 图像加载实现
```cpp
// src/navigo_map_server/src/map_io.cpp

void loadMapFromYaml(
  const std::string & yaml_filename,
  nav_msgs::msg::OccupancyGrid & msg)
{
  // 解析 YAML 文件
  YAML::Node doc = YAML::LoadFile(yaml_filename);
  
  auto image_file_name = yaml_filename_to_image_path(yaml_filename, doc);
  
  LoadParameters loadParameters;
  loadParameters.resolution = doc["resolution"].as<double>();
  // ... 解析其他参数

  // 使用 ImageMagick 加载图像
  Magick::Image img(image_file_name);
  
  // 设置地图元数据
  msg.info.width = img.columns();
  msg.info.height = img.rows();
  msg.info.resolution = loadParameters.resolution;
  msg.info.origin.position.x = loadParameters.origin[0];
  msg.info.origin.position.y = loadParameters.origin[1];
  
  // 转换像素值到占用概率
  msg.data.resize(msg.info.width * msg.info.height);
  
  #pragma omp parallel for schedule(static)
  for (size_t j = 0; j < msg.info.height; j++) {
    for (size_t i = 0; i < msg.info.width; i++) {
      auto pixel = img.pixelColor(i, msg.info.height - j - 1);
      // Trinary 模式: FREE_SPACE=0, LETHAL_OBSTACLE=100, NO_INFORMATION=-1
      int8_t map_cell = interpretValue(pixel, loadParameters);
      msg.data[j * msg.info.width + i] = map_cell;
    }
  }
}
```

#### 地图模式说明

| 模式 | 说明 | 值映射 |
|-----|------|-------|
| **Trinary** | 三态模式 | 0=空闲, 100=占用, -1=未知 |
| **Scale** | 缩放模式 | 连续值 0-100 |
| **Raw** | 原始模式 | 直接使用像素值 |

## 4. 服务接口

### 4.1 GetMap 服务
- **服务名**: `/map_server/map`
- **类型**: `nav_msgs/srv/GetMap`
- **功能**: 获取当前加载的地图

### 4.2 LoadMap 服务
- **服务名**: `/map_server/load_map`
- **类型**: `nav2_msgs/srv/LoadMap`
- **功能**: 动态加载新地图

## 5. 话题接口

### 发布话题
| 话题名 | 消息类型 | QoS | 说明 |
|-------|---------|-----|------|
| `/map` | nav_msgs/OccupancyGrid | transient_local | 占用栅格地图 |

## 6. 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|-------|------|-------|------|
| yaml_filename | string | "" | 地图YAML配置文件路径 |
| topic_name | string | "map" | 地图发布话题名 |
| frame_id | string | "map" | 地图坐标系ID |

## 7. 使用示例

### 启动地图服务器
```bash
ros2 run navigo_map_server map_server --ros-args \
    -p yaml_filename:=/path/to/map.yaml \
    -p topic_name:=map \
    -p frame_id:=map
```

### 在 launch 文件中配置
```python
# navigation_launch.py 中的配置
Node(
    package='navigo_map_server',
    executable='map_server',
    name='map_server',
    parameters=[{
        'yaml_filename': map_yaml_file,
        'topic_name': 'map',
        'frame_id': 'map'
    }],
)
```

## 8. 与其他模块的关系

```
                    ┌─────────────────┐
                    │   map.yaml      │
                    │   + map.pgm     │
                    └────────┬────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │   Map Server    │
                    │  (Lifecycle)    │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
              ▼              ▼              ▼
     ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
     │   Global    │ │   Local     │ │   RViz      │
     │  Costmap    │ │  Costmap    │ │ 可视化      │
     │(static_layer)│ │             │ │             │
     └─────────────┘ └─────────────┘ └─────────────┘
```

## 9. 注意事项

1. **地图文件路径**: YAML文件中的 `image` 路径可以是相对路径（相对于YAML文件）或绝对路径
2. **坐标系**: 地图的 `frame_id` 必须与定位系统发布的 TF 变换一致
3. **分辨率**: 分辨率影响导航精度和计算量，典型值为 0.05m/pixel
4. **QoS设置**: 使用 `transient_local` 确保后加入的订阅者能收到地图
