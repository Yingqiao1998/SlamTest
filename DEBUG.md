# ROS2 SLAM 节点调试指南

## 概述
本项目是一个ROS2组合节点(component)，编译为动态库(.so)形式。已配置好VS Code调试环境。

## 调试方法

### 方法1: 远程调试 (推荐)
这是最简单的调试方法：

1. **启动带gdbserver的节点**
   ```bash
   ros2 launch slam slam.launch.py debug:=true
   ```
   
2. **在VS Code中启动调试**
   - 按 `F5` 或者在调试面板选择 `(gdb) Remote Debug SLAM` 配置
   - VS Code会自动连接到gdbserver

### 方法2: 直接调试
1. **构建项目 (Debug模式)**
   - 在VS Code中按 `Ctrl+Shift+P`
   - 选择 `Tasks: Run Task` -> `build-debug`
   
2. **启动调试**
   - 选择 `(gdb) ROS2 SLAM Debug` 配置
   - 按 `F5` 启动

### 方法3: 附加到进程
1. **正常启动节点**
   ```bash
   ros2 launch slam slam.launch.py
   ```
   
2. **附加调试器**
   - 选择 `(gdb) Attach to ROS2 SLAM` 配置
   - 从进程列表中选择 `component_container_mt`

## 调试技巧

### 设置断点
- 直接在源码行号左侧点击设置断点
- 主要调试入口在 `src/slamNode.cc` 的构造函数和回调函数

### 常用断点位置
- `SlamComponent::SlamComponent()` - 节点初始化
- `SlamComponent::droneAttitudeWorldCallback()` - 无人机姿态回调
- `SlamComponent::gimbalAttitudeCallback()` - 云台姿态回调
- `SlamComponent::targetRealGeoCallback()` - 目标位置回调

### 查看变量
- 鼠标悬停查看变量值
- 在调试控制台中使用gdb命令
- 监视窗口添加需要跟踪的变量

## 文件说明

### VS Code配置文件
- `.vscode/launch.json` - 调试配置
- `.vscode/tasks.json` - 构建任务配置
- `.vscode/c_cpp_properties.json` - C++ IntelliSense配置

### Launch文件
- `launch/slam.launch.py` - 支持调试模式的启动文件
  - `debug:=true` 启用gdbserver模式
  - `debug:=false` 正常运行模式(默认)

## 故障排除

### 常见问题
1. **找不到符号**
   - 确保项目已用Debug模式构建
   - 检查 `CMAKE_BUILD_TYPE=Debug` 是否生效

2. **无法连接gdbserver**
   - 检查端口3000是否被占用
   - 确保防火墙允许本地连接

3. **断点不生效**
   - 确保源码路径正确
   - 检查编译命令是否包含 `-g` 选项

### 构建命令
```bash
# 清理构建
rm -rf build install log

# Debug模式构建
colcon build --packages-select slam --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 检查动态库是否包含调试符号
file build/slam/libslam_component.so
# 应该显示 "not stripped"
```

## 注意事项
- 确保ROS2环境已正确设置
- 调试模式下性能会有所下降
- 使用gdbserver时，节点会等待调试器连接后才开始运行
