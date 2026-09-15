# VidarAR 库技术文档

## 1. 概述

VidarAR 是一个面向增强现实（AR）领域的地理坐标投影库。

当前提供地理坐标与视频像素坐标的实时互转能力，后续将持续扩展其他 AR 相关技术。

经典应用场景是无人机视频 AR：载机位姿随镜头实时变化，
地面标注（航线、边界、目标点）需要稳定地“钉”在视频画面中对应位置，
这正是地理坐标投影要解决的核心问题：
**把地图上的地理标注（点/线/面），实时投影到相机视频画面的对应像素位置上**，供上层绘制引擎渲染。

一句话描述输入输出：

```
载机位姿(经纬高 + 欧拉角) + 相机内参 + 地理标注  ──▶  每个标注点的像素坐标(u, v)
```

---

## 2. 使用场景

| 场景 | 说明 |
|---|---|
| **视频 AR 叠加显示** | 在无人机回传的视频画面上，实时叠加显示地面目标的标注线、区域边界、关键点位 |
| **态势标绘** | 将指挥系统标绘的地理图形（航线、管制区、威胁圈）随镜头视角动态呈现 |
| **目标指示** | 地面目标（建筑、车辆）在画面中的位置随无人机姿态变化实时跟踪 |
| **像素反解定位** | 操作员点击画面上任意像素点，反算出该点对应的地理经纬度，用于目标定位与测报 |

典型工作流：无人机回传视频 + 飞控姿态数据 → VidarAR 计算各标注的像素坐标 → 叠加绘制后编码输出/推流。

---

## 3. 解决的问题

不使用本库时，自行实现"地理坐标 → 视频像素"投影会遇到以下难题，VidarAR 全部内置解决：

| 难题 | VidarAR 的解决方案 |
|---|---|
| **坐标变换链路复杂** | 内置五级坐标变换，调用方只传经纬高和姿态角 |
| **镜头畸变** | 内置 5 参数畸变模型（k1/k2/k3 径向 + p1/p2 切向），使用标定结果直接参与投影 |
| **姿态数据抖动** | 内置多级姿态平滑滤波，低频姿态数据逐视频帧平滑输出，画面稳定不抖动 |
| **多分辨率适配** | 相机参数按分辨率档位注册，一套标定数据服务多个画面规格 |
| **画幅内外裁剪** | 目标飞出画面时的坐标规整处理，标注进出画面平滑自然 |

---

## 4. 核心概念

### 坐标系约定

| 坐标系 | 约定 |
|---|---|
| 地理坐标 | WGS84，纬度/经度单位为度，高度为海拔米 |
| 姿态角 | 欧拉角（俯仰/偏航/滚转），单位为度 |
| 像素坐标 | 原点位于画面**左上角**，u 向右，v 向下 |

### 投影结果回调

`update_pose()` 内部完成姿态平滑与坐标转换后，
投影结果以 `vidar_geometry_pixel_list` 结构体在**调用线程内同步**回调返回：

```cpp
struct vidar_geometry_pixel_list   // 投影结果集合
{
    std::vector<vidar_geometry_pixel> geometries;
};

struct vidar_geometry_pixel       // 单个图形的投影结果
{
    int                             id;      // 图形 ID, 与输入几何对应
    enum vidar_geometry_type        type;    // 几何类型
    std::vector<vidar_pixel_point>  points;  // 各点的像素坐标
};

struct vidar_pixel_point          // 单个点的投影结果
{
    int               id;      // 点 ID, 与输入坐标序号对应
    struct vidar_pixel pixel;  // 像素坐标 (u, v)
};
```

回调内直接遍历结构体读取各图形、各点的像素坐标即可。
`result` 指针仅在回调返回前有效，需保留请拷贝。

---

## 5. 使用流程

### 第一步：创建转换器

```cpp
#include "vidar_geopixel.h"

// 投影结果回调
void on_pixel_result(const vidar_geometry_pixel_list* result, uint32_t pts, void* user_data)
{
    // result: 投影结果集合, 仅在回调返回前有效, 需保留请拷贝
    for (const vidar_geometry_pixel& geo : result->geometries)
    {
        for (const vidar_pixel_point& pt : geo.points)
        {
            // pt.id / pt.pixel.u / pt.pixel.v
        }
    }
    // pts : 与视频帧对应的时间戳(毫秒)
}

vidar_geopixel_context* ctx = vidar_geopixel_create(on_pixel_result, nullptr);
```

### 第二步：注册相机参数

```cpp
vidar_camera_param cam = {};
cam.video_type   = 0;        // 1920x1080
cam.video_width  = 1920;
cam.video_height = 1080;
cam.fx = 1650.0;  cam.fy = 1650.0;   // 焦距 (标定结果)
cam.cx = 960.0;   cam.cy = 540.0;    // 光心
cam.k1 = -0.28;   cam.k2 = 0.12;     // 畸变系数
// ... 其余字段按标定报告填写

vidar_geopixel_set_camera_params(ctx, &cam);
// 有多路分辨率时, 换参数重复调用即可
```

### 第三步：设置地图标注

```cpp
// 一条边界线 (两个端点)
vidar_geometry line;
line.id   = 1;
line.type = VIDAR_GEOMETRY_LINE_STRING;
line.coordinates = {
    { 30.259500, 120.138800, 43.5 },   // 端点 A: 经纬高
    { 30.261200, 120.141500, 41.0 },   // 端点 B
};

vidar_geometry_list list;
list.geometries.push_back(line);

vidar_geopixel_set_geometries(ctx, &list);
```

标注变化时（增删图形）重新调用本接口整体替换。

### 第四步：（可选）配置显示参数

```cpp
// 显示半径 500 米, 只显示点和线, 虚线距离保持默认
vidar_geopixel_config(ctx,
                      500.0,                                        // radius
                      VIDAR_DISPLAY_POINT | VIDAR_DISPLAY_LINE,     // display_flags
                      -1.0);                                        // dashed_line_distance, -1 不修改
```

三个参数均支持"传 -1 不修改"，可渐进式调整。

### 第五步：逐帧更新位姿

随视频流逐帧调用，投影结果在回调中同步返回：

```cpp
vidar_lla position = { 30.265000, 120.135000, 120.0 };   // 载机经纬高
vidar_euler_angle attitude = { -35.0, 45.0, 2.0 };        // 俯仰/偏航/滚转(度)

vidar_geopixel_update_pose(ctx,
                           0,               // videoType: 当前视频档位
                           &position,
                           &attitude,
                           frame_pts_ms);   // 本帧时间戳
```

### 第六步：销毁

```cpp
vidar_geopixel_close(ctx);   // 传 NULL 安全
```

---

## 6. 同步转换接口

只需单点换算时，使用 `vidar_converter.h` 的无状态接口：

```cpp
#include "vidar_converter.h"

// 经纬高 → 像素
vidar_camera_param cam  = /* 标定结果 */;
vidar_lla position      = { 30.265000, 120.135000, 120.0 };
vidar_euler_angle attitude = { -35.0, 45.0, 2.0 };
vidar_lla target        = { 30.259500, 120.138800, 43.5 };
vidar_pixel pixel;

int ret = vidar_geopixel_lla_to_pixel(&cam, &position, &attitude, &target, &pixel);
if (0 == ret) {
    // pixel.u / pixel.v 可用
}

// 像素 → 经纬高 (点击定位)
vidar_pixel click = { 640, 360 };
vidar_lla out;
vidar_geopixel_pixel_to_lla(&cam, &position, &attitude, &click,
                            80.0,   // 相对目标点的高度(米)
                            &out);
```

无需 create/close，直接调用。

---

## 7. 使用约定

| 约定 | 说明 |
|---|---|
| **线程模型** | 所有接口非线程安全，须在同一线程串行调用；投影回调在 `update_pose()` 调用线程内同步触发 |
| **回调数据有效期** | 回调 `result` 指针仅在回调返回前有效，调用方如需保留须自行拷贝 |
| **数据所有权** | 传入本库的结构体指针在函数返回后即可释放，库内部自行拷贝 |
| **调用顺序** | `update_pose()` 前须完成相机参数注册与标注设置，否则返回 -1 |

---

