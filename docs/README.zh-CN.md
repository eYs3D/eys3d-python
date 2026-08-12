# eYs3D 立体深度相机 Python 驱动程序

[![Python](https://img.shields.io/badge/Python-3.8%20%E2%80%93%203.13-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)

**Language:** [English](../README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`pyeys3d` 是 eYs3D 立体深度相机的官方 Python 驱动程序,以 pipeline 为
核心的 API(`Pipeline` / `Config` / `FrameSet`)直接调用 eSPDI C API,
输出彩色图像、深度图像与点云。支持 CPython 3.8–3.13,Linux(x86_64、aarch64)与 Windows(x64)。

### 支持的相机

| 模组 | 产品型号 | USB | 状态 |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | 量产 |
| **R77** | YX8072 | USB 2.0 | 量产 |
| **G62** | YX8081 | USB 2.0 | 量产 |

---

## 功能

- YUYV 与 **MJPEG** 色彩,解码为 `rgb8`(单色模组 G62 / R77 输出灰度,R = G = B)
- **宽幅 L\|R 色彩分割** —— 并列立体模式下左右眼以独立帧输出(模式目录中
  `split_lr` 的模式)
- **深度后处理滤波** —— 空间 / 时序 / 补洞
- **点云反投影** —— XYZ 与 XYZRGB,optical 坐标惯例
- 来自相机 rectify log 的**设备 intrinsics**(K / D / R / P)
- **热插拔恢复** —— watchdog 在 USB 断线后重新打开设备
- **相机控制**可在启动时与运行中设置 —— IR 强度、曝光、白平衡、电源频率
- **每帧 metadata** —— 序号、硬件时戳、主机时钟采集时间与传输掉帧计数
- **设备绑定** —— 多相机环境下以序列号或 USB 拓扑指定相机
- **`examples/viewer.py`** —— 上述所有能力收在同一个界面上,而且在相机运行中就能改

## 安装

预编 wheel 挂在每个 [GitHub
Release](https://github.com/eYs3D/eys3d-python/releases)。下载对应你 Python
版本与平台的 wheel(Linux x86_64 / aarch64 或 Windows x64,CPython 3.8–3.13)后安装
即可 —— wheel 已内含 eSPDI 运行环境。Linux 上相机以 UVC 视频设备枚举:
一般桌面会自动授予登录用户访问权,但 headless 或精简环境可能需将用户
加入 `video` 组(`sudo usermod -aG video $USER` 后重新登录)才能读取
`/dev/video*`。
Windows 另需两个多数机器已具备的系统组件:

- [Visual C++ 2015–2022 可再发行组件包
  (x64)](https://aka.ms/vs/17/release/vc_redist.x64.exe) —— 缺了它
  `import pyeys3d` 会以「DLL load failed / 找不到指定的模块」失败
- 系统 OpenCL 运行时,任一 GPU 驱动都会安装

```bash
pip install pyeys3d-1.0.0-cp310-cp310-linux_x86_64.whl   # Linux
pip install pyeys3d-1.0.0-cp310-cp310-win_amd64.whl      # Windows
```

从源码安装 —— 需要 C++17 编译器;CMake 与 Ninja 未安装时构建过程会
自动获取:

- Linux:GCC(`apt install build-essential`)
- Windows:[Visual Studio 2022 Build
  Tools](https://visualstudio.microsoft.com/visual-studio-build-tools/),
  勾选 **使用 C++ 的桌面开发** workload


```bash
git clone https://github.com/eYs3D/eys3d-python.git
cd eys3d-python
pip install .
```

## 快速开始

不带参数时,会自动检测连接的相机并以其特色模式(机型在目录中定义的
默认模式)打开:

```python
import pyeys3d as ey
import cv2

with ey.Pipeline() as pipeline:
    pipeline.start(ey.Config())            # 自动检测 + 特色模式
    dev = pipeline.device_info
    intr = pipeline.intrinsics             # 未校准时为 None
    print(f"Opened {dev.model}  serial {dev.serial_number}"
          f"  firmware {dev.firmware_version}")
    if intr is not None:
        print(f"  fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.cx:.1f} "
              f"cy={intr.cy:.1f}  baseline {intr.baseline_mm:.2f} mm")
    colorizer = ey.Colorizer(pipeline)     # 深度 -> rgb8 色阶
    # 反投影需要校准数据,未校准的机器没有。
    pc = ey.PointCloud(pipeline) if intr is not None else None
    while True:
        frames = pipeline.wait_for_frames(timeout_ms=1000)
        if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
            break
        if frames is None:
            continue
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if color is None or depth is None:
            continue
        dmm = depth.get_data()             # (H, W) uint16,1 单位 = 1 mm
        verts = pc.calculate(depth)[0] if pc else ()   # (N, 3) float32,单位米

        center_mm = int(dmm[dmm.shape[0] // 2, dmm.shape[1] // 2])
        center = f"{center_mm:5d} mm" if center_mm else " no data"
        nearest = f"{'no points':>23}"
        if len(verts):
            x, y, z = (int(v * 1000) for v in verts[verts[:, 2].argmin()])
            nearest = f"X{x:+5d} Y{y:+5d} Z{z:5d} mm"
        print(f"\rcenter {center}   nearest {nearest}", end="", flush=True)

        cv2.imshow("color", color.get_data_bgr())
        cv2.imshow("depth", colorizer.colorize_bgr(dmm))
```

## 示例

`examples/` 按主题各放一份可运行的文件。`quickstart.py` 与 `viewer.py`
是起点;其余各份在 `01` 的彩色 + 深度基底上各加一个主题:

| 文件 | 说明 |
|---|---|
| `quickstart.py` | 上面那个程序,可以直接复制 |
| `viewer.py` | 全功能展示,由界面上的菜单驱动 |
| `hello_depth.py` | 相机能动 —— 没有窗口,也不需要额外的包 |
| `00_enumerate.py` | 接了什么,以及它有哪些视频模式 |
| `01_basic_color_depth.py` | 彩色 + 深度,以及 `start()` 时应用的设置 |
| `02_pointcloud.py` / `03_pointcloud_open3d.py` | 把深度当成 3D 点云,pyglet 与 Open3D 各一 |
| `04_capture.py` | 快照、片段录制与回放 |
| `05_runtime_controls.py` | 哪些设置能在串流中改 |
| `06_multicam.py` | 多台相机,一台一个 process |

**手上有相机的话,就从 `viewer.py` 开始。** 它把所有设置放在同一个界面,
而且在相机运行中就能改 —— 视频模式、深度裁剪、滤波、IR、曝光、白平衡 ——
还有快照、片段录制与 3D 点云,一次涵盖所有连接的相机。要用到这些,一个
参数都不用传:

```bash
pip install opencv-python "pyglet>=2"
python examples/viewer.py
```

各示例的参数、按键表与设置说明见
**[examples/README.md](../examples/README.md)**
([在线浏览](https://github.com/eYs3D/eys3d-python/tree/main/examples))。

wheel 只安装 library 本体 —— 示例请从同一个 Release 页下载
`pyeys3d-<版本>-examples.zip`,或使用本 repo。

## API 概览

### `Context` — 枚举设备

轻量对象 —— 需要重新扫描时随时建一个。

```python
ctx = ey.Context()
for dev in ctx.query_devices():
    print(dev)
# DeviceInfo(model='G100P', serial_number='8036259M200025', usb_port='2-2:1.0',
#            pid=385, usb_port_type=3, usb_speed='USB3.0',
#            firmware_version='YX80362-B01-...')
```

### `Config` — 声明式配置

从各机型在 `pyeys3d/modes/<MODEL>.yaml` 内的模式目录选择。`enable_device`
的所有参数皆可省略:省略 model 会自动检测连接的相机,省略 `mode_id` 则
使用该机型的**特色模式**(目录中定义的默认模式;各机型清单见下方
「支持的视频模式」):

```python
ey.Config()                                    # 自动检测 + 特色模式
ey.Config().enable_device("G100P")             # 指定机型,特色模式
config = (ey.Config()
          .enable_device("G100P", mode_id=1)
          .set_ir_value(3)                           # 机型相关范围;-1 = 默认
          .set_auto_exposure(True))
```

其他可选的相机控制 —— 每一项都只在有设置时才于 `start()` 应用,没设就
保持原样:

| 方法 | 参数 |
|---|---|
| `set_auto_exposure(enabled)` | `True` / `False`;关掉时用 `set_exposure` 填值 |
| `set_exposure(value)` | 手动曝光,register units(会把自动曝光关掉) |
| `set_auto_white_balance(enabled)` | `True` / `False`;关掉时用 `set_white_balance` 填值 |
| `set_white_balance(value)` | 手动白平衡,register units(会把自动白平衡关掉);**仅彩色机型** |
| `set_power_line_frequency(mode)` | 防闪烁(让曝光与市电同步,以免灯光闪烁在画面上造成条纹):`1` 50 Hz / `2` 60 Hz |
| `set_ir_value(level)` | IR 投射器强度(机型相关范围;0 = 关闭),或用 `-1` 取下述随模式而定的默认值 |
| `set_depth_range(near_mm, far_mm)` | 丢弃 `[near_mm, far_mm]` 之外的深度;未设置 = 机型默认,`far_mm` 最多 16383(14-bit 深度上限) |
| `set_depth_quality_registers(source)` | 固件深度调校配置:`True`(默认)= 内置的机型配置,`False` = 保留固件默认,或给一个自定义配置文件的路径 |

这些控制会被限制在相机实际支持的范围内,不支持的请求会在主机端抛出
`ValueError`、不会送到设备。范围固定的值由 setter 当场拦下;需要同时知道
相机与模式才验得了的 —— 单色机型(G62 / R77)没有白平衡,视频模式只能在
它所声明的 USB 链路上打开(见下方「支持的视频模式」)—— 则在 `start()`
解析配置时拦下。

深度串流开始并稳定后(约数秒),pipeline 会在后台把机型的深度质量寄存器配置写入固件,
USB 重连后也会重写(固件在重新枚举时会复位)。内置配置位于
`pyeys3d/quality/DM_Quality_Cfg/<PART>_DM_Quality_Register_Setting.cfg`(部号:G100+ `YX80362`、R77 `YX8072`、G62 `YX8081`,另有 `DEFAULT`)—— 每行一组 `address,mask,value` 十六进制;
`set_depth_quality_registers(source)` 可传自定义文件路径替换,或传 `False`
保留固件默认。

IR 投射器是唯一在 `start()` 时必定写入的控制 ——
显式调用 `set_ir_value` 时以其为准
(0 = 任何模式都关闭);未设置时,默认值跟着模式的需要走 —— 模式含深度
(立体匹配需要投射器)或模组为单色(G62 / R77 的感光元件看得见 IR,
它就是场景照明,少了它彩色模式只会一片黑)时采用机型目录的默认值;
彩色感光元件上的纯彩色模式则关闭,让点阵图案不进入彩色图像。

### `Pipeline` — 串流管线

持有一台已打开的设备。`start(config)` → `wait_for_frames(timeout_ms)` 返回
`FrameSet`(超时为 `None`)→ `stop()`;`poll_for_frames()` 是非阻塞版:有更新
的一组就返回,否则 `None`。

```python
pipeline = ey.Pipeline()
pipeline.start(config)
try:
    frames = pipeline.wait_for_frames(timeout_ms=1000)
    color = frames.get_color_frame()
    depth = frames.get_depth_frame()
finally:
    pipeline.stop()
```

或使用 context manager:

```python
with ey.Pipeline() as pipeline:
    pipeline.start(config)
    ...
```

非阻塞版本 `pipeline.poll_for_frames()`:有更新的一组就返回,否则返回
`None`(不阻塞),适合在事件循环中轮询。

`start()` 后即可查:`pipeline.device_info`(model / serial / usb_port)、
`pipeline.color_profile` / `depth_profile`(各流的 `StreamProfile(width,
height, fps)`,该模式没有的为 `None`),第一帧前就知道。

USB 断线时 watchdog 会自动重开设备;期间 `wait_for_frames` 返回 `None`、
`pipeline.is_connected` 为 `False`(`reconnect_count` 统计重开次数)。
`pipeline.frames_dropped` 逐流报告相机已产出但主机未收到的帧数 ——
持续上升说明 USB 带宽或调度有问题。

自动恢复只涵盖出图后的断线。若在 `start()` 进行中拔除,断线发生在 SDK
的打开调用内,驱动无法中断它,`start()` 可能一直卡到相机重新接上为止。
调用 `start()` 前请先确认相机已接妥。

### 运行中调整相机控制

以上控制,除了 depth range 与质量寄存器配置,在 `Pipeline` 上都有同名的
runtime 版本 ——
流式传输中直接调用,相机立即生效、无需重启:

```python
pipeline.set_ir_value(4)            # 按机型范围校验
pipeline.set_auto_exposure(False)
pipeline.set_exposure(-6)           # 会先切为手动曝光
```

`pipeline.get_*()`(如 `get_exposure()`、`get_ir_value()`)从设备读回
当前值,不支持的控制返回 `None`。运行中设置的值在 USB 断线后仍保留 ——
热插拔 watchdog 重连时会重新应用最后状态。

曝光与白平衡的单位是设备 register units —— 曝光可能为负值(模组使用
有符号 log2 刻度,如 `-13` ≈ 1/8192 秒)。每个查询都返回
`ControlRange(min, max, step, default)`,runtime setter 会据此校验:
`get_exposure_range()` 报告模组共用的固定寄存器范围,
`get_white_balance_range()` 报告设备所报告的范围,`get_ir_range()` 则取自
机型目录 —— IR 寄存器接受超出机型合格范围的值,因此以目录为准。

`set_temporal_filter` 要求 `start()` 时已通过 `Config.with_filters(...)`
启用时序滤波(视差流在打开时就固定了,只能调参、不能事后开启)。

### 多相机

一个 `Pipeline` 对应一台相机。多台相机可以跑在同一个 process 里,而
`06_multicam.py` 仍然让每台相机各占一个 process —— 这个模式值得照抄:
某一台卡死时不会把其他相机一起拖垮。不过在 Windows 上也有把它们合到一个
process 的理由:窗口不在前台的 process 会被系统降速,所以一台相机一个
viewer 时,除了当前聚焦的那台以外,其余相机都会以偏低的帧率运行。

在同一个 process 内,相机是一台一台设置的 —— `start()`、
`Context.query_devices()` 与驱动自己的重连会彼此轮流,因为 SDK 的设备
记账是以 process 为单位。从多条线程打开是安全的,但不会更快:打开动作
会排队。不同 process 不共用这份状态,可以并行打开。

相机本身则是一次只能被一个 process 打开。第二个 process 去打开已被打开的
相机,会在 `start()` 收到带 eSPDI 错误码的 `RuntimeError`,而持有它的
process 完全不受影响、继续正常出图 —— 所以 `viewer.py` 开着就足以让下一
个示例失败,关闭它即可释放该相机。

接了多台相机时,选择必须唯一:按序号(子串匹配)或 USB port(精确匹配)
绑定:

```python
config.enable_device("G100P", mode_id=1, usb_port="2-2:1.0")
# Windows 的 port 识别是设备路径的 instance 区段,对同一物理 port 固定
# 不变,例:usb_port="6&35c4e9&0&0000" —— 从 00_enumerate.py 的输出复制
config.enable_device("G100P", mode_id=1, serial_number="8036259M200025")
```

选择有歧义时会直接报错,并列出每台候选相机的 model / 序号 / USB port;
`examples/00_enumerate.py` 也打印相同的识别信息。序号与 USB port 两者都给
时,相机必须**同时**匹配(钉“这台序号在这个 port”);若没有相机两者都
匹配则报错列清单。要让相机跨 port 追踪,只钉序号即可。

### `Frame` 属性

| 属性 | 说明 |
|---|---|
| `domain` | `FrameDomain.COLOR_RGB8`(色彩)或 `FrameDomain.DEPTH_MM`(深度) |
| `width`, `height` | 影像尺寸(像素) |
| `frame_number` | 设备端的逐流计数;interleave 模式下一次跳 2,两个流共用一个序列 |
| `hw_timestamp_us` | 硬件时戳(USB DMA 完成瞬间,μs) |
| `timestamp` | 映射到主机时钟的采集时间(epoch 秒,可与 `time.time()` 直接比较) |
| `get_data()` | 色彩:`(H, W, 3)` uint8 rgb8;深度:`(H, W)` uint16 mm |

`get_data()` 返回的是零拷贝的**只读** view;要修改像素请先复制
(`img = frame.get_data().copy()`)。

宽幅 L\|R 分割模式下,右眼彩色帧从同一组 frame set 获取:
`frames.get_right_color_frame()`。

### 内参

`pipeline.intrinsics` 给出当前视频模式对应的相机模型,原样透传设备存的值:

```python
intr = pipeline.intrinsics                            # 未校正时为 None
print(intr.width, intr.height, intr.baseline_mm)      # 例:1280 720 59.93
print(intr.fx, intr.fy, intr.cx, intr.cy)             # rectified pinhole
print(intr.K, intr.D, intr.R, intr.P)                 # 完整模型
```

图像交付时**已经 rectify 过**,所以 `fx`/`fy`/`cx`/`cy`(与 `P` 同值)才是
你收到的图像所遵循的模型;`K` 与 `D` 描述的是 rectify 之前的原始感光元件,
不可再套用到已交付的图像上。完整字段表见 `docs/api.md`。

### 滤波与点云

用 `with_filters` 声明深度后处理链。pipeline 原生执行整条链,输出的深度
已完成滤波且单位为毫米;不挂滤波时走固件毫米快速路径。

```python
config = (ey.Config()
          .enable_device("G100P", mode_id=1)
          .with_filters(
              ey.SpatialFilter(alpha=0.5, delta=20, magnitude=2, holes_fill=0),
              ey.TemporalFilter(alpha=0.4, delta=20, persistence=3),
              ey.HoleFillingFilter(ey.HoleFill.FARTHEST_AROUND)))
pipeline.start(config)

frames = pipeline.wait_for_frames()
depth  = frames.get_depth_frame().get_data()   # (H, W) uint16 mm,已滤波

pc = ey.PointCloud(pipeline)
verts, colors = pc.calculate(frames.get_depth_frame(), frames.get_color_frame())
# verts:  (N, 3) float32 米,optical 坐标(X 右、Y 下、Z 前)
# colors: (N, 3) uint8;未传彩色帧时为 None
```

彩色帧是可选的 —— 传入得到 XYZRGB,省略(`pc.calculate(depth)`)则是
更轻量的纯 XYZ 点云。

深度由校正后的左眼计算,因此本来就与该眼同一个视点:没有第二颗传感器需要
重投影,贴图、测量、叠加前也不需要任何对位步骤。若模式输出的深度分辨率小于
彩色(scale-down 模式,包含 G100+ 与 R77 的 USB 2 特色模式),把像素索引
乘上高度比即可 —— 两张图是同一个视角的两种尺寸。

滤波链顺序固定(空间 → 时序 → 补洞),与参数传入顺序无关。括号内为默认值:

| 滤波 | 参数 |
|---|---|
| `SpatialFilter` | `alpha` 平滑 0–1,1 = 不平滑(0.5);`delta` 边缘阈值,视差单位(20);`magnitude` 执行遍数 1–5(2);`holes_fill` 可桥接的最大空洞宽,0 = 关(0) |
| `TemporalFilter` | `alpha` 当前帧混合权重 0–1(0.4);`delta` 跳变阈值,视差单位(20);`persistence` 断点保持帧数 0–8(3) |
| `HoleFillingFilter` | `mode` = `HoleFill.OFF` / `FROM_LEFT` / `FARTHEST_AROUND`(默认)/ `NEAREST_AROUND` |

### `Colorizer` — 深度上色

把深度帧转成 rgb8 图像(建一次、每帧 `colorize`):

```python
colorizer = ey.Colorizer(pipeline)      # 范围取自 depth clip
rgb = colorizer.colorize(frames.get_depth_frame())   # (H, W, 3) uint8 rgb8
```

`ey.Colorizer(pipeline)` 范围取自 depth clip(`min_mm` / `max_mm` 可覆写);
空洞(深度 0)显示为黑色。`mode='grayscale'` 改为灰度呈现(默认为 JET
color map)。

## 支持的视频模式

特色模式 —— 未指定 `mode_id` 时 `Config()` 打开的模式:

| 机型 | 链路 | 特色模式 |
|---|---|---|
| G100+ | USB 3 | `1` — L'+D 1280x720@60 interleave (SDK 30fps) |
| G100+ | USB 2 | `56` — L'+D 1280x720@24 + 640x360 depth interleave (USB 2.0, SDK 12fps) |
| R77 | USB 2 | `2` — L'+D 1280x920@30 + 640x460 depth |
| G62 | USB 2 | `1` — L'+D 640x480@25 |

这些名称就是目录自己的写法:`L` / `R` 是原始的左右眼,`L'` / `R'` 是
rectify 过的,`D` 是深度,并列的一对则写成 `<width>(x2)x<height>`。

每个模式都声明了自己需要的 USB 链路,也只在该链路上打开 —— 要求协商到的
链路承载不了的模式会抛出 `ValueError`。所以 G100+ 的特色模式跟着链路走:
USB 3 是模式 `1`,USB 2 是模式 `56` —— 拿 60 fps 的帧率与全尺寸深度,
换成较慢链路承载得了的规格(24 fps、640x360 深度)。

模式目录位于 `pyeys3d/modes/`;YAML 文件也随 wheel 一起安装
(`pyeys3d/modes/<MODEL>.yaml`),装好包就能直接查完整模式表:

- `pyeys3d/modes/G100P.yaml` — 80 个模式(55 个 USB 3、25 个 USB 2)
- `pyeys3d/modes/R77.yaml`  — 9 个模式(MJPEG + YUYV,含宽幅 L\|R)
- `pyeys3d/modes/G62.yaml`  — 15 个模式(MJPEG + YUYV,含宽幅 L\|R)

程序列出:

```python
from pyeys3d.modes import load_catalog
for mid, mode in sorted(load_catalog("G100P").items()):
    yuyv = "YUYV" if mode.color.fmt == 0 else "MJPEG"
    print(f"  {mid}: {mode.name}  color={yuyv}")
```

## 诊断

`PYEYS3D_LOG_LEVEL` 控制原生层的日志详细度:`none` / `error` / `warn`
(默认)/ `info` / `debug`。设 `PYEYS3D_TIMING=1` 可在 pipeline 停止时
输出各阶段耗时(色彩解码、深度转换+滤波)。`PYEYS3D_PC_THREADS` 可指定
`PointCloud` 重投影使用的 worker 数(默认 4,上限为核心数)—— 这两趟是
内存带宽受限,增加 worker 的收益低于核心数所暗示的。原生错误信息带有 eSPDI 错误码名称与提示,例如
`APC_OpenDevice2 failed: rc=-27 APC_NOT_SUPPORT_RES (the device rejected
this mode)`(设备拒绝了这个模式)。目录里的模式若指定在错误的 USB 链路上
打开,会更早被拦下 —— `start()` 直接抛出 `ValueError`。

## 兼容性

Python ≥ 3.8。Linux x86_64 和 aarch64(Jetson),及 Windows 10/11 x64。

## 支持

问题与错误反馈:<support@eys3d.com>。为了让问题一眼可诊断,请附上以
`PYEYS3D_LOG_LEVEL=info` 重跑的失败命令、启动时打印的 `device_info` 那一行
(型号 / 序列号 / 固件),以及你的操作系统与 Python 版本。开发环境设置与
测试 / lint 关卡见 [CONTRIBUTING.md](../CONTRIBUTING.md)。

## 许可

Apache-2.0。详见 `LICENSE`。
