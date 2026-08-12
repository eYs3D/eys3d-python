# pyeys3d 示例程序

**Language:** [English](README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`pyeys3d` 驱动程序的可运行示例。每份文件独立、附完整注释,而且只讲一个
主题:挑你需要的那份来读、复制过去,再把自己的程序用不到的部分删掉。

有两份是起点。`quickstart.py` 是最小的完整程序,可复制到你自己的项目里;
`viewer.py` 靠界面上的菜单驱动全部功能,想知道一台相机能做什么,这是
最快的路。其余各份各自加一个主题:`hello_depth.py` 完全不画任何画面,
只证明相机能动;`00_enumerate.py` 列出接了什么;`01` 是彩色 + 深度的
基础;`02`–`06` 各自在这个基础上加一个功能。

## 安装

从本包所在的同一个 Release 页,下载对应你 Python 版本与平台
(Linux x86_64 / aarch64 或 Windows x64)的 `pyeys3d` wheel 安装:

```bash
pip install pyeys3d-<版本>-cp310-cp310-linux_x86_64.whl   # Linux
pip install pyeys3d-<版本>-cp310-cp310-win_amd64.whl      # Windows
```

`hello_depth.py` 与 `00_enumerate.py` 不需要其他包。会绘制画面的示例另需
这些:

```bash
pip install opencv-python           # 所有会开窗口的示例
pip install "pyglet>=2"             # 02_pointcloud.py、viewer.py 的 3D 窗口
pip install open3d                  # 03_pointcloud_open3d.py
```

## 示例一览

| 文件 | 说明 | 需要 |
|---|---|---|
| [`quickstart.py`](#quickstartpy) | 最小的完整程序 | opencv |
| [`viewer.py`](#viewerpy) | 全功能展示,由菜单驱动 | opencv, pyglet |
| [`hello_depth.py`](#hello_depthpy) | 相机能动 | — |
| [`00_enumerate.py`](#00_enumeratepy) | 接了什么,以及它有哪些模式 | — |
| [`01_basic_color_depth.py`](#01_basic_color_depthpy) | 彩色 + 深度,以及开流时应用的设置 | opencv |
| [`02_pointcloud.py`](#02_pointcloudpy--03_pointcloud_open3dpy) | 把深度当成 3D 点云 | opencv, pyglet |
| [`03_pointcloud_open3d.py`](#02_pointcloudpy--03_pointcloud_open3dpy) | 同一个点云的 Open3D 版 | opencv, open3d |
| [`04_capture.py`](#04_capturepy) | 存快照与片段,以及回放 | opencv |
| [`05_runtime_controls.py`](#05_runtime_controlspy) | 哪些设置能在串流中改 | opencv |
| [`06_multicam.py`](#06_multicampy) | 多台相机,一台一个 process | opencv |

## `01`–`06` 共用的参数

每个编号示例都接受同样的七个,一个参数对应它调用的一个 API。没列在这里的
都是示例用一行明白写死的默认值 —— 要改就改那一行,不是加参数。

| 参数 | 对应 API | 默认值 |
|---|---|---|
| `--model MODEL` | `Config.enable_device(model)` | 自动检测 |
| `--mode MODE_ID` | `Config.enable_device(mode_id=)` | 该机型的特色模式 |
| `--serial SERIAL` | `Config.enable_device(serial_number=)` | 任意(子串匹配) |
| `--usb-port PORT` | `Config.enable_device(usb_port=)` | 任意(精确匹配) |
| `--ir-value LEVEL` | `Config.set_ir_value()` | 机型默认;`0` 为关闭 |
| `--depth-range NEAR_MM FAR_MM` | `Config.set_depth_range()` | 机型默认 |
| `--filters` | `Config.with_filters(Spatial, Temporal, HoleFilling)` | 关闭 |

`hello_depth.py`、`quickstart.py` 与 `00_enumerate.py` 一个都不接受:前两份
预期只有一台相机,第三份则什么都不打开。`viewer.py` 只接受 `--out`,因为
其余一切都在界面上。每份示例会在这之上加自己主题的参数 —— 下面各节有列,
`--help` 则列得最全。

`--serial` 与 `--usb-port` 是你从多台里挑一台的方式。接了不止一台而两者
都没给时,`Config` 宁可报错也不会猜,以免打开错的那台。

## 从这里开始

### `quickstart.py`

```bash
python quickstart.py
```

项目 README 的快速开始,可直接运行:开场打印设备身份与内参,接着开
彩色 + 深度窗口,并在同一行状态栏显示中心距离、点云最近点,以及每帧的
编号与时间戳。把这份复制到自己的项目里当起点 —— 它不从其他示例 import
任何东西。

它预期只有一台相机,也不接受任何参数。

### `viewer.py`

```bash
python viewer.py
python viewer.py --out /tmp/captures
```

所有相机设置收在同一个界面,而且在相机运行中就能改 —— 想知道一台相机
能做什么、或要不改任何东西就端到端验证一台相机,这是最快的路。每台相机
会有自己的 Color / Depth 窗口,外加一个装着菜单的 Controls 窗口。

| 按键 | 作用 |
|---|---|
| <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> | 在菜单格之间移动 |
| <kbd>-</kbd> / <kbd>+</kbd> | 调整选中的值,或切换开关 |
| <kbd>Enter</kbd> | 应用暂存的视频模式 / 深度裁剪 / 滤波更改(会重开串流) |
| <kbd>p</kbd> | 打开 / 关闭 3D 点云窗口 |
| <kbd>s</kbd> | 存一组快照 |
| <kbd>r</kbd> | 开始 / 停止录制片段 |
| <kbd>d</kbd> | 把相机属性恢复为默认值 |
| <kbd>x</kbd> | 硬件复位相机 |
| <kbd>q</kbd> / <kbd>ESC</kbd> | 关闭当前聚焦的相机 |

`--out DIR` 是唯一的参数,而且只是因为界面上没地方放它;它会在启动时
解析并打印出来(`captures -> ...`),所以每一行 `saved` 指的都是找得到
的文件。

菜单里有视频模式、IR、电源频率、自动 / 手动曝光、自动 / 手动白平衡、
深度裁剪与三种深度滤波。视频模式、深度裁剪与滤波在选好但还没提交时会
标上 `*`,并由 <kbd>Enter</kbd> 一起应用,因为这三者都在 `start()` 时
固定,改动就得重开串流 —— 每改一次相机就会消失几秒。其余的则是按下按键
当下就生效。录制片段期间,<kbd>Enter</kbd> 与 <kbd>x</kbd> 会被挡下。

接了多台相机时,每台都先停在模式选择画面而不直接串流 —— Controls 窗口
已经开着,预览还没有。选一个模式后按 <kbd>Enter</kbd> 启动那一台,或按
<kbd>q</kbd> 把它关掉。挑模式时要把共用的 USB 总线算进去:多台相机分享
同一个主机控制器的带宽,而一个特色模式本身就可能占掉 USB 3 链路的大半。
要给哪台送按键,就点一下那台相机的窗口。

viewer 做不到、而编号示例做得到的:每帧 metadata(`01 --frame-meta`)、
开场打印的完整相机模型(`01`),以及片段回放(`04 --play`)。

## 其余各份,每份一个主题

### `hello_depth.py`

```bash
python hello_depth.py
```

没有窗口,除 `pyeys3d` 外不需要任何包。它会持续打印画面中心的距离,
所以在这里失败就是相机、驱动或线缆的问题,绝不会是显示代码。它预期只有
一台相机,也不接受任何参数;接了多台时,请在文件里的 `enable_device()`
调用中指名其中一台。

### `00_enumerate.py`

```bash
python 00_enumerate.py
```

先列出每个**已连接**机型的完整视频模式目录:id、该模式需要的 USB 链路、
分辨率与帧率,并在每条链路的特色模式上标 `*`(不指定 `mode_id` 时
`start()` 开的就是它)—— 接着每台相机一行摘要。用 `--mode` 之前先跑它,
看看有哪些 id。

### `01_basic_color_depth.py`

```bash
python 01_basic_color_depth.py
python 01_basic_color_depth.py --mode 3 --filters --frame-meta
```

其他示例的基础:Color (Left)、模式有分割 L|R 时的 Color (Right),以及
Depth 各开在自己的窗口,鼠标移到哪里就读出该点的 RGB 或距离,设备里存的
完整相机模型(K / D / R / P)在开场打印。

| 参数 | 作用 |
|---|---|
| `--frame-meta` | 每秒打印一次某一帧的编号、它的硬件与主机时间戳,以及它距采集所经过的时间 —— 这些字段用来和其他传感器对齐时间,以及测量延迟 |

### `02_pointcloud.py` / `03_pointcloud_open3d.py`

```bash
python 02_pointcloud.py
python 03_pointcloud_open3d.py
```

01 再加上实时 3D 点云,每帧都由深度图与设备存的内参重建。两份文件是
同一个程序、只换显示层 —— `02` 用 pyglet/OpenGL,`03` 用 Open3D。

点云窗口中:拖动旋转、中键拖动平移、滚轮缩放,<kbd>R</kbd> 复位视角、
<kbd>Q</kbd> / <kbd>ESC</kbd> 关闭。

### `04_capture.py`

```bash
python 04_capture.py                      # 开窗口;按 s 存一组
python 04_capture.py --snapshot           # 存一组就退出,不开窗口
python 04_capture.py --record 10          # 录十秒后退出
python 04_capture.py --play capture/clips/20260101-120000-000
```

存快照与片段,以及回放。一切都落在 `--out`(默认 `./capture/`)底下:

```
capture/snapshots/<stamp>_color.png            按 s,或 --snapshot
capture/snapshots/<stamp>_depth.png            原始 16-bit,1 单位 = 1 mm
capture/snapshots/<stamp>_depth_preview.png    可直接看的呈现
capture/snapshots/<stamp>_cloud.ply            MeshLab / Open3D
capture/clips/<stamp>/                         一段录下的片段
    color/000000.jpg  depth/000000.png         成对的帧组
    metadata.jsonl                             索引 + 校准数据
```

| 参数 | 作用 |
|---|---|
| `--out DIR` | 快照与片段的写入位置(默认 `capture`) |
| `--snapshot` | 存一组就退出,不开窗口也不用按键。它会先给自动曝光时间收敛,所以写下的是第一张曝光正确的帧,而不是第一张到达的 |
| `--record SECONDS` | 录这么多秒的片段,然后退出 |
| `--play DIR` | 以录制时的速度回放一段片段 |

`metadata.jsonl` 第一行带着设备、深度范围与内参;之后每一行索引一组帧。
深度 PNG 是 uint16、一单位一毫米,所以大多数看图软件会显示成几乎全黑;
旁边的 `_depth_preview.png` 才是可直接看的呈现。要读回数值请用
`cv2.imread(path, cv2.IMREAD_UNCHANGED)`。

### `05_runtime_controls.py`

```bash
python 05_runtime_controls.py
```

哪些设置能在相机串流中改变。通过 `Config` 设的东西只在 `start()` 时
应用一次;这份展示的是 `Pipeline` 上的对应版本 —— IR、自动 / 手动曝光、
自动 / 手动白平衡(仅彩色机型)与电源频率 —— 放在自己窗口里的方向键
菜单中。

| 按键 | 作用 |
|---|---|
| <kbd>↑</kbd> <kbd>↓</kbd> | 选择一个控制项 |
| <kbd>-</kbd> / <kbd>+</kbd>(或 <kbd>←</kbd> <kbd>→</kbd>) | 调整它;AE / AWB 则是开关切换 |
| <kbd>d</kbd> | 恢复默认 —— IR 回机型默认值,AE / AWB 回自动 |
| <kbd>x</kbd> | 硬件复位相机,让它在 USB 上重新枚举 |
| <kbd>q</kbd> / <kbd>ESC</kbd> | 退出 |

每次更改后数值都会从设备读回,而且扛得住 USB 断线:热插拔 watchdog
会在重连时重新应用最后的状态。按 <kbd>x</kbd> 就能看到 —— Link 那一行
会跟着相机掉下去再回来。

### `06_multicam.py`

```bash
python 06_multicam.py
python 06_multicam.py --model G100P --mode 3
```

对每一台连接的相机同时跑 01,一台相机一个 process。父进程先枚举,再为
每台设备 spawn 一个以它读到的序列号绑定的子进程,然后等待;每个子进程
开一个 `color | depth` 窗口,标题带着它的帧率。`--model` / `--serial` /
`--usb-port` 用来缩小要开哪几台,其余的共用参数则转发给每一个子进程。

## 多台相机:一台一个 process,还是全都在一个 process

`06_multicam.py` 与 `viewer.py` 演示这两种安排,选哪个是取舍,不是偏好:

- **一相机一 process**(`06`)让它们彼此独立 —— 某一台卡死不会把其他相机
  一起拖垮 —— 而且打开动作是并行的,因为 SDK 以 process 为单位的记账并不
  跨 process 共用。在 Windows 上,窗口不在前台的那几台相机会以偏低的帧率
  运行:系统会把后台 process 降速。
- **同一个 process**(`viewer.py`)没有这种落差 —— 不论点的是哪个窗口,
  所有相机都留在前台 —— 但这些相机必须一台一台设置,所以 N 台相机就是
  N 次打开接连跑完。

同一个 process 内由不同线程打开的相机,也出于同样的理由被驱动串行化,
调用方不需要做任何事。

## 这些文件怎么组织

`example_helpers.py` 收纳这些示例要成为一个程序所需的骨架 —— 共用参数、
控制台编码、启动时打印的设备摘要,以及 OpenCV 窗口 —— 且不含任何
`pyeys3d` 调用。读者是为了 API 调用而来的,那些就留在教它的那份示例里,
即使因此和另一份重复也一样。

两份示例做同一件事的地方,就用一模一样的字写,这样把它们 diff 起来只会
看到后面那份多做了什么。真正不同的地方 —— `02` 的点云窗口由主循环驱动、
`viewer.py` 的跑在自己的线程上 —— 就让它们分开,而不是合并成两边都
不好用的东西。

## 注意事项

- **Windows**:wheel 需要 Visual C++ 2015–2022 可再发行组件包(x64)与
  系统 OpenCL 运行时(任一 GPU 驱动都会安装)。
- 完整 API 参考文档:[`docs/api.md`](../docs/api.md) —— 源码与本示例包内都有。
