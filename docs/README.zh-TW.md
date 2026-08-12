# eYs3D 立體深度相機 Python 驅動程式

[![Python](https://img.shields.io/badge/Python-3.8%20%E2%80%93%203.13-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)

**Language:** [English](../README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`pyeys3d` 是 eYs3D 立體深度相機的官方 Python 驅動程式,以 pipeline 為
核心的 API(`Pipeline` / `Config` / `FrameSet`)直接呼叫 eSPDI C API,
輸出彩色影像、深度影像與點雲。支援 CPython 3.8–3.13,Linux(x86_64、aarch64)與 Windows(x64)。

### 支援的相機

| 模組 | 產品型號 | USB | 狀態 |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | 量產 |
| **R77** | YX8072 | USB 2.0 | 量產 |
| **G62** | YX8081 | USB 2.0 | 量產 |

---

## 功能

- YUYV 與 **MJPEG** 色彩,解碼為 `rgb8`(單色模組 G62 / R77 輸出灰階,R = G = B)
- **寬幅 L\|R 色彩分割** —— 並列立體模式下左右眼以獨立影格輸出(模式目錄中
  `split_lr` 的模式)
- **深度後處理濾波** —— 空間 / 時序 / 補洞
- **點雲反投影** —— XYZ 與 XYZRGB,optical 座標慣例
- 來自相機 rectify log 的**裝置 intrinsics**(K / D / R / P)
- **熱插拔復原** —— watchdog 在 USB 斷線後重新開啟裝置
- **相機控制**可於啟動時與執行中設定 —— IR 強度、曝光、白平衡、電源頻率
- **每幀 metadata** —— 序號、硬體時戳、主機時鐘擷取時間與傳輸掉幀計數
- **裝置綁定** —— 多相機環境下以序號或 USB 拓樸指定相機
- **`examples/viewer.py`** —— 上述所有功能收在同一個畫面上,而且在相機執行中就能改

## 安裝

預編 wheel 掛在每個 [GitHub
Release](https://github.com/eYs3D/eys3d-python/releases)。下載對應你 Python
版本與平台的 wheel(Linux x86_64 / aarch64 或 Windows x64,CPython 3.8–3.13)後安裝
即可 —— wheel 已內含 eSPDI 執行環境。Linux 上相機以 UVC 視訊裝置列舉:
一般桌面會自動授予登入使用者存取權,但 headless 或精簡環境可能需將使用者
加入 `video` 群組(`sudo usermod -aG video $USER` 後重新登入)才能讀取
`/dev/video*`。
Windows 另需兩個多數機器已具備的系統元件:

- [Visual C++ 2015–2022 可轉散發套件
  (x64)](https://aka.ms/vs/17/release/vc_redist.x64.exe) —— 缺了它
  `import pyeys3d` 會以「DLL load failed / 找不到指定的模組」失敗
- 系統 OpenCL 執行環境,任一 GPU 驅動都會安裝

```bash
pip install pyeys3d-1.0.0-cp310-cp310-linux_x86_64.whl   # Linux
pip install pyeys3d-1.0.0-cp310-cp310-win_amd64.whl      # Windows
```

從原始碼安裝 —— 需要 C++17 編譯器;CMake 與 Ninja 未安裝時建置過程會
自動抓取:

- Linux:GCC(`apt install build-essential`)
- Windows:[Visual Studio 2022 Build
  Tools](https://visualstudio.microsoft.com/visual-studio-build-tools/),
  勾選 **使用 C++ 的桌面開發** workload


```bash
git clone https://github.com/eYs3D/eys3d-python.git
cd eys3d-python
pip install .
```

## 快速開始

不帶參數時,會自動偵測連接的相機並以其特色模式(機型在目錄中定義的
預設模式)開啟:

```python
import pyeys3d as ey
import cv2

with ey.Pipeline() as pipeline:
    pipeline.start(ey.Config())            # 自動偵測 + 特色模式
    dev = pipeline.device_info
    intr = pipeline.intrinsics             # 未校正時為 None
    print(f"Opened {dev.model}  serial {dev.serial_number}"
          f"  firmware {dev.firmware_version}")
    if intr is not None:
        print(f"  fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.cx:.1f} "
              f"cy={intr.cy:.1f}  baseline {intr.baseline_mm:.2f} mm")
    colorizer = ey.Colorizer(pipeline)     # 深度 -> rgb8 色階
    # 反投影需要校正資料,未校正的裝置沒有這份資料。
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
        dmm = depth.get_data()             # (H, W) uint16,1 單位 = 1 mm
        verts = pc.calculate(depth)[0] if pc else ()   # (N, 3) float32,單位公尺

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

## 範例

`examples/` 依主題各放一支可執行的檔案。`quickstart.py` 與 `viewer.py`
是起點;其餘各支在 `01` 的彩色 + 深度基底上各加一個主題:

| 檔案 | 說明 |
|---|---|
| `quickstart.py` | 上面那支程式,可以直接複製 |
| `viewer.py` | 全功能展示,由畫面上的選單驅動 |
| `hello_depth.py` | 相機能動 —— 沒有視窗,也不需要額外套件 |
| `00_enumerate.py` | 接了什麼,以及它有哪些影像模式 |
| `01_basic_color_depth.py` | 彩色 + 深度,以及 `start()` 時套用的設定 |
| `02_pointcloud.py` / `03_pointcloud_open3d.py` | 把深度當成 3D 點雲,pyglet 與 Open3D 各一 |
| `04_capture.py` | 快照、錄影與回放 |
| `05_runtime_controls.py` | 哪些設定能在串流中改 |
| `06_multicam.py` | 多台相機,一台一個 process |

**手上有相機的話,就從 `viewer.py` 開始。** 它把所有設定放在同一個畫面,
而且在相機執行中就能改 —— 影像模式、深度裁切、濾波、IR、曝光、白平衡 ——
還有快照、錄影與 3D 點雲,一次涵蓋所有連接的相機。要用到這些,一個參數
都不必傳:

```bash
pip install opencv-python "pyglet>=2"
python examples/viewer.py
```

各範例的參數、按鍵表與設定說明見
**[examples/README.md](../examples/README.md)**
([線上瀏覽](https://github.com/eYs3D/eys3d-python/tree/main/examples))。

wheel 只安裝 library 本體 —— 範例請從同一個 Release 頁下載
`pyeys3d-<版本>-examples.zip`,或使用本 repo。

## API 概覽

### `Context` — 列舉裝置

輕量物件 —— 需要重新掃描時隨時建立一個。

```python
ctx = ey.Context()
for dev in ctx.query_devices():
    print(dev)
# DeviceInfo(model='G100P', serial_number='8036259M200025', usb_port='2-2:1.0',
#            pid=385, usb_port_type=3, usb_speed='USB3.0',
#            firmware_version='YX80362-B01-...')
```

### `Config` — 宣告式設定

從各機型在 `pyeys3d/modes/<MODEL>.yaml` 內的模式目錄選擇。`enable_device`
的所有參數皆可省略:省略 model 會自動偵測連接的相機,省略 `mode_id` 則
使用該機型的**特色模式**(目錄中定義的預設模式;各機型清單見下方
「支援的影像模式」):

```python
ey.Config()                                    # 自動偵測 + 特色模式
ey.Config().enable_device("G100P")             # 指定機型,特色模式
config = (ey.Config()
          .enable_device("G100P", mode_id=1)
          .set_ir_value(3)                           # 機型相關範圍;-1 = 預設
          .set_auto_exposure(True))
```

其他可選的相機控制 —— 每一項都只在有設定時才於 `start()` 套用,沒設就
維持原樣:

| 方法 | 參數 |
|---|---|
| `set_auto_exposure(enabled)` | `True` / `False`;關掉時用 `set_exposure` 填值 |
| `set_exposure(value)` | 手動曝光,register units(會把自動曝光關掉) |
| `set_auto_white_balance(enabled)` | `True` / `False`;關掉時用 `set_white_balance` 填值 |
| `set_white_balance(value)` | 手動白平衡,register units(會把自動白平衡關掉);**僅彩色機型** |
| `set_power_line_frequency(mode)` | 防閃爍(讓曝光與市電同步,以免燈光閃爍在畫面上造成條紋):`1` 50 Hz / `2` 60 Hz |
| `set_ir_value(level)` | IR 投射器強度(機型相關範圍;0 = 關閉),或用 `-1` 取下述隨模式而定的預設值 |
| `set_depth_range(near_mm, far_mm)` | 丟棄 `[near_mm, far_mm]` 之外的深度;未設定 = 機型預設,`far_mm` 最多 16383(14-bit 深度上限) |
| `set_depth_quality_registers(source)` | 韌體深度調校設定檔:`True`(預設)= 內建的機型設定檔,`False` = 保留韌體預設,或給一個自訂設定檔的路徑 |

控制項會被限制在相機支援的範圍內,不支援的請求會在主機端丟出
`ValueError`、不會送到裝置。範圍固定的值由 setter 當場擋下;需要同時知道
相機與模式才驗得了的 —— 單色機型(G62 / R77)沒有白平衡,影像模式只能在
它所宣告的 USB 連結上開啟(見「支援的影像模式」)—— 則在 `start()` 解析
設定時擋下。

深度串流開始並穩定後(約數秒),pipeline 會在背景把機型的深度品質暫存器設定檔寫入韌體,
USB 重連後亦會重寫(韌體在重新列舉時會重置)。內建設定檔位於
`pyeys3d/quality/DM_Quality_Cfg/<PART>_DM_Quality_Register_Setting.cfg`(部號:G100+ `YX80362`、R77 `YX8072`、G62 `YX8081`,另有 `DEFAULT`)—— 每行一組 `address,mask,value` 十六進位;
`set_depth_quality_registers(source)` 可傳自訂檔案路徑替換,或傳 `False`
保留韌體預設。

IR 投射器是唯一在 `start()` 時必定寫入的控制 ——
明確呼叫 `set_ir_value` 時以其為準
(0 = 任何模式都關閉);未設定時,預設值跟著模式的需要走 —— 模式含深度
(立體匹配需要投射器)或模組為單色(G62 / R77 的感光元件看得見 IR,
它就是場景照明,少了它彩色模式只會一片黑)時採用機型目錄的預設值;
彩色感光元件上的純彩色模式則關閉,讓點陣圖案不進入彩色影像。

### `Pipeline` — 串流管線

持有一台已開啟的裝置。`start(config)` → `wait_for_frames(timeout_ms)` 回傳
`FrameSet`(逾時為 `None`)→ `stop()`;`poll_for_frames()` 是非阻塞版:有更新
的一組就回傳,否則 `None`。

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

非阻塞版本 `pipeline.poll_for_frames()`:有更新的一組就回傳,否則回傳
`None`(不阻塞),適合在事件迴圈中輪詢。

`start()` 後即可查:`pipeline.device_info`(model / serial / usb_port)、
`pipeline.color_profile` / `depth_profile`(各串流的 `StreamProfile(width,
height, fps)`,該模式沒有的為 `None`),第一幀前就知道。

USB 斷線時 watchdog 會自動重開裝置;期間 `wait_for_frames` 回傳 `None`、
`pipeline.is_connected` 為 `False`(`reconnect_count` 統計重開次數)。
`pipeline.frames_dropped` 逐串流回報相機已產出但主機未收到的幀數 ——
持續上升代表 USB 頻寬或排程有問題。

自動復原只涵蓋出圖後的斷線。若在 `start()` 進行中拔除,斷線會發生在 SDK
的開啟呼叫內,驅動無法中斷它,`start()` 可能一直卡到相機重新接上為止。
呼叫 `start()` 前請先確認相機已接妥。

### 執行中調整相機控制

以上控制,除了 depth range 與品質暫存器設定檔,在 `Pipeline` 上都有同名的
runtime 版本 ——
串流中直接呼叫,相機立即生效、不需重啟:

```python
pipeline.set_ir_value(4)            # 依機型範圍驗證
pipeline.set_auto_exposure(False)
pipeline.set_exposure(-6)           # 會先切為手動曝光
```

`pipeline.get_*()`(如 `get_exposure()`、`get_ir_value()`)從裝置讀回
目前值,不支援的控制回 `None`。執行中設定的值在 USB 斷線後仍保留 ——
熱插拔 watchdog 重連時會重新套用最後狀態。

曝光與白平衡的單位是裝置 register units —— 曝光可能是負值(模組使用
有號 log2 刻度,如 `-13` ≈ 1/8192 秒)。每個查詢都回傳
`ControlRange(min, max, step, default)`,runtime setter 會依此驗證:
`get_exposure_range()` 回報模組共用的固定暫存器範圍,
`get_white_balance_range()` 回報裝置回報的範圍,`get_ir_range()` 則取自
機型目錄 —— IR 暫存器接受超出機型合格範圍的值,因此以目錄為準。

`set_temporal_filter` 需要 `start()` 時已透過 `Config.with_filters(...)`
啟用時序濾波(視差串流在開啟時就固定了,只能調參、不能事後開啟)。

### 多相機

一個 `Pipeline` 對應一台相機。多台相機可以跑在同一個 process 裡,而
`06_multicam.py` 仍然讓每台相機各佔一個 process —— 這個模式值得照抄:
某一台卡死時不會把其他相機一起拖垮。不過在 Windows 上也有把它們合到同一
個 process 的理由:視窗不在前景的 process 會被系統降速,所以一台相機一個
viewer 時,除了目前聚焦的那台以外,其餘相機都會以偏低的幀率執行。

在同一個 process 內,相機是一台一台設定的 —— `start()`、
`Context.query_devices()` 與驅動程式自己的重連會彼此輪流,因為 SDK 的
裝置記帳是以 process 為單位。從多條執行緒開啟是安全的,但不會比較快:
開啟動作會排隊。不同 process 不共用這份狀態,可以並行開啟。

相機本身則是一次只能被一個 process 開啟。第二個 process 去開已被開啟的
相機,會在 `start()` 收到帶 eSPDI 錯誤碼的 `RuntimeError`,而持有它的
process 完全不受影響、繼續正常出圖 —— 所以 `viewer.py` 開著就足以讓下一
支範例失敗,關閉它即可釋放該相機。

接了多台相機時,選擇必須唯一:以序號(子字串比對)或 USB port(完全比對)
綁定:

```python
config.enable_device("G100P", mode_id=1, usb_port="2-2:1.0")
# Windows 的 port 識別是裝置路徑的 instance 區段,對同一實體 port 固定
# 不變,例:usb_port="6&35c4e9&0&0000" —— 從 00_enumerate.py 的輸出複製
config.enable_device("G100P", mode_id=1, serial_number="8036259M200025")
```

選擇有歧義時會直接報錯,並列出每台候選相機的 model / 序號 / USB port;
`examples/00_enumerate.py` 也印出相同的識別資訊。序號與 USB port 兩者都給
時,相機必須**同時**符合(釘「這台序號在這個 port」);若沒有相機兩者都
符合則報錯列清單。要讓相機跨 port 追蹤,只釘序號即可。

### `Frame` 屬性

| 屬性 | 說明 |
|---|---|
| `domain` | `FrameDomain.COLOR_RGB8`(色彩)或 `FrameDomain.DEPTH_MM`(深度) |
| `width`, `height` | 影像尺寸(像素) |
| `frame_number` | 裝置端的逐串流計數;interleave 模式下一次跳 2,兩串流共用一個序列 |
| `hw_timestamp_us` | 硬體時戳(USB DMA 完成瞬間,μs) |
| `timestamp` | 對映到主機時鐘的擷取時間(epoch 秒,可與 `time.time()` 直接比較) |
| `get_data()` | 色彩: `(H, W, 3)` uint8 rgb8;深度: `(H, W)` uint16 mm |

`get_data()` 回傳的是零拷貝的**唯讀** view;要修改像素請先複製
(`img = frame.get_data().copy()`)。

寬幅 L\|R 分割模式下,右眼彩色影格從同一組 frame set 取得:
`frames.get_right_color_frame()`。

### 內參

`pipeline.intrinsics` 給出目前影像模式對應的相機模型,原樣透傳裝置存的值:

```python
intr = pipeline.intrinsics                            # 未校正時為 None
print(intr.width, intr.height, intr.baseline_mm)      # 例:1280 720 59.93
print(intr.fx, intr.fy, intr.cx, intr.cy)             # rectified pinhole
print(intr.K, intr.D, intr.R, intr.P)                 # 完整模型
```

影像交付時**已經 rectify 過**,所以 `fx`/`fy`/`cx`/`cy`(與 `P` 同值)才是
你收到的影像所遵循的模型;`K` 與 `D` 描述的是 rectify 之前的原始感光元件,
不可再套用到已交付的影像上。完整欄位表見 `docs/api.md`。

### 濾波與點雲

以 `with_filters` 宣告深度後處理鏈。pipeline 原生執行整條鏈,輸出的深度
已完成濾波且單位為毫米;不掛濾波時走韌體毫米快速路徑。

```python
config = (ey.Config()
          .enable_device("G100P", mode_id=1)
          .with_filters(
              ey.SpatialFilter(alpha=0.5, delta=20, magnitude=2, holes_fill=0),
              ey.TemporalFilter(alpha=0.4, delta=20, persistence=3),
              ey.HoleFillingFilter(ey.HoleFill.FARTHEST_AROUND)))
pipeline.start(config)

frames = pipeline.wait_for_frames()
depth  = frames.get_depth_frame().get_data()   # (H, W) uint16 mm,已濾波

pc = ey.PointCloud(pipeline)
verts, colors = pc.calculate(frames.get_depth_frame(), frames.get_color_frame())
# verts:  (N, 3) float32 公尺,optical 座標(X 右、Y 下、Z 前)
# colors: (N, 3) uint8;未傳彩色影格時為 None
```

彩色影格是可選的 —— 傳入得到 XYZRGB,省略(`pc.calculate(depth)`)則是
較輕量的純 XYZ 點雲。

深度由校正後的左眼計算,因此本來就與該眼同一個視點:沒有第二顆感測器需要
重投影,貼圖、量測、疊圖前也不需要任何對位步驟。若模式輸出的深度解析度小於
彩色(scale-down 模式,包含 G100+ 與 R77 的 USB 2 特色模式),把像素索引
乘上高度比即可 —— 兩張圖是同一個視角的兩種尺寸。

濾波鏈順序固定(空間 → 時序 → 補洞),與參數傳入順序無關。括號內為預設值:

| 濾波 | 參數 |
|---|---|
| `SpatialFilter` | `alpha` 平滑 0–1,1 = 不平滑(0.5);`delta` 邊緣門檻,視差單位(20);`magnitude` 執行趟數 1–5(2);`holes_fill` 可橋接的最大空洞寬,0 = 關(0) |
| `TemporalFilter` | `alpha` 當前幀混合權重 0–1(0.4);`delta` 跳變門檻,視差單位(20);`persistence` 斷點保持幀數 0–8(3) |
| `HoleFillingFilter` | `mode` = `HoleFill.OFF` / `FROM_LEFT` / `FARTHEST_AROUND`(預設)/ `NEAREST_AROUND` |

### `Colorizer` — 深度上色

把深度影格轉成 rgb8 影像(建一次、每幀 `colorize`):

```python
colorizer = ey.Colorizer(pipeline)      # 範圍取自 depth clip
rgb = colorizer.colorize(frames.get_depth_frame())   # (H, W, 3) uint8 rgb8
```

`ey.Colorizer(pipeline)` 範圍取自 depth clip(`min_mm` / `max_mm` 可覆寫);
洞(深度 0)顯示為黑色。`mode='grayscale'` 改以灰階呈現(預設為 JET
color map)。

## 支援的影像模式

特色模式 —— 未指定 `mode_id` 時 `Config()` 開啟的模式:

| 機型 | 連結 | 特色模式 |
|---|---|---|
| G100+ | USB 3 | `1` — L'+D 1280x720@60 interleave (SDK 30fps) |
| G100+ | USB 2 | `56` — L'+D 1280x720@24 + 640x360 depth interleave (USB 2.0, SDK 12fps) |
| R77 | USB 2 | `2` — L'+D 1280x920@30 + 640x460 depth |
| G62 | USB 2 | `1` — L'+D 640x480@25 |

這些名稱就是目錄自己的寫法:`L` / `R` 是原始的左右眼,`L'` / `R'` 是
rectify 過的,`D` 是深度,並列的一對則寫成 `<width>(x2)x<height>`。

每個模式都宣告了自己需要的 USB 連結,也只能在該連結上開啟 —— 若要求的
模式是協商後的連結承載不了的,會丟出 `ValueError`。因此 G100+ 的特色模式
跟著連結走:USB 3 是模式 `1`,USB 2 是模式 `56`,以 60 fps 的幀率與全尺寸
深度換取較慢連結載得動的規格(24 fps、640x360 深度)。

模式目錄位於 `pyeys3d/modes/`;YAML 檔也隨 wheel 一起安裝
(`pyeys3d/modes/<MODEL>.yaml`),裝好套件就能直接查完整模式表:

- `pyeys3d/modes/G100P.yaml` — 80 個模式(55 個 USB 3、25 個 USB 2)
- `pyeys3d/modes/R77.yaml`  — 9 個模式(MJPEG + YUYV,含 wide L\|R)
- `pyeys3d/modes/G62.yaml`  — 15 個模式(MJPEG + YUYV,含 wide L\|R)

程式列出:

```python
from pyeys3d.modes import load_catalog
for mid, mode in sorted(load_catalog("G100P").items()):
    yuyv = "YUYV" if mode.color.fmt == 0 else "MJPEG"
    print(f"  {mid}: {mode.name}  color={yuyv}")
```

## 診斷

`PYEYS3D_LOG_LEVEL` 控制原生層的 log 詳細度:`none` / `error` / `warn`
(預設)/ `info` / `debug`。設 `PYEYS3D_TIMING=1` 可在 pipeline 停止時
輸出各階段耗時(色彩解碼、深度轉換+濾波)。`PYEYS3D_PC_THREADS` 可指定
`PointCloud` 重投影使用的 worker 數(預設 4,上限為核心數)—— 這兩趟是
記憶體頻寬受限,增加 worker 的效益低於核心數所暗示的。原生錯誤訊息帶有 eSPDI 錯誤碼名稱與提示,例如
`APC_OpenDevice2 failed: rc=-27 APC_NOT_SUPPORT_RES (the device rejected
this mode)`。目錄中的模式若開在錯誤的 USB 連結上,會更早被攔下 —— 由
`start()` 丟出 `ValueError`。

## 相容性

Python ≥ 3.8。Linux x86_64 與 aarch64(Jetson),及 Windows 10/11 x64。

## 支援

問題與錯誤回報:<support@eys3d.com>。為了讓問題一眼可診斷,請附上以
`PYEYS3D_LOG_LEVEL=info` 重跑的失敗指令、啟動時印出的 `device_info` 那一行
(機型 / 序號 / 韌體),以及你的作業系統與 Python 版本。開發環境設定與
測試 / lint 關卡見 [CONTRIBUTING.md](../CONTRIBUTING.md)。

## 授權

Apache-2.0。詳見 `LICENSE`。
