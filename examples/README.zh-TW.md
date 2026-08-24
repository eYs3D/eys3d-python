# pyeys3d 範例程式

**Language:** [English](README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`pyeys3d` 驅動程式的可執行範例。每支檔案獨立、附完整註解,而且只講一個
主題:挑你需要的那支來讀、複製過去,再把自己的程式用不到的部分刪掉。

有兩支是起點。`quickstart.py` 是最小的完整程式,可複製到自己的專案;
`viewer.py` 用畫面上的選單驅動每一項功能 —— 想知道一台相機能做什麼,
這是最快的路。其餘各支各自加一個主題:`hello_depth.py` 完全不畫任何畫面
就證明相機能動,`00_enumerate.py` 列出接了什麼,`01` 是彩色 + 深度的
基底,`02`–`06` 各自在這個基底上加一個功能。

## 安裝

從本包所在的同一個 Release 頁,下載對應你 Python 版本與平台
(Linux x86_64 / aarch64 或 Windows x64)的 `pyeys3d` wheel 安裝:

```bash
pip install pyeys3d-<版本>-cp310-cp310-linux_x86_64.whl   # Linux
pip install pyeys3d-<版本>-cp310-cp310-win_amd64.whl      # Windows
```

`hello_depth.py` 與 `00_enumerate.py` 不需要其他套件。會繪圖的範例則需要
這些:

```bash
pip install opencv-python           # 所有會開視窗的範例
pip install "pyglet>=2"             # 02_pointcloud.py、viewer.py 的 3D 視窗
pip install open3d                  # 03_pointcloud_open3d.py
```

## 範例一覽

| 檔案 | 說明 | 需要 |
|---|---|---|
| [`quickstart.py`](#quickstartpy) | 最小的完整程式 | opencv |
| [`viewer.py`](#viewerpy) | 全功能展示,由選單驅動 | opencv, pyglet |
| [`hello_depth.py`](#hello_depthpy) | 相機能動 | — |
| [`00_enumerate.py`](#00_enumeratepy) | 接了什麼,以及它有哪些模式 | — |
| [`01_basic_color_depth.py`](#01_basic_color_depthpy) | 彩色 + 深度,以及開流時套用的設定 | opencv |
| [`02_pointcloud.py`](#02_pointcloudpy--03_pointcloud_open3dpy) | 把深度當成 3D 點雲 | opencv, pyglet |
| [`03_pointcloud_open3d.py`](#02_pointcloudpy--03_pointcloud_open3dpy) | 同一個點雲的 Open3D 版 | opencv, open3d |
| [`04_capture.py`](#04_capturepy) | 存快照與錄影,以及回放 | opencv |
| [`05_runtime_controls.py`](#05_runtime_controlspy) | 哪些設定能在串流中改 | opencv |
| [`06_multicam.py`](#06_multicampy) | 多台相機,一台一個 process | opencv |

## `01`–`06` 共用的參數

每支編號範例都接受同樣的七個,一個參數對應它呼叫的一個 API。沒列在這裡的
都是範例用一行明白寫死的預設值 —— 要改就改那一行,不是加參數。

| 參數 | 對應 API | 預設值 |
|---|---|---|
| `--model MODEL` | `Config.enable_device(model)` | 自動偵測 |
| `--mode MODE_ID` | `Config.enable_device(mode_id=)` | 該機型的特色模式 |
| `--serial SERIAL` | `Config.enable_device(serial_number=)` | 任意(子字串比對) |
| `--usb-port PORT` | `Config.enable_device(usb_port=)` | 任意(完全比對) |
| `--ir-value LEVEL` | `Config.set_ir_value()` | 機型預設;`0` 為關閉 |
| `--depth-range NEAR_MM FAR_MM` | `Config.set_depth_range()` | 機型預設 |
| `--filters` | `Config.with_filters(Spatial, Temporal, HoleFilling)` | 關閉 |

`hello_depth.py`、`quickstart.py` 與 `00_enumerate.py` 一個都不接受:前兩支
預期只有一台相機,第三支則什麼都不開。`viewer.py` 只接受 `--out`,因為其餘
一切都在畫面上。每支範例會在這之上加自己主題的參數 —— 下面各節有列,
`--help` 則列得最完整。

`--serial` 與 `--usb-port` 是你從多台裡挑一台的方式。接了不只一台而兩者
都沒給時,`Config` 寧可報錯也不會猜,以免開錯台。

## 從這裡開始

### `quickstart.py`

```bash
python quickstart.py
```

專案 README 的快速開始,可直接執行:開場印出裝置身分與內參,接著開
彩色 + 深度視窗,並在同一行狀態列顯示中心距離、點雲最近點,以及每幀的
編號與時間戳。把這支複製到自己的專案當起點 —— 它不從其他範例 import
任何東西。

它預期只有一台相機,也不接受任何參數。

### `viewer.py`

```bash
python viewer.py
python viewer.py --out /tmp/captures
```

所有相機設定收在同一個畫面,而且在相機執行中就能改 —— 想知道一台相機
能做什麼、或要不改任何東西就完整驗證一台相機,這是最快的路。每台相機
會有自己的 Color / Depth 視窗,外加一個裝著選單的 Controls 視窗。

| 按鍵 | 動作 |
|---|---|
| <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> | 在選單格之間移動 |
| <kbd>-</kbd> / <kbd>+</kbd> | 調整選取的值,或切換開關 |
| <kbd>Enter</kbd> | 套用已選好的影像模式 / 深度裁切 / 濾波變更(會重開串流) |
| <kbd>p</kbd> | 開啟 / 關閉 3D 點雲視窗 |
| <kbd>s</kbd> | 存一組快照 |
| <kbd>r</kbd> | 開始 / 停止錄影 |
| <kbd>d</kbd> | 把相機屬性還原為預設值 |
| <kbd>x</kbd> | 硬體重置相機 |
| <kbd>q</kbd> / <kbd>ESC</kbd> | 關閉目前聚焦的相機 |

`--out DIR` 是唯一的參數,而且只是因為畫面上沒有地方放它;它會在啟動時
解析並印出(`captures -> ...`),所以每一行 `saved` 指的都是找得到的檔案。

選單中涵蓋影像模式、IR、電源頻率、自動 / 手動曝光、自動 / 手動白平衡、
深度裁切與三種深度濾波。影像模式、深度裁切與濾波在選好但還沒送出時會標上
`*`,並由 <kbd>Enter</kbd> 一起套用,因為這三者都在 `start()` 時固定,
改動就得重開串流 —— 每改一次相機就會消失幾秒。其餘的則是按下按鍵當下
就生效。錄影期間 <kbd>Enter</kbd> 與 <kbd>x</kbd> 會被擋下。

接了多台相機時,每台都先停在模式選擇畫面而不直接串流 —— Controls 視窗
會出現,預覽則不會。選一個模式後按 <kbd>Enter</kbd> 啟動那一台,或按
<kbd>q</kbd> 把它關掉。挑選時要把共用的 USB 匯流排放在心上:這些相機
分食同一個主控制器的頻寬,而單單一個特色模式就可能吃掉 USB 3 連結的
大半。要對哪一台送按鍵,就點一下那台相機的視窗。

檢視器做不到、而編號範例做得到的:每幀 metadata(`01 --frame-meta`)、
開場印出的完整相機模型(`01`),以及片段回放(`04 --play`)。

## 其餘各支,一支一個主題

### `hello_depth.py`

```bash
python hello_depth.py
```

沒有視窗,除 `pyeys3d` 外不需要任何套件。它會持續印出畫面中心的距離,
所以在這裡失敗就是相機、驅動程式或線材的問題,絕不會是顯示程式碼。
它預期只有一台相機,也不接受任何參數;接了多台時,請在檔案裡的
`enable_device()` 呼叫中指名其中一台。

### `00_enumerate.py`

```bash
python 00_enumerate.py
```

先列出每個**已連接**機型的完整影像模式目錄:id、該模式需要的 USB 連結、
解析度與幀率,並在每條連結的特色模式上標 `*`(不指定 `mode_id` 時
`start()` 開的就是它)—— 接著印出每台相機一行摘要。用 `--mode`
之前先跑它,看看有哪些 id。

### `01_basic_color_depth.py`

```bash
python 01_basic_color_depth.py
python 01_basic_color_depth.py --mode 3 --filters --frame-meta
```

其他範例的基底:Color (Left)、模式有分割 L|R 時的 Color (Right),以及
Depth 各開在自己的視窗,游標移到哪裡就讀出該點的 RGB 或距離,而裝置裡
存的完整相機模型(K / D / R / P)在開場印出。

| 參數 | 作用 |
|---|---|
| `--frame-meta` | 每秒印一次某一幀的編號、它的硬體與主機時間戳,以及它距擷取所經過的時間 —— 這些欄位用來和其他感測器對齊時間,以及量測延遲 |

### `02_pointcloud.py` / `03_pointcloud_open3d.py`

```bash
python 02_pointcloud.py
python 03_pointcloud_open3d.py
```

01 再加上即時 3D 點雲,每幀都由深度影像與裝置存的內參重建。兩支檔案是
同一個程式、只換顯示層 —— `02` 用 pyglet/OpenGL,`03` 用 Open3D。點雲
視窗中:拖曳可旋轉、中鍵拖曳平移、滾輪縮放,<kbd>R</kbd> 重設視角、
<kbd>Q</kbd> / <kbd>ESC</kbd> 關閉。

### `04_capture.py`

```bash
python 04_capture.py                      # 開視窗;按 s 存一組
python 04_capture.py --snapshot           # 存一組就結束,不開視窗
python 04_capture.py --record 10          # 錄十秒後結束
python 04_capture.py --play capture/clips/20260101-120000-000
```

存快照與錄影,以及回放。一切都落在 `--out`(預設 `./capture/`)底下:

```
capture/snapshots/<stamp>_color.png            按 s,或 --snapshot
capture/snapshots/<stamp>_depth.png            原始 16-bit,1 單位 = 1 mm
capture/snapshots/<stamp>_depth_preview.png    可直接看的呈現
capture/snapshots/<stamp>_cloud.ply            MeshLab / Open3D
capture/clips/<stamp>/                         一段錄下的片段
    color/000000.jpg  depth/000000.png         成對的影格組
    metadata.jsonl                             索引 + 校正資料
```

| 參數 | 作用 |
|---|---|
| `--out DIR` | 快照與錄影的寫入位置(預設 `capture`) |
| `--snapshot` | 存一組就結束,不開視窗也不用按鍵。它會先給自動曝光時間收斂,所以寫下的是第一張曝光正確的影格,而不是第一張到達的 |
| `--record SECONDS` | 錄這麼多秒的片段,然後結束 |
| `--play DIR` | 以錄製時的速度回放一段片段 |

`metadata.jsonl` 第一行帶著裝置、深度範圍與內參;之後每一行索引一組影格。
深度 PNG 是 uint16、一單位一毫米,所以大多數看圖軟體會顯示成幾乎全黑;
旁邊的 `_depth_preview.png` 才是可直接看的呈現。要讀回數值請用
`cv2.imread(path, cv2.IMREAD_UNCHANGED)`。

### `05_runtime_controls.py`

```bash
python 05_runtime_controls.py
```

哪些設定能在相機串流中改變。透過 `Config` 設定的東西只在 `start()` 時
套用一次;這支展示的是 `Pipeline` 上的對應版本 —— IR、自動 / 手動曝光、
自動 / 手動白平衡(僅彩色機型)與電源頻率 —— 放在自己視窗裡的方向鍵
選單中。

| 按鍵 | 動作 |
|---|---|
| <kbd>↑</kbd> <kbd>↓</kbd> | 選擇一個控制項 |
| <kbd>-</kbd> / <kbd>+</kbd>(或 <kbd>←</kbd> <kbd>→</kbd>) | 調整它;AE / AWB 則是開關切換 |
| <kbd>d</kbd> | 還原預設 —— IR 回機型預設值,AE / AWB 回自動 |
| <kbd>x</kbd> | 硬體重置相機,讓它在 USB 上重新列舉 |
| <kbd>q</kbd> / <kbd>ESC</kbd> | 結束 |

每次變更後數值都會從裝置讀回,而且撐得過 USB 斷線:熱插拔 watchdog
會在重連時重新套用最後的狀態。按 <kbd>x</kbd> 就能看到 —— Link 那一行
會跟著相機掉下去再回來。

### `06_multicam.py`

```bash
python 06_multicam.py
python 06_multicam.py --model G100P --mode 3
```

對每一台連接的相機同時跑 01,一台相機一個 process。父行程先列舉,再為
每台裝置 spawn 一個以它讀到的序號綁定的子行程,然後等待;每個子行程開
一個 `color | depth` 視窗,標題帶著它的幀率。`--model` / `--serial` /
`--usb-port` 用來縮小要開哪幾台,其餘的共用參數則轉發給每一個子行程。

## 多台相機:一台一個 process,還是全部同一個 process

`06_multicam.py` 與 `viewer.py` 示範這兩種安排,而選擇是取捨,不是偏好:

- **一相機一 process**(`06`)讓它們彼此獨立 —— 某一台卡死不會拖垮其他
  相機 —— 而且開啟動作是並行的,因為 SDK 以 process 為單位的記帳並不跨
  process 共用。在 Windows 上,視窗不在前景的那幾台相機會以偏低的幀率
  執行:系統會把背景 process 降速。
- **同一個 process**(`viewer.py`)沒有這種落差 —— 不論點的是哪個視窗,
  所有相機都留在前景 —— 但這些相機必須一台一台設定,所以 N 台相機就是
  N 次開啟接連跑完。

同一個 process 內由不同執行緒開啟的相機,也基於同樣理由被驅動程式序列化,
呼叫端不需要做任何事。

## 這些檔案怎麼組織

`example_helpers.py` 收納這些範例要成為一支程式所需的骨架 —— 共用參數、
主控台編碼、啟動時印出的裝置摘要,以及 OpenCV 視窗 —— 且不含任何
`pyeys3d` 呼叫。讀者是為了 API 呼叫而來的,那些就留在教它的那支範例裡,
即使因此和另一支重複也一樣。

兩支範例做同一件事的地方,就用一模一樣的字寫,這樣把它們 diff 起來只會
看到後面那支多做了什麼。真正不同的地方 —— `02` 的點雲視窗由主迴圈驅動、
`viewer.py` 的跑在自己的執行緒上 —— 就讓它們分開,而不是合併成兩邊都
不好用的東西。

## 注意事項

- **Windows**:wheel 需要 Visual C++ 2015–2022 可轉散發套件(x64)與
  系統 OpenCL 執行環境(任一 GPU 驅動都會安裝)。
- 完整 API 參考文件:[`docs/api.md`](../docs/api.md) —— 原始碼與本範例包內都有。
