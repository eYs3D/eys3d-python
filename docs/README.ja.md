# eYs3D ステレオ深度カメラ Python ドライバ

[![Python](https://img.shields.io/badge/Python-3.8%20%E2%80%93%203.13-blue)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)

**Language:** [English](../README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`pyeys3d` は eYs3D ステレオ深度カメラの公式 Python ドライバです。
pipeline 中心の API(`Pipeline` / `Config` / `FrameSet`)が eSPDI C API
を直接呼び出し、カラー・深度・ポイントクラウドを出力します。CPython
3.8–3.13、Linux(x86_64・aarch64)と Windows(x64)対応。

### 対応カメラ

| モジュール | 製品型番 | USB | ステータス |
|---|---|---|---|
| **G100+** | YX80362 | USB 3.2 Gen1 | 量産 |
| **R77** | YX8072 | USB 2.0 | 量産 |
| **G62** | YX8081 | USB 2.0 | 量産 |

---

## 機能

- YUYV と **MJPEG** カラー、`rgb8` にデコード(モノクロモジュールの
  G62 / R77 はグレースケール、R = G = B)
- **ワイド L\|R カラー分割** —— 並列ステレオモードで左右眼を独立フレームとして
  出力(モードカタログの `split_lr` モード)
- **深度ポストプロセッシングフィルタ** —— spatial / temporal / hole filling
- **ポイントクラウド再投影** —— XYZ と XYZRGB、optical 座標系
- カメラの rectify log からの**デバイス内部パラメータ**(K / D / R / P)
- **ホットプラグ復旧** —— watchdog が USB 切断後にデバイスを再オープン
- **カメラ制御**は起動時にもストリーミング中にも設定可能 —— IR 強度、
  露出、ホワイトバランス、電源周波数
- **フレーム毎メタデータ** —— シリアル番号、ハードウェアタイムスタンプ、
  ホスト時計の撮像時刻、伝送ドロップカウンタ
- **デバイスバインド** —— 複数カメラ環境でシリアル番号または USB
  トポロジでカメラを固定
- **`examples/viewer.py`** —— 上記すべての機能を 1 つの画面に置き、
  カメラを動かしたまま変更

## インストール

ビルド済み wheel は各 [GitHub
Release](https://github.com/eYs3D/eys3d-python/releases) に添付されています。
Python バージョンとプラットフォーム(Linux x86_64 / aarch64 または Windows x64、CPython
3.8–3.13)に合う wheel をダウンロードしてインストールするだけです —— wheel
は eSPDI ランタイムを同梱しています。Linux ではカメラは UVC ビデオデバイス
として列挙されます。通常のデスクトップ環境では自動的にアクセス権が付与
されますが、ヘッドレスや最小構成では `/dev/video*` を読むためにユーザーを
`video` グループに追加する必要がある場合があります(`sudo usermod -aG
video $USER` の後、再ログイン)。
Windows では、ほとんどのマシンに既に入っている 2 つのシステム
コンポーネントが必要です:

- [Visual C++ 2015–2022 再頒布可能パッケージ
  (x64)](https://aka.ms/vs/17/release/vc_redist.x64.exe) —— これが無いと
  `import pyeys3d` が「DLL load failed / 指定されたモジュールが
  見つかりません」で失敗します
- システムの OpenCL ランタイム —— GPU ドライバに同梱

```bash
pip install pyeys3d-1.0.0-cp310-cp310-linux_x86_64.whl   # Linux
pip install pyeys3d-1.0.0-cp310-cp310-win_amd64.whl      # Windows
```

ソースからのビルド —— C++17 コンパイラが必要です。CMake と Ninja が
未インストールならビルド時に自動取得されます:

- Linux:GCC(`apt install build-essential`)
- Windows:[Visual Studio 2022 Build
  Tools](https://visualstudio.microsoft.com/visual-studio-build-tools/)
  の **C++ によるデスクトップ開発** ワークロード


```bash
git clone https://github.com/eYs3D/eys3d-python.git
cd eys3d-python
pip install .
```

## クイックスタート

引数なしの場合、接続中のカメラを自動検出し、そのシグネチャモード(モデルの
カタログで定義された既定モード)で開きます:

```python
import pyeys3d as ey
import cv2

with ey.Pipeline() as pipeline:
    pipeline.start(ey.Config())            # 自動検出 + シグネチャモード
    dev = pipeline.device_info
    intr = pipeline.intrinsics             # 未校正なら None
    print(f"Opened {dev.model}  serial {dev.serial_number}"
          f"  firmware {dev.firmware_version}")
    if intr is not None:
        print(f"  fx={intr.fx:.1f} fy={intr.fy:.1f} cx={intr.cx:.1f} "
              f"cy={intr.cy:.1f}  baseline {intr.baseline_mm:.2f} mm")
    colorizer = ey.Colorizer(pipeline)     # 深度 -> rgb8 カラーマップ
    # 再投影には、未校正の機体が持たないキャリブレーションが必要です。
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
        dmm = depth.get_data()             # (H, W) uint16、1 単位 = 1 mm
        verts = pc.calculate(depth)[0] if pc else ()   # (N, 3) float32、単位はメートル

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

## サンプル

`examples/` には主題ごとに実行可能なファイルが 1 つずつあります。
`quickstart.py` と `viewer.py` が出発点で、残りは `01` のカラー + 深度の
土台にそれぞれ 1 主題を加えます:

| ファイル | 内容 |
|---|---|
| `quickstart.py` | 上のプログラムを、そのままコピーできる形で |
| `viewer.py` | 全機能を画面上のメニューから操作 |
| `hello_depth.py` | カメラが動くこと —— ウィンドウも追加パッケージも無し |
| `00_enumerate.py` | 何が接続され、どのビデオモードを持つか |
| `01_basic_color_depth.py` | カラー + 深度、および `start()` で適用される設定 |
| `02_pointcloud.py` / `03_pointcloud_open3d.py` | 深度を 3D ポイントクラウドに、pyglet と Open3D で |
| `04_capture.py` | スナップショット、クリップ録画と再生 |
| `05_runtime_controls.py` | ストリーミング中に変更できる設定 |
| `06_multicam.py` | 複数カメラ、カメラごとに 1 プロセス |

**カメラが手元にあるなら、`viewer.py` から始めてください。** すべての設定を
1 つの画面に置き、カメラを動かしたまま変更します —— ビデオモード、深度
クリップ、フィルタ、IR、露出、ホワイトバランス —— スナップショット、
クリップ録画、3D クラウドも、接続中の全カメラに対して一度に。そこへ届く
のに何かを渡す必要はありません:

```bash
pip install opencv-python "pyglet>=2"
python examples/viewer.py
```

サンプルごとのフラグ、キー表、セットアップは
**[examples/README.md](../examples/README.md)**
([オンラインで見る](https://github.com/eYs3D/eys3d-python/tree/main/examples))
にあります。

wheel がインストールするのはライブラリ本体のみです —— 同じ Release ページ
から `pyeys3d-<version>-examples.zip` をダウンロードするか、本リポジトリを
使ってください。

## API 概要

### `Context` — デバイス列挙

軽量オブジェクト —— 再スキャンが必要になったら都度生成します。

```python
ctx = ey.Context()
for dev in ctx.query_devices():
    print(dev)
# DeviceInfo(model='G100P', serial_number='8036259M200025', usb_port='2-2:1.0',
#            pid=385, usb_port_type=3, usb_speed='USB3.0',
#            firmware_version='YX80362-B01-...')
```

### `Config` — 宣言的設定

`pyeys3d/modes/<MODEL>.yaml` のモードカタログから選択します。`enable_device`
の引数はすべて省略可能です。model を省略すると接続中のカメラを自動検出し、
`mode_id` を省略するとそのモデルの**シグネチャモード**(カタログで定義
された既定モード。モデル別一覧は下の「対応するビデオモード」)が使われます:

```python
ey.Config()                                    # 自動検出 + シグネチャモード
ey.Config().enable_device("G100P")             # このモデル、シグネチャモード
config = (ey.Config()
          .enable_device("G100P", mode_id=1)
          .set_ir_value(3)                           # モデル依存の範囲;-1 = 既定
          .set_auto_exposure(True))
```

その他の任意のカメラ制御 —— いずれも設定したときだけ `start()` で適用され、
未設定ならそのまま触れられません:

| メソッド | 引数 |
|---|---|
| `set_auto_exposure(enabled)` | `True` / `False`;オフにした場合は `set_exposure` で値を設定 |
| `set_exposure(value)` | 手動露出、レジスタ単位(自動露出はオフになります) |
| `set_auto_white_balance(enabled)` | `True` / `False`;オフにした場合は `set_white_balance` で値を設定 |
| `set_white_balance(value)` | 手動ホワイトバランス、レジスタ単位(自動 WB はオフになります);**カラーモデルのみ** |
| `set_power_line_frequency(mode)` | フリッカー低減(露出を商用電源に同期させ、照明のフリッカーが画像に縞を出さないようにする):`1` 50 Hz / `2` 60 Hz |
| `set_ir_value(level)` | IR プロジェクタの強度(モデル依存の範囲;0 = オフ)、または後述のモード対応の既定値を使う `-1` |
| `set_depth_range(near_mm, far_mm)` | `[near_mm, far_mm]` の外側の深度を捨てる;未設定 = モデル既定値、`far_mm` は最大 16383(14 bit 深度の上限) |
| `set_depth_quality_registers(source)` | ファームウェアの深度チューニングプロファイル:`True`(既定)= 同梱のモデル別プロファイル、`False` = ファームウェア既定のまま、またはカスタムプロファイルファイルのパス |

カメラ制御はカメラが対応する範囲に制限されるため、非対応の要求はホスト側で
`ValueError` になり、デバイスには届きません。範囲が固定の値は setter 自身が
拒否します。どのカメラとモードを選んだかに依存するもの —— モノクロモデル
(G62 / R77)にホワイトバランスは無く、ビデオモードは自身が宣言する USB
リンクでしか開けません(「対応するビデオモード」を参照)—— は、設定が
解決される `start()` で拒否されます。

深度ストリーミングが開始して安定した後(数秒後)、pipeline はモデルの
深度品質レジスタプロファイルをバックグラウンドでファームウェアに書き込み、USB 再接続後
にも再適用します(ファームウェアは再列挙時にリセットされます)。同梱
プロファイルは `pyeys3d/quality/DM_Quality_Cfg/<PART>_DM_Quality_Register_Setting.cfg`
(型番:G100+ `YX80362`、R77 `YX8072`、G62 `YX8081`、加えて `DEFAULT`。1 行に `address,mask,value`
の 16 進トリプル)。`set_depth_quality_registers(source)` に独自ファイル
のパスを渡して差し替えるか、`False` でファームウェア既定のままにできます。

IR プロジェクタは `start()` 時に必ず書き込まれる唯一の
制御です —— 明示的な `set_ir_value` が優先されます(0 = どのモード
でもオフ)。未設定なら既定値はモードの要件に従います:モードに深度が
ある場合(ステレオマッチングにはプロジェクタが必要)、またはモノクロ
モジュール(G62 / R77 のセンサーは IR を感知するためシーンの照明も兼ね、
IR 無しではカラーモードは真っ黒のままです)の場合はモデルカタログの
既定値、カラーセンサーでのカラーのみのモードではオフとなり、ドット
パターンがカラー画像に入り込むのを防ぎます。

### `Pipeline` — ストリーミングオーケストレータ

開いたデバイスを 1 台保持します。`start(config)` → `wait_for_frames(timeout_ms)`
は `FrameSet`(タイムアウト時は `None`)を返し → `stop()`。`poll_for_frames()`
はノンブロッキング版で、より新しいセットがあれば返し、無ければ `None`。

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

または context manager で:

```python
with ey.Pipeline() as pipeline:
    pipeline.start(config)
    ...
```

ノンブロッキング版 `pipeline.poll_for_frames()`:新しいセットがあれば返し、
なければ `None` を返します(ブロックしません)。

`start()` 後に確認できます:`pipeline.device_info`(model / serial /
usb_port)、`pipeline.color_profile` / `depth_profile`(各ストリームの
`StreamProfile(width, height, fps)`、モードに無いものは `None`)。最初のフレーム前
に分かります。

USB 切断時は watchdog がデバイスを自動で再オープンします。その間
`wait_for_frames` は `None` を返し、`pipeline.is_connected` は `False`
です(`reconnect_count` が再オープン回数)。`pipeline.frames_dropped` は
カメラが生成したがホストに届かなかったフレーム数をストリーム別に報告
します —— 増え続ける場合は USB 帯域かスケジューリングに問題があります。

自動復旧の対象はストリーミング開始後の切断です。`start()` の実行中に
抜かれた場合、切断は SDK のオープン呼び出しの内側で起きるためドライバ
からは中断できず、`start()` はカメラが接続し直されるまでブロックする
ことがあります。`start()` を呼ぶ前にカメラを接続しておいてください。

### ストリーミング中のカメラ制御

上記の制御のうち depth range と品質レジスタプロファイルを除くものには、
`Pipeline` 上に同名のランタイム版があります —— ストリーミング中に呼べば即座に反映され、再起動は不要です:

```python
pipeline.set_ir_value(4)            # モデルの範囲で検証
pipeline.set_auto_exposure(False)
pipeline.set_exposure(-6)           # 先に手動露出へ切り替え
```

`pipeline.get_*()`（`get_exposure()`、`get_ir_value()` など）はデバイス
から現在値を読み返します(非対応の制御は `None`)。ランタイムで設定した値
は USB 切断後も維持され、ホットプラグ watchdog が再接続時に最終状態を
再適用します。

露出とホワイトバランスはデバイスのレジスタ単位です —— 露出は負値になる
ことがあります(モジュールは符号付き log2 スケール、例:`-13` ≈
1/8192 秒)。各クエリは `ControlRange(min, max, step, default)` を返し、
ランタイム setter はこれに対して検証します:`get_exposure_range()` は
モジュール共通の固定レジスタ範囲、`get_white_balance_range()` はデバイスが
報告する範囲、`get_ir_range()` はモデルカタログ由来です —— IR レジスタは
モデルの適格範囲を超える値も受け付けるため、カタログが正となります。

`set_temporal_filter` は `start()` 時に `Config.with_filters(...)` で
temporal フィルタを有効にしてある場合のみ使えます(視差ストリームは
オープン時に固定されるため、再調整はできても後から有効化はできません)。

### 複数カメラ

1 つの `Pipeline` が 1 台のカメラを所有します。複数台を 1 プロセスで動かす
こともできますが、`06_multicam.py` はあえてカメラごとにプロセスを分けて
います —— 1 台がハングしても他を巻き込まないため、真似する価値のある
パターンです。ただし Windows では 1 プロセスにまとめる理由もあります:
ウィンドウが前面にないプロセスは OS に速度を落とされるため、カメラごとに
ビューアを分けると、フォーカスされている 1 台以外はフレームレートが
落ちた状態で動きます。

1 つのプロセス内では、カメラは 1 台ずつセットアップされます —— `start()`、
`Context.query_devices()`、そしてドライバ自身の再接続は互いに順番を譲り
合います。SDK がデバイスの管理情報をプロセス単位で持っているためです。
複数スレッドから開くのは安全ですが速くはなりません:オープンは順番待ちに
なります。別プロセスはこの状態を共有せず、並行して開きます。

カメラ自体は同時に 1 プロセスからしか開けません。すでに開かれている
カメラを別のプロセスが開こうとすると、`start()` が eSPDI のエラーコードを
含む `RuntimeError` を送出し、保持している側のプロセスは影響を受けずに
ストリーミングを続けます —— つまり `viewer.py` を起動したままにするだけで
次のサンプルは失敗し、閉じればカメラは解放されます。

複数台接続時は選択が一意である必要があります:
シリアル番号(部分一致)か USB ポート(完全一致)でバインド:

```python
config.enable_device("G100P", mode_id=1, usb_port="2-2:1.0")
# Windows のポート識別子はデバイスパスのインスタンス部分(物理ポートごと
# に固定)。例:usb_port="6&35c4e9&0&0000" —— 00_enumerate.py の出力から
# コピーする
config.enable_device("G100P", mode_id=1, serial_number="8036259M200025")
```

選択が曖昧な場合は、各候補カメラの model / シリアル / USB ポートを列挙
したエラーになります。`examples/00_enumerate.py` も同じ識別子を表示
します。シリアルと USB ポートを両方指定した場合、カメラは**両方**に一致
する必要があります(「このシリアルをこのポートで」)。両方に一致するカメラ
が無ければ一覧付きでエラーになります。ポートをまたいで同じ機体を追うには
シリアルだけを指定してください。

### `Frame` プロパティ

| プロパティ | 説明 |
|---|---|
| `domain` | `FrameDomain.COLOR_RGB8`(カラー)または `FrameDomain.DEPTH_MM`(深度) |
| `width`, `height` | 画像サイズ(ピクセル) |
| `frame_number` | デバイス側のストリーム別カウンタ。interleave モードでは 2 ずつ進み、2 つのストリームが 1 つの系列を共有します |
| `hw_timestamp_us` | ハードウェアタイムスタンプ(USB DMA 完了時点、μs) |
| `timestamp` | ホスト時計へマップした撮像時刻(epoch 秒、`time.time()` と直接比較可能) |
| `get_data()` | カラー: `(H, W, 3)` uint8 rgb8 / 深度: `(H, W)` uint16 mm |

`get_data()` が返すのはゼロコピーの**読み取り専用** view です。ピクセルを
変更する場合は先にコピーしてください(`img = frame.get_data().copy()`)。

ワイド L\|R 分割モードでは、右眼カラーフレームも同じ frame set から
取得できます:`frames.get_right_color_frame()`。

### 内部パラメータ

`pipeline.intrinsics` は現在のビデオモードに対応するカメラモデルを、
デバイスが保持しているそのままの形で返します:

```python
intr = pipeline.intrinsics                            # 未校正なら None
print(intr.width, intr.height, intr.baseline_mm)      # 例: 1280 720 59.93
print(intr.fx, intr.fy, intr.cx, intr.cy)             # rectified pinhole
print(intr.K, intr.D, intr.R, intr.P)                 # 完全なモデル
```

フレームは**すでに rectify 済み**で届くため、受け取る画像に対応するのは
`fx`/`fy`/`cx`/`cy`(`P` と同じ値)です。`K` と `D` は rectify 前の生の
センサーを表すもので、配信済みフレームに再適用してはいけません。
全フィールドは `docs/api.md` を参照してください。

### フィルタとポイントクラウド

深度後処理チェーンは `with_filters` で宣言します。pipeline がチェーンを
ネイティブに実行し、深度はフィルタ適用済みのミリメートル値で届きます。
フィルタ無しの場合はファームウェアのミリメートル高速パスを使います。

```python
config = (ey.Config()
          .enable_device("G100P", mode_id=1)
          .with_filters(
              ey.SpatialFilter(alpha=0.5, delta=20, magnitude=2, holes_fill=0),
              ey.TemporalFilter(alpha=0.4, delta=20, persistence=3),
              ey.HoleFillingFilter(ey.HoleFill.FARTHEST_AROUND)))
pipeline.start(config)

frames = pipeline.wait_for_frames()
depth  = frames.get_depth_frame().get_data()   # (H, W) uint16 mm、フィルタ済み

pc = ey.PointCloud(pipeline)
verts, colors = pc.calculate(frames.get_depth_frame(), frames.get_color_frame())
# verts:  (N, 3) float32 メートル、optical 座標(X 右、Y 下、Z 前)
# colors: (N, 3) uint8。カラーフレーム省略時は None
```

カラーフレームは省略可能です —— 渡せば XYZRGB、省略
(`pc.calculate(depth)`)すれば軽量な XYZ のみのクラウドになります。

深度は rectify 済みの左眼から計算されるため、その眼と同一の視点をもとから
共有しています —— 再投影すべき第二のセンサは無く、テクスチャ、計測、重ね
合わせの前にアライメント処理は不要です。深度がカラーより小さいラスタで届く
モード(scale-down モード。G100+ と R77 の USB 2 シグネチャモードを含む)
では、ピクセル座標を高さ比で換算してください —— 同じ視野の 2 つのサイズです。

チェーンの順序は固定(spatial → temporal → hole filling)で、引数の順序
に依りません。括弧内は既定値:

| フィルタ | パラメータ |
|---|---|
| `SpatialFilter` | `alpha` 平滑化 0–1、1 = なし(0.5);`delta` エッジ閾値、視差単位(20);`magnitude` パス数 1–5(2);`holes_fill` 橋渡しする最大幅、0 = オフ(0) |
| `TemporalFilter` | `alpha` 現フレーム混合率 0–1(0.4);`delta` ギャップ閾値、視差単位(20);`persistence` 欠損時の保持フレーム数 0–8(3) |
| `HoleFillingFilter` | `mode` = `HoleFill.OFF` / `FROM_LEFT` / `FARTHEST_AROUND`(既定)/ `NEAREST_AROUND` |

### `Colorizer` — 深度の可視化

深度フレームを rgb8 画像に変換します(一度生成し、毎フレーム `colorize`):

```python
colorizer = ey.Colorizer(pipeline)      # 範囲は depth clip から
rgb = colorizer.colorize(frames.get_depth_frame())   # (H, W, 3) uint8 rgb8
```

`ey.Colorizer(pipeline)` は depth clip の範囲を使います(`min_mm` / `max_mm`
で上書き可)。穴(深度 0)は黒です。`mode='grayscale'` でグレースケール
表示になります(既定は JET カラーマップ)。

## 対応するビデオモード

シグネチャモード —— `mode_id` を省略したとき `Config()` が開くモード:

| モデル | リンク | シグネチャモード |
|---|---|---|
| G100+ | USB 3 | `1` — L'+D 1280x720@60 interleave (SDK 30fps) |
| G100+ | USB 2 | `56` — L'+D 1280x720@24 + 640x360 depth interleave (USB 2.0, SDK 12fps) |
| R77 | USB 2 | `2` — L'+D 1280x920@30 + 640x460 depth |
| G62 | USB 2 | `1` — L'+D 640x480@25 |

名前はカタログのものをそのまま使っています:`L` / `R` は生の左右眼、
`L'` / `R'` は rectify 済みの左右眼、`D` は深度、並列ペアは
`<width>(x2)x<height>` と表記されます。

各モードは必要とする USB リンクを宣言しており、そのリンクでしか開けません
—— ネゴシエートされたリンクが伝送できないモードを要求すると `ValueError`
になります。したがって G100+ のシグネチャモードもリンクに従い、USB 3 では
モード `1`、USB 2 では `56` になります。後者は 60 fps のレートとフルサイズ
の深度を、低速リンクで運べる範囲(24 fps、640x360 深度)と引き換えにした
モードです。

モードカタログは `pyeys3d/modes/` にあります。YAML は wheel
にも同梱される(`pyeys3d/modes/<MODEL>.yaml`)ため、インストール済み
パッケージからそのまま全モード表を参照できます:

- `pyeys3d/modes/G100P.yaml` — 80 モード(USB 3 が 55、USB 2 が 25)
- `pyeys3d/modes/R77.yaml`  — 9 モード(MJPEG + YUYV、wide L\|R 含む)
- `pyeys3d/modes/G62.yaml`  — 15 モード(MJPEG + YUYV、wide L\|R 含む)

プログラムから一覧:

```python
from pyeys3d.modes import load_catalog
for mid, mode in sorted(load_catalog("G100P").items()):
    yuyv = "YUYV" if mode.color.fmt == 0 else "MJPEG"
    print(f"  {mid}: {mode.name}  color={yuyv}")
```

## 診断

`PYEYS3D_LOG_LEVEL` はネイティブ層のログ詳細度を制御します:`none` /
`error` / `warn`(既定)/ `info` / `debug`。`PYEYS3D_TIMING=1` を設定
すると、pipeline 停止時に各ステージの所要時間(カラーデコード、深度変換
+フィルタ)をログ出力します。`PYEYS3D_PC_THREADS` は `PointCloud` の
再投影に使うワーカ数を指定します(既定 4、コア数が上限)。この 2 パスは
メモリ帯域律速なので、ワーカを増やしてもコア数ほどには効きません。ネイティブの
エラーには eSPDI エラーコード名とヒントが付きます。例:
`APC_OpenDevice2 failed: rc=-27 APC_NOT_SUPPORT_RES (the device rejected
this mode)`。カタログにあるモードを誤った USB リンクで要求した場合は、
その手前で `start()` が `ValueError` を送出します。

## 互換性

Python ≥ 3.8。Linux x86_64 と aarch64(Jetson)、および Windows 10/11 x64。

## サポート

質問やバグ報告:<support@eys3d.com>。問題を一目で診断できるよう、
`PYEYS3D_LOG_LEVEL=info` を付けて再実行した失敗コマンド、起動時に表示される
`device_info` の行(モデル / シリアル / ファームウェア)、および OS と
Python のバージョンを添えてください。開発環境のセットアップとテスト /
lint ゲートは [CONTRIBUTING.md](../CONTRIBUTING.md) にあります。

## ライセンス

Apache-2.0。詳細は `LICENSE` を参照。
