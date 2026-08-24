# pyeys3d サンプルプログラム

**Language:** [English](README.md) · [日本語](README.ja.md) · [繁體中文](README.zh-TW.md) · [简体中文](README.zh-CN.md)

`pyeys3d` ドライバの実行可能なサンプルです。各ファイルは自己完結でコメント
付き、1 つの主題だけを扱います —— 必要なものを読み、コピーして、自分の
プログラムが使わない部分を削ってください。

出発点は 2 つあります。`quickstart.py` は完結する最小のプログラムで、自分の
プロジェクトへコピーするためのものです。`viewer.py` はすべての機能を画面上の
メニューから操作するもので、カメラに何ができるかを見るには一番の近道です。
残りはそれぞれ 1 つずつ主題を足します:`hello_depth.py` は表示を一切使わずに
カメラが動くことを確かめ、`00_enumerate.py` は何が接続されているかを一覧し、
`01` はカラー + 深度のベースで、`02`–`06` はそのベースに 1 つずつ機能を
足します。

## セットアップ

本バンドルと同じ Release ページから、Python バージョンとプラットフォーム
(Linux x86_64 / aarch64 または Windows x64)に合う `pyeys3d` wheel を
ダウンロードしてインストールします:

```bash
pip install pyeys3d-<version>-cp310-cp310-linux_x86_64.whl   # Linux
pip install pyeys3d-<version>-cp310-cp310-win_amd64.whl      # Windows
```

`hello_depth.py` と `00_enumerate.py` は他のパッケージ不要です。描画を
行うサンプルには以下が必要です:

```bash
pip install opencv-python           # ウィンドウを開くサンプル全部
pip install "pyglet>=2"             # 02_pointcloud.py、viewer.py の 3D ウィンドウ
pip install open3d                  # 03_pointcloud_open3d.py
```

## サンプル一覧

| ファイル | 説明 | 必要 |
|---|---|---|
| [`quickstart.py`](#quickstartpy) | 完結する最小のプログラム | opencv |
| [`viewer.py`](#viewerpy) | 全機能をメニューから操作 | opencv, pyglet |
| [`hello_depth.py`](#hello_depthpy) | カメラが動くことの確認 | — |
| [`00_enumerate.py`](#00_enumeratepy) | 何が接続され、どのモードを持つか | — |
| [`01_basic_color_depth.py`](#01_basic_color_depthpy) | カラー + 深度、および起動時に適用される設定 | opencv |
| [`02_pointcloud.py`](#02_pointcloudpy--03_pointcloud_open3dpy) | 深度を 3D ポイントクラウドとして表示 | opencv, pyglet |
| [`03_pointcloud_open3d.py`](#02_pointcloudpy--03_pointcloud_open3dpy) | 同じクラウドを Open3D で | opencv, open3d |
| [`04_capture.py`](#04_capturepy) | スナップショットとクリップの保存、そして再生 | opencv |
| [`05_runtime_controls.py`](#05_runtime_controlspy) | ストリーミング中に変更できる設定 | opencv |
| [`06_multicam.py`](#06_multicampy) | 複数カメラ、カメラごとに 1 プロセス | opencv |

## `01`–`06` 共通のオプション

番号付きサンプルはいずれも同じ 7 つを受け取ります —— 呼び出す API 1 つに
つき 1 つです。ここに無いものはサンプルが 1 行で設定している既定値です。
フラグではなく、その行を書き換えてください。

| フラグ | 対応 API | デフォルト |
|---|---|---|
| `--model MODEL` | `Config.enable_device(model)` | 自動検出 |
| `--mode MODE_ID` | `Config.enable_device(mode_id=)` | モデルのシグネチャモード |
| `--serial SERIAL` | `Config.enable_device(serial_number=)` | 任意(部分一致) |
| `--usb-port PORT` | `Config.enable_device(usb_port=)` | 任意(完全一致) |
| `--ir-value LEVEL` | `Config.set_ir_value()` | モデル既定値;`0` でオフ |
| `--depth-range NEAR_MM FAR_MM` | `Config.set_depth_range()` | モデル既定値 |
| `--filters` | `Config.with_filters(Spatial, Temporal, HoleFilling)` | オフ |

`hello_depth.py`・`quickstart.py`・`00_enumerate.py` はフラグを受け取り
ません。前の 2 つはカメラ 1 台を前提とし、3 つ目は何も開かないためです。
`viewer.py` が受け取るのは `--out` だけで、それ以外はすべて画面上にある
からです。各サンプルは自分の主題のフラグをこれに足します —— 下の各節に
挙げてあり、`--help` で全部確認できます。

`--serial` と `--usb-port` は、複数台から 1 台を選ぶための手段です。2 台
以上つながっていてどちらも指定が無い場合、`Config` は当てずっぽうで違う
カメラを開くのではなく、エラーにします。

## まずはここから

### `quickstart.py`

```bash
python quickstart.py
```

プロジェクト README のクイックスタートを、そのまま実行できる形にした
ものです:起動時にデバイス情報と内部パラメータを表示し、続いてカラー +
深度のウィンドウを開き、中心距離・クラウドの最近点・各フレームの番号と
タイムスタンプを 1 行のステータス行に表示します。出発点として自分の
プロジェクトへコピーしてください —— 他のサンプルから何も import して
いません。

カメラ 1 台を前提とし、フラグは受け取りません。

### `viewer.py`

```bash
python viewer.py
python viewer.py --out /tmp/captures
```

カメラのすべての設定を 1 つの画面に置き、カメラを動かしたまま変更します
—— カメラに何ができるかを見るにも、1 台を端から端まで検証するにも、
何も書き換えずに済む一番の近道です。各カメラは Color / Depth ウィンドウ
に加えて、メニューを載せた Controls ウィンドウを持ちます。

| キー | 動作 |
|---|---|
| <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> | メニューのセル間を移動 |
| <kbd>-</kbd> / <kbd>+</kbd> | 選択中の値を増減、または切り替え |
| <kbd>Enter</kbd> | 保留中のビデオモード / 深度クリップ / フィルタ変更を適用(ストリームを開き直します) |
| <kbd>p</kbd> | 3D ポイントクラウドウィンドウを開く / 閉じる |
| <kbd>s</kbd> | スナップショット一式を保存 |
| <kbd>r</kbd> | クリップ録画の開始 / 停止 |
| <kbd>d</kbd> | カメラのプロパティを既定値に戻す |
| <kbd>x</kbd> | カメラをハードウェアリセット |
| <kbd>q</kbd> / <kbd>ESC</kbd> | フォーカス中のカメラを閉じる |

オプションは `--out DIR` だけで、それも画面上に置き場所がないからです。
起動時に解決されて表示されるため(`captures -> ...`)、`saved` の各行は
必ず見つけられるファイル名を示します。

メニューにはビデオモード、IR、電源周波数、自動 / 手動露出、自動 / 手動
ホワイトバランス、深度クリップ、3 つの深度フィルタが並びます。ビデオ
モード・深度クリップ・フィルタは保留中は `*` が付き、<kbd>Enter</kbd> で
まとめて適用されます。3 つとも `start()` 時に固定されるもので、変更すると
ストリームを開き直すため、そのたびにカメラが数秒間いなくなるからです。
それ以外はキーを押した時点で反映されます。クリップ録画中は
<kbd>Enter</kbd> と <kbd>x</kbd> を受け付けません。

複数台のカメラが接続されている場合、各カメラはストリーミングせずに
モードピッカーで止まります —— Controls ウィンドウは開きますが、プレビュー
は開きません。モードを選んで <kbd>Enter</kbd> でそのカメラを開始するか、
<kbd>q</kbd> で閉じます。選ぶときは共有する USB バスを念頭に置いて
ください:カメラは 1 つのホストコントローラの帯域を分け合い、シグネチャ
モードだけで USB 3 リンクの大半を占めることもあります。キーを送りたい
カメラのウィンドウをクリックしてください。

ビューアにできず、番号付きサンプルにできること:フレーム毎メタデータ
(`01 --frame-meta`)、起動時の完全なカメラモデル(`01`)、クリップ再生
(`04 --play`)。

## 残り —— それぞれ 1 つの主題

### `hello_depth.py`

```bash
python hello_depth.py
```

ウィンドウも、`pyeys3d` 以外のパッケージも使いません。画像中心の距離を
更新のたびに表示するので、ここで失敗するならカメラかドライバかケーブル
であって、表示コードではありえません。カメラ 1 台を前提とし、フラグは
受け取りません。複数台つながっている場合は、ファイル内の
`enable_device()` の呼び出しで 1 台を指定してください。

### `00_enumerate.py`

```bash
python 00_enumerate.py
```

まず**接続中の**モデルごとに完全なビデオモードカタログを表示します:
id、そのモードが必要とする USB リンク、解像度とフレームレート、そして
各リンクのシグネチャモード(`mode_id` を指定しないとき `start()` が開く
モード)に `*` を付けます。続いてカメラごとに要約行を表示します。
`--mode` を使う前に実行して、どの id があるかを確かめてください。

### `01_basic_color_depth.py`

```bash
python 01_basic_color_depth.py
python 01_basic_color_depth.py --mode 3 --filters --frame-meta
```

他のサンプルが積み上げるベースです。Color (Left)、モードが L|R を分割
する場合は Color (Right)、そして Depth がそれぞれ独立したウィンドウで
開き、カーソルを乗せるとその位置の RGB または距離が読めます。保存されて
いる完全なカメラモデル(K / D / R / P)は起動時に表示されます。

| フラグ | 効果 |
|---|---|
| `--frame-meta` | 1 秒に 1 回、フレーム 1 枚の番号、ハードウェアとホストのタイムスタンプ、その経過時間を表示 —— 他のセンサーと時刻を揃えるため、そして遅延を測るためのフィールドです |

### `02_pointcloud.py` / `03_pointcloud_open3d.py`

```bash
python 02_pointcloud.py
python 03_pointcloud_open3d.py
```

01 に、深度画像と保存された内部パラメータから毎フレーム再構築される
ライブ 3D ポイントクラウドを足したものです。2 つのファイルは表示レイヤ
だけが違う同じプログラムで、`02` は pyglet/OpenGL、`03` は Open3D です。
クラウドウィンドウでは、ドラッグで回転、中ボタンドラッグで平行移動、
スクロールでズーム、<kbd>R</kbd> で視点をリセット、<kbd>Q</kbd> /
<kbd>ESC</kbd> で閉じます。

### `04_capture.py`

```bash
python 04_capture.py                      # ウィンドウ表示、s で 1 組保存
python 04_capture.py --snapshot           # 1 組保存して終了(ウィンドウ無し)
python 04_capture.py --record 10          # 10 秒録画して終了
python 04_capture.py --play capture/clips/20260101-120000-000
```

スナップショットとクリップの保存、そして再生。出力はすべて
`--out`(既定は `./capture/`)の下に置かれます:

```
capture/snapshots/<stamp>_color.png            s を押す、または --snapshot
capture/snapshots/<stamp>_depth.png            生の 16 bit、1 単位 = 1 mm
capture/snapshots/<stamp>_depth_preview.png    閲覧用のレンダリング
capture/snapshots/<stamp>_cloud.ply            MeshLab / Open3D
capture/clips/<stamp>/                         録画したクリップ 1 本
    color/000000.jpg  depth/000000.png         対になったフレームセット
    metadata.jsonl                             インデックス + キャリブレーション
```

| フラグ | 効果 |
|---|---|
| `--out DIR` | スナップショットとクリップの書き込み先(既定 `capture`) |
| `--snapshot` | 1 組保存して終了。ウィンドウもキー入力も不要です。先に自動露出が収束するのを待つため、書き出されるのは最初に届いたフレームではなく、最初に適正露出になったフレームです |
| `--record SECONDS` | その秒数だけクリップを録画して終了 |
| `--play DIR` | クリップを録画時のペースで再生 |

`metadata.jsonl` の 1 行目にはデバイス、深度範囲、内部パラメータが入り、
以降の各行がフレームセット 1 組を索引します。深度 PNG は 1 単位 = 1 mm の
uint16 なので、たいていの画像ビューアではほぼ真っ黒に見えます。隣にある
`_depth_preview.png` が閲覧用のレンダリングです。値を読み戻すには
`cv2.imread(path, cv2.IMREAD_UNCHANGED)` を使ってください。

### `05_runtime_controls.py`

```bash
python 05_runtime_controls.py
```

カメラのストリーミング中にどの設定を変えられるかを示します。`Config`
経由の設定は `start()` 時に 1 度だけ適用されます。ここで扱うのはその
`Pipeline` 側の対応物 —— IR、自動 / 手動露出、自動 / 手動ホワイト
バランス(カラーモデルのみ)、電源周波数 —— で、矢印キーのメニューを
独立したウィンドウに置きます。

| キー | 動作 |
|---|---|
| <kbd>↑</kbd> <kbd>↓</kbd> | 制御項目を選択 |
| <kbd>-</kbd> / <kbd>+</kbd>(または <kbd>←</kbd> <kbd>→</kbd>) | 値を増減。AE / AWB はオン・オフの切り替え |
| <kbd>d</kbd> | 既定値に復帰 —— IR はモデル既定値、AE / AWB は自動へ |
| <kbd>x</kbd> | カメラをハードウェアリセットし、USB 上で再列挙させる |
| <kbd>q</kbd> / <kbd>ESC</kbd> | 終了 |

値は変更のたびにデバイスから読み返され、USB 切断後も維持されます:
ホットプラグ watchdog が再接続時に最終状態を再適用します。<kbd>x</kbd>
を押すとその様子が見えます —— Link の行がカメラの切断と復帰を追います。

### `06_multicam.py`

```bash
python 06_multicam.py
python 06_multicam.py --model G100P --mode 3
```

接続中のすべてのカメラに対する 01 を、カメラごとに 1 プロセスで同時に
実行します。親プロセスが列挙し、読み取ったシリアルで固定した子プロセスを
デバイスごとに起動して待ちます。各子プロセスは `color | depth` の
ウィンドウを 1 つ開き、タイトルにフレームレートを出します。`--model` /
`--serial` / `--usb-port` で開くカメラを絞り込み、残りの共通フラグは
すべての子プロセスへ転送されます。

## 複数カメラ:カメラごとに 1 プロセスか、1 プロセスにまとめるか

`06_multicam.py` と `viewer.py` は 2 通りの構成を示します。どちらを選ぶかは
好みではなくトレードオフです:

- **カメラごとに 1 プロセス**(`06`)は各カメラを独立させ —— 1 台がハング
  しても他を巻き込みません —— オープン処理も並列に走ります。SDK のプロセス
  単位の管理情報はプロセス間で共有されないためです。Windows ではウィンドウ
  が前面にないカメラはフレームレートが落ちた状態で動きます:OS がバック
  グラウンドのプロセスの速度を落とすためです。
- **1 プロセス**(`viewer.py`)にはこの差がありません —— どのウィンドウを
  クリックしても全カメラが前面のままです —— 代わりにカメラは 1 台ずつ
  セットアップする必要があるため、N 台なら N 回のオープンが連続します。

1 つのプロセスの別スレッドから開いたカメラも、同じ理由でドライバが直列化
します。呼び出し側で何かする必要はありません。

## ファイルの組み立て方

`example_helpers.py` はサンプルが 1 つのプログラムとして成立するための
足回り —— 共通フラグ、コンソールのエンコーディング、起動時に表示する
デバイス概要、そして OpenCV のウィンドウ —— をまとめたもので、`pyeys3d`
の呼び出しは 1 つも含みません。読み手が見に来た API 呼び出しは、たとえ
他のファイルと重複しても、それを教えるサンプルの中に残してあります。

2 つのサンプルが同じことをしている箇所は、同じ字面で書いてあります。
差分を取れば、後のサンプルが足した分だけが見えるようにするためです。
本当に違う箇所 —— `02` のクラウドウィンドウはメインループから駆動され、
`viewer.py` のものは専用スレッドで動きます —— は、どちらにも役立たない
何かに統合せず、別々のままにしてあります。

## 補足

- **Windows**:wheel には Visual C++ 2015–2022 再頒布可能パッケージ
  (x64)と、システムの OpenCL ランタイム(GPU ドライバに同梱)が必要です。
- 完全な API リファレンス:[`docs/api.md`](../docs/api.md) —— リポジトリと本サンプルアーカイブの両方に含まれます。
