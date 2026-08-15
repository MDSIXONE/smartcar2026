# 主流程复制：国赛版与国赛额外任务版（2026-08-16）

## 目的

省赛主流程（`ucar_2026`）复制为两份独立副本，供国赛使用：

1. **`ucar_2026_national`** —— 国赛正式版：流程与省赛完全一致，唯一区别是地图墙位改动：
   把编号图中 148 与 159 之间的竖直墙壁（x=0，y 1.5~2.0，即 65 号格与 66 号格之间的墙）
   移动到 147 与 158 之间（x=-0.5，y 1.5~2.0，即 64 号格与 65 号格之间）。
2. **`ucar_2026_extra`** —— 国赛现场随机任务版：流程暂与省赛一致，地图沿用国赛布局
   （同一场地），后续在此副本上开发随机任务逻辑。

三场比赛各自独立：省赛（`ucar_2026`）、国赛（`ucar_2026_national`）、
国赛额外任务（`ucar_2026_extra`）。

## 地图改动的语义

- 网格编号图中 148=(0.0, 2.0)、159=(0.0, 1.5)，两点之间（x=0 竖线）原有墙；
  147=(-0.5, 2.0)、158=(-0.5, 1.5) 之间无墙。
- 改后：147~158 之间新增墙，148~159 之间墙被移除（65/66 格之间变空，64/65 格之间新增墙）。
- **真车导航 pgm 同步改版**（`ucar_nav/maps/iflysse_field_walls_national.{pgm,yaml}` 新增）：
  以 `iflysse_field_walls_without_middle_vertices` 为底图，把 x=0（y 1.53~2.0）的
  竖直墙段（col 248-251 × row 100..147）整体平移到 x=-0.5（col 198-201），其余像素
  与省赛版完全一致（逐像素 diff = 0）。
- 国赛两个包的 `2026.launch` 中 `map_file` 默认值改为
  `$(find ucar_nav)/maps/iflysse_field_walls_national.yaml`；省赛包 `ucar_2026` 仍用
  原图不动。move_base 全局代价地图与 `navigation_scan_relay` 静态墙过滤均从 `/map`
  读取，加载新图后自动与国赛物理场地一致，流程代码零改动。

## 涉及文件

- 新增目录 `ucar_ws/src/ucar_2026_national/`（整体复制自 `ucar_2026`）：
  - 所有文件内容中包名引用 `ucar_2026` → `ucar_2026_national`
    （CMakeLists.txt project、package.xml name、launch 中 `$(find ...)`/`pkg=`、
    start/stop/handoff 脚本、urdf 内容等）
  - `urdf/ucar_2026_visual.urdf` 重命名为 `urdf/ucar_2026_national_visual.urdf`
    （与 2026.launch 内容引用保持一致）
  - `config/production_full_grid_all_numbered.json` 原样复制（网格点坐标不变）
  - `config/production_full_grid_all_numbered.png` **新增**：改墙后的国赛版编号图
    （根目录 `production_full_grid_all_numbered.png` 保持省赛版原样不动）
- 新增目录 `ucar_ws/src/ucar_2026_extra/`（同上，包名替换为 `ucar_2026_extra`，
  urdf 重命名为 `ucar_2026_extra_visual.urdf`，config/ 含同一张国赛版编号图）。
- 新增 `ucar_ws/src/ucar_nav/maps/iflysse_field_walls_national.{pgm,yaml}`：
  国赛版真车导航地图（省赛图墙位平移 0.5 m）。
- 修改 `ucar_ws/src/ucar_2026_national/launch/2026.launch` 与
  `ucar_ws/src/ucar_2026_extra/launch/2026.launch`：`map_file` 默认值指向
  `iflysse_field_walls_national.yaml`。
- `__pycache__` 与 `*.pyc` 未复制。

## 验证

- 两新包文件清单与源包一致（23 个文件 + 新增 config png + urdf 重命名）。
- `launch/2026.launch` XML 解析通过（三个包均验证）。
- `config/production_full_grid_all_numbered.json` JSON 有效，459 个编号点与源一致。
- 包名引用完整性：正则 `ucar_2026(?!_national)` / `ucar_2026(?!_extra)` 全量扫描
  两新包全部文本文件，零残留；`$(find ...)` 引用与重命名后 urdf 文件名一致。
- 编号图像素验证：旧墙位（x=0，px 782~794）暗像素清零，新墙位（x=-0.5，px 633~645）
  按原墙形状绘制 1512 像素；observer 复核标签完整、观感自然。
- 导航地图像素验证：`iflysse_field_walls_national.pgm` 与省赛图逐像素 diff = 0
  （除移动的墙段矩形外）；新墙位（col 198-201 × row 100..147）为占用像素、
  旧墙位（col 248-251）已清除。
- 两个国赛包 `2026.launch` XML 解析通过，`map_file` 指向的 yaml/pgm 存在。
- 测试文件 `test/test_production_task_geometry.py` 对网格 JSON 的相对路径引用复制后仍有效。

## 实车反馈（2026-08-16 晚上）

- 用户在小车运行 `start_2026.sh`（mission 模式）后反馈：地图仍是省赛版，
  车在国赛场地"实际没有墙的地方绕了一圈"。原因：此前分析 pgm 时把图像 y 轴
  方向搞反（图像顶部应为 y=+3.0，误按 y=-3.0 处理），错误得出"该区域无墙"
  的结论，导致国赛版未同步生成新导航地图。
- 修正：生成国赛版导航地图并让两个国赛包的 `map_file` 指向它。
  小车部署后 `map_server`/`move_base`/`navigation_scan_relay` 均按国赛墙位工作。

## 已知限制

- 两新包需在小车 Ubuntu 18.04 上随 `catkin_make` 编译后才会出现在 `$(find ...)` 解析中；
  编译命令与 `ucar_2026` 相同（包名换成 `ucar_2026_national` / `ucar_2026_extra`）。
- 国赛版导航地图 `iflysse_field_walls_national` 需随 `ucar_nav` 一起部署到小车
  `~/ucar_ws/src/ucar_nav/maps/`（scp 或 git pull）；部署后需重启主流程使 `map_server`
  加载新图。
- 若现场物理墙位与编号图新布局（147-158 有墙、148-159 无墙）不符，需重新建图，
  勿直接沿用本图。
- `ucar_2026_extra` 地图暂与国赛版一致；若随机任务使用不同场地布局，后续单独调整。
- 根目录 `production_full_grid_all_numbered.png/json` 仍为省赛版资源，未被修改。
