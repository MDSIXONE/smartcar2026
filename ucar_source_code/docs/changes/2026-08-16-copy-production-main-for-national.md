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
- 真车导航 pgm（`iflysse_field_walls_without_middle_vertices`）该区域本无中间墙
  （简化版地图，导航靠激光实时避障），因此**流程代码零改动**，静态地图无需同步修改。

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
- `__pycache__` 与 `*.pyc` 未复制。

## 验证

- 两新包文件清单与源包一致（23 个文件 + 新增 config png + urdf 重命名）。
- `launch/2026.launch` XML 解析通过（三个包均验证）。
- `config/production_full_grid_all_numbered.json` JSON 有效，459 个编号点与源一致。
- 包名引用完整性：正则 `ucar_2026(?!_national)` / `ucar_2026(?!_extra)` 全量扫描
  两新包全部文本文件，零残留；`$(find ...)` 引用与重命名后 urdf 文件名一致。
- 编号图像素验证：旧墙位（x=0，px 782~794）暗像素清零，新墙位（x=-0.5，px 633~645）
  按原墙形状绘制 1512 像素；observer 复核标签完整、观感自然。
- 测试文件 `test/test_production_task_geometry.py` 对网格 JSON 的相对路径引用复制后仍有效。

## 已知限制

- 两新包需在小车 Ubuntu 18.04 上随 `catkin_make` 编译后才会出现在 `$(find ...)` 解析中；
  编译命令与 `ucar_2026` 相同（包名换成 `ucar_2026_national` / `ucar_2026_extra`）。
- 国赛场地若与编号图新布局不符（物理墙位），需现场以激光/建图校准，勿直接沿用本
  简化版 pgm 的假设。
- `ucar_2026_extra` 地图暂与国赛版一致；若随机任务使用不同场地布局，后续单独调整。
- 根目录 `production_full_grid_all_numbered.png/json` 仍为省赛版资源，未被修改。
