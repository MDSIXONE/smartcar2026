# 生产地图墙体拐角与端点像素修正（2026-08-18）

## 目的

修正生产地图中正交墙拐角留下的缺角像素，并统一三个开放墙端的端点长度：139、148、152 各沿墙体方向延长半个墙宽。省赛地图仍以 148-159 墙位为准；国赛和额外任务先按省赛几何修正，再把这段墙从 148-159 平移到 147-158。

## 涉及文件

- `ucar_ws/src/ucar_nav/maps/iflysse_field_walls_without_middle_vertices.pgm`：省赛运行地图。
- `ucar_ws/src/ucar_nav/maps/iflysse_field_walls_national.pgm`：国赛运行地图。
- `production_full_grid_all_numbered.png`：省赛编号预览图。
- `ucar_ws/src/ucar_2026/config/production_full_grid_all_numbered.png`：省赛包内编号预览图，新增并与根目录预览图一致。
- `ucar_ws/src/ucar_2026_national/config/production_full_grid_all_numbered.png`、`ucar_ws/src/ucar_2026_extra/config/production_full_grid_all_numbered.png`：国赛墙位版编号预览图。
- `tools/fix_production_map_pixels.py`：像素修正与两套地图同步工具。
- `docs/operations.md`：本地重新生成与核对命令。

## 几何修正

- 补齐 136、138、140、141、149、151 等正交拐角的 2×2 像素缺角。
- 139 的上部竖墙、152 的右侧竖墙下端各补 2 行；省赛 148 的竖墙上端补 2 行。
- 国赛将省赛 148-159 墙段平移到 147-158，因此 148 的上端补点在国赛图中对应平移后的 147 墙端；国赛 152 与 139 的修正保持原编号位置。

## 验证结果

- `python tools/fix_production_map_pixels.py` 成功执行两次，第二次结果幂等。
- 两份 PGM 各新增 48 个占用像素，均为白色背景 `254` 改为黑色占用 `0`，无像素被清除。
- 139 的最终补点为第 50-51 行；第 52-53 行保持背景，确认端点与原墙连续。
- 国赛 PGM 仍满足 147-158 墙段存在、148-159 原墙段空置；省赛 PGM 保持 148-159 墙段。
- 省赛根目录 PNG 与 `ucar_2026` 包内 PNG 的 RGB 像素一致；国赛与额外任务 PNG 一致。
- 未在本机编译或启动 ROS；未发送运动命令。
- 已通过当前局域网动态发现并确认车端为 `ucar-mini`（Ubuntu 18.04.6），将两份运行时 PGM 上传到 `~/ucar_ws/src/ucar_nav/maps/`。
- 车端与本地 SHA-256 一致：省赛 `8cc09f4d5384a8e0ca622dd531ef86fd10032a487ef6fd6d8c689438370d2e7b`，国赛 `0e5b9c27143208b7a0b6893a545947e8e20db386b6e81bb73cd6a3639d6c27d7`；未创建车端备份文件。

## 已知限制

- 本次只替换了车端运行时 PGM，未启动或重启 ROS/导航主流程；下次启动对应主流程时会加载新地图，若已有 `map_server` 常驻则需先重启它。
- 历史未被当前 2026 主流程引用的 `iflysse_2026_direct.pgm` 未修改。
