# 2026-11-05 测试视频

归档当日从机载设备导出的 22 段原始录像：18 段任务录像、4 段相机姿态采样。时间均为上海时间（Asia/Shanghai），覆盖 14:29:06—17:06:52；视频合计 565,040,499 字节（约 565 MB），累计约 5 分 40 秒。

每段保留 `liveview.mkv`、逐帧时间记录 `frames.csv` 和会话信息 `session.json`，共 66 个原始文件。全部文件的 SHA-256 与设备源文件一致；22 段视频共 3548 帧，已核对视频解码帧数、会话帧数及 CSV 行数一致。

日期由录像开始/停止日志与会话标识交叉确认。设备当前开机时钟未校准；历史 5 段时钟异常采样已由 2026-10-30 记录确认，另有 5 段 2026-10-17 录像，均未纳入本目录。

此目录记录测试数据；测试结论仍需结合任务记录分析。

`manifest.json` 包含视频索引、起止时间、大小和帧数；`SHA256SUMS` 包含全部原始文件的校验值。可在本目录运行 `shasum -a 256 -c SHA256SUMS` 复核。

| 开始时间 | 类型 | 时长（秒） | 大小（MB） | 视频 |
| --- | --- | ---: | ---: | --- |
| 14:29:06 | 相机姿态采样 | 26.912 | 40.32 | [01 · run_472874_579_53f84ca0](prediction_recordings/run_472874_579_53f84ca0/liveview.mkv) |
| 14:29:52 | 任务录像 | 0.597 | 0.92 | [02 · run_519405_579_bb5fd0c3](prediction_recordings/run_519405_579_bb5fd0c3/liveview.mkv) |
| 14:30:00 | 任务录像 | 0.626 | 1.24 | [03 · run_526917_579_47cf23c3](prediction_recordings/run_526917_579_47cf23c3/liveview.mkv) |
| 14:30:53 | 任务录像 | 0.492 | 0.99 | [04 · run_579469_579_df7a10a2](prediction_recordings/run_579469_579_df7a10a2/liveview.mkv) |
| 14:31:24 | 任务录像 | 0.495 | 0.97 | [05 · run_610995_579_241e54ab](prediction_recordings/run_610995_579_241e54ab/liveview.mkv) |
| 14:32:25 | 任务录像 | 0.494 | 0.77 | [06 · run_672557_579_52194907](prediction_recordings/run_672557_579_52194907/liveview.mkv) |
| 14:32:52 | 相机姿态采样 | 27.121 | 40.02 | [07 · run_699581_579_075908d4](prediction_recordings/run_699581_579_075908d4/liveview.mkv) |
| 14:33:42 | 任务录像 | 0.591 | 0.94 | [08 · run_749133_579_156533d2](prediction_recordings/run_749133_579_156533d2/liveview.mkv) |
| 15:07:51 | 相机姿态采样 | 27.318 | 44.72 | [09 · run_1461475_605_6846af0a](prediction_recordings/run_1461475_605_6846af0a/liveview.mkv) |
| 15:08:30 | 任务录像 | 27.118 | 38.77 | [10 · run_1500504_605_f467fc5d](prediction_recordings/run_1500504_605_f467fc5d/liveview.mkv) |
| 15:09:05 | 任务录像 | 5.900 | 8.69 | [11 · run_1535008_605_005293aa](prediction_recordings/run_1535008_605_005293aa/liveview.mkv) |
| 15:09:44 | 任务录像 | 5.300 | 7.36 | [12 · run_1574076_605_095dabbc](prediction_recordings/run_1574076_605_095dabbc/liveview.mkv) |
| 15:12:29 | 任务录像 | 17.636 | 31.03 | [13 · run_84683_590_b50acc4e](prediction_recordings/run_84683_590_b50acc4e/liveview.mkv) |
| 15:25:34 | 任务录像 | 21.978 | 34.20 | [14 · run_869940_590_f8128033](prediction_recordings/run_869940_590_f8128033/liveview.mkv) |
| 15:26:31 | 任务录像 | 22.777 | 45.00 | [15 · run_926999_590_b163fdd6](prediction_recordings/run_926999_590_b163fdd6/liveview.mkv) |
| 15:27:01 | 任务录像 | 40.898 | 90.96 | [16 · run_957025_590_ea7fea6e](prediction_recordings/run_957025_590_ea7fea6e/liveview.mkv) |
| 15:28:10 | 任务录像 | 26.220 | 53.78 | [17 · run_1026091_590_01ce098e](prediction_recordings/run_1026091_590_01ce098e/liveview.mkv) |
| 16:03:15 | 任务录像 | 36.685 | 62.93 | [18 · run_285815_567_253b6f36](prediction_recordings/run_285815_567_253b6f36/liveview.mkv) |
| 16:04:33 | 任务录像 | 14.340 | 21.55 | [19 · run_363910_567_0a8e310a](prediction_recordings/run_363910_567_0a8e310a/liveview.mkv) |
| 16:40:32 | 相机姿态采样 | 22.917 | 24.05 | [20 · run_2523146_567_c8ab9844](prediction_recordings/run_2523146_567_c8ab9844/liveview.mkv) |
| 17:04:06 | 任务录像 | 5.098 | 6.83 | [21 · run_3937364_567_435b935d](prediction_recordings/run_3937364_567_435b935d/liveview.mkv) |
| 17:06:43 | 任务录像 | 8.764 | 9.01 | [22 · run_4094247_567_e51639f8](prediction_recordings/run_4094247_567_e51639f8/liveview.mkv) |
