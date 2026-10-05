# tests 测试包说明

本文档描述 `reusable_model/tests/` 测试包的职责、依赖关系、用例构成与运行方法。测试包位于 `reusable_model` 包的外层目录，通过绝对导入 `reusable_model.xxx` 引用被测模块，当前共包含 **128** 个测试用例，全部通过。

## 1. 测试总览

| 测试文件 | 对应被测模块 | 主要测试点（覆盖的功能） |
| --- | --- | --- |
| `conftest.py` | —（测试基础设施） | 将仓库根目录写入 `sys.path`，使 `import reusable_model...` 可解析；当前不定义 fixture |
| `test_boxes.py` | `reusable_model.geometry.boxes` | 边界框面积/中心、IoU/最小 IoU/重叠分数、裁剪、bbox↔mask 转换、参数校验 |
| `test_detection_npz.py` | `reusable_model.io.detection_npz` | 检测结果打包/解包的往返一致性、空数据、缺失 mask 回填、自定义字符串字段、异常载荷 |
| `test_gridmap.py` | `reusable_model.gridmap.occupancy`、`reusable_model.gridmap.regions`、`reusable_model.gridmap.distance_field`、`reusable_model.gridmap.descent` | 占据栅格构造、连通域标注、距离场创建、下降方向计算 |
| `test_hardware.py` | `reusable_model.hardware.serial_discovery`、`reusable_model.hardware.modbus_gripper` | 按 ID 枚举串口/端口探测、Modbus 夹爪空运行（dry-run）与参数校验 |
| `test_io.py` | `reusable_model.io.yaml_config`、`reusable_model.io.binary_envelope` | YAML 读写往返、缺失/非法文件处理、二进制信封打包解包 |
| `test_motion.py` | `reusable_model.motion.planar_ik` | 平面两连杆逆运动学求解、可达/不可达、关节限位、肘部圆几何 |
| `test_rotations3d.py` | `reusable_model.geometry.rotations3d` | 欧拉角↔旋转矩阵往返、旋转矩阵判定/断言、轴角旋转、两向量间旋转、角度限幅 |
| `test_runtime.py` | `reusable_model.runtime.subprocess_bridge`、`reusable_model.runtime.ctypes_loader` | 子进程桥接收发与关闭、动态库查找、就绪轮询、原生调用异常 |
| `test_speed_profile.py` | `reusable_model.motion.speed_profile` | 加减速计算、目标速度斜坡、制动速度、运动策略选择、参数校验 |
| `test_stdout_filter.py` | `reusable_model.runtime.stdout_filter` | 过滤流按子串丢弃行、空行处理、部分行刷新、安装过滤器、静默 logger |
| `test_tracking.py` | `reusable_model.tracking.iou_tracker`、`reusable_model.tracking.stable_streak` | IoU 跟踪器命中/失配/过期/重置、键控跟踪存储、主目标选择、稳定帧计数 |
| `test_vision.py` | `reusable_model.vision.pinhole`、`reusable_model.vision.depth`、`reusable_model.vision.planes`、`reusable_model.vision.pointcloud` | 针孔投影与反投影、深度采样、平面拟合与射线求交、点云降采样/高度过滤 |

## 2. 依赖关系与调用层级

### 2.1 第三方依赖

- `pytest`：测试运行器、`pytest.raises`、`pytest.approx`、`@pytest.fixture`、`monkeypatch`、内置 `tmp_path`。
- `numpy`：数值断言与构造测试数据。
- `math`：`test_speed_profile.py` 中的开方计算。
- `sys` / `pathlib` / `io.StringIO` / `logging`：标准库，用于路径、流与日志测试。

### 2.2 测试文件对 `reusable_model` 子包的导入依赖（逐一列出）

- `test_boxes.py` ← `reusable_model.geometry.boxes`：`bbox_area`、`bbox_center`、`bbox_to_mask`、`clip_bbox`、`iou`、`iou_min`、`mask_center`、`overlap_score`
- `test_detection_npz.py` ← `reusable_model.io.detection_npz`：`pack_detections`、`unpack_detections`
- `test_gridmap.py` ← `reusable_model.gridmap.descent`：`descent_direction`；← `reusable_model.gridmap.distance_field`：`DistanceField`、`create_distance_field_for_region`；← `reusable_model.gridmap.occupancy`：`FREE`、`OCCUPIED`、`GridGeometry`、`OccupancyGrid`；← `reusable_model.gridmap.regions`：`RegionInfo`、`label_connected`
- `test_hardware.py` ← `reusable_model.hardware.modbus_gripper`：`ModbusGripper`；← `reusable_model.hardware.serial_discovery`：`list_by_id_ports`、`probe_port`
- `test_io.py` ← `reusable_model.io.binary_envelope`：`pack`、`unpack`；← `reusable_model.io.yaml_config`：`dump_yaml`、`load_yaml`
- `test_motion.py` ← `reusable_model.motion.planar_ik`：`PlanarTwoLink`、`PlanarTwoLinkIK`、`elbow_circle`
- `test_rotations3d.py` ← `reusable_model.geometry.rotations3d`：`assert_rotation_matrix`、`euler_to_matrix`、`is_rotation_matrix`、`matrix_to_euler`、`rotation_between`、`rotation_from_axis`、`wrap_to_limits`
- `test_runtime.py` ← `reusable_model.runtime.ctypes_loader`：`CFunction`、`NativeCallError`、`find_library_file`、`poll_until_ready`；← `reusable_model.runtime.subprocess_bridge`：`SubprocessBridge`
- `test_speed_profile.py` ← `reusable_model.motion.speed_profile`：`BrakingProfile`
- `test_stdout_filter.py` ← `reusable_model.runtime.stdout_filter`：`FilteredStream`、`install_filter`、`quiet_loggers`
- `test_tracking.py` ← `reusable_model.tracking.iou_tracker`：`IoUTracker`、`KeyedTrackerStore`、`pick_primary_track_id`；← `reusable_model.tracking.stable_streak`：`StableStreak`
- `test_vision.py` ← `reusable_model.vision.depth`：`sample_depth`；← `reusable_model.vision.pinhole`：`Intrinsics`、`pixel_to_3d`、`project`、`ray_direction`；← `reusable_model.vision.planes`：`fit_plane`、`fit_plane_oriented`、`ray_plane_intersection`；← `reusable_model.vision.pointcloud`：`backproject_depth`、`filter_by_height`、`voxel_downsample`

### 2.3 conftest.py 提供的公共设施

`conftest.py` 当前**不定义 fixture**，它只承担导入路径设置：计算仓库根目录（`tests/` 的上级的上级）并插入 `sys.path`，从而让所有测试文件都能 `import reusable_model...`。因此它作用于**全部**测试文件。

实际测试中使用的 fixture 均为文件内局部定义或 pytest 内置：

- `test_speed_profile.py` 的 `p`：返回一个默认 `BrakingProfile()` 实例。
- `test_vision.py` 的 `intr`：返回一个内参为 `fx=fy=500.0, cx=320.0, cy=240.0, width=640, height=480` 的 `Intrinsics`。
- pytest 内置 `tmp_path`（`test_hardware.py`、`test_io.py`、`test_runtime.py`）与 `monkeypatch`（`test_stdout_filter.py`）。

### 2.4 测试与实现代码的层级关系

```
Garden/
└── reusable_model/                 # 被测包（实现代码）
    ├── geometry/  io/  gridmap/  hardware/  motion/  runtime/  tracking/  vision/
    └── tests/           # 本测试包（位于包外层子目录）
        ├── conftest.py
        └── test_*.py
```

`tests` 是 `reusable_model` 包内的子目录，测试文件统一使用 `reusable_model.xxx.yyy` 绝对导入被测模块；`conftest.py` 负责保证仓库根目录在 `sys.path` 上。

## 3. 各测试模块职责与用例定义

### 3.1 test_boxes.py

- **测试目标**：`reusable_model.geometry.boxes` 中的度量、转换与校验函数。
- **关键用例**：
  - `TestMetrics`
    - `test_bbox_area`：面积计算，并验证反向框被裁剪为 0。
    - `test_iou_exact_and_disjoint`：完全重合 IoU=1、完全分离 IoU=0。
    - `test_iou_known_ratio`：已知重叠比（交集 25、并集 175）。
    - `test_iou_min_nested_box_is_one`：嵌套框的最小 IoU 为 1。
    - `test_overlap_score_is_max`：重叠分数取 IoU 与最小 IoU 的较大者。
    - `test_extra_fields_ignored`：边界框多余字段被忽略。
  - `TestConversion`
    - `test_clip_bbox`：越界框裁剪至图像范围。
    - `test_bbox_to_mask`：bbox 转布尔掩码的形状/类型/取值。
    - `test_bbox_to_mask_out_of_image_is_empty`：完全在图像外的框生成空掩码。
    - `test_bbox_center`：边界框中心计算。
    - `test_mask_center`：掩码质心；全零掩码返回 `None`。
  - `TestValidation`
    - `test_bbox_requires_four_elements`：元素数不足抛 `ValueError`。
    - `test_bbox_rejects_non_numeric`：非数值元素抛 `TypeError`。
    - `test_clip_rejects_bad_resolution`：非法分辨率（负值/非整型）抛异常。
- **边界与异常路径**：反向框、图像外框、空掩码、元素不足、非数值、负/非整分辨率。

### 3.2 test_detection_npz.py

- **测试目标**：`reusable_model.io.detection_npz` 的 `pack_detections` / `unpack_detections`。
- **关键用例**：
  - `TestRoundTrip`
    - `test_full_round_trip`：多条检测结果打包再解包后字段一致（含 `name_zh`）。
    - `test_empty_list_round_trip`：空列表仍产出有效载荷，解包为空列表。
    - `test_empty_bytes`：空字节流解包为空列表。
    - `test_missing_mask_filled_from_bbox`：缺失 mask 时由 bbox 回填。
    - `test_custom_string_fields`：自定义字符串字段经 `str_fields` 保留。
  - `TestValidation`
    - `test_mask_must_be_2d`：非二维 mask 抛 `ValueError`。
    - `test_confidence_required`：缺少 confidence 抛 `ValueError`。
    - `test_corrupt_payload_rejected`：损坏载荷抛 `ValueError`。
- **边界与异常路径**：空输入、空字节、缺字段、非二维掩码、损坏载荷。

### 3.3 test_gridmap.py

- **测试目标**：占据栅格、连通域、距离场、下降方向。
- **关键用例**：
  - `TestOccupancy`
    - `test_from_array_geometry`：从数组构造栅格，校验尺寸/类型/边界。
    - `test_shape_mismatch_rejected`：数据与几何尺寸不符抛 `ValueError`。
    - `test_grid_geometry_validation`：零分辨率抛 `ValueError`。
  - `TestRegions`
    - `test_label_connected`：连通域标注标签与区域信息。
    - `test_min_area_filters`：小于 `min_area_cells` 的区域被过滤。
  - `TestDistanceField`
    - `test_create_for_region`：为指定区域创建距离场对象。
    - `test_own_resolution_decoupled_from_map`：距离场分辨率独立于地图。
  - `TestDescent`
    - `test_descent_heading_toward_lower_field`：朝向指向距离场下降方向。
    - `test_descent_none_in_invalid_region`：无效区域返回 `None`。
- **边界与异常路径**：尺寸不匹配、零分辨率、面积过小区域、全无穷距离场。

### 3.4 test_hardware.py

- **测试目标**：串口发现与 Modbus 夹爪（空运行）。
- **关键用例**：
  - `TestSerialDiscovery`
    - `test_list_by_id_ports`：按 `*` 通配符枚举所有 by-id 端口。
    - `test_list_by_id_ports_pattern`：按 `*FTDI*` 模式过滤端口。
    - `test_probe_port`：探测回调返回真/假/抛异常三种情况的处理。
  - `TestModbusGripper`
    - `test_dry_run_never_touches_serial`：空运行不打开端口，且每个动作被记录。
    - `test_port_required`：端口类型错误抛 `TypeError`、空白端口抛 `ValueError`。
- **边界与异常路径**：空目录、探测异常、非法端口类型/空白端口。

### 3.5 test_io.py

- **测试目标**：`reusable_model.io.yaml_config` 与 `reusable_model.io.binary_envelope`。
- **关键用例**：
  - `TestYamlConfig`
    - `test_dump_load_round_trip`：YAML 写出后读回与原数据一致。
    - `test_load_missing_with_default`：文件缺失且 `allow_missing=True` 时返回默认值。
    - `test_load_missing_rejected`：文件缺失且未允许时抛 `FileNotFoundError`。
    - `test_load_invalid_yaml_raises`：非法 YAML 抛异常。
  - `TestBinaryEnvelope`
    - `test_round_trip_raw`：raw 后端下头部与附件往返一致。
    - `test_no_attachments`：无附件时返回空附件列表。
    - `test_round_trip_empty_header`：空头部往返。
    - `test_corrupt_payload_rejected`：损坏载荷抛异常。
- **边界与异常路径**：缺失文件、非法 YAML、空头部、无附件、损坏载荷。

### 3.6 test_motion.py

- **测试目标**：`reusable_model.motion.planar_ik` 的两连杆逆运动学与肘部圆。
- **关键用例**：
  - `TestPlanarTwoLinkIK`
    - `test_reachable_target`：完全伸直可达目标时解出 q1=q2=0。
    - `test_out_of_reach_fails`：超出工作空间求解失败。
    - `test_joint_limits_respected`：给定关节限位时结果不越界。
    - `test_result_has_expected_fields`：结果对象包含 `success`/`q1`/`q2`。
  - `TestElbowCircle`
    - `test_circle_geometry`：肘部圆心、半径与正交基向量正确。
    - `test_out_of_reach_returns_none`：不可达时返回 `None`。
- **边界与异常路径**：不可达目标、关节限位边界。

### 3.7 test_rotations3d.py

- **测试目标**：`reusable_model.geometry.rotations3d` 的 3D 旋转工具。
- **关键用例**：
  - `TestRotationMatrices`
    - `test_euler_round_trip`：ZYX 欧拉角↔矩阵往返一致。
    - `test_euler_round_trip_proper_euler`：XYX 真欧拉序列往返一致。
    - `test_is_rotation_matrix`：正交矩阵判定（含单位阵与非旋转矩阵）。
    - `test_assert_rotation_matrix_passes_through`：断言函数对合法矩阵返回非空。
    - `test_rotation_between_known`：+x 到 +y 的旋转映射正确。
    - `test_rotation_between_same_vector_is_identity`：同向量间旋转为单位阵。
    - `test_rotation_between_opposite_vectors`：对径向量旋转 180 度。
    - `test_rotation_from_axis`：机体轴旋转映射与正交性。
  - `TestWrapToLimits`
    - `test_wraps_outside_limits`：超限值被回绕到限位区间内。
    - `test_no_change_when_inside`：区间内数值保持不变。
- **边界与异常路径**：同向/对径向量、越界回绕、非旋转矩阵。

### 3.8 test_runtime.py

- **测试目标**：子进程桥接与 ctypes 动态库加载。
- **关键用例**：
  - `TestSubprocessBridge`
    - `test_send_lines_and_clean_close`：可发送命令行/数据，上下文退出后子进程干净结束。
    - `test_sentinel_closes_child`：哨兵信号关闭子进程。
    - `test_bad_argv_rejected`：空参数抛 `ValueError`、裸字符串抛 `TypeError`。
  - `TestCtypesLoader`
    - `test_find_library_file`：按名查找 `.so` 文件，未命中返回 `None`。
    - `test_poll_until_ready`：轮询在满足条件时返回真。
    - `test_poll_timeout`：超时返回假。
    - `test_native_call_error`：原生调用异常的 code 与消息。
    - `test_cfunction_requires_func`：缺少函数对象抛 `TypeError`。
- **边界与异常路径**：非法 argv、超时、库缺失、空函数对象。

### 3.9 test_speed_profile.py

- **测试目标**：`reusable_model.motion.speed_profile.BrakingProfile` 的速度/加速度曲线。
- **关键用例**：
  - `TestAcceleration`
    - `test_forward_acceleration` / `test_backward_acceleration`：前向/后向加速度符号与幅值。
    - `test_angular_uses_angular_acceleration`：角模式使用角加速度。
    - `test_step_clamp_on_long_interval`：长间隔下单步限制生效。
    - `test_rejects_bad_interval`：零间隔抛 `ValueError`。
  - `TestTargetVelocity`
    - `test_ramp_never_overshoots`：斜坡不越过目标速度。
    - `test_decel_floor`：减速下限计算。
  - `TestBrakingVelocity`
    - `test_known_value`：制动速度等于 `sqrt(2·a·d)`。
    - `test_sign_matches_distance`：符号随距离符号变化。
    - `test_rejects_zero_distance`：零距离抛 `ValueError`。
  - `TestMotionStrategy`
    - `test_result_within_bounds`、`test_angular_mode_runs`：结果落在合理区间。
    - `test_target_velocity_mode`：目标速度模式的斜坡取值。
    - `test_stop_when_no_motion`：临近目标时减速/停止。
    - `test_max_velocity_clamp`：结果被限幅在最大速度。
    - `test_requires_a_target`：缺少目标时抛 `ValueError`。
  - `TestValidation`
    - `test_signed_accelerations`：加速度符号约束校验。
    - `test_damping_factors`：阻尼因子取值范围校验。
- **边界与异常路径**：零间隔、零距离、越界距离、缺失目标、非法构造参数。

### 3.10 test_stdout_filter.py

- **测试目标**：`reusable_model.runtime.stdout_filter` 的输出过滤与日志静默。
- **关键用例**：
  - `TestFilteredStream`
    - `test_drops_matching_lines`：包含子串的行被丢弃。
    - `test_drops_blank_line_after_dropped` / `test_drop_blank_disabled`：丢弃后空行开关行为。
    - `test_partial_line_flushed`：未以换行结束的内容在 `flush()` 后输出。
    - `test_non_string_coerced`：非字符串被强制转为字符串。
    - `test_fileno_forwards`：`fileno()` 转发到底层流。
    - `test_rejects_none_drop_list`：丢弃列表为 `None` 抛 `TypeError`。
  - `TestInstallFilter`
    - `test_installs_on_requested_streams`：按需安装过滤器且重复调用为空操作。
  - `TestQuietLoggers`
    - `test_prefix_matching`：按前缀将 logger 级别调高。
    - `test_rejects_none`：`None` 参数抛 `TypeError`。
- **边界与异常路径**：部分行、空行、非字符串输入、重复安装、`None` 参数。

### 3.11 test_tracking.py

- **测试目标**：IoU 跟踪、键控跟踪存储、主目标选择、稳定帧计数。
- **关键用例**：
  - `TestIoUTracker`
    - `test_immediate_confirmation_at_min_hits_1`：`min_hits=1` 时立即确认。
    - `test_same_track_reassigned`：重叠检测沿用同一 track_id。
    - `test_new_track_when_disjoint`：不相交检测生成新轨迹。
    - `test_min_hits_gates_confirmation`：`min_hits=3` 时需连续命中才分配 ID。
    - `test_track_expires_after_max_age`：超过 `max_age` 后轨迹过期。
    - `test_reset`：重置后轨迹编号从 1 重新开始。
    - `test_active_track_ids`：活动轨迹集合正确。
    - `test_validation`：非法 `iou_threshold`/`max_age`/`min_hits` 抛 `ValueError`。
  - `TestKeyedTrackerStore`
    - `test_keys_are_isolated`：不同键的轨迹相互隔离。
    - `test_none_key_uses_default`：`None` 键使用默认桶。
  - `TestPickPrimary`
    - `test_largest_box_wins`：选择面积最大的目标。
    - `test_none_when_no_ids`：无 track_id 时返回 `None`。
  - `TestStableStreak`
    - `test_becomes_stable_after_threshold`：达到阈值后转为稳定。
    - `test_jump_resets_streak`：位置跳变重置计数。
    - `test_change_of_track_id_resets`：track_id 变化重置计数。
    - `test_missing_detection_zeroes_streak`：缺失检测时计数清零。
    - `test_bbox_required_with_track_id`：有 track_id 却无 bbox 抛 `ValueError`。
    - `test_validation`：非法 `min_stable_frames`/`stable_iou` 抛 `ValueError`。
- **边界与异常路径**：单帧/连续帧、空帧、ID 切换、跳变、无 bbox、非法参数。

### 3.12 test_vision.py

- **测试目标**：`reusable_model.vision` 的针孔模型、深度采样、平面拟合与点云处理。
- **关键用例**：
  - `TestPinhole`
    - `test_project_pixel_round_trip`：像素→3D→像素往返，深度以毫米单位返回。
    - `test_principal_axis_projects_to_center`：主光轴投影到图像中心。
    - `test_ray_direction_unit_length`：射线方向为单位向量且 z 为正。
    - `test_intrinsics_validation`：`fx=0` 抛 `ValueError`。
  - `TestDepth`
    - `test_sample_depth_median`：窗口内取中值深度。
    - `test_sample_depth_filters_zeros`：零值被过滤。
    - `test_sample_depth_none_when_invalid`：无有效深度返回 `None`。
  - `TestPlanes`
    - `test_fit_plane_horizontal`：水平面拟合的法向量平行于 z 轴。
    - `test_fit_plane_oriented_points_up`：定向拟合使法向朝上。
    - `test_ray_plane_intersection`：射线与平面求交。
    - `test_ray_plane_parallel_returns_none`：射线与平面平行返回 `None`。
  - `TestPointCloud`
    - `test_backproject_depth`：深度反投影生成点云并标记有效点。
    - `test_voxel_downsample`：体素降采样减少点数。
    - `test_filter_by_height`：按高度区间过滤点。
- **边界与异常路径**：全零深度、无效内参、平行射线、空/稀疏点云。

## 4. 运行与验证方法

在仓库中执行以下命令（本机解释器为 `python3`）：

- 全量测试：
  ```
  cd /home/moke/Coding/Garden/reusable_model && python3 -m pytest -q
  ```
- 单文件测试：
  ```
  python3 -m pytest tests/test_vision.py -q
  ```
- 详细输出：
  ```
  python3 -m pytest -v
  ```
- doctest（对 `reusable_model` 包执行模块级文档测试）：
  ```
  cd /home/moke/Coding/Garden && PYTHONPATH=/home/moke/Coding/Garden python3 -m pytest --doctest-modules reusable_model -q
  ```

逃避缓存写入的推荐方式（避免在受限环境下写 `__pycache__`）：
```
cd /home/moke/Coding/Garden/reusable_model && PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider
```

**预期结果**：当前共 **128** 个用例全部通过。

## 5. 新增测试指引

1. **命名与放置**：按被测模块名创建 `test_<模块>.py`，测试类命名 `TestXxx`，测试函数命名 `test_<行为>`。
2. **导入**：使用绝对导入 `from reusable_model.<子包>.<模块> import ...`，`conftest.py` 已保证导入路径可用，无需在文件内改动 `sys.path`。
3. **组织用例**：按功能分组到测试类中，一个函数聚焦一个行为；用 `# 准备 / # 执行 / # 断言` 等中文注释标注结构。
4. **常用断言**：浮点比较使用 `pytest.approx(...)` 或 `np.testing.assert_allclose(...)`；异常路径使用 `with pytest.raises(...)`。
5. **临时文件**：需要文件系统时使用 pytest 内置 `tmp_path` fixture；需要替换全局对象时使用 `monkeypatch`。
6. **注释语言**：本包注释与 docstring 统一使用简体中文；不要修改字符串字面量、`# type: ignore[...]` 等具有功能性含义的内容。
7. **验证**：新增后用 `python3 -m py_compile tests/*.py` 做语法检查，再运行全量测试确保保持全绿。
