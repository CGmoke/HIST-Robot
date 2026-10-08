#!/usr/bin/env bash
# greeting_autostart.sh —— 迎宾动作栈开机自启动入口
#
# 启动内容（greeting_teleop/greeting_bringup.launch.py）：
#   1) greeting_body     motion_server          —— /greeting/play_motion 动作服务端（实机 臂/头/腰）
#   2) greeting_teleop   joy_mapper             —— /sbus_data/event 按键 -> /greeting/panel_command
#   3) greeting_teleop   panel_command_bridge   —— /greeting/panel_command -> /greeting/play_motion
#   4) greeting_teleop   flow_command_bridge    —— /greeting/panel_command -> /greeting/control_cmd
#   5) greeting_voice    speak_action_server    —— /greeting/speak 动作服务端（编排层语音播报）
#   6) greeting_orchestrator orchestrator_node  —— 讲稿状态机（动作 + 语音同步）
#
#   说明：只起 speak_action_server，不起 voice_greet_tts_node（后者同样监听 G+A/B/C/D，
#         会与 joy_mapper 的 goto:<段号> 撞车）；用 with_voice:=false 可关闭语音服务端。
#
# 手柄操作（默认映射见 src/greeting_teleop/config/teleop_map.yaml）：
#   先把 E 三档开关拨到「上」激活组合键，再按 A / B / C 触发 鞠躬 / 挥手 / 拍照；
#   E 离开「上」会下发 release=manual，中止正在执行的动作。
#   · 若希望「开机后直接按键触发、无需先拨 E」：把 teleop_map.yaml 的 guard 段删掉，
#     并把动作写进常驻按键表 keys:（例如 a: motion:salute_bow）。
#   · 平台层 A 键在 robot_status != Running 时用于启动自检；
#     self-check 完成后 robot_control 才使能，/arm/cmd 才有订阅者，动作才可能被执行。
#
# 环境变量（均可不设，使用默认值）：
#   GREETING_WS                 工作空间根，默认 /home/nvidia/greeting-body-motion/greeting_ws
#   GREETING_SPEED_LIMIT        动作速度上限，默认 0.7（硬约束 ≤0.7）
#   GREETING_ARM_CURRENT        手臂电流上限(A)，默认 5.0
#   GREETING_REQUIRED_GROUPS    硬预检通道分组，默认 ['arm']
#   GREETING_BRIDGE_SPEED_SCALE 手柄触发动作速度缩放，默认 0.5
#   GREETING_LOG_DIR            日志/锁文件目录，默认 <GREETING_WS>/log
#   GREETING_EXTRA_ARGS         附加 launch 参数（原样拼到命令行末尾）
#
# 手动调试运行：
#   bash /home/nvidia/greeting-body-motion/greeting_ws/scripts/greeting_autostart.sh
set -o errexit
set -o nounset
set -o pipefail

WS="${GREETING_WS:-/home/nvidia/greeting-body-motion/greeting_ws}"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
XOS_SETUP="/home/nvidia/xos/setup.bash"
WS_SETUP="${WS}/install/setup.bash"
LOG_DIR="${GREETING_LOG_DIR:-${WS}/log}"

SPEED_LIMIT="${GREETING_SPEED_LIMIT:-0.7}"
ARM_CURRENT="${GREETING_ARM_CURRENT:-5.0}"
REQUIRED_GROUPS="${GREETING_REQUIRED_GROUPS:-['arm']}"
BRIDGE_SPEED_SCALE="${GREETING_BRIDGE_SPEED_SCALE:-0.5}"
EXTRA_ARGS="${GREETING_EXTRA_ARGS:-}"

log() { printf '[greeting-autostart] %s\n' "$*" >&2; }

# ROS/平台的 setup.bash 会引用未定义变量，source 期间必须临时关闭 nounset
source_env() {
    set +o nounset
    # shellcheck disable=SC1090
    source "$1"
    set -o nounset
}

# ---------------------------------------------------------------- 单实例保护
# 双实例会导致 /arm|/head|/waist/cmd 出现重复发布者、电机互相打架，必须拦住。
mkdir -p "$LOG_DIR" 2>/dev/null || true
LOCK_FILE="${LOG_DIR}/greeting_autostart.lock"
if : >>"$LOCK_FILE" 2>/dev/null; then
    exec 9>>"$LOCK_FILE"
    if ! flock -n 9; then
        log "已有实例在运行（锁 ${LOCK_FILE}），本次启动放弃"
        exit 1
    fi
else
    log "警告：无法创建锁文件 ${LOCK_FILE}（跨用户权限），跳过单实例保护"
fi

# ---------------------------------------------------------------- 环境加载
if [ ! -f "${WS_SETUP}" ]; then
    log "错误：未找到 ${WS_SETUP}"
    log "请先编译：source ${XOS_SETUP} && cd ${WS} && colcon build --symlink-install"
    exit 1
fi

# shellcheck disable=SC1090
source_env "${ROS_SETUP}"
log "已 source ${ROS_SETUP}"

if [ -f "${XOS_SETUP}" ]; then
    source_env "${XOS_SETUP}"
    log "已 source ${XOS_SETUP}（平台消息包 bodyctrl_msgs / ros2_bridge_msgs）"
else
    log "警告：未找到 ${XOS_SETUP}，实机平台消息包可能缺失"
fi

source_env "${WS_SETUP}"
log "已 source ${WS_SETUP}"

# journald 下关掉 ANSI 颜色，日志更干净
export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-0}"
# ROS_DOMAIN_ID / RMW_IMPLEMENTATION 不在此处设置：与本机平台进程保持一致
# （均未设置 -> 默认域 0、默认 rmw_fastrtps_cpp）。

log "启动迎宾动作栈：speed_limit=${SPEED_LIMIT} arm_current=${ARM_CURRENT}"
log "required_groups=${REQUIRED_GROUPS} bridge_speed_scale=${BRIDGE_SPEED_SCALE}"

# ---------------------------------------------------------------- 启动
# shellcheck disable=SC2086
exec ros2 launch greeting_teleop greeting_bringup.launch.py \
    "speed_limit:=${SPEED_LIMIT}" \
    "arm_current:=${ARM_CURRENT}" \
    "required_groups:=${REQUIRED_GROUPS}" \
    "bridge_speed_scale:=${BRIDGE_SPEED_SCALE}" \
    ${EXTRA_ARGS}