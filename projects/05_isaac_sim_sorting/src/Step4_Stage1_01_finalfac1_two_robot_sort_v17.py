# Step4_Stage1_01_finalfac1_two_robot_sort_v12.py
# -*- coding: utf-8 -*-
"""
협동3 / Step4 finalfac1 box_assets QR면 고정+카메라줌+QR디코드 통합 v12

환경:
- finalfac1.usd

고정 로봇 매핑:
- robot1 = /World/FFW_BG2/ffw_bg2_follower/joints
- robot2 = /World/FFW_BG2_01/ffw_bg2_follower/joints

사용자 지정 시나리오:
1) 상자 QR에는 today/day2/day3 정보가 들어있다.
2) 새 상자는 이전 상자가 최종 목적지 영역에 도달한 뒤에만 스폰한다.
3) robot1은 QR 정보 그대로 today/not_today 1차 분류를 수행한다.
   - QR target=today    -> route=today
   - QR target=day2/3   -> route=not_today
4) today 상자는 robot1 왼팔 분류 후 최종목적지 영역에 도달하면 다음 상자를 스폰한다.
5) day2/day3 상자는 robot1 오른팔/not_today 동작이 끝난 뒤 2초 후 robot2가 사전에 읽은 target 정보로 2차 분류한다.
   - target=day2 -> robot2 route=today
   - target=day3 -> robot2 route=not_today
6) day2/day3 상자가 최종목적지 영역에 도달하면 다음 상자를 스폰한다.
7) 상자는 despawn하지 않고 stage 위에 계속 유지한다.

v08 수정:
- 다음 상자 스폰이 늦어지는 원인 수정.
- 기존 final gate 판정은 상자 bbox 중심점이 목적지 bbox 안에 들어와야 HIT였다.
- 사용자가 말한 "통과하거나 걸침" 기준에 맞게 bbox overlap 판정으로 변경했다.
- 상자 일부라도 최종분류칸 bbox와 겹치면 FINAL GATE HIT 처리한다.

v09 수정:
- robot1 작업대 상단 카메라 prim 생성.
- robot1 카메라가 상자 QR 정보를 읽는 구조로 변경한다.
- 분류 동작은 QR camera read 결과를 기준으로 결정한다.

v10 수정:
- 상자 윗면에 실제 QR 이미지가 보이도록 TopQR_Visual을 textured mesh plane으로 다시 생성한다.
- QR plane은 box prim의 child로 붙어서 상자와 같이 움직이도록 한다.

v11 통합:
- v10 + v11 + v12를 한 파일에 통합한다.
- robot1 작업대 상단 카메라 prim 생성.
- 카메라 RGB 캡처를 시도한다.
- OpenCV QRCodeDetector로 실제 QR decode를 시도한다.
- 실제 decode 성공 시 그 payload로 robot1/robot2 route를 결정한다.
- decode 실패 시 실험이 멈추지 않도록 user:qr_payload fallback을 사용한다.
- 분류 중에는 현재 상자 정보만 터미널에 집중 출력한다.
- 현재 출력 정보:
  전체 물품 중 몇 번째 물품인지, 출고일, target, QR read source, 최종분류 성공 여부.

v12 수정:
- box_assets USD에 이미 박힌 QR을 사용한다. TopQR_Visual 신규 부착은 기본 OFF로 변경했다.
- qr_face_terminal_tester 실험 결과 QR 부착면은 local -Y로 확인되어, 스폰 시 y_neg 면이 위로 오도록 고정한다.
- robot1 QR 카메라 기본값을 근접/줌인 설정으로 변경한다.
  기본 위치 z=2.20, focal_length=55, aperture=18, resolution=1024x1024.
- QR debug 이미지를 기본 저장하여 실제 카메라 디코드 실패 시 화면 확인이 쉽도록 한다.

주의:
- 컨베이어벨트 OFF/ON 제어 없음.
- 카메라 trigger 제어 없음.
- robot2는 QR을 다시 읽지 않고 robot1에서 이미 읽은 target 정보를 사용한다.
- v06에서 route의 팔 매핑을 사용자 시나리오에 맞게 교정했다.
  route=today     -> 왼팔
  route=not_today -> 오른팔
- 따라서 robot1 왼팔이 움직인 경우는 today 처리이며 robot2가 절대 움직이면 안 된다.
"""

from __future__ import annotations

import argparse
import math
import csv
from datetime import datetime, timedelta
import os
import re
import time
import zipfile
import random
import subprocess
import shlex
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple


# =============================================================================
# ARGPARSE
# =============================================================================

parser = argparse.ArgumentParser()

parser.add_argument("--usd", default="/home/rokey/dev_ws/isaac_sim/isaac_step4/finalfac1.usd", help="Path to finalfac1.usd")
parser.add_argument("--headless", action="store_true", help="Run Isaac without GUI")
parser.add_argument("--dry-run", action="store_true", help="Open USD, print conveyor/joint discovery, then exit")
parser.add_argument("--max-time", type=float, default=0.0, help="0 means run until user stops with Ctrl+C or closes Isaac.")

# Box / conveyor spawn
parser.add_argument("--spawn-track-path", default="/World/ConveyorTrack")
parser.add_argument("--spawn-track-index", type=int, default=0)
parser.add_argument("--box-count", type=int, default=6)
parser.add_argument("--sort-delay", type=float, default=3.0, help="Seconds to let conveyor carry box before robot1 sort starts")
parser.add_argument("--continuous-feed", action="store_true", default=True)
parser.add_argument("--no-continuous-feed", dest="continuous_feed", action="store_false")
parser.add_argument("--spawn-interval", type=float, default=0.2)
parser.add_argument("--max-boxes", type=int, default=0, help="0 means unlimited")
parser.add_argument("--keep-sorted-boxes", action="store_true", default=True)
parser.add_argument("--no-keep-sorted-boxes", dest="keep_sorted_boxes", action="store_false")
parser.add_argument("--camera-detection-log-every", type=int, default=120)
parser.add_argument("--overhead-camera-visual", action="store_true", default=True)
parser.add_argument("--no-overhead-camera-visual", dest="overhead_camera_visual", action="store_false")
parser.add_argument("--overhead-camera-z-offset", type=float, default=2.0)
parser.add_argument("--stage1-camera-sensor-index", type=int, default=1, help="Camera/sensor index used as Stage1 worktable detector.")
parser.add_argument("--stage2-camera-sensor-index", type=int, default=2, help="Camera/sensor index used as Stage2 worktable detector for day2/day3.")
parser.add_argument("--stage1-force-robot-slot", type=int, default=1, choices=[0, 1, 2], help="0 means use sensor map, 1/2 forces robot slot for Stage1.")
parser.add_argument("--stage2-force-robot-slot", type=int, default=2, choices=[0, 1, 2], help="0 means use sensor map, 1/2 forces robot slot for Stage2.")
parser.add_argument("--time-control-mode", action="store_true", default=True, help="v34: use timed robot sequence instead of camera detection.")
parser.add_argument("--stage2-delay-after-robot1", type=float, default=2.0, help="Seconds after robot1 right/not_today motion END before robot2 sorts day2/day3.")
parser.add_argument("--robot1-start-delay-after-spawn", type=float, default=3.0, help="Seconds after spawn before robot1 starts first sort.")
parser.add_argument("--skip-robot2-for-today", action="store_true", default=True, help="Today does not require stage2 by default.")
parser.add_argument("--no-skip-robot2-for-today", dest="skip_robot2_for_today", action="store_false")
parser.add_argument("--final-gate-enabled", action="store_true", default=True, help="Wait until current box overlaps/passes final classification zone before next spawn.")
parser.add_argument("--no-final-gate-enabled", dest="final_gate_enabled", action="store_false")
parser.add_argument("--final-gate-today-index", type=int, default=5)
parser.add_argument("--final-gate-day2-index", type=int, default=12)
parser.add_argument("--final-gate-day3-index", type=int, default=15)
parser.add_argument("--final-gate-today-path", default="")
parser.add_argument("--final-gate-day2-path", default="")
parser.add_argument("--final-gate-day3-path", default="")
parser.add_argument("--final-gate-xy-margin", type=float, default=0.12)
parser.add_argument("--final-gate-timeout-steps", type=int, default=3600)
parser.add_argument("--final-gate-log-every", type=int, default=30)
parser.add_argument("--final-gate-hit-mode", default="overlap", choices=["overlap", "center"], help="overlap means box bbox can just touch/overlap final gate; center requires bbox center inside.")
parser.add_argument("--final-gate-require-z-overlap", action="store_true", default=False)
parser.add_argument("--final-gate-max-wait-sec", type=float, default=1.0)
parser.add_argument("--despawn-after-final-gate-sec", type=float, default=3.0)
parser.add_argument("--transit-service-enabled", action="store_true", default=True)
parser.add_argument("--no-transit-service-enabled", dest="transit_service_enabled", action="store_false")
parser.add_argument("--transit-service-name", default="/sim/transit_package")
parser.add_argument("--transit-service-timeout-sec", type=float, default=5.0)
parser.add_argument("--transit-ros-domain-id", default="119")
parser.add_argument("--transit-ros-localhost-only", default="0")
parser.add_argument("--transit-rmw-implementation", default="rmw_cyclonedds_cpp")
parser.add_argument("--transit-cyclonedds-uri", default="file:///home/rokey/.ros/cyclonedds_wifi.xml")
parser.add_argument("--transit-helper-path", default="/home/rokey/dev_ws/isaac_sim/isaac_step4/transit_package_client_helper.py")
parser.add_argument("--transit-ros-setups", default="/opt/ros/humble/setup.bash;/home/rokey/cobot3_ws/install/setup.bash")
parser.add_argument("--robot1-today-route", default="today", choices=["today", "not_today"], help="robot1 QR target=today route. v04 default: today.")
parser.add_argument("--robot1-not-today-route", default="not_today", choices=["today", "not_today"], help="robot1 QR target=day2/day3 route. v04 default: not_today.")
parser.add_argument("--robot2-day2-route", default="today", choices=["today", "not_today"], help="robot2 target=day2 route. v05 default: today.")
parser.add_argument("--robot2-day3-route", default="not_today", choices=["today", "not_today"], help="robot2 target=day3 route. v05 default: not_today.")
parser.add_argument("--post-cycle-idle-wait", type=float, default=1.0, help="Extra idle wait before next supply.")
parser.add_argument("--add-top-qr-visual", action="store_true", default=False, help="Legacy option: add a new QR texture plane on top. v12 default is OFF because box_assets already contain QR.")
parser.add_argument("--no-add-top-qr-visual", dest="add_top_qr_visual", action="store_false")
parser.add_argument("--top-qr-size", type=float, default=0.18)
parser.add_argument("--top-qr-z-offset", type=float, default=0.012)
parser.add_argument("--robot1-camera-enabled", action="store_true", default=True)
parser.add_argument("--no-robot1-camera-enabled", dest="robot1_camera_enabled", action="store_false")
parser.add_argument("--robot1-camera-path", default="/World/Step4FinalfacIntegration/Robot1_QR_OverheadCamera")
parser.add_argument("--robot1-camera-pos", default="-0.1,1.05,4.08", help="Robot1 QR overhead camera world position x,y,z. v12 default is closer to enlarge the asset QR.")
parser.add_argument("--robot1-camera-focal-length", type=float, default=55.0)
parser.add_argument("--robot1-camera-horizontal-aperture", type=float, default=18.0, help="Smaller aperture narrows FOV. v12 default zooms in on the QR.")
parser.add_argument("--robot1-camera-resolution", default="1024,1024")
parser.add_argument("--qr-real-decode", action="store_true", default=True, help="Try real camera image QR decode before fallback.")
parser.add_argument("--no-qr-real-decode", dest="qr_real_decode", action="store_false")
parser.add_argument("--qr-decode-save-debug", action="store_true", default=True)
parser.add_argument("--qr-decode-debug-dir", default="/home/rokey/dev_ws/isaac_sim/isaac_step4/qr_debug")
parser.add_argument("--qr-decode-warmup-frames", type=int, default=8)
parser.add_argument("--qr-decode-max-attempts", type=int, default=5)
parser.add_argument("--item-log-only-current", action="store_true", default=True)
# Sort-start trigger: robot starts sorting only after box enters the work-table area.
parser.add_argument("--sort-start-trigger-track-path", default="/World/ConveyorTrack_01", help="Work-table area track path. Default is the square area between green/yellow lanes.")
parser.add_argument("--sort-start-trigger-track-index", type=int, default=1, help="Fallback ConveyorTrack index for work-table sort-start trigger.")
parser.add_argument("--sort-start-trigger-size", default="0.60,0.60,0.50", help="Sort-start trigger size x,y,z for each of the two work-area sensors.")
parser.add_argument("--sort-start-trigger-centers", default="33.75807,-0.91425,1.24929;35.84186,1.03315,1.24929", help="Semicolon-separated two sensor centers x,y,z. Empty string falls back to track-center trigger.")
parser.add_argument("--robot1-sort-start-sensor-index", type=int, default=0, help="1-based sort-start sensor index for robot1. 0 means any sensor. Default 0 avoids waiting forever if sensor order is reversed.")
parser.add_argument("--robot2-sort-start-sensor-index", type=int, default=2, help="1-based sort-start sensor index for robot2. 0 means any sensor.")
parser.add_argument("--no-robot2-sort-trigger", dest="require_robot2_sort_trigger", action="store_false", help="Disable robot2 wait trigger and fall back to robot2-delay.")
parser.set_defaults(require_robot2_sort_trigger=True)
parser.add_argument("--sort-start-trigger-xy-margin", type=float, default=0.05)
parser.add_argument("--sort-start-trigger-timeout-steps", type=int, default=900)
parser.add_argument("--sort-start-trigger-log-every", type=int, default=60, help="Print waiting status every N simulation steps while waiting for sort-start trigger.")
parser.add_argument("--sort-start-trigger-visual", action="store_true", default=True)
parser.add_argument("--no-sort-start-trigger-visual", dest="sort_start_trigger_visual", action="store_false")
parser.add_argument("--robot2-delay", type=float, default=2.0, help="Seconds to wait before robot2 sort after robot1 not_today route")
parser.add_argument("--box-size", type=str, default="0.20,0.20,0.15")
parser.add_argument("--box-mass", type=float, default=0.35)
# QR / package asset integration
parser.add_argument("--qr-enabled", action="store_true", default=True, help="Use QR package assets when available.")
parser.add_argument("--no-qr-enabled", dest="qr_enabled", action="store_false")
parser.add_argument("--box-assets-zip", default="/home/rokey/dev_ws/isaac_sim/isaac_step4/box_assets.zip")
parser.add_argument("--qr-codes-zip", default="/home/rokey/dev_ws/isaac_sim/isaac_step4/qr_codes.zip")
parser.add_argument("--asset-cache-dir", default="/home/rokey/dev_ws/isaac_sim/isaac_step4/step4_qr_asset_cache")
parser.add_argument("--package-csv", default="/home/rokey/dev_ws/isaac_sim/isaac_step4/packages_2026-06-08.csv", help="Daily package CSV. qr_id is serial key; route_zone is used for date classification.")
parser.add_argument("--today-date", default="2026-06-08", help="Business 기준일 YYYY-MM-DD or YYYYMMDD. route_zone equal to this is target=today.")
parser.add_argument("--target-dates", default="today=20260606,day2=20260607,day3=20260608", help="QR date to target map seed. Format: today=YYYYMMDD,day2=YYYYMMDD,day3=YYYYMMDD")
parser.add_argument("--qr-unmapped-date-target", default="day3", choices=["today", "day2", "day3"], help="Fallback target for QR dates not listed in --target-dates")
parser.add_argument("--package-sample-mode", default="target_sequence", choices=["target_sequence", "asset_order", "random"], help="How to choose package assets for each cycle.")
parser.add_argument("--asset-box-scale", default="1.0,1.0,1.0", help="Scale applied to referenced package USD root.")
parser.add_argument("--box-qr-face-up", default="y_neg", choices=["x_pos", "x_neg", "y_pos", "y_neg", "z_pos", "z_neg"], help="Which local face of box_assets should face upward. v12 default y_neg from QR face test.")
parser.add_argument("--qr-log-only", action="store_true", default=False, help="Do not use QR target for route; only log decoded QR.")

# Height / collision protection
parser.add_argument("--max-sortable-box-height", type=float, default=0.24, help="If measured box visual height exceeds this, apply tall-box policy.")
parser.add_argument("--tall-box-policy", default="proxy", choices=["proxy", "skip", "allow"], help="proxy caps collision height, skip avoids robot motion, allow does nothing.")
parser.add_argument("--collision-proxy-height-cap", type=float, default=0.16, help="Max collider height used by proxy mode.")
parser.add_argument("--collision-proxy-xy-scale", type=float, default=0.92, help="XY shrink factor for proxy collider to avoid gripper edge snag.")
parser.add_argument("--disable-asset-collisions-for-proxy", action="store_true", default=True)
parser.add_argument("--no-disable-asset-collisions-for-proxy", dest="disable_asset_collisions_for_proxy", action="store_false")
parser.add_argument("--spawn-z-offset", type=float, default=0.05)
parser.add_argument("--fallback-spawn-pos", type=str, default="0,0,1.30")

# Conveyor stop override paths, optional
parser.add_argument("--track0-path", default="")
parser.add_argument("--track1-path", default="")
parser.add_argument("--track4-path", default="")
parser.add_argument("--track8-path", default="")

# Robot joint roots
parser.add_argument("--robot1-joint-root", default="/World/FFW_BG2/ffw_bg2_follower/joints")
parser.add_argument("--robot2-joint-root", default="/World/FFW_BG2_01/ffw_bg2_follower/joints")
parser.add_argument("--auto-discover-joint-roots", action="store_true", default=True)
parser.add_argument("--swap-robots", dest="swap_robots", action="store_true", default=False, help="Do not use by default. robot1=FFW_BG2, robot2=FFW_BG2_01.")
parser.add_argument("--no-swap-robots", dest="swap_robots", action="store_false", help="Do not swap robot role roots; use raw robot1/robot2 roots.")
parser.add_argument("--sensor1-robot-slot", default="auto", choices=["auto", "1", "2"], help="Which raw robot slot belongs to sort-start sensor 1. auto=nearest robot to sensor center.")
parser.add_argument("--sensor2-robot-slot", default="auto", choices=["auto", "1", "2"], help="Which raw robot slot belongs to sort-start sensor 2. auto=nearest robot to sensor center.")
parser.add_argument("--sensor-robot-map-mode", default="nearest", choices=["nearest", "index"], help="nearest maps sensors to nearest robot root; index maps sensor1->slot1, sensor2->slot2 unless overridden.")

# Motion timing
parser.add_argument("--motion-steps", type=int, default=5, help="Isaac updates per interpolation point; v06 default is about 2x faster")
parser.add_argument("--settle-steps", type=int, default=15)
parser.add_argument("--tap-via-interp", type=int, default=4)
parser.add_argument("--tap-push-waypoints", type=int, default=7)
parser.add_argument("--tap-retreat-interp", type=int, default=4)
parser.add_argument("--drag-via-interp", type=int, default=4)
parser.add_argument("--drag-push-waypoints", type=int, default=7)
parser.add_argument("--second-drag-via-interp", type=int, default=10, help="Slower interpolation for second drag approach, especially edge_clear_high->pre3.")
parser.add_argument("--second-drag-push-waypoints", type=int, default=14, help="Slower interpolation for second drag end2 push.")
parser.add_argument("--motion-complete-check", action="store_true", default=False, help="Unused in v22. All completion judgement is removed.")
parser.add_argument("--no-motion-complete-check", dest="motion_complete_check", action="store_false")
parser.add_argument("--motion-complete-xy-displacement", type=float, default=0.18, help="XY displacement from motion start treated as sort complete.")
parser.add_argument("--motion-complete-min-updates", type=int, default=2)
parser.add_argument("--pre-via-interp", type=int, default=8)
parser.add_argument("--drag-contact-waypoints", type=int, default=12)
parser.add_argument("--drag-waypoints", type=int, default=18)
parser.add_argument("--drag-retreat-interp", type=int, default=8)

# Optional box snapping for visual debugging
parser.add_argument("--snap-after-sort", action="store_true", help="After robot motion, snap active box to lane/debug positions")
parser.add_argument("--today-drop-pos", default="0,1.0,1.30")
parser.add_argument("--day2-drop-pos", default="1.0,1.0,1.30")
parser.add_argument("--day3-drop-pos", default="1.0,-1.0,1.30")
parser.add_argument("--robot2-work-pos", default="1.0,0.0,1.30")

# Final destination tracks
parser.add_argument("--destination-track-today-index", type=int, default=5, help="Final destination ConveyorTrack index for today.")
parser.add_argument("--destination-track-day2-index", type=int, default=12, help="Final destination ConveyorTrack index for day2.")
parser.add_argument("--destination-track-day3-index", type=int, default=15, help="Final destination ConveyorTrack index for day3.")
parser.add_argument("--destination-track-today-path", default="", help="Optional exact destination track path for today.")
parser.add_argument("--destination-track-day2-path", default="", help="Optional exact destination track path for day2.")
parser.add_argument("--destination-track-day3-path", default="", help="Optional exact destination track path for day3.")
parser.add_argument("--destination-trigger-xy-margin", type=float, default=0.08)
parser.add_argument("--destination-timeout-steps", type=int, default=1800)
parser.add_argument("--destination-log-every", type=int, default=90)
parser.add_argument("--no-wait-destination", dest="wait_destination", action="store_false", help="Do not wait for final destination track before despawn.")
parser.set_defaults(wait_destination=True)

args, _ = parser.parse_known_args()


# =============================================================================
# START ISAAC SIM
# =============================================================================

try:
    from isaacsim import SimulationApp
except Exception:
    from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": bool(args.headless)})

# Isaac imports must come after SimulationApp
import omni.usd
import omni.timeline
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

try:
    import numpy as np
except Exception:
    np = None

try:
    import cv2
except Exception:
    cv2 = None

try:
    # Isaac Sim 4.x/5.x path variants differ. Keep both.
    from isaacsim.sensors.camera import Camera as IsaacCamera
except Exception:
    try:
        from omni.isaac.sensor import Camera as IsaacCamera
    except Exception:
        IsaacCamera = None


# =============================================================================
# COPIED ROBOT MOTION DATA FROM Step3_Stage3_06_unified_qr_sorting_pipeline_v17
# =============================================================================

DRIVE_STIFFNESS = float(os.environ.get("BG2_DRIVE_STIFFNESS", "16000.0"))
DRIVE_DAMPING = float(os.environ.get("BG2_DRIVE_DAMPING", "900.0"))
DRIVE_MAX_FORCE = float(os.environ.get("BG2_DRIVE_MAX_FORCE", "350000.0"))

JOINT_LIMITS_DEG = {
    "left": {
        "j1": (-179.9087371826172, 179.9087371826172),
        "j2": (0.0, 179.9087371826172),
        "j3": (-179.9087371826172, 179.9087371826172),
        "j4": (-168.22613525390625, 61.79922866821289),
        "j5": (-179.9087371826172, 179.9087371826172),
        "j6": (-89.9543685913086, 89.9543685913086),
        "j7": (-104.28404235839844, 90.55024719238281),
    },
    "right": {
        "j1": (-179.9087371826172, 179.9087371826172),
        "j2": (-179.9087371826172, 0.0),
        "j3": (-179.9087371826172, 179.9087371826172),
        "j4": (-168.22613525390625, 61.79922866821289),
        "j5": (-179.9087371826172, 179.9087371826172),
        "j6": (-89.9543685913086, 89.9543685913086),
        "j7": (-90.55024719238281, 104.28404235839844),
    },
}
JOINT_LIMIT_MARGIN_DEG = float(os.environ.get("BG2_JOINT_LIMIT_MARGIN_DEG", "2.0"))

# v05/v02 user-captured tap-only poses from finalfac1.usd.
# 전체 동작:
#   initial -> pre1 -> pre2 -> end1 -> end2 -> end3
# 복귀:
#   end3 -> end2 -> initial

# v11/v10/v05 initial transition policy requested by user:
#   init1 -> init2 -> init3(return/forward)
# Reason:
#   Direct or interpolated init1->init3 made the hands sweep forward and hit the conveyor.
#   Therefore the first boot transition is now explicitly staged.
INIT1_LEFT_RAW = {
    "j1": 0.0, "j2": 90.0, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
INIT2_LEFT_RAW = {
    "j1": 0.0, "j2": 90.0, "j3": 90.0, "j4": -90.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
INIT3_LEFT_RAW = {
    "j1": -90.0, "j2": 0.0, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}

INIT1_RIGHT_RAW = {
    "j1": 0.0, "j2": -90.0, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
INIT2_RIGHT_RAW = {
    "j1": 0.0, "j2": -90.0, "j3": -90.0, "j4": -90.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
INIT3_RIGHT_RAW = {
    "j1": -90.0, "j2": 0.0, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}

# Compatibility aliases used by old helper names.
# SIDE_STANDBY means init1, FORWARD_STANDBY means init3(return pose).
SIDE_STANDBY_LEFT_RAW = INIT1_LEFT_RAW
SIDE_STANDBY_RIGHT_RAW = INIT1_RIGHT_RAW
FORWARD_STANDBY_LEFT_RAW = INIT3_LEFT_RAW
FORWARD_STANDBY_RIGHT_RAW = INIT3_RIGHT_RAW

# Existing helper names now mean the safe return/hold pose, not the first boot pose.
PARALLEL_STANDBY_LEFT_RAW = FORWARD_STANDBY_LEFT_RAW
PARALLEL_STANDBY_RIGHT_RAW = FORWARD_STANDBY_RIGHT_RAW

LEFT_TAP_INITIAL_RAW = FORWARD_STANDBY_LEFT_RAW
LEFT_TAP_PRE_1_RAW = {
    "j1": -90.0, "j2": 2.0, "j3": 10.0, "j4": -40.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
LEFT_TAP_PRE_2_RAW = {
    "j1": -40.0, "j2": 2.0, "j3": 10.0, "j4": -40.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
LEFT_TAP_END_1_RAW = {
    "j1": -40.0, "j2": 2.0, "j3": -50.0, "j4": -40.0, "j5": 30.0, "j6": 0.0, "j7": 0.0
}
LEFT_TAP_END_2_RAW = {
    "j1": -30.0, "j2": 50.0, "j3": -20.0, "j4": -70.0, "j5": 10.0, "j6": 0.0, "j7": 0.0
}
LEFT_TAP_END_3_RAW = {
    "j1": -35.0, "j2": 50.0, "j3": -70.0, "j4": -75.0, "j5": 40.0, "j6": -35.0, "j7": -30.0
}
LEFT_TAP_END_4_RAW = {
    "j1": -35.0, "j2": 5.0, "j3": -70.0, "j4": -75.0, "j5": 30.0, "j6": 20.0, "j7": -30.0
}

RIGHT_TAP_INITIAL_RAW = FORWARD_STANDBY_RIGHT_RAW
RIGHT_TAP_PRE_1_RAW = {
    "j1": -90.0, "j2": -2.0, "j3": -9.30, "j4": -40.0, "j5": 0.70, "j6": 0.0, "j7": 0.0
}
RIGHT_TAP_PRE_2_RAW = {
    "j1": -40.0, "j2": -2.0, "j3": -9.30, "j4": -40.0, "j5": 0.70, "j6": 0.0, "j7": 0.0
}
RIGHT_TAP_END_1_RAW = {
    "j1": -40.0, "j2": -2.0, "j3": 50.70, "j4": -40.0, "j5": -29.30, "j6": 0.0, "j7": 0.0
}
RIGHT_TAP_END_2_RAW = {
    "j1": -30.0, "j2": -50.0, "j3": 20.0, "j4": -70.0, "j5": -10.0, "j6": 0.0, "j7": 0.0
}
RIGHT_TAP_END_3_RAW = {
    "j1": -35.0, "j2": -50.0, "j3": 70.0, "j4": -75.0, "j5": -40.0, "j6": -35.0, "j7": 30.0
}
RIGHT_TAP_END_4_RAW = {
    "j1": -35.0, "j2": -5.0, "j3": 70.0, "j4": -75.0, "j5": -30.0, "j6": 20.0, "j7": 30.0
}

# Compatibility names used by old helper functions.
# contact=end3, end=end4
LEFT_TAP_CONTACT_RAW = LEFT_TAP_END_3_RAW
LEFT_TAP_END_15CM_RAW = LEFT_TAP_END_4_RAW
RIGHT_TAP_CONTACT_RAW = RIGHT_TAP_END_3_RAW
RIGHT_TAP_END_15CM_RAW = RIGHT_TAP_END_4_RAW

# Existing route naming:
#   today/day2 -> right tap
#   not_today/day3 -> left tap
TODAY_RIGHT_TAP_APPROACH_WAYPOINTS_RAW = [
    RIGHT_TAP_PRE_1_RAW,
    RIGHT_TAP_PRE_2_RAW,
    RIGHT_TAP_END_1_RAW,
    RIGHT_TAP_END_2_RAW,
    RIGHT_TAP_END_3_RAW,
    RIGHT_TAP_END_4_RAW,
]
NOT_TODAY_LEFT_TAP_APPROACH_WAYPOINTS_RAW = [
    LEFT_TAP_PRE_1_RAW,
    LEFT_TAP_PRE_2_RAW,
    LEFT_TAP_END_1_RAW,
    LEFT_TAP_END_2_RAW,
    LEFT_TAP_END_3_RAW,
    LEFT_TAP_END_4_RAW,
]

# Drag data restored in v21.
# Route mapping:
#   route=today     : right tap then opposite left drag
#   route=not_today : left tap then opposite right drag

RIGHT_DRAG_PRE_1_RAW = {
    "j1": -40.0, "j2": -2.0, "j3": 50.0, "j4": -100.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
RIGHT_DRAG_PRE_2_RAW = {
    "j1": -30.0, "j2": -2.0, "j3": 90.0, "j4": -80.0, "j5": -30.0, "j6": 0.0, "j7": 0.0
}
RIGHT_DRAG_END_1_RAW = {
    "j1": -30.0, "j2": -2.0, "j3": 0.0, "j4": -46.0, "j5": -15.0, "j6": 0.0, "j7": 0.0
}
RIGHT_DRAG_PRE_3_RAW = {
    "j1": -20.0, "j2": -2.0, "j3": 90.0, "j4": -80.0, "j5": -30.0, "j6": 0.0, "j7": 0.0
}
RIGHT_DRAG_END_2_RAW = {
    "j1": -20.0, "j2": -2.0, "j3": -5.0, "j4": -59.0, "j5": -10.0, "j6": 0.0, "j7": 0.0
}
RIGHT_DRAG_RETURN_RAW = {
    "j1": -90.0, "j2": 0.0, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
RIGHT_DRAG_BODY_HIGH_CLEAR_RAW = {
    "j1": -60.0, "j2": -55.0, "j3": 45.0, "j4": -95.0, "j5": -20.0, "j6": 0.0, "j7": 0.0
}
RIGHT_DRAG_EDGE_CLEAR_HIGH_RAW = {
    "j1": -75.0, "j2": -65.0, "j3": 30.0, "j4": -110.0, "j5": -10.0, "j6": 0.0, "j7": 0.0
}

LEFT_DRAG_PRE_1_RAW = {
    "j1": -40.0, "j2": 2.0, "j3": -50.0, "j4": -100.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
LEFT_DRAG_PRE_2_RAW = {
    "j1": -30.0, "j2": 2.0, "j3": -90.0, "j4": -80.0, "j5": 30.0, "j6": 0.0, "j7": 0.0
}
LEFT_DRAG_END_1_RAW = {
    "j1": -30.0, "j2": 2.0, "j3": 0.0, "j4": -46.0, "j5": 15.0, "j6": 0.0, "j7": 0.0
}
LEFT_DRAG_PRE_3_RAW = {
    "j1": -20.0, "j2": 2.0, "j3": -90.0, "j4": -80.0, "j5": 30.0, "j6": 0.0, "j7": 0.0
}
LEFT_DRAG_END_2_RAW = {
    "j1": -20.0, "j2": 2.0, "j3": 5.0, "j4": -59.0, "j5": 10.0, "j6": 0.0, "j7": 0.0
}
LEFT_DRAG_RETURN_RAW = {
    "j1": -90.0, "j2": 0.0, "j3": 0.0, "j4": 0.0, "j5": 0.0, "j6": 0.0, "j7": 0.0
}
LEFT_DRAG_BODY_HIGH_CLEAR_RAW = {
    "j1": -60.0, "j2": 55.0, "j3": -45.0, "j4": -95.0, "j5": 20.0, "j6": 0.0, "j7": 0.0
}
LEFT_DRAG_EDGE_CLEAR_HIGH_RAW = {
    "j1": -75.0, "j2": 65.0, "j3": -30.0, "j4": -110.0, "j5": 10.0, "j6": 0.0, "j7": 0.0
}

TODAY_RIGHT_DRAG_APPROACH_WAYPOINTS_RAW = [
    RIGHT_DRAG_PRE_1_RAW,
    RIGHT_DRAG_PRE_2_RAW,
    RIGHT_DRAG_END_1_RAW,
    RIGHT_DRAG_RETURN_RAW,
    RIGHT_DRAG_BODY_HIGH_CLEAR_RAW,
    RIGHT_DRAG_EDGE_CLEAR_HIGH_RAW,
    RIGHT_DRAG_PRE_3_RAW,
    RIGHT_DRAG_END_2_RAW,
]
NOT_TODAY_LEFT_DRAG_APPROACH_WAYPOINTS_RAW = [
    LEFT_DRAG_PRE_1_RAW,
    LEFT_DRAG_PRE_2_RAW,
    LEFT_DRAG_END_1_RAW,
    LEFT_DRAG_RETURN_RAW,
    LEFT_DRAG_BODY_HIGH_CLEAR_RAW,
    LEFT_DRAG_EDGE_CLEAR_HIGH_RAW,
    LEFT_DRAG_PRE_3_RAW,
    LEFT_DRAG_END_2_RAW,
]


# =============================================================================
# CONFIG
# =============================================================================

TEST_ROOT = "/World/Step4FinalfacIntegration"
BOX_ROOT = TEST_ROOT + "/Boxes"
CAMERA_ROOT = TEST_ROOT + "/Cameras"

TARGET_SEQUENCE = ["today", "day2", "day3", "today", "day2", "day3"]

TARGET_COLORS = {
    "today": (0.10, 0.80, 0.20),
    "day2": (0.10, 0.35, 1.00),
    "day3": (1.00, 0.45, 0.10),
}

ROBOT_STOP_TRACKS = {
    1: [0, 1],
    2: [8],
}

DESTINATION_TRACK_DEFAULTS = {
    "today": 5,
    "day2": 12,
    "day3": 15,
}



# =============================================================================
# BASIC HELPERS
# =============================================================================

def parse_vec3(text: str) -> Gf.Vec3d:
    parts = [float(x.strip()) for x in str(text).split(",")]
    if len(parts) != 3:
        raise ValueError(f"Vec3 must be x,y,z: {text}")
    return Gf.Vec3d(parts[0], parts[1], parts[2])


def parse_tuple3(text: str) -> Tuple[float, float, float]:
    v = parse_vec3(text)
    return (float(v[0]), float(v[1]), float(v[2]))


BOX_SIZE = parse_tuple3(args.box_size)

PACKAGE_CATALOG = None
PACKAGE_CURSOR_BY_TARGET = {"today": 0, "day2": 0, "day3": 0}
PACKAGE_ROUTE_DB_BY_QR = None


def get_stage():
    return omni.usd.get_context().get_stage()


def ensure_xform(stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    return prim


def remove_prim(stage, path: str):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)


def get_or_add_translate_op(prim):
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op
    return xf.AddTranslateOp()


def get_or_add_scale_op(prim):
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            return op
    return xf.AddScaleOp()


def set_local_pos(prim, pos):
    get_or_add_translate_op(prim).Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))


def set_local_scale(prim, scale):
    get_or_add_scale_op(prim).Set(Gf.Vec3d(float(scale[0]), float(scale[1]), float(scale[2])))


# v12: box_assets already contain the QR texture.
# qr_face_terminal_tester result: local -Y face must face upward.
QR_FACE_UP_QUAT_WXYZ = {
    "z_pos": (1.0, 0.0, 0.0, 0.0),
    "z_neg": (0.0, 1.0, 0.0, 0.0),
    "x_pos": (0.7071068, 0.0, -0.7071068, 0.0),
    "x_neg": (0.7071068, 0.0, 0.7071068, 0.0),
    "y_pos": (0.7071068, 0.7071068, 0.0, 0.0),
    "y_neg": (0.7071068, -0.7071068, 0.0, 0.0),
}


def force_box_qr_face_up(stage, box_prim_path: str, face_up: str = "y_neg") -> bool:
    """
    Rotate the spawned package-root Xform so the asset QR face points upward.

    Important:
    - This must be applied to the spawned wrapper Xform, not to a child Mesh.
    - It preserves the existing translate/scale ops and inserts an orient op.
    - v12 fixed default is y_neg because the QR was visually confirmed there.
    """
    prim = stage.GetPrimAtPath(box_prim_path)
    if not prim or not prim.IsValid():
        print(f"[QR FACE UP][FAIL] invalid prim: {box_prim_path}")
        return False

    face_up = str(face_up or "y_neg")
    if face_up not in QR_FACE_UP_QUAT_WXYZ:
        print(f"[QR FACE UP][WARN] unknown face_up={face_up}; fallback=y_neg")
        face_up = "y_neg"

    w, x, y, z = QR_FACE_UP_QUAT_WXYZ[face_up]

    xform = UsdGeom.Xformable(prim)
    translate_value = None
    scale_value = None

    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            translate_value = op.Get()
        elif op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_value = op.Get()

    xform.ClearXformOpOrder()

    if translate_value is not None:
        xform.AddTranslateOp().Set(translate_value)

    xform.AddOrientOp().Set(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))

    if scale_value is not None:
        xform.AddScaleOp().Set(scale_value)

    prim.CreateAttribute("user:qr_face_up", Sdf.ValueTypeNames.String).Set(face_up)
    prim.CreateAttribute("user:qr_face_up_quat_wxyz", Sdf.ValueTypeNames.String).Set(
        f"{w:.7f},{x:.7f},{y:.7f},{z:.7f}"
    )
    print(
        f"[QR FACE UP] box={box_prim_path} face_up={face_up} "
        f"quat_wxyz=({w:.6f},{x:.6f},{y:.6f},{z:.6f})"
    )
    return True


def get_world_pos(stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    cache = UsdGeom.XformCache()
    mat = cache.GetLocalToWorldTransform(prim)
    t = mat.ExtractTranslation()
    return Gf.Vec3d(t[0], t[1], t[2])


def set_world_translate(stage, path: str, pos):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return
    set_local_pos(prim, pos)


def sim_steps(count: int):
    for _ in range(max(1, int(count))):
        simulation_app.update()


def sim_wait_seconds(seconds: float):
    end = time.time() + max(0.0, float(seconds))
    while time.time() < end and simulation_app.is_running():
        simulation_app.update()


def lerp_pose(a: Dict[str, float], b: Dict[str, float], t: float) -> Dict[str, float]:
    return {k: float(a[k]) * (1.0 - t) + float(b[k]) * t for k in a.keys()}


def other_arm(arm: str) -> str:
    return "right" if arm == "left" else "left"


def clamp_joint_for_arm(arm: str, joint: str, value: float) -> float:
    lower, upper = JOINT_LIMITS_DEG[arm][joint]
    lower += JOINT_LIMIT_MARGIN_DEG
    upper -= JOINT_LIMIT_MARGIN_DEG
    if lower > upper:
        lower, upper = JOINT_LIMITS_DEG[arm][joint]
    return max(lower, min(upper, float(value)))


def clamp_pose_for_arm(arm: str, pose: Dict[str, float]) -> Dict[str, float]:
    return {k: clamp_joint_for_arm(arm, k, v) for k, v in pose.items()}


def side_standby_pose_for_arm(arm: str) -> Dict[str, float]:
    raw = SIDE_STANDBY_LEFT_RAW if arm == "left" else SIDE_STANDBY_RIGHT_RAW
    return clamp_pose_for_arm(arm, raw)


def forward_standby_pose_for_arm(arm: str) -> Dict[str, float]:
    raw = FORWARD_STANDBY_LEFT_RAW if arm == "left" else FORWARD_STANDBY_RIGHT_RAW
    return clamp_pose_for_arm(arm, raw)


def init_standby_pose_for_arm(arm: str, idx: int) -> Dict[str, float]:
    idx = int(idx)
    if arm == "left":
        raw = INIT1_LEFT_RAW if idx == 1 else INIT2_LEFT_RAW if idx == 2 else INIT3_LEFT_RAW
    else:
        raw = INIT1_RIGHT_RAW if idx == 1 else INIT2_RIGHT_RAW if idx == 2 else INIT3_RIGHT_RAW
    return clamp_pose_for_arm(arm, raw)


def safe_up_pose_for_arm(arm: str) -> Dict[str, float]:
    # Safe hold/return pose is init3/forward-parallel.
    return forward_standby_pose_for_arm(arm)


def all_bg2_joint_names():
    return [f"arm_l_joint{i}" for i in range(1, 8)] + [f"arm_r_joint{i}" for i in range(1, 8)]


# =============================================================================
# STAGE OPEN
# =============================================================================

def open_stage_blocking(usd_path: str):
    ctx = omni.usd.get_context()
    print(f"[OPEN USD] {usd_path}")
    ctx.open_stage(usd_path)

    for _ in range(900):
        simulation_app.update()
        try:
            if not ctx.is_stage_loading():
                break
        except Exception:
            pass

    stage = ctx.get_stage()
    if stage is None:
        raise RuntimeError("Failed to open USD stage")

    print(f"[OPEN USD DONE] {stage.GetRootLayer().identifier}")
    return stage


# =============================================================================
# CONVEYOR DISCOVERY + ENABLE ONLY GATE
# =============================================================================

def conveyor_track_index_from_name(name: str) -> Optional[int]:
    if name == "ConveyorTrack":
        return 0
    m = re.match(r"^ConveyorTrack[_]?(\d+)$", name)
    if m:
        return int(m.group(1))
    m = re.match(r"^conveyortrack[_]?(\d+)$", name, flags=re.IGNORECASE)
    if m:
        return int(m.group(1))
    return None


def discover_conveyor_tracks(stage) -> List[str]:
    paths = []
    for prim in stage.Traverse():
        name = prim.GetName()
        if name == "ConveyorTrack" or re.match(r"^ConveyorTrack[_]?\d+$", name) or re.match(r"^conveyortrack[_]?\d+$", name, flags=re.IGNORECASE):
            paths.append(str(prim.GetPath()))
    return sorted(paths)


def resolve_track_path(stage, track_index: int, explicit_path: str = "") -> Optional[str]:
    if explicit_path:
        prim = stage.GetPrimAtPath(explicit_path)
        if prim.IsValid():
            return explicit_path
        print(f"[CONVEYOR][WARN] explicit track path invalid: {explicit_path}")

    # root-level common forms first
    candidates = []
    if track_index == 0:
        candidates += ["/World/ConveyorTrack", "/World/conveyortrack", "/World/conveyorTrack"]
    candidates += [
        f"/World/ConveyorTrack_{track_index}",
        f"/World/ConveyorTrack_{track_index:02d}",
        f"/World/ConveyorTrack{track_index}",
        f"/World/ConveyorTrack{track_index:02d}",
        f"/World/conveyortrack_{track_index}",
        f"/World/conveyortrack{track_index}",
    ]
    for p in candidates:
        if stage.GetPrimAtPath(p).IsValid():
            return p

    # recursive fallback by basename index
    all_tracks = discover_conveyor_tracks(stage)
    exact = []
    for p in all_tracks:
        idx = conveyor_track_index_from_name(p.split("/")[-1])
        if idx == track_index:
            exact.append(p)

    if exact:
        # Prefer path closer to /World root, then lexical
        exact.sort(key=lambda x: (x.count("/"), x))
        return exact[0]

    return None


class ConveyorEnableGate:
    def __init__(self, stage):
        self.stage = stage
        self.original_enabled = {}
        self.track_overrides = {
            0: args.track0_path,
            1: args.track1_path,
            4: args.track4_path,
            8: args.track8_path,
        }

    def find_nodes_under_track(self, track_index: int) -> List[Usd.Prim]:
        track_path = resolve_track_path(self.stage, track_index, self.track_overrides.get(track_index, ""))
        if not track_path:
            print(f"[CONVEYOR][WARN] Track {track_index} not resolved")
            return []

        nodes = []
        prefix = track_path + "/"
        for prim in self.stage.Traverse():
            p = str(prim.GetPath())
            if not p.startswith(prefix):
                continue
            if prim.GetName() != "ConveyorNode":
                continue
            attr = prim.GetAttribute("inputs:enabled")
            if attr and attr.IsValid():
                nodes.append(prim)

        print(f"[CONVEYOR] Track {track_index} path={track_path} nodes={len(nodes)}")
        return nodes

    def pause_for_robot(self, robot_id: int):
        tracks = ROBOT_STOP_TRACKS.get(robot_id, [])
        print(f"[CONVEYOR] robot{robot_id} sorting -> OFF tracks {tracks}")
        for idx in tracks:
            for node in self.find_nodes_under_track(idx):
                p = str(node.GetPath())
                attr = node.GetAttribute("inputs:enabled")
                if p not in self.original_enabled:
                    self.original_enabled[p] = bool(attr.Get())
                attr.Set(False)
                print(f"[CONVEYOR] OFF {p}")

    def resume_for_robot(self, robot_id: int):
        tracks = ROBOT_STOP_TRACKS.get(robot_id, [])
        print(f"[CONVEYOR] robot{robot_id} done -> RESTORE tracks {tracks}")
        for idx in tracks:
            for node in self.find_nodes_under_track(idx):
                p = str(node.GetPath())
                attr = node.GetAttribute("inputs:enabled")
                original = self.original_enabled.get(p, True)
                attr.Set(bool(original))
                print(f"[CONVEYOR] RESTORE {p} enabled={original}")


# =============================================================================
# BOX SPAWN ON CONVEYOR TRACK
# =============================================================================

def bbox_for_prim(stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    try:
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], useExtentsHint=True)
        box = cache.ComputeWorldBound(prim).ComputeAlignedBox()
        mn = box.GetMin()
        mx = box.GetMax()
        if not all(math.isfinite(float(v)) for v in [mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]]):
            return None
        if mx[0] < mn[0] or mx[1] < mn[1] or mx[2] < mn[2]:
            return None
        return mn, mx
    except Exception as e:
        print(f"[BBOX][WARN] {path}: {e}")
        return None


def track_spawn_position(stage, track_path: str) -> Gf.Vec3d:
    # Prefer Belt child top surface if available
    belt_path = track_path + "/Belt"
    bb = bbox_for_prim(stage, belt_path)
    if bb is None:
        bb = bbox_for_prim(stage, track_path)

    if bb is None:
        print("[SPAWN][WARN] bbox failed; using fallback spawn pos")
        return parse_vec3(args.fallback_spawn_pos)

    mn, mx = bb
    cx = (float(mn[0]) + float(mx[0])) * 0.5
    cy = (float(mn[1]) + float(mx[1])) * 0.5
    top_z = float(mx[2])
    z = top_z + BOX_SIZE[2] * 0.5 + float(args.spawn_z_offset)
    return Gf.Vec3d(cx, cy, z)



# =============================================================================
# SORT-START WORK AREA TRIGGER
# =============================================================================

SORT_START_VIS_ROOT = TEST_ROOT + "/SortStartTriggerVisuals"
OVERHEAD_CAMERA_ROOT = TEST_ROOT + "/OverheadWorktableCameras"


def sort_make_bbox_from_center_size(center: Gf.Vec3d, size: Tuple[float, float, float]):
    sx, sy, sz = (float(size[0]), float(size[1]), float(size[2]))
    return (
        (float(center[0]) - sx * 0.5, float(center[1]) - sy * 0.5, float(center[2]) - sz * 0.5),
        (float(center[0]) + sx * 0.5, float(center[1]) + sy * 0.5, float(center[2]) + sz * 0.5),
    )


def sort_bbox_tuple_for_path(stage, path: str):
    bb = bbox_for_prim(stage, path)
    if bb is None:
        return None
    mn, mx = bb
    return (
        (float(mn[0]), float(mn[1]), float(mn[2])),
        (float(mx[0]), float(mx[1]), float(mx[2])),
    )


def sort_bbox_xy_center(bb):
    if bb is None:
        return None
    mn, mx = bb
    return ((float(mn[0]) + float(mx[0])) * 0.5, (float(mn[1]) + float(mx[1])) * 0.5)


def sort_bbox_center3(bb):
    if bb is None:
        return None
    mn, mx = bb
    return Gf.Vec3d(
        (float(mn[0]) + float(mx[0])) * 0.5,
        (float(mn[1]) + float(mx[1])) * 0.5,
        (float(mn[2]) + float(mx[2])) * 0.5,
    )


def sort_xy_center_inside(point_xy, trig_bb, margin: float = 0.0) -> bool:
    if point_xy is None or trig_bb is None:
        return False
    x, y = float(point_xy[0]), float(point_xy[1])
    mn, mx = trig_bb
    return (
        float(mn[0]) - margin <= x <= float(mx[0]) + margin and
        float(mn[1]) - margin <= y <= float(mx[1]) + margin
    )


def parse_sort_start_centers(raw: str) -> List[Gf.Vec3d]:
    centers = []
    for chunk in str(raw or "").split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        vals = [float(x.strip()) for x in chunk.split(",") if x.strip()]
        if len(vals) != 3:
            raise ValueError(f"--sort-start-trigger-centers entry must be x,y,z: {chunk}")
        centers.append(Gf.Vec3d(vals[0], vals[1], vals[2]))
    return centers


def infer_env_offset_from_preferred_path(stage, preferred_track_path: str) -> Gf.Vec3d:
    """
    vector env에서는 base 좌표에 Env_XX root translation을 더해 센서를 복제한다.
    single GUI/finalfac에서는 offset=(0,0,0).
    """
    p = str(preferred_track_path or "")
    m = re.match(r"^(/World/VecTrainEnv/Env_\\d+)(/|$)", p)
    if not m:
        return Gf.Vec3d(0.0, 0.0, 0.0)
    root_path = m.group(1)
    pos = get_world_pos(stage, root_path)
    if pos is None:
        return Gf.Vec3d(0.0, 0.0, 0.0)
    return Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2]))


def resolve_sort_start_track_path(stage, preferred_path: str = "") -> Optional[str]:
    candidates = []
    if preferred_path:
        candidates.append(preferred_path)
    candidates.append(str(getattr(args, "sort_start_trigger_track_path", "")))
    for p in candidates:
        if p and stage.GetPrimAtPath(p).IsValid():
            return p

    try:
        p = resolve_track_path(stage, int(args.sort_start_trigger_track_index), "")
        if p and stage.GetPrimAtPath(p).IsValid():
            return p
    except Exception:
        pass
    return None


def _make_single_sort_start_spec(label: str, center: Gf.Vec3d, size: Tuple[float, float, float], source: str, source_path: str = "") -> dict:
    trig_bb = sort_make_bbox_from_center_size(center, size)
    return {
        "label": label,
        "source": source,
        "source_path": source_path,
        "bbox": trig_bb,
        "center": [float(center[0]), float(center[1]), float(center[2])],
        "size": [float(size[0]), float(size[1]), float(size[2])],
    }


def make_sort_start_trigger_spec(stage, preferred_track_path: str = "", label: str = "work_area") -> dict:
    """
    로봇 앞 작업대 시작 센서.
    v14/v09: 단일 센서가 아니라 사용자 지정 2개 센서를 union으로 사용한다.

    센서 기본값:
      sensor_01 center=(33.75807, -0.91425, 1.24929), size=(0.6,0.6,0.5)
      sensor_02 center=(35.84186, +1.03315, 1.24929), size=(0.6,0.6,0.5)

    박스 중심 XY가 둘 중 하나에 들어오면 분류 시작.
    """
    size = parse_tuple3(args.sort_start_trigger_size)
    centers = parse_sort_start_centers(getattr(args, "sort_start_trigger_centers", ""))
    children = []

    if centers:
        offset = infer_env_offset_from_preferred_path(stage, preferred_track_path)
        for idx, c in enumerate(centers, start=1):
            center = Gf.Vec3d(float(c[0]) + float(offset[0]), float(c[1]) + float(offset[1]), float(c[2]) + float(offset[2]))
            child_label = f"{label}_sensor_{idx:02d}"
            spec = _make_single_sort_start_spec(child_label, center, size, "explicit_center", f"center_{idx:02d}")
            children.append(spec)
            print(
                f"[SORT TRIGGER] {child_label} explicit_center "
                f"center=({center[0]:+.5f},{center[1]:+.5f},{center[2]:+.5f}) "
                f"size=({size[0]:.3f},{size[1]:.3f},{size[2]:.3f})"
            )

        return {
            "label": label,
            "source": "multi_explicit_center",
            "source_path": "explicit_centers",
            "children": children,
            "bbox": children[0]["bbox"] if children else None,
            "center": children[0]["center"] if children else [0, 0, 0],
            "size": children[0]["size"] if children else [0.6, 0.6, 0.5],
        }

    # Fallback: old single track-center behavior.
    track_path = resolve_sort_start_track_path(stage, preferred_track_path)
    if track_path:
        bb = bbox_for_prim(stage, track_path + "/Belt") or bbox_for_prim(stage, track_path)
        if bb is not None:
            mn, mx = bb
            center = Gf.Vec3d(
                (float(mn[0]) + float(mx[0])) * 0.5,
                (float(mn[1]) + float(mx[1])) * 0.5,
                float(mx[2]) + float(size[2]) * 0.5,
            )
            print(
                f"[SORT TRIGGER] {label} source_track={track_path} "
                f"center=({center[0]:+.3f},{center[1]:+.3f},{center[2]:+.3f}) "
                f"size=({size[0]:.3f},{size[1]:.3f},{size[2]:.3f})"
            )
            return _make_single_sort_start_spec(label, center, size, "track_bbox_center", track_path)

    fallback = parse_vec3(args.fallback_spawn_pos)
    center = Gf.Vec3d(float(fallback[0]), float(fallback[1]), float(fallback[2]))
    print(
        f"[SORT TRIGGER] {label} fallback center=({center[0]:+.3f},{center[1]:+.3f},{center[2]:+.3f}) "
        f"size=({size[0]:.3f},{size[1]:.3f},{size[2]:.3f})"
    )
    return _make_single_sort_start_spec(label, center, size, "fallback", "")


def sort_trigger_children(spec: dict) -> List[dict]:
    children = spec.get("children")
    if isinstance(children, list) and children:
        return children
    return [spec]


def select_sort_start_sensor_spec(spec: dict, sensor_index: int, label: str) -> dict:
    """
    sensor_index:
      0  -> any sensor union
      1+ -> selected single sensor only
    """
    idx = int(sensor_index)
    children = sort_trigger_children(spec)
    if idx <= 0:
        out = dict(spec)
        out["label"] = label
        print(f"[SORT TRIGGER SELECT] {label}: ANY sensor count={len(children)}  # robot moves if box enters any selected sensor")
        return out

    if idx > len(children):
        raise RuntimeError(f"{label}: requested sensor index {idx}, but only {len(children)} sort-start sensor(s) exist")

    child = dict(children[idx - 1])
    child["label"] = label
    print(
        f"[SORT TRIGGER SELECT] {label}: sensor={idx} "
        f"center=({child['center'][0]:+.5f},{child['center'][1]:+.5f},{child['center'][2]:+.5f}) "
        f"size=({child['size'][0]:.3f},{child['size'][1]:.3f},{child['size'][2]:.3f})"
    )
    return child


def create_sort_start_trigger_visual(stage, spec: dict, name_suffix: str = ""):
    if not bool(getattr(args, "sort_start_trigger_visual", True)):
        return []
    paths = []
    try:
        ensure_xform(stage, SORT_START_VIS_ROOT)
        safe_suffix = re.sub(r"[^A-Za-z0-9_]+", "_", str(name_suffix or "work_area"))

        for idx, child in enumerate(sort_trigger_children(spec), start=1):
            path = f"{SORT_START_VIS_ROOT}/WorkAreaStartTrigger_{safe_suffix}_{idx:02d}"
            remove_prim(stage, path)
            center = child.get("center", [0, 0, 0])
            size = child.get("size", [0.6, 0.6, 0.5])

            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.35, 1.0)])
            try:
                cube.CreateDisplayOpacityAttr([0.22])
            except Exception:
                pass

            prim = cube.GetPrim()
            set_local_pos(prim, Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
            set_local_scale(prim, Gf.Vec3d(float(size[0]), float(size[1]), float(size[2])))
            prim.CreateAttribute("user:visual_only_no_collision", Sdf.ValueTypeNames.Bool).Set(True)
            prim.CreateAttribute("user:trigger_role", Sdf.ValueTypeNames.String).Set("sort_start_work_area")
            prim.CreateAttribute("user:trigger_sensor_index", Sdf.ValueTypeNames.Int).Set(int(idx))
            print(f"[SORT TRIGGER VISUAL] sensor={idx:02d} path={path}")
            paths.append(path)

        return paths
    except Exception as e:
        print(f"[SORT TRIGGER VISUAL][WARN] failed: {e}")
        return paths


def detect_sort_start_trigger(stage, box_path: str, spec: dict) -> Tuple[bool, Tuple[float, float]]:
    box_bb = sort_bbox_tuple_for_path(stage, box_path)
    box_xy = sort_bbox_xy_center(box_bb)
    for idx, child in enumerate(sort_trigger_children(spec), start=1):
        hit = sort_xy_center_inside(box_xy, child.get("bbox"), margin=float(args.sort_start_trigger_xy_margin))
        if hit:
            try:
                spec["last_hit_sensor"] = idx
                spec["last_hit_label"] = child.get("label", f"sensor_{idx:02d}")
            except Exception:
                pass
            return True, box_xy if box_xy is not None else (float("nan"), float("nan"))
    return False, box_xy if box_xy is not None else (float("nan"), float("nan"))


def wait_for_sort_start_trigger(stage, box_path: str, spec: dict, label: str = "box") -> bool:
    timeout = max(1, int(args.sort_start_trigger_timeout_steps))
    log_every = max(1, int(getattr(args, "sort_start_trigger_log_every", 60)))

    for step_i in range(1, timeout + 1):
        hit, xy = detect_sort_start_trigger(stage, box_path, spec)
        if hit:
            print(
                f"[SORT TRIGGER HIT] {label} step={step_i}/{timeout} "
                f"sensor={spec.get('last_hit_sensor', '?')} "
                f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f}) source={spec.get('source_path', '')}"
            )
            return True

        if step_i == 1 or step_i % log_every == 0:
            print(
                f"[SORT TRIGGER WAIT] {label} step={step_i}/{timeout} "
                f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f}) "
                f"sensors={len(sort_trigger_children(spec))}"
            )

        simulation_app.update()

    hit, xy = detect_sort_start_trigger(stage, box_path, spec)
    print(
        f"[SORT TRIGGER TIMEOUT] {label} hit={int(hit)} "
        f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f}) source={spec.get('source_path', '')}"
    )
    for idx, child in enumerate(sort_trigger_children(spec), start=1):
        bb = child.get("bbox")
        if bb:
            mn, mx = bb
            inside = sort_xy_center_inside(xy, bb, margin=float(args.sort_start_trigger_xy_margin))
            print(
                f"  [SORT TRIGGER RANGE] sensor={idx:02d} "
                f"x=[{float(mn[0]):+.3f},{float(mx[0]):+.3f}] "
                f"y=[{float(mn[1]):+.3f},{float(mx[1]):+.3f}] "
                f"center_inside={int(inside)}"
            )
    return bool(hit)



def wait_for_specific_sensor(stage, box_path: str, all_spec: dict, sensor_idx: int, label: str) -> bool:
    selected = select_sort_start_sensor_spec(all_spec, int(sensor_idx), label)
    return wait_for_sort_start_trigger(stage, box_path, selected, label=label)


def robot_xy_for_root(stage, root: str):
    pos_path, pos = representative_robot_pos(stage, root)
    if pos is None:
        return None, pos_path
    return (float(pos[0]), float(pos[1])), pos_path


def sensor_xy_from_child(child: dict):
    center = child.get("center", None)
    if not center or len(center) < 2:
        return None
    return (float(center[0]), float(center[1]))


def choose_robot_slot_for_sensor(stage, sensor_idx: int, child: dict, robot_roots_by_slot: dict) -> int:
    override = str(args.sensor1_robot_slot if int(sensor_idx) == 1 else args.sensor2_robot_slot)
    if override in {"1", "2"}:
        return int(override)

    if str(args.sensor_robot_map_mode) == "index":
        return 1 if int(sensor_idx) == 1 else 2

    sxy = sensor_xy_from_child(child)
    if sxy is None:
        return 1 if int(sensor_idx) == 1 else 2

    best_slot = None
    best_d2 = None
    for slot, root in robot_roots_by_slot.items():
        rxy, pos_path = robot_xy_for_root(stage, root)
        if rxy is None:
            continue
        d2 = (rxy[0] - sxy[0]) ** 2 + (rxy[1] - sxy[1]) ** 2
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best_slot = int(slot)

    if best_slot is None:
        return 1 if int(sensor_idx) == 1 else 2
    return int(best_slot)


def build_sensor_robot_map(stage, all_spec: dict, robot_roots_by_slot: dict) -> dict:
    """
    핵심:
    - 더 이상 robot1/robot2 라벨로 먼저 정하지 않는다.
    - sensor N에 박스가 들어오면 sensor N 앞에 매칭된 실제 robot root가 움직인다.
    - conveyor gate는 sensor zone index 기준으로 pause한다.
    """
    mapping = {}
    children = sort_trigger_children(all_spec)

    for idx, child in enumerate(children, start=1):
        slot = choose_robot_slot_for_sensor(stage, idx, child, robot_roots_by_slot)
        root = robot_roots_by_slot[int(slot)]
        rxy, pos_path = robot_xy_for_root(stage, root)
        sxy = sensor_xy_from_child(child)
        mapping[int(idx)] = {
            "sensor_idx": int(idx),
            "robot_slot": int(slot),
            "joint_root": root,
            "label": f"sensor{idx}_robot_slot{slot}",
            "sensor_xy": sxy,
            "robot_xy": rxy,
            "robot_pos_path": pos_path,
        }
        print(
            f"[SENSOR ROBOT MAP] sensor={idx} -> robot_slot={slot} "
            f"sensor_xy=({sxy[0]:+.3f},{sxy[1]:+.3f}) "
            f"robot_xy=({rxy[0]:+.3f},{rxy[1]:+.3f}) root={root}"
            if sxy is not None and rxy is not None else
            f"[SENSOR ROBOT MAP] sensor={idx} -> robot_slot={slot} root={root}"
        )

    return mapping


def get_hit_sensor_idx(spec: dict) -> int:
    try:
        return int(spec.get("last_hit_sensor", 0))
    except Exception:
        return 0


def other_sensor_idx(sensor_idx: int, all_spec: dict) -> int:
    children = sort_trigger_children(all_spec)
    if len(children) <= 1:
        return int(sensor_idx)
    for idx in range(1, len(children) + 1):
        if idx != int(sensor_idx):
            return idx
    return int(sensor_idx)




# =============================================================================
# QR / PACKAGE ASSET HELPERS
# =============================================================================

def safe_extract_zip(zip_path: str, cache_dir: Path, marker_name: str) -> Optional[Path]:
    zp = Path(str(zip_path)).expanduser()
    if not zp.exists():
        print(f"[ASSET][WARN] zip not found: {zp}")
        return None

    cache_dir.mkdir(parents=True, exist_ok=True)
    marker = cache_dir / marker_name
    try:
        if not marker.exists():
            print(f"[ASSET] extracting {zp} -> {cache_dir}")
            with zipfile.ZipFile(str(zp), "r") as zf:
                zf.extractall(str(cache_dir))
            marker.write_text(str(time.time()), encoding="utf-8")
        else:
            print(f"[ASSET] using cache {cache_dir}")
        return cache_dir
    except Exception as e:
        print(f"[ASSET][WARN] extract failed {zp}: {e}")
        return None


def parse_target_dates(raw: str) -> Dict[str, str]:
    out = {}
    for chunk in str(raw or "").split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "=" not in chunk:
            continue
        k, v = chunk.split("=", 1)
        k = k.strip()
        v = v.strip()
        if k in {"today", "day2", "day3"} and re.match(r"^\d{8}$", v):
            out[k] = v
    return out



def normalize_date_iso(value: str) -> str:
    raw = str(value or "").strip()
    if not raw:
        return ""
    if re.match(r"^\d{8}$", raw):
        return f"{raw[0:4]}-{raw[4:6]}-{raw[6:8]}"
    if re.match(r"^\d{4}-\d{2}-\d{2}$", raw):
        return raw
    return raw


def date_to_compact(value: str) -> str:
    iso = normalize_date_iso(value)
    if re.match(r"^\d{4}-\d{2}-\d{2}$", iso):
        return iso.replace("-", "")
    return str(value or "")


def route_zone_to_target(route_zone: str, fallback: str = "day3") -> str:
    route_iso = normalize_date_iso(route_zone)
    today_iso = normalize_date_iso(getattr(args, "today_date", "2026-06-08"))
    try:
        route_dt = datetime.strptime(route_iso, "%Y-%m-%d")
        today_dt = datetime.strptime(today_iso, "%Y-%m-%d")
        delta_days = (route_dt.date() - today_dt.date()).days
        if delta_days == 0:
            return "today"
        if delta_days == 1:
            return "day2"
        if delta_days == 2:
            return "day3"
        print(f"[PACKAGE CSV][WARN] route_zone={route_iso} is delta={delta_days} from today={today_iso}; fallback={fallback}")
        return fallback if fallback in {"today", "day2", "day3"} else "day3"
    except Exception as e:
        print(f"[PACKAGE CSV][WARN] bad route_zone={route_zone} today={today_iso}: {e}; fallback={fallback}")
        return fallback if fallback in {"today", "day2", "day3"} else "day3"


def load_package_route_db_by_qr() -> dict:
    global PACKAGE_ROUTE_DB_BY_QR
    if PACKAGE_ROUTE_DB_BY_QR is not None:
        return PACKAGE_ROUTE_DB_BY_QR

    db = {}
    csv_path = Path(str(getattr(args, "package_csv", ""))).expanduser()
    if not csv_path.exists():
        print(f"[PACKAGE CSV][WARN] not found: {csv_path}. Falling back to legacy QR date mapping.")
        PACKAGE_ROUTE_DB_BY_QR = db
        return db

    try:
        with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
            reader = csv.DictReader(f)
            for row_i, row in enumerate(reader, start=1):
                qr_id = str(row.get("qr_id", "") or "").strip()
                if not qr_id:
                    continue
                route_zone = normalize_date_iso(row.get("route_zone", ""))
                target = route_zone_to_target(route_zone, fallback=str(getattr(args, "qr_unmapped_date_target", "day3")))
                rec = {
                    "row_index": row_i,
                    "package_id": str(row.get("package_id", "") or "").strip(),
                    "customer_name": str(row.get("customer_name", "") or "").strip(),
                    "route_zone": route_zone,
                    "route_zone_raw": str(row.get("route_zone", "") or "").strip(),
                    "qr_id": qr_id,
                    "target": target,
                }
                db[qr_id] = rec

        counts = {"today": 0, "day2": 0, "day3": 0}
        for rec in db.values():
            if rec.get("target") in counts:
                counts[rec["target"]] += 1
        print(
            f"[PACKAGE CSV] loaded={len(db)} file={csv_path} today_date={normalize_date_iso(getattr(args, 'today_date', '2026-06-08'))} "
            f"target_counts=today:{counts['today']} day2:{counts['day2']} day3:{counts['day3']}"
        )
    except Exception as e:
        print(f"[PACKAGE CSV][WARN] failed to read {csv_path}: {e}. Falling back to legacy QR date mapping.")
        db = {}

    PACKAGE_ROUTE_DB_BY_QR = db
    return db


def package_route_record_for_qr_payload(payload: str) -> Optional[dict]:
    qr_id = str(payload or "").strip()
    if not qr_id:
        return None
    db = load_package_route_db_by_qr()
    rec = db.get(qr_id)
    if rec:
        return dict(rec)
    return None


def target_for_qr_payload(payload: str, fallback: str = "day3") -> str:
    """
    v17: QR payload is only qr_id / serial key.
    Actual route date comes from package CSV row.route_zone.
    """
    rec = package_route_record_for_qr_payload(payload)
    if rec:
        print(
            f"[PACKAGE CSV MATCH] qr_id={rec.get('qr_id')} package_id={rec.get('package_id')} "
            f"route_zone={rec.get('route_zone')} -> target={rec.get('target')}"
        )
        return str(rec.get("target") or fallback)

    m = re.search(r"QR_(\d{8})_(\d{3})", str(payload or ""))
    if not m:
        return fallback if fallback in {"today", "day2", "day3"} else "day3"

    date = m.group(1)
    rules = parse_target_dates(args.target_dates)
    for target, target_date in rules.items():
        if date == target_date:
            print(f"[PACKAGE CSV MISS][LEGACY TARGET] qr_id={payload} date={date} -> target={target}")
            return target
    print(f"[PACKAGE CSV MISS][LEGACY FALLBACK] qr_id={payload} date={date} -> target={args.qr_unmapped_date_target}")
    return str(args.qr_unmapped_date_target)


def target_date_for_requested_target(target: str) -> Optional[str]:
    return parse_target_dates(args.target_dates).get(str(target))


def parse_package_asset_name(path: Path) -> Optional[dict]:
    m = re.match(r"^PKG_(\d{8})_(\d{3})\.usd$", path.name)
    if not m:
        return None
    date, serial = m.group(1), m.group(2)
    payload = f"QR_{date}_{serial}"
    csv_rec = package_route_record_for_qr_payload(payload) or {}
    return {
        "date": date,
        "serial": serial,
        "payload": payload,
        "package_id": str(csv_rec.get("package_id", "PKG_" + payload[3:] if payload.startswith("QR_") else payload)),
        "customer_name": str(csv_rec.get("customer_name", "")),
        "route_zone": str(csv_rec.get("route_zone", "")),
        "target": target_for_qr_payload(payload, fallback=str(args.qr_unmapped_date_target)),
        "asset_path": str(path),
        "qr_filename": f"{payload}.png",
        "qr_path": "",
    }



def rewrite_box_asset_qr_texture_paths(asset_root: Path, qr_root: Path) -> int:
    """
    Robust QR texture path fix.

    v24 text rewriting was not enough for binary .usd assets.
    v25 opens each package USD with pxr.Usd and rewrites any asset/string attr
    containing qr_codes/QR_YYYYMMDD_NNN.png to the currently extracted qr_root path.
    """
    if not asset_root.exists() or not qr_root.exists():
        return 0

    changed_files = 0

    def local_qr_for_value(value):
        raw = ""
        try:
            if isinstance(value, Sdf.AssetPath):
                raw = value.path or value.resolvedPath or ""
            else:
                raw = str(value or "")
        except Exception:
            raw = str(value or "")

        m = re.search(r"(QR_\d{8}_\d{3}\.png)", raw)
        if not m:
            return None
        qr_name = m.group(1)
        local_qr = qr_root / qr_name
        if not local_qr.exists():
            print(f"[QR TEXTURE FIX][WARN] local QR missing: {local_qr}")
            return None
        return str(local_qr)

    for usd_path in sorted(asset_root.glob("PKG_*.usd")):
        file_changed = False

        # First try USD API. Works for binary .usd.
        try:
            pkg_stage = Usd.Stage.Open(str(usd_path))
            if pkg_stage:
                for prim in pkg_stage.Traverse():
                    for attr in prim.GetAttributes():
                        try:
                            val = attr.Get()
                        except Exception:
                            continue

                        new_path = local_qr_for_value(val)
                        if not new_path:
                            continue

                        try:
                            # Most texture attrs are asset-path typed.
                            attr.Set(Sdf.AssetPath(new_path))
                            file_changed = True
                            print(f"[QR TEXTURE FIX] {usd_path.name} {prim.GetPath()}.{attr.GetName()} -> {new_path}")
                        except Exception:
                            try:
                                attr.Set(new_path)
                                file_changed = True
                                print(f"[QR TEXTURE FIX] {usd_path.name} {prim.GetPath()}.{attr.GetName()} -> {new_path}")
                            except Exception as e:
                                print(f"[QR TEXTURE FIX][WARN] attr set failed {usd_path.name} {prim.GetPath()}.{attr.GetName()}: {e}")

                if file_changed:
                    pkg_stage.GetRootLayer().Save()
                    changed_files += 1
                    continue
        except Exception as e:
            print(f"[QR TEXTURE FIX][WARN] USD API open failed {usd_path.name}: {e}")

        # Fallback for text/usda-ish files.
        try:
            data = usd_path.read_bytes()
            original = data
            for qr_file in qr_root.glob("QR_*.png"):
                old_patterns = [
                    b"/home/rokey/cobot3_ws/scratch/qr_codes/" + qr_file.name.encode("utf-8"),
                    b"scratch/qr_codes/" + qr_file.name.encode("utf-8"),
                ]
                for old in old_patterns:
                    if old in data:
                        data = data.replace(old, str(qr_file).encode("utf-8"))
            if data != original:
                usd_path.write_bytes(data)
                changed_files += 1
                print(f"[QR TEXTURE FIX] binary/text fallback rewrote {usd_path.name}")
        except Exception as e:
            print(f"[QR TEXTURE FIX][WARN] fallback failed {usd_path.name}: {e}")

    if changed_files:
        print(f"[QR TEXTURE FIX] changed_files={changed_files} qr_root={qr_root}")
    else:
        print(f"[QR TEXTURE FIX] no package files changed. qr_root={qr_root}")
    return changed_files


def build_package_catalog() -> dict:
    global PACKAGE_CATALOG
    if PACKAGE_CATALOG is not None:
        return PACKAGE_CATALOG

    catalog = {
        "all": [],
        "by_target": {"today": [], "day2": [], "day3": []},
        "by_payload": {},
        "asset_root": "",
        "qr_root": "",
    }

    if not bool(args.qr_enabled):
        print("[QR] disabled by --no-qr-enabled")
        PACKAGE_CATALOG = catalog
        return catalog

    cache_root = Path(str(args.asset_cache_dir)).expanduser()
    asset_cache = safe_extract_zip(args.box_assets_zip, cache_root, ".box_assets_extracted")
    qr_cache = safe_extract_zip(args.qr_codes_zip, cache_root, ".qr_codes_extracted")

    if asset_cache is None:
        PACKAGE_CATALOG = catalog
        return catalog

    asset_root = asset_cache / "box_assets"
    qr_root = (qr_cache or cache_root) / "qr_codes"
    catalog["asset_root"] = str(asset_root)
    catalog["qr_root"] = str(qr_root)

    if not asset_root.exists():
        print(f"[ASSET][WARN] asset root not found: {asset_root}")
        PACKAGE_CATALOG = catalog
        return catalog

    try:
        rewrite_box_asset_qr_texture_paths(asset_root, qr_root)
    except Exception as e:
        print(f"[QR TEXTURE FIX][WARN] failed: {e}")

    for usd in sorted(asset_root.glob("PKG_*.usd")):
        rec = parse_package_asset_name(usd)
        if not rec:
            continue
        qr_path = qr_root / rec["qr_filename"]
        if qr_path.exists():
            rec["qr_path"] = str(qr_path)
        catalog["all"].append(rec)
        catalog["by_target"].setdefault(rec["target"], []).append(rec)
        catalog["by_payload"][rec["payload"]] = rec

    for t in ["today", "day2", "day3"]:
        print(f"[QR CATALOG] target={t:5s} packages={len(catalog['by_target'].get(t, []))}")
    print(f"[QR CATALOG] total={len(catalog['all'])} asset_root={catalog['asset_root']} qr_root={catalog['qr_root']}")

    PACKAGE_CATALOG = catalog
    return catalog


def choose_package_record(box_idx: int, requested_target: str) -> Optional[dict]:
    catalog = build_package_catalog()
    if not catalog.get("all"):
        return None

    mode = str(args.package_sample_mode)

    if mode == "random":
        candidates = catalog["by_target"].get(requested_target) or catalog["all"]
        return dict(random.choice(candidates))

    if mode == "asset_order":
        return dict(catalog["all"][int(box_idx) % len(catalog["all"])])

    # target_sequence mode: choose a package whose QR maps to requested target.
    candidates = catalog["by_target"].get(requested_target) or catalog["all"]
    cursor = PACKAGE_CURSOR_BY_TARGET.get(requested_target, 0)
    rec = dict(candidates[cursor % len(candidates)])
    PACKAGE_CURSOR_BY_TARGET[requested_target] = cursor + 1
    return rec


def set_collision_enabled_recursive(stage, root_path: str, enabled: bool):
    prefix = str(root_path).rstrip("/")
    changed = 0
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if not (p == prefix or p.startswith(prefix + "/")):
            continue
        try:
            api = UsdPhysics.CollisionAPI(prim)
            attr = api.GetCollisionEnabledAttr()
            if attr and attr.IsValid():
                attr.Set(bool(enabled))
                changed += 1
        except Exception:
            pass
    return changed


def add_collision_proxy_cube(stage, root_path: str, visual_height: float):
    proxy_path = f"{root_path}/CollisionProxy_Capped"
    remove_prim(stage, proxy_path)

    cap_h = max(0.02, min(float(visual_height), float(args.collision_proxy_height_cap)))
    sx = max(0.02, float(BOX_SIZE[0]) * float(args.collision_proxy_xy_scale))
    sy = max(0.02, float(BOX_SIZE[1]) * float(args.collision_proxy_xy_scale))

    cube = UsdGeom.Cube.Define(stage, proxy_path)
    cube.CreateSizeAttr(1.0)
    try:
        cube.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    except Exception:
        pass

    prim = cube.GetPrim()
    set_local_pos(prim, Gf.Vec3d(0.0, 0.0, 0.0))
    set_local_scale(prim, Gf.Vec3d(sx, sy, cap_h))

    UsdPhysics.CollisionAPI.Apply(prim)
    prim.CreateAttribute("user:collision_proxy", Sdf.ValueTypeNames.Bool).Set(True)
    prim.CreateAttribute("user:collision_proxy_height_cap", Sdf.ValueTypeNames.Float).Set(float(cap_h))

    print(f"[COLLISION PROXY] {proxy_path} scale=({sx:.3f},{sy:.3f},{cap_h:.3f})")
    return proxy_path


def bbox_height_for_path(stage, path: str) -> float:
    bb = bbox_for_prim(stage, path)
    if bb is None:
        return float(BOX_SIZE[2])
    mn, mx = bb
    return max(0.0, float(mx[2]) - float(mn[2]))


def apply_height_collision_policy(stage, box_path: str) -> dict:
    h = bbox_height_for_path(stage, box_path)
    info = {"height": float(h), "tall": bool(h > float(args.max_sortable_box_height)), "policy": str(args.tall_box_policy)}

    if not info["tall"]:
        print(f"[HEIGHT GUARD] {box_path} height={h:.3f} <= max={float(args.max_sortable_box_height):.3f} ok")
        return info

    print(
        f"[HEIGHT GUARD] {box_path} height={h:.3f} > max={float(args.max_sortable_box_height):.3f} "
        f"policy={args.tall_box_policy}"
    )

    if str(args.tall_box_policy) == "proxy":
        if bool(args.disable_asset_collisions_for_proxy):
            n = set_collision_enabled_recursive(stage, box_path, False)
            print(f"[HEIGHT GUARD] disabled asset collisions under {box_path}: {n}")
        add_collision_proxy_cube(stage, box_path, h)
        info["proxy"] = True
    return info


def read_box_qr_payload(stage, box_path: str) -> str:
    prim = stage.GetPrimAtPath(box_path)
    if not prim.IsValid():
        return ""
    attr = prim.GetAttribute("user:qr_payload")
    if attr and attr.IsValid():
        try:
            return str(attr.Get() or "")
        except Exception:
            return ""
    return ""


def infer_target_from_box_qr(stage, box_path: str, fallback_target: str) -> Tuple[str, str]:
    payload = read_box_qr_payload(stage, box_path)
    if not payload:
        return fallback_target, ""
    target = target_for_qr_payload(payload, fallback=fallback_target)
    return target, payload




def parse_qr_payload_date(payload: str) -> str:
    m = re.search(r"QR_(\d{8})_(\d{3})", str(payload or ""))
    if not m:
        return ""
    return m.group(1)


def format_yyyymmdd(date_str: str) -> str:
    d = str(date_str or "")
    if re.match(r"^\d{8}$", d):
        return f"{d[0:4]}-{d[4:6]}-{d[6:8]}"
    return d or "unknown"


ROBOT1_QR_CAMERA_SENSOR = None


def parse_resolution(s: str):
    parts = [p.strip() for p in str(s).split(",")]
    if len(parts) != 2:
        return (640, 480)
    return (max(32, int(float(parts[0]))), max(32, int(float(parts[1]))))


def create_robot1_qr_camera(stage):
    """
    v11:
    Creates USD camera prim and tries to create an Isaac camera sensor object for RGB capture.
    """
    global ROBOT1_QR_CAMERA_SENSOR
    if not bool(getattr(args, "robot1_camera_enabled", True)):
        return ""

    try:
        ensure_xform(stage, CAMERA_ROOT)
        cam_path = str(args.robot1_camera_path)
        remove_prim(stage, cam_path)
        cam = UsdGeom.Camera.Define(stage, cam_path)
        prim = cam.GetPrim()
        set_local_pos(prim, parse_vec3(args.robot1_camera_pos))

        # Default USD camera looks along local -Z.
        # With no rotation at high +Z, this is a top-down camera in world coordinates.
        cam.CreateFocalLengthAttr(float(args.robot1_camera_focal_length))
        aperture = float(getattr(args, "robot1_camera_horizontal_aperture", 18.0))
        cam.CreateHorizontalApertureAttr(aperture)
        cam.CreateVerticalApertureAttr(aperture)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
        prim.CreateAttribute("user:camera_role", Sdf.ValueTypeNames.String).Set("robot1_top_qr_reader")
        print(
            f"[QR CAMERA CREATE] robot1 camera path={cam_path} pos={args.robot1_camera_pos} "
            f"focal={float(args.robot1_camera_focal_length):.1f} aperture={aperture:.1f}"
        )

        ROBOT1_QR_CAMERA_SENSOR = None
        if IsaacCamera is not None and np is not None:
            try:
                res = parse_resolution(args.robot1_camera_resolution)
                ROBOT1_QR_CAMERA_SENSOR = IsaacCamera(
                    prim_path=cam_path,
                    resolution=res,
                )
                ROBOT1_QR_CAMERA_SENSOR.initialize()
                print(f"[QR CAMERA SENSOR] initialized IsaacCamera path={cam_path} resolution={res}")
            except Exception as e:
                ROBOT1_QR_CAMERA_SENSOR = None
                print(f"[QR CAMERA SENSOR][WARN] init failed. USD camera remains. reason={e}")
        else:
            print("[QR CAMERA SENSOR][WARN] IsaacCamera or numpy unavailable. Real image decode may fallback.")

        return cam_path
    except Exception as e:
        print(f"[QR CAMERA][WARN] create failed: {e}")
        return ""



def capture_robot1_camera_rgba():
    """
    Returns RGBA/RGB ndarray if IsaacCamera capture works.
    """
    global ROBOT1_QR_CAMERA_SENSOR
    if ROBOT1_QR_CAMERA_SENSOR is None or np is None:
        return None

    for _ in range(max(0, int(args.qr_decode_warmup_frames))):
        simulation_app.update()

    for attempt in range(1, max(1, int(args.qr_decode_max_attempts)) + 1):
        try:
            simulation_app.update()
            frame = ROBOT1_QR_CAMERA_SENSOR.get_rgba()
            if frame is None:
                continue
            arr = np.asarray(frame)
            if arr.size == 0:
                continue
            print(f"[QR CAMERA CAPTURE] attempt={attempt} shape={arr.shape}")
            return arr
        except Exception as e:
            print(f"[QR CAMERA CAPTURE][WARN] attempt={attempt} failed: {e}")
            simulation_app.update()
    return None


def decode_qr_from_image_array(img):
    """
    Decode QR using OpenCV QRCodeDetector.
    """
    if img is None or cv2 is None or np is None:
        return "", "unavailable"

    try:
        arr = np.asarray(img)
        if arr.ndim == 3 and arr.shape[-1] == 4:
            bgr = cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_RGBA2BGR)
        elif arr.ndim == 3 and arr.shape[-1] == 3:
            bgr = cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_RGB2BGR)
        else:
            return "", "bad_image_shape"

        detector = cv2.QRCodeDetector()
        data, points, _ = detector.detectAndDecode(bgr)
        if data:
            return str(data), "opencv_qrcode_detector"

        # Try grayscale + threshold as a second little crowbar.
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        data2, points2, _ = detector.detectAndDecode(th)
        if data2:
            return str(data2), "opencv_qrcode_detector_threshold"

        return "", "opencv_no_decode"
    except Exception as e:
        return "", f"opencv_exception:{e}"


def maybe_save_qr_debug_image(img, box_id: str):
    if not bool(getattr(args, "qr_decode_save_debug", False)):
        return ""
    if img is None or cv2 is None or np is None:
        return ""
    try:
        out_dir = Path(str(args.qr_decode_debug_dir))
        out_dir.mkdir(parents=True, exist_ok=True)
        out = out_dir / f"{box_id}_robot1_qr_camera.png"
        arr = np.asarray(img)
        if arr.ndim == 3 and arr.shape[-1] == 4:
            bgr = cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_RGBA2BGR)
        elif arr.ndim == 3 and arr.shape[-1] == 3:
            bgr = cv2.cvtColor(arr.astype(np.uint8), cv2.COLOR_RGB2BGR)
        else:
            return ""
        cv2.imwrite(str(out), bgr)
        print(f"[QR CAMERA DEBUG] saved={out}")
        return str(out)
    except Exception as e:
        print(f"[QR CAMERA DEBUG][WARN] save failed: {e}")
        return ""


def robot1_camera_read_qr(stage, box_rec: dict) -> dict:
    """
    v11 integrated QR reader:
    1) Try real camera RGB capture + OpenCV QRCodeDetector.
    2) If real decode fails, fallback to box user:qr_payload so the logistics demo does not stall.
    """
    box_path = box_rec.get("box_path", "")
    box_id = str(box_rec.get("box_id", "BOX_UNKNOWN"))
    decoded_payload = ""
    read_source = "none"
    debug_image = ""

    if bool(getattr(args, "qr_real_decode", True)):
        img = capture_robot1_camera_rgba()
        debug_image = maybe_save_qr_debug_image(img, box_id)
        decoded_payload, read_source = decode_qr_from_image_array(img)
        if decoded_payload:
            print(f"[QR CAMERA DECODE][REAL OK] box={box_id} payload={decoded_payload} source={read_source}")
        else:
            print(f"[QR CAMERA DECODE][REAL FAIL] box={box_id} reason={read_source}")

    payload = decoded_payload
    if not payload:
        payload = read_box_qr_payload(stage, box_path)
        read_source = "fallback:user:qr_payload"
    if not payload:
        payload = str(box_rec.get("qr_payload", "") or "")
        read_source = "fallback:box_rec"

    package_csv_rec = package_route_record_for_qr_payload(payload) or {}
    target = target_for_qr_payload(payload, fallback=str(box_rec.get("requested_target", "day3")))

    if package_csv_rec:
        ship_date = str(package_csv_rec.get("route_zone", "") or "unknown")
        ship_date_raw = date_to_compact(ship_date)
        package_id = str(package_csv_rec.get("package_id", "") or ("PKG_" + payload[3:] if str(payload).startswith("QR_") else payload))
        customer_name = str(package_csv_rec.get("customer_name", "") or "")
        csv_route_zone = str(package_csv_rec.get("route_zone", "") or "")
        print(
            f"[PACKAGE CSV ROUTE] qr_id={payload} package_id={package_id} "
            f"route_zone={csv_route_zone} today_date={normalize_date_iso(getattr(args, 'today_date', '2026-06-08'))} -> target={target}"
        )
    else:
        ship_date_raw = parse_qr_payload_date(payload)
        ship_date = format_yyyymmdd(ship_date_raw)
        package_id = "PKG_" + str(payload)[3:] if str(payload).startswith("QR_") else str(payload or box_id)
        customer_name = ""
        csv_route_zone = ""
        print(f"[PACKAGE CSV ROUTE][MISS] qr_id={payload}. Using legacy ship_date={ship_date} target={target}")

    info = {
        "item_index": int(box_rec.get("box_idx", 0)) + 1,
        "box_id": box_id,
        "qr_payload": str(payload),
        "package_id": package_id,
        "customer_name": customer_name,
        "route_zone": csv_route_zone,
        "ship_date_raw": ship_date_raw,
        "ship_date": ship_date,
        "target": target,
        "qr_read_source": read_source,
        "qr_real_decode_ok": bool(decoded_payload),
        "qr_debug_image": debug_image,
    }
    print(
        f"[QR CAMERA READ] item={info['item_index']} box={info['box_id']} "
        f"payload={info['qr_payload']} package_id={info.get('package_id', '')} "
        f"route_zone={info.get('route_zone', '') or info['ship_date']} ship_date={info['ship_date']} target={info['target']} "
        f"source={info['qr_read_source']} real_decode_ok={int(info['qr_real_decode_ok'])}"
    )
    return info


def log_current_item_status(info: dict, status: str, success=None):
    msg = (
        f"[ITEM STATUS] item={info.get('item_index')} "
        f"box={info.get('box_id')} "
        f"ship_date={info.get('ship_date')} "
        f"target={info.get('target')} "
        f"qr_source={info.get('qr_read_source', '?')} "
        f"status={status}"
    )
    if success is not None:
        msg += f" success={int(bool(success))}"
    print(msg)





def target_to_sg2_line_for_transit_service(target: str) -> str:
    target = str(target or "")
    if target == "today":
        return "sg2_in_01"
    if target == "day2":
        return "sg2_in_02"
    if target == "day3":
        return "sg2_in_03"
    return "sg2_in_03"


def package_id_for_transit_service(qr_info: dict, box_rec: dict) -> str:
    csv_package_id = str(qr_info.get("package_id", "") or box_rec.get("package_id", "") or "").strip()
    if csv_package_id:
        return csv_package_id

    payload = str(qr_info.get("qr_payload", "") or box_rec.get("qr_payload", "") or "")
    csv_rec = package_route_record_for_qr_payload(payload)
    if csv_rec and csv_rec.get("package_id"):
        return str(csv_rec.get("package_id"))

    if payload.startswith("QR_"):
        return "PKG_" + payload[3:]
    if payload.startswith("PKG_"):
        return payload
    return str(qr_info.get("box_id") or box_rec.get("box_id") or "UNKNOWN_PACKAGE")


def log_transit_package_service_call(qr_info: dict, box_rec: dict):
    package_id = package_id_for_transit_service(qr_info, box_rec)
    target_line = target_to_sg2_line_for_transit_service(str(qr_info.get("target", box_rec.get("target", ""))))
    service_name = str(getattr(args, "transit_service_name", "/sim/transit_package"))
    timeout_sec = max(0.5, float(getattr(args, "transit_service_timeout_sec", 5.0)))
    domain_id = str(getattr(args, "transit_ros_domain_id", "119") or "119")
    localhost_only = str(getattr(args, "transit_ros_localhost_only", "0") or "0")
    rmw_impl = str(getattr(args, "transit_rmw_implementation", "rmw_cyclonedds_cpp") or "").strip()
    cyclonedds_uri = str(getattr(args, "transit_cyclonedds_uri", "file:///home/rokey/.ros/cyclonedds_wifi.xml") or "").strip()
    helper_path = str(getattr(args, "transit_helper_path", "/home/rokey/dev_ws/isaac_sim/isaac_step4/transit_package_client_helper.py"))

    print(f"[TRANSIT PACKAGE SERVICE CALL] package_id={package_id} target_line={target_line} service={service_name}")
    print(f"[TRANSIT PACKAGE HELPER CALL] helper={helper_path} domain={domain_id} localhost={localhost_only} rmw={rmw_impl or '<unset>'} cyclonedds={cyclonedds_uri or '<unset>'}")

    if not bool(getattr(args, "transit_service_enabled", True)):
        print("[TRANSIT PACKAGE HELPER SKIP] transit_service_enabled=False")
        return False

    setup_cmds = []
    for setup in str(getattr(args, "transit_ros_setups", "") or "").split(";"):
        setup = setup.strip()
        if setup:
            setup_cmds.append(f"source {shlex.quote(setup)} >/dev/null 2>&1")

    cmd_parts = []
    cmd_parts.append("unset PYTHONHOME PYTHONPATH PYTHONEXECUTABLE PYTHONNOUSERSITE PYTHONUSERBASE VIRTUAL_ENV")
    cmd_parts.append(f"export ROS_DOMAIN_ID={shlex.quote(domain_id)}")
    cmd_parts.append(f"export ROS_LOCALHOST_ONLY={shlex.quote(localhost_only)}")
    if rmw_impl:
        cmd_parts.append(f"export RMW_IMPLEMENTATION={shlex.quote(rmw_impl)}")
    if cyclonedds_uri:
        cmd_parts.append(f"export CYCLONEDDS_URI={shlex.quote(cyclonedds_uri)}")
    cmd_parts.extend(setup_cmds)
    cmd_parts.append(
        " ".join([
            "/usr/bin/python3",
            shlex.quote(helper_path),
            "--service", shlex.quote(service_name),
            "--package-id", shlex.quote(package_id),
            "--target-line", shlex.quote(target_line),
            "--domain-id", shlex.quote(domain_id),
            "--localhost-only", shlex.quote(localhost_only),
            "--timeout-sec", shlex.quote(f"{timeout_sec:.2f}"),
        ])
    )
    cmd = " && ".join(cmd_parts)

    try:
        result = subprocess.run(
            ["bash", "-lc", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout_sec + 3.0,
            env={
                "HOME": os.environ.get("HOME", "/home/rokey"),
                "USER": os.environ.get("USER", "rokey"),
                "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
                "ROS_DOMAIN_ID": domain_id,
                "ROS_LOCALHOST_ONLY": localhost_only,
                "RMW_IMPLEMENTATION": rmw_impl,
                "CYCLONEDDS_URI": cyclonedds_uri,
                "LANG": "C.UTF-8",
                "LC_ALL": "C.UTF-8",
            },
        )

        out = (result.stdout or "").strip()
        err = (result.stderr or "").strip()

        print(f"[TRANSIT PACKAGE HELPER RETURN] code={result.returncode}")
        if out:
            print("[TRANSIT PACKAGE HELPER STDOUT]")
            print(out)
        if err:
            print("[TRANSIT PACKAGE HELPER STDERR]")
            print(err)

        if result.returncode == 0:
            print(f"[TRANSIT PACKAGE HELPER OK] package_id={package_id} target_line={target_line}")
            return True

        print(f"[TRANSIT PACKAGE HELPER FAIL] package_id={package_id} target_line={target_line}")
        return False

    except subprocess.TimeoutExpired as e:
        print(f"[TRANSIT PACKAGE HELPER TIMEOUT] timeout={timeout_sec + 3.0:.1f}s")
        if getattr(e, "stdout", None):
            print("[TRANSIT PACKAGE HELPER TIMEOUT STDOUT]")
            print(str(e.stdout).strip())
        if getattr(e, "stderr", None):
            print("[TRANSIT PACKAGE HELPER TIMEOUT STDERR]")
            print(str(e.stderr).strip())
        return False
    except Exception as e:
        print(f"[TRANSIT PACKAGE HELPER ERROR] {type(e).__name__}: {e}")
        return False


def log_arrived_box_and_despawn(stage, box_rec: dict, qr_info: dict, gate_ok: bool):
    wait_sec = max(0.0, float(getattr(args, "despawn_after_final_gate_sec", 3.0)))
    if wait_sec > 0:
        sim_wait_seconds(wait_sec)

    box_path = str(box_rec.get("box_path", ""))
    print(
        f"[ARRIVED BOX LOG] item={qr_info.get('item_index')} "
        f"box={qr_info.get('box_id')} "
        f"ship_date={qr_info.get('ship_date')} "
        f"target={qr_info.get('target')} "
        f"qr_source={qr_info.get('qr_read_source', '?')} "
        f"final_success={int(bool(gate_ok))} "
        f"path={box_path}"
    )
    log_transit_package_service_call(qr_info, box_rec)
    despawn_box(stage, box_path)
    removed = not stage.GetPrimAtPath(box_path).IsValid()
    print(f"[ARRIVED BOX DESPAWN] box={qr_info.get('box_id')} removed={int(bool(removed))} path={box_path}")
    return bool(removed)


def add_top_qr_visual(stage, box_path: str, qr_png_path: str, qr_payload: str = ""):
    """
    v10:
    Actually creates a visible QR textured mesh plane on the TOP face of the spawned box.

    v09 only stored/used user:qr_payload and left a mostly-white visual marker.
    That was not real camera image recognition.

    This plane is created as a child of the box prim, so it moves with the box.
    """
    if not bool(getattr(args, "add_top_qr_visual", True)):
        return ""
    if not qr_png_path or not Path(str(qr_png_path)).exists():
        print(f"[TOP QR][WARN] missing qr png: {qr_png_path}")
        return ""

    try:
        bb = bbox_for_prim(stage, box_path)
        if bb is None:
            print(f"[TOP QR][WARN] bbox failed for {box_path}")
            return ""

        box_prim = stage.GetPrimAtPath(box_path)
        cache = UsdGeom.XformCache()
        parent_world = cache.GetLocalToWorldTransform(box_prim)
        parent_inv = parent_world.GetInverse()

        mn, mx = bb
        cx = (float(mn[0]) + float(mx[0])) * 0.5
        cy = (float(mn[1]) + float(mx[1])) * 0.5
        top_z = float(mx[2]) + float(args.top_qr_z_offset)

        world_center = Gf.Vec3d(cx, cy, top_z)
        local_center = parent_inv.Transform(world_center)

        qr_path = f"{box_path}/TopQR_Visual"
        safe_name = re.sub(r"[^A-Za-z0-9_]+", "_", str(qr_payload or Path(str(qr_png_path)).stem))
        mat_path = f"{box_path}/TopQR_Material_{safe_name}"
        pbr_path = f"{mat_path}/PreviewSurface"
        tex_path = f"{mat_path}/QRTexture"
        st_path = f"{mat_path}/PrimvarReader_st"

        remove_prim(stage, qr_path)
        remove_prim(stage, mat_path)

        s = float(args.top_qr_size)
        lc = Gf.Vec3d(local_center)

        mesh = UsdGeom.Mesh.Define(stage, qr_path)
        mesh.CreatePointsAttr([
            Gf.Vec3f(float(lc[0] - s * 0.5), float(lc[1] - s * 0.5), float(lc[2])),
            Gf.Vec3f(float(lc[0] + s * 0.5), float(lc[1] - s * 0.5), float(lc[2])),
            Gf.Vec3f(float(lc[0] + s * 0.5), float(lc[1] + s * 0.5), float(lc[2])),
            Gf.Vec3f(float(lc[0] - s * 0.5), float(lc[1] + s * 0.5), float(lc[2])),
        ])
        mesh.CreateFaceVertexCountsAttr([4])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
        mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)])
        mesh.SetNormalsInterpolation("constant")

        pv = mesh.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
        pv.Set([
            Gf.Vec2f(0.0, 0.0),
            Gf.Vec2f(1.0, 0.0),
            Gf.Vec2f(1.0, 1.0),
            Gf.Vec2f(0.0, 1.0),
        ])

        prim = mesh.GetPrim()
        prim.CreateAttribute("user:top_qr_visual", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("user:qr_payload", Sdf.ValueTypeNames.String).Set(str(qr_payload or ""))
        prim.CreateAttribute("user:qr_png", Sdf.ValueTypeNames.String).Set(str(qr_png_path))

        mat = UsdShade.Material.Define(stage, mat_path)

        pbr = UsdShade.Shader.Define(stage, pbr_path)
        pbr.CreateIdAttr("UsdPreviewSurface")
        pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
        pbr.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

        st_reader = UsdShade.Shader.Define(stage, st_path)
        st_reader.CreateIdAttr("UsdPrimvarReader_float2")
        st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")

        tex = UsdShade.Shader.Define(stage, tex_path)
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(qr_png_path))
        tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_reader.ConnectableAPI(), "result")
        pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")

        mat.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(prim).Bind(mat)

        print(
            f"[TOP QR] visible textured QR attached box={box_path} "
            f"qr={qr_payload} png={qr_png_path} local_center=({float(lc[0]):+.3f},{float(lc[1]):+.3f},{float(lc[2]):+.3f}) size={s:.3f}"
        )
        return qr_path
    except Exception as e:
        print(f"[TOP QR][WARN] failed for {box_path}: {e}")
        return ""


def create_box(stage, box_id: str, target: str, pos: Gf.Vec3d, package_record: Optional[dict] = None) -> str:
    ensure_xform(stage, TEST_ROOT)
    ensure_xform(stage, BOX_ROOT)

    path = f"{BOX_ROOT}/{box_id}_{target}"
    remove_prim(stage, path)

    package_record = package_record or None
    used_asset = False

    if package_record and package_record.get("asset_path") and Path(package_record["asset_path"]).exists():
        prim = UsdGeom.Xform.Define(stage, path).GetPrim()
        prim.GetReferences().AddReference(str(package_record["asset_path"]))
        set_local_pos(prim, pos)
        set_local_scale(prim, parse_vec3(args.asset_box_scale))
        force_box_qr_face_up(stage, path, str(getattr(args, "box_qr_face_up", "y_neg")))
        used_asset = True

        # Ensure referenced root participates as one rigid body. If child asset already has rigid body, this is harmless enough.
        try:
            rb = UsdPhysics.RigidBodyAPI.Apply(prim)
            rb.CreateRigidBodyEnabledAttr(True)
            rb.CreateKinematicEnabledAttr(False)
        except Exception as e:
            print(f"[ASSET][WARN] RigidBodyAPI on asset root failed: {e}")

        try:
            mass = UsdPhysics.MassAPI.Apply(prim)
            mass.CreateMassAttr(float(args.box_mass))
        except Exception:
            pass

        qr_payload = str(package_record.get("payload", ""))
        qr_path = str(package_record.get("qr_path", ""))
        prim.CreateAttribute("user:package_asset", Sdf.ValueTypeNames.String).Set(str(package_record["asset_path"]))
        prim.CreateAttribute("user:qr_png", Sdf.ValueTypeNames.String).Set(qr_path)
        prim.CreateAttribute("user:qr_payload", Sdf.ValueTypeNames.String).Set(qr_payload)
        prim.CreateAttribute("user:asset_target", Sdf.ValueTypeNames.String).Set(str(package_record.get("target", target)))

        print(
            f"[SPAWN ASSET] {path} asset={Path(package_record['asset_path']).name} "
            f"qr={qr_payload} expected_target={package_record.get('target', target)} "
            f"pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})"
        )

        # Let reference compose before bbox/height policy.
        sim_steps(3)

        # v12: box_assets already contain QR. Legacy TopQR_Visual is OFF by default.
        # Use --add-top-qr-visual only for old cube/texture experiments.
        add_top_qr_visual(stage, path, qr_path, qr_payload)
        apply_height_collision_policy(stage, path)

    if not used_asset:
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*TARGET_COLORS[target])])

        prim = cube.GetPrim()
        set_local_pos(prim, pos)
        set_local_scale(prim, BOX_SIZE)

        UsdPhysics.CollisionAPI.Apply(prim)

        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.CreateRigidBodyEnabledAttr(True)
        rb.CreateKinematicEnabledAttr(False)

        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr(float(args.box_mass))

        fallback_payload = f"QR_FALLBACK_{box_id}"
        prim.CreateAttribute("user:qr_payload", Sdf.ValueTypeNames.String).Set(fallback_payload)
        print(f"[SPAWN CUBE] {path} target={target} pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})")

    prim = stage.GetPrimAtPath(path)
    prim.CreateAttribute("user:box_id", Sdf.ValueTypeNames.String).Set(box_id)
    prim.CreateAttribute("user:target", Sdf.ValueTypeNames.String).Set(target)
    prim.CreateAttribute("user:step4_source", Sdf.ValueTypeNames.String).Set("finalfac_conveyortrack_dual_robot_sort_qr_v12")
    return path


def despawn_box(stage, path: str):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)
        simulation_app.update()


# =============================================================================
# ROBOT JOINT DISCOVERY + CONTROL
# =============================================================================

def count_joint_names_under_root(stage, root: str) -> int:
    return sum(1 for name in all_bg2_joint_names() if stage.GetPrimAtPath(f"{root}/{name}").IsValid())


def discover_joint_roots(stage) -> List[str]:
    roots = []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if count_joint_names_under_root(stage, p) >= 14:
            roots.append(p)
    # Prefer actual joints-looking path and shallow-ish paths
    roots = sorted(set(roots), key=lambda x: (0 if x.endswith("/joints") else 1, x.count("/"), x))
    return roots


def representative_robot_pos(stage, joint_root: str):
    """
    Print-friendly representative world position for a robot role.
    joint_root itself often has no visible translation, so try nearby parent paths.
    """
    candidates = [str(joint_root)]
    p = str(joint_root).rstrip("/")
    parts = p.split("/")
    for cut in range(len(parts), 1, -1):
        candidates.append("/".join(parts[:cut]))
    for c in candidates:
        if not c:
            continue
        try:
            pos = get_world_pos(stage, c)
            if pos is not None:
                return c, pos
        except Exception:
            pass
    return str(joint_root), None


def resolve_joint_roots(stage) -> Tuple[str, str]:
    r1 = args.robot1_joint_root
    r2 = args.robot2_joint_root

    valid1 = count_joint_names_under_root(stage, r1) >= 14
    valid2 = count_joint_names_under_root(stage, r2) >= 14

    if valid1 and valid2:
        return r1, r2

    roots = discover_joint_roots(stage)
    print(f"[JOINT DISCOVERY] found {len(roots)} candidate root(s)")
    for i, root in enumerate(roots[:10], start=1):
        print(f"  {i:02d}. {root} count={count_joint_names_under_root(stage, root)}/14")

    if not valid1 and len(roots) >= 1:
        r1 = roots[0]
        print(f"[JOINT DISCOVERY] robot1 auto root -> {r1}")
    if not valid2 and len(roots) >= 2:
        r2 = roots[1]
        print(f"[JOINT DISCOVERY] robot2 auto root -> {r2}")

    if count_joint_names_under_root(stage, r1) < 14:
        raise RuntimeError(
            "robot1 joint root not found. Use --dry-run and pass --robot1-joint-root <.../joints>"
        )
    if count_joint_names_under_root(stage, r2) < 14:
        raise RuntimeError(
            "robot2 joint root not found. Use --dry-run and pass --robot2-joint-root <.../joints>"
        )

    return r1, r2


def set_drive_target(stage, joint_path: str, target_deg: float) -> bool:
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        print(f"[JOINT][WARN] missing: {joint_path}")
        return False

    try:
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    except Exception:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")

    drive.CreateTargetPositionAttr(float(target_deg))
    drive.GetTargetPositionAttr().Set(float(target_deg))

    drive.CreateStiffnessAttr(float(DRIVE_STIFFNESS))
    drive.GetStiffnessAttr().Set(float(DRIVE_STIFFNESS))

    drive.CreateDampingAttr(float(DRIVE_DAMPING))
    drive.GetDampingAttr().Set(float(DRIVE_DAMPING))

    try:
        drive.CreateMaxForceAttr(float(DRIVE_MAX_FORCE))
        drive.GetMaxForceAttr().Set(float(DRIVE_MAX_FORCE))
    except Exception:
        pass

    return True


def apply_arm_pose(stage, joint_root: str, arm: str, pose: Dict[str, float]) -> int:
    pose = clamp_pose_for_arm(arm, pose)
    prefix = "arm_l_joint" if arm == "left" else "arm_r_joint"
    count = 0
    for i in range(1, 8):
        name = f"{prefix}{i}"
        if set_drive_target(stage, f"{joint_root}/{name}", float(pose[f"j{i}"])):
            count += 1
    return count


def apply_both_parallel(stage, joint_root: str, label: str) -> int:
    c = apply_arm_pose(stage, joint_root, "left", forward_standby_pose_for_arm("left"))
    c += apply_arm_pose(stage, joint_root, "right", forward_standby_pose_for_arm("right"))
    print(f"[ROBOT] {label} init3/return standby {c}/14")
    return c


def apply_both_init_pose(stage, joint_root: str, label: str, init_idx: int) -> int:
    init_idx = int(init_idx)
    c = apply_arm_pose(stage, joint_root, "left", init_standby_pose_for_arm("left", init_idx))
    c += apply_arm_pose(stage, joint_root, "right", init_standby_pose_for_arm("right", init_idx))
    if init_idx == 1:
        detail = "left(j3=0,j4=0) right(j3=0,j4=0,j5=+0.70)"
    elif init_idx == 2:
        detail = "left(j3=+90,j4=-90) right(j3=-90,j4=-90,j5=+0.70)"
    else:
        detail = "return/forward pose"
    print(f"[ROBOT] {label} INIT{init_idx} applied {c}/14 {detail}")
    return c


def apply_both_side_parallel(stage, joint_root: str, label: str) -> int:
    # Compatibility wrapper. This is now init1, not the old side pose.
    return apply_both_init_pose(stage, joint_root, label, 1)


def transition_both_side_to_forward(stage, joint_root: str, label: str):
    """
    User requested staged initial motion:
      init1 -> init2 -> init3(return pose)

    Do not interpolate directly from init1 to init3.
    That direct sweep made the hands move forward and collide with the conveyor.
    After the first staged transition, the robot normally stays/returns at init3.
    """
    if not hasattr(transition_both_side_to_forward, "_done_roots"):
        transition_both_side_to_forward._done_roots = set()

    key = str(joint_root)
    if key in transition_both_side_to_forward._done_roots:
        apply_both_parallel(stage, joint_root, label)
        sim_steps(args.settle_steps)
        return

    for init_idx in (1, 2, 3):
        apply_both_init_pose(stage, joint_root, label, init_idx)
        sim_steps(args.settle_steps)

    transition_both_side_to_forward._done_roots.add(key)


def apply_active_keep_other_parallel(stage, joint_root: str, active_arm: str, active_pose: Dict[str, float]) -> Tuple[int, int]:
    hold_arm = other_arm(active_arm)
    hold_count = apply_arm_pose(stage, joint_root, hold_arm, forward_standby_pose_for_arm(hold_arm))
    active_count = apply_arm_pose(stage, joint_root, active_arm, active_pose)
    return active_count, hold_count

# =============================================================================
# COPIED ROBOT MOTION EXECUTION
# =============================================================================

def tap_data_for_route(route: str):
    if route == "today":
        return {
            "arm": "right",
            "return_pose": clamp_pose_for_arm("right", INIT3_RIGHT_RAW),
            "sequence": [
                clamp_pose_for_arm("right", RIGHT_TAP_PRE_1_RAW),
                clamp_pose_for_arm("right", RIGHT_TAP_PRE_2_RAW),
                clamp_pose_for_arm("right", RIGHT_TAP_END_1_RAW),
                clamp_pose_for_arm("right", RIGHT_TAP_END_2_RAW),
            ],
        }
    if route == "not_today":
        return {
            "arm": "left",
            "return_pose": clamp_pose_for_arm("left", INIT3_LEFT_RAW),
            "sequence": [
                clamp_pose_for_arm("left", LEFT_TAP_PRE_1_RAW),
                clamp_pose_for_arm("left", LEFT_TAP_PRE_2_RAW),
                clamp_pose_for_arm("left", LEFT_TAP_END_1_RAW),
                clamp_pose_for_arm("left", LEFT_TAP_END_2_RAW),
            ],
        }
    raise ValueError(route)


def drag_data_for_route(route: str):
    """
    v06 semantic arm mapping fixed to user's scenario.

    route=today:
      LEFT arm push/sort.

    route=not_today:
      RIGHT arm push/sort.

    This fixes the previous bug where day2/day3 used not_today but the left arm moved,
    causing the sequence to look like "today/left then robot2".
    """
    if route == "today":
        return {
            "arm": "left",
            "return_pose": clamp_pose_for_arm("left", LEFT_DRAG_RETURN_RAW),
            "sequence": [clamp_pose_for_arm("left", p) for p in NOT_TODAY_LEFT_DRAG_APPROACH_WAYPOINTS_RAW],
            "labels": ["pre1", "pre2", "end1", "mid_initial3", "body_high_clear", "edge_clear_high", "pre3", "end2"],
        }
    if route == "not_today":
        return {
            "arm": "right",
            "return_pose": clamp_pose_for_arm("right", RIGHT_DRAG_RETURN_RAW),
            "sequence": [clamp_pose_for_arm("right", p) for p in TODAY_RIGHT_DRAG_APPROACH_WAYPOINTS_RAW],
            "labels": ["pre1", "pre2", "end1", "mid_initial3", "body_high_clear", "edge_clear_high", "pre3", "end2"],
        }
    raise ValueError(route)


def box_xy_center(stage, box_path: str) -> Optional[Tuple[float, float]]:
    bb = sort_bbox_tuple_for_path(stage, box_path)
    if bb is None:
        return None
    return sort_bbox_xy_center(bb)


def xy_dist(a, b) -> float:
    if a is None or b is None:
        return 0.0
    return math.sqrt((float(a[0]) - float(b[0])) ** 2 + (float(a[1]) - float(b[1])) ** 2)


def make_motion_completion_fn(stage, box_path: str, start_xy, label: str):
    updates = {"n": 0, "done": False}
    if not bool(args.motion_complete_check) or not box_path:
        return None

    threshold = max(0.0, float(args.motion_complete_xy_displacement))
    min_updates = max(0, int(args.motion_complete_min_updates))

    def _check():
        if updates["done"]:
            return True
        updates["n"] += 1
        if updates["n"] < min_updates:
            return False
        cur_xy = box_xy_center(stage, box_path)
        d = xy_dist(start_xy, cur_xy)
        if d >= threshold:
            updates["done"] = True
            print(
                f"[UNUSED_MOTION_COMPLETE_DISABLED] {label} disp={d:.3f} >= {threshold:.3f} "
                f"start_xy=({start_xy[0]:+.3f},{start_xy[1]:+.3f}) "
                f"cur_xy=({cur_xy[0]:+.3f},{cur_xy[1]:+.3f})"
                if start_xy and cur_xy else
                f"[UNUSED_MOTION_COMPLETE_DISABLED] {label} disp={d:.3f} >= {threshold:.3f}"
            )
            return True
        return False

    return _check


def interpolate_active(
    stage,
    joint_root: str,
    arm: str,
    start_pose: Dict[str, float],
    end_pose: Dict[str, float],
    n: int,
    label: str,
    completion_fn=None,
):
    """
    v22:
    completion_fn is accepted for old call compatibility but intentionally ignored.
    No mid-motion abort.
    """
    last_pose = dict(start_pose)
    for i in range(1, max(1, n) + 1):
        t = i / max(1, n)
        pose = lerp_pose(start_pose, end_pose, t)
        active, hold = apply_active_keep_other_parallel(stage, joint_root, arm, pose)
        last_pose = pose
        if i == 1 or i == n:
            print(f"[CMD] {label} {i:03d}/{n:03d} arm={arm} active={active}/7 hold={hold}/7")
        sim_steps(args.motion_steps)

    sim_steps(args.settle_steps)
    return False, last_pose


def follow_waypoints_from_parallel(stage, joint_root: str, arm: str, waypoints: List[Dict[str, float]], interp: int, label: str):
    prev = safe_up_pose_for_arm(arm)
    for wi, target in enumerate(waypoints, start=1):
        interpolate_active(stage, joint_root, arm, prev, target, interp, f"{label}_via_{wi:02d}")
        prev = target


def reverse_retreat_to_parallel(stage, joint_root: str, arm: str, start_pose: Dict[str, float], reverse_waypoints: List[Dict[str, float]], interp: int, label: str):
    prev = start_pose
    for wi, target in enumerate(reverse_waypoints, start=1):
        interpolate_active(stage, joint_root, arm, prev, target, interp, f"{label}_retreat_{wi:02d}")
        prev = target


def reverse_return_path(stage, joint_root: str, arm: str, current_pose: Dict[str, float], visited: List[Dict[str, float]], return_pose: Dict[str, float], interp: int, label: str):
    """
    Current motion-specific return:
      current pose -> visited waypoints in reverse -> return/initial pose.
    """
    return_targets = list(reversed(visited)) + [return_pose]
    prev = current_pose
    for ri, target_pose in enumerate(return_targets, start=1):
        _, prev = interpolate_active(
            stage,
            joint_root,
            arm,
            prev,
            target_pose,
            interp,
            f"{label}_return_{ri:02d}",
            completion_fn=None,
        )


def run_motion_phase(stage, joint_root: str, robot_label: str, route: str, phase_name: str, data: dict, box_path: str = "", completion_start_xy=None) -> bool:
    """
    v23:
    No 판정조건.
    Execute the whole provided motion phase.
    Return directly from final pose to initial3/return pose.
    """
    arm = data["arm"]
    sequence = data["sequence"]
    return_pose = data["return_pose"]

    print(f"[{phase_name.upper()} START] {robot_label} route={route} arm={arm}")
    prev = return_pose
    current_pose = prev

    labels = data.get("labels")
    if not labels:
        labels = ["pre1", "pre2"] + [f"end{i}" for i in range(1, max(1, len(sequence) - 1))]

    for label_name, target_pose in zip(labels, sequence):
        if label_name in {"pre3", "edge_clear_high"}:
            interp = int(args.second_drag_via_interp)
        elif label_name == "end2":
            interp = int(args.second_drag_push_waypoints)
        elif "initial" in label_name or "return" in label_name or "clear" in label_name:
            interp = args.tap_retreat_interp
        else:
            interp = args.drag_via_interp if label_name.startswith("pre") else args.drag_push_waypoints
        _, current_pose = interpolate_active(
            stage,
            joint_root,
            arm,
            prev,
            target_pose,
            interp,
            f"{robot_label}_{route}_{phase_name}_{label_name}",
            completion_fn=None,
        )
        prev = current_pose

    print(f"[{phase_name.upper()} END] {robot_label} route={route} full phase done -> direct return initial3")
    _, _ = interpolate_active(
        stage,
        joint_root,
        arm,
        current_pose,
        return_pose,
        args.tap_retreat_interp,
        f"{robot_label}_{route}_{phase_name}_return_initial3",
        completion_fn=None,
    )
    apply_both_parallel(stage, joint_root, robot_label)
    sim_steps(args.settle_steps)
    return False


def execute_stage3_v17_route_motion(stage, joint_root: str, robot_label: str, route: str, box_path: str = "", completion_start_xy=None):
    """
    v23 drag-only primitive.

    route=today:
      right drag full

    route=not_today:
      left drag full

    No tap.
    No motion-completion judgement.
    No mid-motion abort.
    """
    print("")
    print("=" * 88)
    print(f"[ROBOT MOTION START] {robot_label} route={route} mode=drag_only_full_v27")
    print("=" * 88)

    transition_both_side_to_forward(stage, joint_root, robot_label)

    run_motion_phase(
        stage,
        joint_root,
        robot_label,
        route,
        "drag",
        drag_data_for_route(route),
        box_path=box_path,
        completion_start_xy=None,
    )

    print(f"[ROBOT MOTION DONE] {robot_label} route={route} completed_full_drag_only")


def secondary_stage2_route_for_target(target: str) -> str:
    # v17 Secondary mapping:
    # day2 -> Stage2 today(+Y), day3 -> Stage2 not_today(-Y)
    if target == "day2":
        return "today"
    if target == "day3":
        return "not_today"
    raise ValueError(target)

def primary_stage1_route_for_target(target: str) -> str:
    """
    Stage1 binary split:
      today -> final today lane
      day2/day3 -> not_today branch toward Stage2 robot
    """
    if target == "today":
        return "today"
    if target in {"day2", "day3"}:
        return "not_today"
    raise ValueError(target)


def target_requires_stage2(target: str) -> bool:
    return str(target) in {"day2", "day3"}


def integrated_sort_plan_for_target(target: str) -> dict:
    stage1_route = primary_stage1_route_for_target(target)
    stage2_route = secondary_stage2_route_for_target(target) if target_requires_stage2(target) else None
    return {
        "final_target": target,
        "stage1_route": stage1_route,
        "stage2_route": stage2_route,
        "requires_stage2": target_requires_stage2(target),
    }



# =============================================================================
# OPTIONAL SNAP HELPERS
# =============================================================================

def set_box_kinematic(stage, path: str, enabled: bool):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return
    rb = UsdPhysics.RigidBodyAPI(prim)
    attr = rb.GetKinematicEnabledAttr()
    if not attr:
        attr = rb.CreateKinematicEnabledAttr()
    attr.Set(bool(enabled))


def snap_box_to(stage, path: str, pos: Gf.Vec3d, label: str):
    print(f"[SNAP] {path} -> {label} pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})")
    set_box_kinematic(stage, path, True)
    set_world_translate(stage, path, pos)
    sim_steps(args.settle_steps)
    set_box_kinematic(stage, path, False)


# =============================================================================
# DRY RUN
# =============================================================================

def print_discovery(stage):
    print("")
    print("=" * 88)
    print("[DRY RUN] ConveyorTrack discovery")
    print("=" * 88)
    tracks = discover_conveyor_tracks(stage)
    for i, p in enumerate(tracks[:200], start=1):
        idx = conveyor_track_index_from_name(p.split("/")[-1])
        mark = ""
        if p == args.spawn_track_path:
            mark = "  <spawn explicit>"
        print(f"{i:03d}. idx={idx} path={p}{mark}")
    print(f"[DRY RUN] total tracks found={len(tracks)}")

    print("")
    print("=" * 88)
    print("[DRY RUN] Requested track resolution")
    print("=" * 88)
    for idx in [0, 1, 4, 8, args.spawn_track_index]:
        override = {0: args.track0_path, 1: args.track1_path, 4: args.track4_path}.get(idx, "")
        p = resolve_track_path(stage, idx, override)
        print(f"track {idx}: {p}")

    spawn_path = args.spawn_track_path if stage.GetPrimAtPath(args.spawn_track_path).IsValid() else resolve_track_path(stage, args.spawn_track_index)
    print(f"spawn track final: {spawn_path}")
    if spawn_path:
        print(f"spawn pos estimate: {track_spawn_position(stage, spawn_path)}")

    print("")
    print("=" * 88)
    print("[DRY RUN] BG2-like joint root discovery")
    print("=" * 88)
    roots = discover_joint_roots(stage)
    for i, r in enumerate(roots[:20], start=1):
        print(f"{i:02d}. {r} count={count_joint_names_under_root(stage, r)}/14")
    print(f"default robot1={args.robot1_joint_root} count={count_joint_names_under_root(stage, args.robot1_joint_root)}/14")
    print(f"default robot2={args.robot2_joint_root} count={count_joint_names_under_root(stage, args.robot2_joint_root)}/14")




# =============================================================================
# OVERHEAD CAMERA TRIGGER + CONTINUOUS FEED HELPERS
# =============================================================================

def create_overhead_worktable_cameras(stage, spec: dict, sensor_robot_map: dict):
    if not bool(getattr(args, "overhead_camera_visual", True)):
        return []
    ensure_xform(stage, OVERHEAD_CAMERA_ROOT)
    created = []
    for idx, child in enumerate(sort_trigger_children(spec), start=1):
        center = child.get("center", [0, 0, 0])
        cam_path = f"{OVERHEAD_CAMERA_ROOT}/WorktableCamera_sensor_{idx:02d}"
        remove_prim(stage, cam_path)
        cam = UsdGeom.Camera.Define(stage, cam_path)
        prim = cam.GetPrim()
        set_local_pos(
            prim,
            Gf.Vec3d(float(center[0]), float(center[1]), float(center[2]) + float(args.overhead_camera_z_offset)),
        )
        try:
            cam.CreateFocalLengthAttr(18.0)
            cam.CreateHorizontalApertureAttr(36.0)
            cam.CreateVerticalApertureAttr(36.0)
            cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
        except Exception:
            pass
        mapped = sensor_robot_map.get(int(idx), {})
        prim.CreateAttribute("user:camera_role", Sdf.ValueTypeNames.String).Set("worktable_overhead_sort_camera")
        prim.CreateAttribute("user:sensor_index", Sdf.ValueTypeNames.Int).Set(int(idx))
        prim.CreateAttribute("user:robot_slot", Sdf.ValueTypeNames.Int).Set(int(mapped.get("robot_slot", 0) or 0))
        print(
            f"[CAMERA CREATE] sensor={idx} robot_slot={mapped.get('robot_slot', '?')} "
            f"path={cam_path} pos=({float(center[0]):+.3f},{float(center[1]):+.3f},{float(center[2]) + float(args.overhead_camera_z_offset):+.3f})"
        )
        created.append(cam_path)
    return created


def camera_detect_sensor_for_box(stage, box_path: str, spec: dict):
    hit, xy = detect_sort_start_trigger(stage, box_path, spec)
    if hit:
        return int(get_hit_sensor_idx(spec)), xy
    return 0, xy


def camera_detect_specific_sensor_for_box(stage, box_path: str, spec: dict, sensor_idx: int):
    """
    v32:
    Stage2 must not depend on the first sensor returned by the union detector.
    Check exactly one camera/sensor bbox.
    """
    children = sort_trigger_children(spec)
    idx = int(sensor_idx)
    box_bb = sort_bbox_tuple_for_path(stage, box_path)
    box_xy = sort_bbox_xy_center(box_bb)

    if idx < 1 or idx > len(children):
        print(f"[CAMERA][WARN] requested sensor={idx}, available={len(children)}")
        return False, box_xy if box_xy is not None else (float("nan"), float("nan"))

    child = children[idx - 1]
    hit = sort_xy_center_inside(box_xy, child.get("bbox"), margin=float(args.sort_start_trigger_xy_margin))
    return bool(hit), box_xy if box_xy is not None else (float("nan"), float("nan"))


def robot_info_for_forced_slot(sensor_robot_map: dict, robot_roots_by_slot: dict, sensor_idx: int, forced_slot: int) -> dict:
    """
    forced_slot:
      0 -> use sensor map
      1/2 -> force a physical robot slot
    """
    if int(forced_slot) in {1, 2}:
        slot = int(forced_slot)
        root = robot_roots_by_slot[int(slot)]
        return {
            "sensor_idx": int(sensor_idx),
            "robot_slot": int(slot),
            "joint_root": root,
            "label": f"forced_robot_slot{slot}_sensor{int(sensor_idx)}",
        }
    return sensor_robot_map[int(sensor_idx)]


def maybe_log_camera_wait(box_rec: dict, xy):
    n = int(box_rec.get("camera_wait_count", 0)) + 1
    box_rec["camera_wait_count"] = n
    log_every = max(1, int(getattr(args, "camera_detection_log_every", 120)))
    if n == 1 or n % log_every == 0:
        if xy is None:
            print(f"[CAMERA WAIT] {box_rec.get('box_id')} state={box_rec.get('state')} target={box_rec.get('target')} box_xy=(nan,nan)")
        else:
            print(f"[CAMERA WAIT] {box_rec.get('box_id')} state={box_rec.get('state')} target={box_rec.get('target')} box_xy=({xy[0]:+.3f},{xy[1]:+.3f})")


def spawn_continuous_box(stage, box_idx: int) -> dict:
    box_id = f"BOX_{box_idx + 1:04d}"
    spawn_track = args.spawn_track_path if stage.GetPrimAtPath(args.spawn_track_path).IsValid() else resolve_track_path(stage, args.spawn_track_index)
    if not spawn_track:
        raise RuntimeError("spawn track not found. Use --dry-run and pass --spawn-track-path.")
    spawn_pos = track_spawn_position(stage, spawn_track)
    requested_target = TARGET_SEQUENCE[box_idx % len(TARGET_SEQUENCE)]
    package_record = choose_package_record(box_idx, requested_target)
    box_path = create_box(stage, box_id, requested_target, spawn_pos, package_record=package_record)
    qr_target, qr_payload = infer_target_from_box_qr(stage, box_path, requested_target)
    if qr_payload:
        print(f"[QR READ] {box_id} payload={qr_payload} -> target={qr_target} requested_sequence_target={requested_target}")
    else:
        print(f"[QR READ][WARN] {box_id} no QR payload. fallback target={requested_target}")
    target = requested_target if bool(args.qr_log_only) else qr_target
    height_info = apply_height_collision_policy(stage, box_path)
    skipped = bool(height_info.get("tall")) and str(args.tall_box_policy) == "skip"
    rec = {
        "box_idx": int(box_idx), "box_id": box_id, "box_path": box_path,
        "requested_target": requested_target, "target": target, "qr_payload": qr_payload,
        "state": "skipped_tall_box" if skipped else "stage1_wait_camera",
        "stage1_sensor": 0, "stage2_sensor": 0,
        "stage1_route": None, "stage2_route": None,
        "done": bool(skipped), "spawn_track": spawn_track,
        "spawned_at": time.time(), "camera_wait_count": 0,
    }
    print(f"[SUPPLY SPAWN] {box_id} target={target} path={box_path} spawn_track={spawn_track} keep_on_stage=1")
    return rec


def execute_robot_sort_no_conveyor_stop(stage, conveyor_gate, robot_info: dict, sensor_idx: int, route: str, box_rec: dict, stage_label: str):
    """
    v02:
    Conveyor enable/disable is intentionally not used.
    Robot motion only.
    """
    robot_slot = int(robot_info["robot_slot"])
    robot_label = f"robot_slot{robot_slot}_{stage_label}"
    print(
        f"[SORT EXEC] {box_rec['box_id']} {stage_label} "
        f"robot_slot={robot_slot} route={route} target={box_rec['target']} conveyor_stop=0"
    )
    execute_stage3_v17_route_motion(
        stage,
        robot_info["joint_root"],
        robot_label,
        route,
        box_path=box_rec["box_path"],
        completion_start_xy=None,
    )
    print(f"[SORT EXEC DONE] {box_rec['box_id']} {stage_label} robot_slot={robot_slot} route={route}")


def process_box_by_overhead_camera(stage, conveyor_gate: ConveyorEnableGate, sensor_robot_map: dict, robot_roots_by_slot: dict, all_spec: dict, box_rec: dict) -> bool:
    """
    Returns True when a robot motion was executed.
    Boxes are never despawned here.

    v32:
    Stage1 and Stage2 cameras are checked explicitly:
      stage1 sensor -> robot_slot1 by default
      stage2 sensor -> robot_slot2 by default
    """
    if bool(box_rec.get("done", False)):
        return False

    box_path = box_rec["box_path"]
    if not stage.GetPrimAtPath(box_path).IsValid():
        box_rec["done"] = True
        box_rec["state"] = "missing_prim"
        return False

    target = str(box_rec["target"])
    plan = integrated_sort_plan_for_target(target)

    if box_rec["state"] == "stage1_wait_camera":
        expected_sensor = int(args.stage1_camera_sensor_index)
        hit, xy = camera_detect_specific_sensor_for_box(stage, box_path, all_spec, expected_sensor)
        if not hit:
            maybe_log_camera_wait(box_rec, xy)
            return False

        sensor_idx = expected_sensor
        if sensor_idx not in sensor_robot_map:
            print(f"[CAMERA][WARN] {box_rec['box_id']} stage1 sensor={sensor_idx} not in map")
            return False

        primary = robot_info_for_forced_slot(
            sensor_robot_map,
            robot_roots_by_slot,
            int(sensor_idx),
            int(args.stage1_force_robot_slot),
        )
        route = plan["stage1_route"]

        print(
            f"[CAMERA DETECT] {box_rec['box_id']} STAGE1 sensor={sensor_idx} "
            f"robot_slot={primary['robot_slot']} target={target} route={route} "
            f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f})"
        )
        execute_robot_sort_with_local_feed_stop(
            stage,
            conveyor_gate,
            primary,
            int(sensor_idx),
            route,
            box_rec,
            "stage1",
        )
        box_rec["stage1_sensor"] = int(sensor_idx)
        box_rec["stage1_robot_slot"] = int(primary["robot_slot"])
        box_rec["stage1_route"] = route

        if bool(plan["requires_stage2"]):
            box_rec["state"] = "stage2_wait_camera"
            box_rec["camera_wait_count"] = 0
            print(
                f"[FLOW] {box_rec['box_id']} target={target} "
                f"Stage1 done. Waiting Stage2 camera sensor={int(args.stage2_camera_sensor_index)} robot_slot={int(args.stage2_force_robot_slot) if int(args.stage2_force_robot_slot) else 'sensor_map'}."
            )
        else:
            box_rec["state"] = "sorted_keep_on_stage"
            box_rec["done"] = True
            print(f"[FLOW] {box_rec['box_id']} target=today sorted. Box remains on stage.")
        return True

    if box_rec["state"] == "stage2_wait_camera":
        expected_sensor = int(args.stage2_camera_sensor_index)
        hit, xy = camera_detect_specific_sensor_for_box(stage, box_path, all_spec, expected_sensor)
        if not hit:
            maybe_log_camera_wait(box_rec, xy)
            return False

        sensor_idx = expected_sensor
        if sensor_idx not in sensor_robot_map:
            print(f"[CAMERA][WARN] {box_rec['box_id']} stage2 sensor={sensor_idx} not in map")
            return False

        secondary = robot_info_for_forced_slot(
            sensor_robot_map,
            robot_roots_by_slot,
            int(sensor_idx),
            int(args.stage2_force_robot_slot),
        )
        route = plan["stage2_route"]

        print(
            f"[CAMERA DETECT] {box_rec['box_id']} STAGE2 sensor={sensor_idx} "
            f"robot_slot={secondary['robot_slot']} target={target} route={route} "
            f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f})"
        )
        execute_robot_sort_with_local_feed_stop(
            stage,
            conveyor_gate,
            secondary,
            int(sensor_idx),
            route,
            box_rec,
            "stage2",
        )
        box_rec["stage2_sensor"] = int(sensor_idx)
        box_rec["stage2_robot_slot"] = int(secondary["robot_slot"])
        box_rec["stage2_route"] = route
        box_rec["state"] = "sorted_keep_on_stage"
        box_rec["done"] = True
        print(f"[FLOW] {box_rec['box_id']} target={target} Stage2 sorted by robot_slot{secondary['robot_slot']}. Box remains on stage.")
        return True

    return False



def make_time_control_robot_info(robot_roots_by_slot: dict, slot: int, sensor_idx: int = 0) -> dict:
    slot = int(slot)
    return {
        "sensor_idx": int(sensor_idx),
        "robot_slot": slot,
        "joint_root": robot_roots_by_slot[slot],
        "label": f"time_control_robot_slot{slot}",
    }



def final_gate_index_for_target(target: str) -> int:
    target = str(target)
    if target == "today":
        return int(args.final_gate_today_index)
    if target == "day2":
        return int(args.final_gate_day2_index)
    if target == "day3":
        return int(args.final_gate_day3_index)
    return int(args.final_gate_day3_index)


def final_gate_override_for_target(target: str) -> str:
    target = str(target)
    if target == "today":
        return str(args.final_gate_today_path or "")
    if target == "day2":
        return str(args.final_gate_day2_path or "")
    if target == "day3":
        return str(args.final_gate_day3_path or "")
    return ""


def make_final_gate_spec(stage, target: str):
    idx = final_gate_index_for_target(target)
    override = final_gate_override_for_target(target)
    path = resolve_track_path(stage, idx, override)
    if not path or not stage.GetPrimAtPath(path).IsValid():
        print(f"[FINAL GATE][WARN] target={target} track index={idx} not resolved override={override}")
        return None

    bb = bbox_for_prim(stage, path + "/Belt") or bbox_for_prim(stage, path)
    if bb is None:
        print(f"[FINAL GATE][WARN] bbox failed target={target} path={path}")
        return None

    mn, mx = bb
    spec = {
        "target": str(target),
        "track_index": int(idx),
        "track_path": path,
        "bbox": (
            (float(mn[0]), float(mn[1]), float(mn[2])),
            (float(mx[0]), float(mx[1]), float(mx[2])),
        ),
    }
    print(
        f"[FINAL GATE SPEC] target={target} track=ConveyorTrack_{idx:02d} path={path} "
        f"x=[{float(mn[0]):+.3f},{float(mx[0]):+.3f}] "
        f"y=[{float(mn[1]):+.3f},{float(mx[1]):+.3f}]"
    )
    return spec


def bbox_xy_overlap(bb_a, bb_b, margin: float = 0.0) -> bool:
    if bb_a is None or bb_b is None:
        return False
    a_mn, a_mx = bb_a
    b_mn, b_mx = bb_b
    ax0, ay0 = float(a_mn[0]), float(a_mn[1])
    ax1, ay1 = float(a_mx[0]), float(a_mx[1])
    bx0, by0 = float(b_mn[0]) - float(margin), float(b_mn[1]) - float(margin)
    bx1, by1 = float(b_mx[0]) + float(margin), float(b_mx[1]) + float(margin)
    return (ax1 >= bx0 and ax0 <= bx1 and ay1 >= by0 and ay0 <= by1)


def bbox_z_overlap(bb_a, bb_b, margin: float = 0.0) -> bool:
    if bb_a is None or bb_b is None:
        return False
    a_mn, a_mx = bb_a
    b_mn, b_mx = bb_b
    az0, az1 = float(a_mn[2]), float(a_mx[2])
    bz0, bz1 = float(b_mn[2]) - float(margin), float(b_mx[2]) + float(margin)
    return az1 >= bz0 and az0 <= bz1


def final_gate_hit(stage, box_path: str, spec: dict) -> tuple:
    """
    v08:
    User said final gate should trigger when the box passes through or even overlaps/grazes it.
    Therefore default mode is bbox XY overlap, not bbox-center-inside.
    """
    box_bb = sort_bbox_tuple_for_path(stage, box_path)
    box_xy = sort_bbox_xy_center(box_bb)

    mode = str(getattr(args, "final_gate_hit_mode", "overlap"))
    if mode == "center":
        hit = sort_xy_center_inside(box_xy, spec.get("bbox"), margin=float(args.final_gate_xy_margin))
    else:
        hit = bbox_xy_overlap(box_bb, spec.get("bbox"), margin=float(args.final_gate_xy_margin))
        if bool(getattr(args, "final_gate_require_z_overlap", False)):
            hit = bool(hit and bbox_z_overlap(box_bb, spec.get("bbox"), margin=0.05))

    return bool(hit), box_xy if box_xy is not None else (float("nan"), float("nan"))


def wait_until_box_reaches_final_gate(stage, box_rec: dict) -> bool:
    if not bool(args.final_gate_enabled):
        print(f"[FINAL GATE SKIP] {box_rec['box_id']} final_gate_enabled=False")
        return True

    target = str(box_rec["target"])
    spec = make_final_gate_spec(stage, target)
    if spec is None:
        print(f"[FINAL GATE SKIP] {box_rec['box_id']} no final gate spec")
        return False

    timeout = max(1, int(args.final_gate_timeout_steps))
    log_every = max(1, int(args.final_gate_log_every))
    max_wait_sec = max(0.05, float(getattr(args, "final_gate_max_wait_sec", 1.0)))
    box_path = box_rec["box_path"]
    final_gate_wait_started = time.time()

    print(
        f"[FINAL GATE WAIT] {box_rec['box_id']} target={target} "
        f"waiting {str(getattr(args, 'final_gate_hit_mode', 'overlap'))} hit on ConveyorTrack_{spec['track_index']:02d} before next spawn"
    )

    for step_i in range(1, timeout + 1):
        hit, xy = final_gate_hit(stage, box_path, spec)
        if hit:
            print(
                f"[FINAL GATE HIT] {box_rec['box_id']} target={target} "
                f"step={step_i}/{timeout} track=ConveyorTrack_{spec['track_index']:02d} "
                f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f})"
            )
            return True

        if time.time() - final_gate_wait_started >= max_wait_sec:
            print(
                f"[FINAL GATE FAST PROCEED] {box_rec['box_id']} target={target} "
                f"elapsed={time.time() - final_gate_wait_started:.2f}s "
                f"track=ConveyorTrack_{spec['track_index']:02d} "
                f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f})"
            )
            return True

        if step_i == 1 or step_i % log_every == 0:
            print(
                f"[FINAL GATE WAIT] {box_rec['box_id']} step={step_i}/{timeout} "
                f"track=ConveyorTrack_{spec['track_index']:02d} "
                f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f})"
            )

        simulation_app.update()

    hit, xy = final_gate_hit(stage, box_path, spec)
    print(
        f"[FINAL GATE TIMEOUT] {box_rec['box_id']} target={target} hit={int(hit)} "
        f"track=ConveyorTrack_{spec['track_index']:02d} box_xy=({xy[0]:+.3f},{xy[1]:+.3f})"
    )
    return bool(hit)



def scenario_robot1_route_for_target(target: str) -> str:
    """
    v06 semantic route mapping:
      QR target=today  -> route=today     -> LEFT arm
      QR target=day2/3 -> route=not_today -> RIGHT arm

    Robot2 is allowed only for day2/day3, i.e. only after robot1 not_today/right-side split.
    """
    target = str(target)
    if target == "today":
        return str(args.robot1_today_route)
    if target in {"day2", "day3"}:
        return str(args.robot1_not_today_route)
    raise ValueError(target)


def scenario_robot2_route_for_target(target: str) -> str:
    """
    v06 semantic route mapping for robot2:
      QR target=day2 -> route=today     -> LEFT arm
      QR target=day3 -> route=not_today -> RIGHT arm
    """
    target = str(target)
    if target == "day2":
        return str(args.robot2_day2_route)
    if target == "day3":
        return str(args.robot2_day3_route)
    return "not_required_today"


def scenario_target_requires_robot2(target: str) -> bool:
    return str(target) in {"day2", "day3"}


def run_simple_two_robot_sort_loop(stage, conveyor_gate, robot_roots_by_slot: dict):
    """
    v03 exact user scenario:
      spawn -> robot1 today/not_today split -> if needed timed robot2 split -> wait final gate -> next spawn.
    """
    active_boxes = []
    spawn_idx = 0
    start = time.time()
    max_boxes = int(getattr(args, "max_boxes", 0))
    max_time = float(getattr(args, "max_time", 0.0))
    spawn_interval = max(0.0, float(getattr(args, "spawn_interval", 1.0)))

    # Fixed mapping requested by user:
    # FFW_BG2 is robot1, FFW_BG2_01 is robot2.
    robot1_slot = 1
    robot2_slot = 2
    robot1 = make_time_control_robot_info(robot_roots_by_slot, robot1_slot, int(args.stage1_camera_sensor_index))
    robot2 = make_time_control_robot_info(robot_roots_by_slot, robot2_slot, int(args.stage2_camera_sensor_index))

    print("")
    print("=" * 96)
    print("[QR CAMERA REAL-DECODE SORT START V11]")
    print("Scenario:")
    print("  robot1 overhead camera reads top QR, then robots choose route from QR info")
    print("  robot1: QR today -> route=today(LEFT), QR day2/day3 -> route=not_today(RIGHT)")
    print("  robot2: QR day2 -> route=today(LEFT), QR day3 -> route=not_today(RIGHT)")
    print("  next box spawns only after current box reaches final destination gate")
    print(f"usd={args.usd}")
    print(f"robot1=/World/FFW_BG2 slot={robot1_slot}")
    print(f"robot2=/World/FFW_BG2_01 slot={robot2_slot}")
    print(f"max_boxes={max_boxes if max_boxes > 0 else 'unlimited'} max_time={max_time if max_time > 0 else 'until_user_stop'}")
    print(f"robot1_start_delay_after_spawn={float(args.robot1_start_delay_after_spawn):.2f}s")
    print(f"robot2_delay_after_robot1_RIGHT_motion_end={float(args.stage2_delay_after_robot1):.2f}s")
    print("conveyor_stop=0")
    print(f"final gates: today=ConveyorTrack_{int(args.final_gate_today_index):02d}, day2=ConveyorTrack_{int(args.final_gate_day2_index):02d}, day3=ConveyorTrack_{int(args.final_gate_day3_index):02d}")
    print("=" * 96)

    try:
        while simulation_app.is_running():
            elapsed = time.time() - start
            if max_time > 0.0 and elapsed >= max_time:
                print(f"[STOP] max_time reached: {elapsed:.1f}s")
                break
            if max_boxes > 0 and spawn_idx >= max_boxes:
                simulation_app.update()
                continue

            # Reaching here means previous cycle fully ended, including final gate hit.
            print("")
            print("#" * 96)
            print(f"[SPAWN GATE OPEN] previous box final-gate complete -> spawn box index={spawn_idx + 1}")
            print("#" * 96)

            rec = spawn_continuous_box(stage, spawn_idx)
            active_boxes.append(rec)
            spawn_idx += 1

            qr_info = robot1_camera_read_qr(stage, rec)
            rec["qr_camera_info"] = qr_info
            target = str(qr_info["target"])
            rec["target"] = target
            rec["ship_date"] = qr_info["ship_date"]
            rec["route_zone"] = qr_info.get("route_zone", qr_info["ship_date"])
            rec["package_id"] = qr_info.get("package_id", "")
            rec["customer_name"] = qr_info.get("customer_name", "")
            rec["qr_payload"] = qr_info["qr_payload"]

            robot1_route = scenario_robot1_route_for_target(target)
            needs_robot2 = scenario_target_requires_robot2(target) and robot1_route == "not_today"
            robot2_route = scenario_robot2_route_for_target(target) if needs_robot2 else "not_required_today"

            log_current_item_status(qr_info, "qr_read_ok_sort_plan_created", success=None)
            print(
                f"[QR PLAN] item={qr_info['item_index']} box={rec['box_id']} QR_target={target} ship_date={qr_info['ship_date']} | "
                f"robot1_route={robot1_route} "
                f"{'(LEFT/today)' if robot1_route == 'today' else '(RIGHT/not_today)'} | "
                f"robot2_route={robot2_route} "
                f"{'(LEFT/day2)' if target == 'day2' and needs_robot2 else '(RIGHT/day3)' if target == 'day3' and needs_robot2 else ''}"
            )

            d1 = max(0.0, float(args.robot1_start_delay_after_spawn))
            if d1 > 0:
                print(f"[ROBOT1 TIMER] {rec['box_id']} waiting {d1:.2f}s after spawn")
                sim_wait_seconds(d1)

            rec["state"] = "robot1_sorting"
            log_current_item_status(qr_info, "robot1_sorting_start", success=None)
            execute_robot_sort_no_conveyor_stop(
                stage,
                conveyor_gate,
                robot1,
                int(args.stage1_camera_sensor_index),
                robot1_route,
                rec,
                "robot1_primary",
            )
            robot1_motion_end = time.time()
            rec["stage1_route"] = robot1_route
            rec["stage1_robot_slot"] = robot1_slot
            rec["stage1_sensor"] = int(args.stage1_camera_sensor_index)
            log_current_item_status(qr_info, "robot1_sorting_done", success=None)

            if needs_robot2:
                delay_after_end = max(0.0, float(args.stage2_delay_after_robot1))
                print(
                    f"[ROBOT2 TIMER] {rec['box_id']} target={target} "
                    f"robot1_right_motion_end={robot1_motion_end:.3f} "
                    f"waiting_after_robot1_end={delay_after_end:.2f}s"
                )
                if delay_after_end > 0:
                    sim_wait_seconds(delay_after_end)

                rec["state"] = "robot2_sorting"
                log_current_item_status(qr_info, "robot2_sorting_start", success=None)
                execute_robot_sort_no_conveyor_stop(
                    stage,
                    conveyor_gate,
                    robot2,
                    int(args.stage2_camera_sensor_index),
                    robot2_route,
                    rec,
                    "robot2_secondary",
                )
                rec["stage2_route"] = robot2_route
                rec["stage2_robot_slot"] = robot2_slot
                rec["stage2_sensor"] = int(args.stage2_camera_sensor_index)
                log_current_item_status(qr_info, "robot2_sorting_done", success=None)
            else:
                rec["stage2_route"] = "not_required_today"
                rec["stage2_robot_slot"] = 0
                rec["stage2_sensor"] = 0
                print(
                    f"[ROBOT2 SKIP] {rec['box_id']} target=today. "
                    f"Robot1 left-push done, now wait today final gate."
                )

            rec["state"] = "waiting_final_gate"
            log_current_item_status(qr_info, "waiting_final_destination", success=None)
            gate_ok = wait_until_box_reaches_final_gate(stage, rec)
            rec["final_gate_ok"] = bool(gate_ok)
            rec["final_gate_track"] = f"ConveyorTrack_{final_gate_index_for_target(target):02d}"
            rec["state"] = "sorted_keep_on_stage"
            rec["done"] = True
            log_current_item_status(qr_info, "final_destination_reached" if gate_ok else "final_destination_timeout", success=gate_ok)
            rec["despawned"] = log_arrived_box_and_despawn(stage, rec, qr_info, gate_ok)

            print(
                f"[CYCLE DONE] {rec['box_id']} target={target} "
                f"robot1={rec.get('stage1_route')}@slot{rec.get('stage1_robot_slot')} "
                f"robot2={rec.get('stage2_route')}@slot{rec.get('stage2_robot_slot')} "
                f"final_gate={rec['final_gate_track']} ok={int(bool(gate_ok))} "
                f"box_kept_on_stage=1"
            )

            if spawn_interval > 0:
                print(f"[NEXT SUPPLY WAIT] {spawn_interval:.2f}s")
                sim_wait_seconds(spawn_interval)

    except KeyboardInterrupt:
        print("[STOP] KeyboardInterrupt")
    finally:
        alive = sum(1 for r in active_boxes if stage.GetPrimAtPath(r.get("box_path", "")).IsValid())
        sorted_count = sum(1 for r in active_boxes if r.get("state") == "sorted_keep_on_stage")
        print("")
        print("=" * 96)
        print("[QR CAMERA REAL-DECODE SORT RESULT V11]")
        print(f"spawned={len(active_boxes)} sorted={sorted_count} alive_on_stage={alive}")
        for r in active_boxes[-30:]:
            print(
                f"{r['box_id']} | target={r['target']:5s} | state={r['state']} | "
                f"robot1={r.get('stage1_route')}@slot{r.get('stage1_robot_slot', '?')} | "
                f"robot2={r.get('stage2_route')}@slot{r.get('stage2_robot_slot', '?')} | "
                f"final_gate={r.get('final_gate_track', '?')} ok={int(bool(r.get('final_gate_ok', False)))}"
            )
        print("=" * 96)


def run_continuous_camera_sort_loop(stage, conveyor_gate, robot_roots_by_slot: dict):
    all_sort_trigger_spec = make_sort_start_trigger_spec(stage, label="work_area_camera")
    create_sort_start_trigger_visual(stage, all_sort_trigger_spec, "work_area_camera")
    sensor_robot_map = build_sensor_robot_map(stage, all_sort_trigger_spec, robot_roots_by_slot)
    create_overhead_worktable_cameras(stage, all_sort_trigger_spec, sensor_robot_map)
    print(
        f"[STAGE CAMERA MAP] stage1_sensor={int(args.stage1_camera_sensor_index)} "
        f"stage1_robot_slot={int(args.stage1_force_robot_slot) if int(args.stage1_force_robot_slot) else 'sensor_map'} | "
        f"stage2_sensor={int(args.stage2_camera_sensor_index)} "
        f"stage2_robot_slot={int(args.stage2_force_robot_slot) if int(args.stage2_force_robot_slot) else 'sensor_map'}"
    )

    active_boxes = []
    spawn_idx = 0
    next_spawn_t = 0.0
    start = time.time()
    max_boxes = int(getattr(args, "max_boxes", 0))
    spawn_interval = max(0.1, float(getattr(args, "spawn_interval", 4.0)))
    max_time = float(getattr(args, "max_time", 0.0))

    print("")
    print("=" * 96)
    print("[CONTINUOUS FEED START]")
    print(f"spawn_interval={spawn_interval:.2f}s max_boxes={max_boxes if max_boxes > 0 else 'unlimited'} max_time={max_time if max_time > 0 else 'until_user_stop'}")
    print("boxes are kept on stage; no final destination wait; no despawn")
    print("=" * 96)

    try:
        while simulation_app.is_running():
            now = time.time()
            elapsed = now - start
            if max_time > 0.0 and elapsed >= max_time:
                print(f"[CONTINUOUS STOP] max_time reached: {elapsed:.1f}s")
                break
            can_spawn = (max_boxes <= 0 or spawn_idx < max_boxes)
            if bool(args.continuous_feed) and can_spawn and now >= next_spawn_t:
                rec = spawn_continuous_box(stage, spawn_idx)
                active_boxes.append(rec)
                spawn_idx += 1
                next_spawn_t = now + spawn_interval

            for rec in active_boxes:
                if process_box_by_overhead_camera(stage, conveyor_gate, sensor_robot_map, robot_roots_by_slot, all_sort_trigger_spec, rec):
                    break

            simulation_app.update()
    except KeyboardInterrupt:
        print("[CONTINUOUS STOP] KeyboardInterrupt")
    finally:
        alive = sum(1 for r in active_boxes if stage.GetPrimAtPath(r.get("box_path", "")).IsValid())
        sorted_count = sum(1 for r in active_boxes if r.get("state") == "sorted_keep_on_stage")
        waiting_count = sum(1 for r in active_boxes if not r.get("done", False))
        print("")
        print("=" * 96)
        print("[CONTINUOUS FEED RESULT V34_CAMERA_MODE]")
        print(f"spawned={len(active_boxes)} sorted={sorted_count} waiting={waiting_count} alive_on_stage={alive}")
        for r in active_boxes[-20:]:
            print(
                f"{r['box_id']} | target={r['target']:5s} | state={r['state']} | "
                f"stage1={r.get('stage1_route')}@sensor{r.get('stage1_sensor')}/robot{r.get('stage1_robot_slot', '?')} | "
                f"stage2={r.get('stage2_route')}@sensor{r.get('stage2_sensor')}/robot{r.get('stage2_robot_slot', '?')}"
            )
        print("=" * 96)


# =============================================================================
# FINAL DESTINATION TRACK WAIT
# =============================================================================

def destination_track_index_for_target(target: str) -> int:
    if target == "today":
        return int(args.destination_track_today_index)
    if target == "day2":
        return int(args.destination_track_day2_index)
    if target == "day3":
        return int(args.destination_track_day3_index)
    return int(DESTINATION_TRACK_DEFAULTS.get(str(target), 5))


def destination_track_override_for_target(target: str) -> str:
    if target == "today":
        return str(args.destination_track_today_path or "")
    if target == "day2":
        return str(args.destination_track_day2_path or "")
    if target == "day3":
        return str(args.destination_track_day3_path or "")
    return ""


def destination_track_path_for_target(stage, target: str) -> Optional[str]:
    idx = destination_track_index_for_target(target)
    override = destination_track_override_for_target(target)
    path = resolve_track_path(stage, idx, override)
    if path and stage.GetPrimAtPath(path).IsValid():
        return path
    print(f"[DEST][WARN] target={target} destination track not resolved. index={idx} override={override}")
    return None


def make_destination_spec(stage, target: str) -> Optional[dict]:
    track_path = destination_track_path_for_target(stage, target)
    if not track_path:
        return None

    bb = bbox_for_prim(stage, track_path + "/Belt") or bbox_for_prim(stage, track_path)
    if bb is None:
        print(f"[DEST][WARN] target={target} destination track bbox failed: {track_path}")
        return None

    mn, mx = bb
    spec = {
        "target": target,
        "track_index": destination_track_index_for_target(target),
        "track_path": track_path,
        "bbox": (
            (float(mn[0]), float(mn[1]), float(mn[2])),
            (float(mx[0]), float(mx[1]), float(mx[2])),
        ),
    }
    print(
        f"[DEST SPEC] target={target} track=ConveyorTrack_{spec['track_index']:02d} "
        f"path={track_path} "
        f"x=[{float(mn[0]):+.3f},{float(mx[0]):+.3f}] "
        f"y=[{float(mn[1]):+.3f},{float(mx[1]):+.3f}]"
    )
    return spec


def destination_hit(stage, box_path: str, spec: dict) -> Tuple[bool, Tuple[float, float]]:
    box_bb = sort_bbox_tuple_for_path(stage, box_path)
    box_xy = sort_bbox_xy_center(box_bb)
    hit = sort_xy_center_inside(box_xy, spec.get("bbox"), margin=float(args.destination_trigger_xy_margin))
    return bool(hit), box_xy if box_xy is not None else (float("nan"), float("nan"))


def wait_for_destination_track(stage, box_path: str, target: str, box_id: str) -> bool:
    if not bool(args.wait_destination):
        print(f"[DEST SKIP] {box_id} target={target} wait_destination=False")
        return True

    spec = make_destination_spec(stage, target)
    if spec is None:
        print(f"[DEST][SKIP] {box_id} target={target} no destination spec")
        return False

    timeout = max(1, int(args.destination_timeout_steps))
    log_every = max(1, int(args.destination_log_every))

    for step_i in range(1, timeout + 1):
        hit, xy = destination_hit(stage, box_path, spec)
        if hit:
            print(
                f"[DEST HIT] {box_id} target={target} step={step_i}/{timeout} "
                f"track=ConveyorTrack_{spec['track_index']:02d} "
                f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f})"
            )
            return True

        if step_i == 1 or step_i % log_every == 0:
            print(
                f"[DEST WAIT] {box_id} target={target} step={step_i}/{timeout} "
                f"track=ConveyorTrack_{spec['track_index']:02d} "
                f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f})"
            )

        simulation_app.update()

    hit, xy = destination_hit(stage, box_path, spec)
    mn, mx = spec["bbox"]
    print(
        f"[DEST TIMEOUT] {box_id} target={target} hit={int(hit)} "
        f"track=ConveyorTrack_{spec['track_index']:02d} "
        f"box_xy=({xy[0]:+.3f},{xy[1]:+.3f}) "
        f"x=[{float(mn[0]):+.3f},{float(mx[0]):+.3f}] "
        f"y=[{float(mn[1]):+.3f},{float(mx[1]):+.3f}]"
    )
    return bool(hit)


# =============================================================================
# MAIN FLOW
# =============================================================================

def run_one_cycle(stage, conveyor_gate: ConveyorEnableGate, robot_roots_by_slot: dict, box_idx: int, target: str):
    box_id = f"BOX_{box_idx + 1:03d}"

    # Resolve spawn track at every cycle so stage edits are reflected.
    if stage.GetPrimAtPath(args.spawn_track_path).IsValid():
        spawn_track = args.spawn_track_path
    else:
        spawn_track = resolve_track_path(stage, args.spawn_track_index)

    if not spawn_track:
        raise RuntimeError(
            "spawn track not found. Use --dry-run and pass --spawn-track-path with exact /World/.../ConveyorTrack path."
        )

    spawn_pos = track_spawn_position(stage, spawn_track)

    requested_target = target
    package_record = choose_package_record(box_idx, requested_target)
    box_path = create_box(stage, box_id, requested_target, spawn_pos, package_record=package_record)

    qr_target, qr_payload = infer_target_from_box_qr(stage, box_path, requested_target)
    if qr_payload:
        print(f"[QR READ] {box_id} payload={qr_payload} -> target={qr_target} requested_sequence_target={requested_target}")
    else:
        print(f"[QR READ][WARN] {box_id} no QR payload. fallback target={requested_target}")

    if bool(args.qr_log_only):
        target = requested_target
        print(f"[QR READ] qr_log_only=True. using requested target={target}")
    else:
        target = qr_target

    height_info = apply_height_collision_policy(stage, box_path)
    if bool(height_info.get("tall")) and str(args.tall_box_policy) == "skip":
        print(f"[HEIGHT GUARD][SKIP] {box_id} tall box. Robot motion skipped to protect gripper.")
        despawn_box(stage, box_path)
        return {
            "box_id": box_id,
            "target": target,
            "primary_route": "skipped_tall_box",
            "secondary_route": None,
            "box_path": box_path,
        }

    all_sort_trigger_spec = make_sort_start_trigger_spec(stage, label="work_area")
    create_sort_start_trigger_visual(stage, all_sort_trigger_spec, "work_area")
    sensor_robot_map = build_sensor_robot_map(stage, all_sort_trigger_spec, robot_roots_by_slot)

    print(f"[FLOW] {box_id} spawned on {spawn_track}. Waiting ANY robot-front sensor")
    any_ready = wait_for_sort_start_trigger(stage, box_path, all_sort_trigger_spec, label=f"{box_id}_any_sensor")
    if not any_ready:
        print(f"[FLOW][SKIP] {box_id} no sensor hit. Robot motion skipped.")
        despawn_box(stage, box_path)
        return {
            "box_id": box_id,
            "target": target,
            "primary_route": "skipped_sensor_timeout",
            "secondary_route": None,
            "box_path": box_path,
        }

    hit_sensor = get_hit_sensor_idx(all_sort_trigger_spec)
    if hit_sensor not in sensor_robot_map:
        raise RuntimeError(f"Hit sensor {hit_sensor} not in sensor_robot_map={sorted(sensor_robot_map.keys())}")

    primary = sensor_robot_map[hit_sensor]
    plan = integrated_sort_plan_for_target(target)
    primary_route = plan["stage1_route"]

    print(
        f"[INTEGRATED PLAN] {box_id} final_target={target} "
        f"stage1_route={plan['stage1_route']} stage2_route={plan['stage2_route']} "
        f"requires_stage2={int(plan['requires_stage2'])}"
    )
    print(
        f"[FLOW][STAGE1] {box_id} sensor={hit_sensor} -> robot_slot={primary['robot_slot']} "
        f"route={primary_route} final_target={target}"
    )
    primary_start_xy = box_xy_center(stage, box_path)
    conveyor_gate.pause_for_robot(int(primary["robot_slot"]))
    execute_stage3_v17_route_motion(
        stage,
        primary["joint_root"],
        f"robot_slot{primary['robot_slot']}_sensor{hit_sensor}",
        primary_route,
        box_path=box_path,
        completion_start_xy=primary_start_xy,
    )
    conveyor_gate.resume_for_robot(int(primary["robot_slot"]))

    if args.snap_after_sort:
        if target == "today":
            snap_box_to(stage, box_path, parse_vec3(args.today_drop_pos), "today_drop")
        else:
            snap_box_to(stage, box_path, parse_vec3(args.robot2_work_pos), "robot2_work")

    secondary_route = None

    # Stage2 robot only for day2/day3.
    # v27: Stage1 handles today/not_today, Stage2 handles day2/day3 final split.
    if bool(plan["requires_stage2"]):
        next_sensor = other_sensor_idx(hit_sensor, all_sort_trigger_spec)
        print(f"[FLOW][STAGE2 WAIT] {box_id} final_target={target} waiting sensor={next_sensor}")

        secondary_ready = wait_for_specific_sensor(
            stage,
            box_path,
            all_sort_trigger_spec,
            next_sensor,
            label=f"{box_id}_secondary_sensor{next_sensor}",
        )

        if secondary_ready:
            secondary = sensor_robot_map[next_sensor]
            secondary_route = plan["stage2_route"]
            print(
                f"[FLOW][STAGE2] {box_id} sensor={next_sensor} -> robot_slot={secondary['robot_slot']} "
                f"route={secondary_route} final_target={target}"
            )

            secondary_start_xy = box_xy_center(stage, box_path)
            conveyor_gate.pause_for_robot(int(secondary["robot_slot"]))
            execute_stage3_v17_route_motion(
                stage,
                secondary["joint_root"],
                f"robot_slot{secondary['robot_slot']}_sensor{next_sensor}",
                secondary_route,
                box_path=box_path,
                completion_start_xy=secondary_start_xy,
            )
            conveyor_gate.resume_for_robot(int(secondary["robot_slot"]))

            if args.snap_after_sort:
                drop = parse_vec3(args.day2_drop_pos if target == "day2" else args.day3_drop_pos)
                snap_box_to(stage, box_path, drop, f"{target}_drop")
        else:
            secondary_route = f"skipped_secondary_sensor{next_sensor}_timeout"
            print(f"[FLOW][SKIP] {box_id} secondary sensor timeout. Secondary robot motion skipped.")

    if not bool(plan["requires_stage2"]):
        secondary_route = "not_required_today"

    dest_ok = wait_for_destination_track(stage, box_path, target, box_id)

    print(
        f"[CYCLE DONE] {box_id} final_target={target} "
        f"stage1_route={primary_route} stage2_route={secondary_route} "
        f"destination_ok={int(dest_ok)}"
    )
    despawn_box(stage, box_path)

    return {
        "box_id": box_id,
        "target": target,
        "final_target": target,
        "stage1_route": primary_route,
        "stage2_route": secondary_route,
        "destination_ok": bool(dest_ok),
        "destination_track": f"ConveyorTrack_{destination_track_index_for_target(target):02d}",
        "primary_route": primary_route,
        "secondary_route": secondary_route,
        "box_path": box_path,
    }



def main():
    stage = open_stage_blocking(args.usd)

    if not stage.GetPrimAtPath("/World/physicsScene").IsValid():
        scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr(9.81)

    if args.dry_run:
        print_discovery(stage)
        return

    raw_robot1_root, raw_robot2_root = resolve_joint_roots(stage)

    print(f"[ROBOT ROOT RAW] slot1_raw={raw_robot1_root}")
    print(f"[ROBOT ROOT RAW] slot2_raw={raw_robot2_root}")

    for raw_label, raw_root in [("slot1_raw", raw_robot1_root), ("slot2_raw", raw_robot2_root)]:
        pos_path, pos = representative_robot_pos(stage, raw_root)
        if pos is not None:
            print(f"[ROBOT ROOT RAW POS] {raw_label} pos_path={pos_path} pos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})")

    # v03 fixed user mapping:
    # FFW_BG2 is robot1, FFW_BG2_01 is robot2. Do not swap.
    robot_roots_by_slot = {1: raw_robot1_root, 2: raw_robot2_root}
    print("[ROBOT ROOT ORDER] fixed: robot1=FFW_BG2, robot2=FFW_BG2_01, swap_ignored")

    print(f"[ROBOT ROOT ORDER] slot1={robot_roots_by_slot[1]}")
    print(f"[ROBOT ROOT ORDER] slot2={robot_roots_by_slot[2]}")

    ensure_xform(stage, TEST_ROOT)
    ensure_xform(stage, BOX_ROOT)

    conveyor_gate = ConveyorEnableGate(stage)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    print("[SIM] play")

    transition_both_side_to_forward(stage, robot_roots_by_slot[1], "robot_slot1")
    transition_both_side_to_forward(stage, robot_roots_by_slot[2], "robot_slot2")
    sim_steps(args.settle_steps)

    create_robot1_qr_camera(stage)

    try:
        run_simple_two_robot_sort_loop(stage, conveyor_gate, robot_roots_by_slot)
    finally:
        timeline.stop()
        sim_steps(10)


try:
    main()
finally:
    simulation_app.close()
