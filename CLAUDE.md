# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

The computer-side half of a two-part system that flies a **DJI Mini SE** with the computer's
**keyboard and mouse**, showing the drone's own video in the same window. The other half is the
Android bridge app (Drone Bridge) at `/home/jean/AndroidStudioProjects/app_proprio`; it has
no control logic, it forwards ROS2 messages to the DJI SDK. Its `PROTOCOLO.md` is the contract
(topics, RPC commands, state sections) — read it before adding anything that talks to the phone.

ROS2 **Humble** `ament_python` workspace (`colcon build`). Everything here is Python.

## Build / run

```bash
./setup_ambiente.sh            # once: venv with pygame + pytest (needs ROS2 Humble + rosbridge installed via apt)
./iniciar.sh                   # sources ROS2 + venv, builds if needed (--build forces it), prints the PC's IP, launches
```

`iniciar.sh` is the one-command start; by hand it is `source /opt/ros/humble/setup.bash`, `source .venv/bin/activate`, `python3 -m colcon build --symlink-install`, `source install/setup.bash`, `ros2 launch ponte_teclado_drone sistema.launch.py`. In the app, type the PC's IP and port `9090`.

- The venv must be `--system-site-packages` (`rclpy` comes from apt). **Build with `python3 -m colcon build`, never bare `colcon build`** — the apt `colcon` has a hard-coded system-Python shebang, so the generated node script would ignore the venv and not find `pygame`. If a build came out wrong: `rm -rf build install log` and rebuild.
- Tests: `cd src/ponte_teclado_drone && python -m pytest test` (pure logic, no ROS2/display needed). There is no formal test for the node itself. To smoke-test it, use `SDL_VIDEODRIVER=dummy`, synthetic `pygame` events and a second ROS2 node watching the topics, **always in an isolated ROS domain (`ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1`, and check `ros2 node list` is empty first) — never on the default domain: the rosbridge and the real phone/drone may be live there and will receive the test's takeoff/land/velocity/stick_mode messages**.

## Architecture

- `controle.py`: all control logic, **pure Python** (no ROS2, no pygame) so it is unit-tested. `Piloto` combines pressed keys (`w a s d espaco shift` for flight, `esquerda direita` for yaw, `cima baixo` for the gimbal — the arrows add to the mouse stick at `VELOCIDADE_GUINADA_TECLAS` / `VELOCIDADE_GIMBAL_TECLAS`), the mouse stick and the armed flag into a `Comando` (x/y/z/yaw/gimbal, normalized to [-1, 1], REP 103 axes) per time step. Keys go through a `Rampa` (0→1 in 0.30 s, back in 0.15 s; WASD is on/off and would otherwise send speed steps) and the diagonal is normalized. `StickMouse` is a virtual joystick (drag anchor = center, dead zone, exponential curve, low-pass `Filtro`); the drag "pulls the image": dragging right = drone yaws left (counter-clockwise, positive yaw), dragging down = camera tilts up (`INVERTER_GIMBAL` flips only the gimbal). `AcaoSegurada` fires T/L/H/G only after the key is held ~0.8 s; T and L are additionally ignored while disarmed (`ACOES_EXIGEM_ARMADO`), H/G always work. Yaw is sent only while armed; the gimbal works disarmed. All feel constants are at the top of the file.
- `piloto_teclado_mouse.py`: the `piloto_teclado_mouse` node + pygame window. Translates pygame events into `Piloto` calls and `Comando` into messages (`/drone/cmd_vel` only while armed, 20 Hz; `/drone/gimbal_pitch_rate` while non-zero plus a few trailing zeros; `/drone/stick_mode` only on arm/disarm changes, because the app re-runs its whole enable sequence on every `true`). Takeoff/land/go-home go through the app's RPC (`/drone/rpc/request`, `flight.take_off`, `flight.land`, `home.go_home`, `flight.cancel_land_or_rth` for G — cancels a landing or RTH, whichever is running, never the takeoff; decided phone-side) and the response is shown in the HUD. Keys 1-4 send `video.set_quality` presets (`PRESETS_VIDEO`); the active preset is shown only after the phone's ok, with measured fps / KB per frame / KB/s. Subscribes to the drone video (decoded straight by pygame from the JPEG), `/phone/telemetria` and `/phone/rpc/response`. The pygame loop owns the main thread and calls `rclpy.spin_once(timeout_sec=0)`.
- `sincronizar_com_celular()`: ~3 s after arming, if fresh telemetry says the phone is NOT in stick mode (its "ASSUMIR CONTROLE" button, app backgrounded, drone disconnected, enable failed) the node disarms itself and shows a warning, so one Enter re-arms. This keeps the node's state in line with the phone, so one Enter re-arms.
- Losing window focus, closing, or Ctrl+C disarms: zero twist, `stick_mode=false` (the DJI remote takes over), gimbal zeros. Esc disarms immediately. Starts disarmed. Panic is not here on purpose — it only exists on the phone.
- Axis convention is **REP 103** (x forward, y left, z up, yaw counter-clockwise). The app converts to DJI's; don't "fix" signs here. The yaw sign conversion in the app is unverified on a real aircraft — if the drone yaws the wrong way, flip it in `MainActivity.subscribeToCommandTopics()` first.

## Mini SE limits to remember when extending this

- The Mini SE gimbal only accepts **pitch** (−90…0°, or up to +20° with `gimbal.set_pitch_range_extension`); roll/yaw are auto-stabilized and the SDK doesn't reject commands for them. Use only pitch; check `gimbal.capabilities` at startup if adding gimbal features.
- Not every RPC command is guaranteed on the Mini SE; `system.probe` reports what the connected drone actually answers. This program has flown a real Mini SE through the app (stick mode, takeoff, landing, go-home/cancel, gimbal); the yaw-sign conversion is still unconfirmed.
