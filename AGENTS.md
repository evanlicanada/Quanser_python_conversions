# Agent Guidelines & Repository Architecture

> ⚠️ **MANDATORY DIRECTIVE FOR ALL FUTURE AGENTS:**
> Any agent modifying this repository, creating new lab scripts, adding or altering library components, or refactoring existing code **MUST update this `AGENTS.md` file**. Always document new files, architectural decisions, and configuration conventions here so subsequent agents maintain full continuity.

---

## 1. Project Mission & Context

This repository is part of the **University of British Columbia (UBC) TLEF Co-op Project**, converting Quanser robotics and control laboratory curricula from legacy MATLAB/Simulink/QUARC into modern, standalone **Python** applications.

- **Primary Target Product:** Quanser QArm (4-DOF robotic manipulator + 1-DOF gripper).
- **Execution Environments:**
  1. **Virtual Digital Twin (`hardware=0`):** Communicates with Quanser Interactive Labs (QLabs) via TCP/IP on port `18900` (`0@tcpip://localhost:18900?nagle='off'`).
  2. **Physical Hardware (`hardware=1`):** Communicates with physical QArm hardware via Quanser HIL SDK over USB.
- **Python Version:** Python 3.13 (PyQt6, PyQtGraph, NumPy).

---

## 2. Repository Layout

```
python conversions/
├── AGENTS.md                                # <-- THIS FILE: Guidelines, architecture, and registry
├── templates/
│   └── python_lab_template.py               # Standard template for all lab conversions
├── modified_library/                        # Local fixes & extensions to Quanser PAL libraries
│   ├── __init__.py                          # Package exports
│   ├── README.md                            # Technical explanation of PAL bug fixes
│   ├── scope.py                             # High-performance MultiScope (rolling buffer, safe event loop)
│   └── qarm.py                              # Enhanced QArm (supports jointMode=1 PWM mode)
├── sp0_play/                                # Skills Progression 0 - Play (Introductory Labs)
│   └── play_qarm_python/
│       ├── digital_twin/
│       │   ├── BasicIO_position_mode.py     # Lab 0: Position mode with GUI & MultiScope
│       │   └── BasicIO_pwm_mode.py          # Lab 0: PWM open-loop mode with GUI & MultiScope
│       └── hardware/
│           └── play.py                      # Hardware teleoperation & plotting script
└── qarm.py                                  # (Legacy workspace prototype; prefer modified_library.qarm)
```

---

## 3. Core Architectural Rules

### Rule 1: Always Use `modified_library` for Scope and QArm PWM
Quanser's vendor libraries (`pal.utilities.scope` and `pal.products.qarm`) live in external system folders that cannot be modified directly. Always use the drop-in replacements in `modified_library`:

1. **`modified_library.scope`:**
   - **Why:** Upstream Quanser `pal.utilities.scope.Signal` creates a new `pg.PlotDataItem` on every single frame, causing over 21,500 graphics items to accumulate. When the plot reaches the visible window ($t \ge 5\text{s}$), it forces full-scene transform updates on every tick, causing catastrophic UI lag. It also has a nested `processEvents()` call that crashes PyQt timer event loops.
   - **Fix:** Uses a single persistent curve per signal with in-place `.setData()` from a fast rolling buffer, plus safe `QThread.currentThread().loopLevel() == 0` event loop detection. Performance jumps from **12 FPS to 132+ FPS**.
   - **Import:**
     ```python
     from modified_library.scope import Scope, XYScope, MultiScope, TYAxis
     ```

2. **`modified_library.qarm`:**
   - **Why:** Upstream Quanser `pal.products.qarm.QArm` hardcodes Position Mode (`j0_mode=0; ...`) and has no support for PWM open-loop voltage control.
   - **Fix:** Subclasses `QArm`, adds `jointMode=0` (Position) and `jointMode=1` (PWM), and adds `myArm.read_write_pwm(pwmCMD, gprCMD, baseLED)` writing to HIL channels `[11000:11003]`.
   - **Import:**
     ```python
     from modified_library.qarm import QArm
     ```

3. **Universal Path Discovery:**
   Always include the following boilerplate near the top of lab scripts so they run seamlessly from any working directory or terminal:
   ```python
   PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
   if PROJECT_ROOT not in sys.path:
       sys.path.insert(0, PROJECT_ROOT)
   ```

---

### Rule 2: Strictly Adhere to `templates/python_lab_template.py`
Every converted lab script must follow the standard structure using `# region:` and `# endregion` tags:

1. **Header Metadata:**
   ```python
   # TEAM: Add name of the experiment here
   # TEAM: Add a description of the experiment here
   ```
2. **`# region: Python level imports`:** Standard library, PyQt6, Quanser HAL/PAL, and `modified_library`.
3. **`# region: Scope constants`:** Scope frame rates, buffer sizes, and **fixed Y-axis limits (`yLim`)**.
4. **`# region: Scope setup or Probe/Observer setup (pick one)`:** `MultiScope` grid initialization and signal attachment.
5. **`# region: Experiment constants`:** Student-tunable variables (`target_sweep`, `oscillationPeriod`, `hardwareMode`, `readMode`).
6. **`# region: Main Loop`:**
   - Step 1: Kill switch setup (`signal.SIGINT` and `KILL_THREAD`)
   - Step 2: Hardware / Digital Twin initialization
   - Step 3: Background data collection thread (`scope_loop`)
   - Step 4: PyQt6 custom control panel (GUI)
   - Step 5: Periodic control loop callback (`control_step`)
   - Step 6: `QTimer` and `app.exec()`
   - Step 7: Clean shutdown sequence (`KILL_THREAD = True`, `myArm.terminate()`)

---

### Rule 3: Always Define Fixed Scope Limits (`yLim`)
PyQtGraph automatically auto-scales any axis where `yLim` is not provided. When joints are stationary, numerical floating-point noise (e.g., $10^{-6}\text{ rad/s}$ velocity or $0.005\text{ rad}$ gravitational sag) causes PyQtGraph to display confusing micro-scale multipliers like `(x1e-06)` and `(x0.001)`.
**Always set explicit limits on scope axes:**
- Joint positions (`Phi`): `yLim=[-3.5, 3.5]` (rad)
- Joint speeds (`Phi dot`): `yLim=[-2.0, 2.0]` (rad/s)
- Motor currents: `yLim=[-2.0, 2.0]` (A)
- Gripper position: `yLim=[0.0, 1.0]`
- PWM duty cycle: `yLim=[-0.5, 0.5]` or `[-1.0, 1.0]`
- Temperatures: `yLim=[20.0, 50.0]` (°C)

---

### Rule 4: Event Loops, Timers, and Thread Safety
1. **Never use `while True: sleep()` in the main thread of a PyQt app.** Use `QTimer` connected to a periodic function (`control_step`) and let `app.exec()` drive the GUI.
2. **Timer intervals must match Scope FPS:**
   ```python
   timer.start(int(1000 / scopeFps)) # e.g. 33 ms for 30 FPS
   ```
3. **Background Data Collection (`scope_loop`):**
   - Runs in a daemon thread.
   - Samples sensor data and calls `multiScope.axes[i].sample(t, data)`.
   - Must loop on `while not KILL_THREAD:`.
   - On shutdown, always set `KILL_THREAD = True` and call `myArm.terminate()` to close HIL sockets and prevent orphaned background connections.

---

## 4. Current Lab Registry

| Lab / File Path | Mode | Description |
|---|---|---|
| `sp0_play/play_qarm_python/digital_twin/BasicIO_position_mode.py` | Position Mode (`jointMode=0`) | Controls 4 joint angles & gripper via PyQt GUI panel; monitors 6 real-time signals on `MultiScope`. |
| `sp0_play/play_qarm_python/digital_twin/BasicIO_pwm_mode.py` | PWM Mode (`jointMode=1`) | Open-loop voltage commands (-12V to 12V); features `-1`, `0`, `+1` directional autofill buttons and custom PWM input to test gravity fall, joint conventions, and hold voltages. |
| `sp0_play/play_qarm_python/hardware/play.py` | Hardware Teleoperation | Uses `QKeyboard` and forward kinematics for hardware joint movement and plotting. |

---

## 5. Maintenance Checklist for Future Agents

Whenever you perform work in this repository, check off each item:
- [ ] Did you create a new lab script? Verify it matches `templates/python_lab_template.py` with all `# region:` blocks.
- [ ] Did you use `modified_library.scope` instead of `pal.utilities.scope`?
- [ ] Did you set explicit `yLim` limits on all scope axes?
- [ ] Did you add graceful shutdown (`KILL_THREAD`, `myArm.terminate()`)?
- [ ] Did you verify python syntax using `py -3.13 -m py_compile <path>`?
- [ ] **DID YOU UPDATE THIS `AGENTS.MD` FILE?** (Register new files in the table above, document new libraries or conventions).
