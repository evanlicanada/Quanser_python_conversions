# Modified Quanser Library (`modified_library`)

This folder contains localized enhancements and bug fixes for Quanser's **PAL (Python Architecture for Laboratories)** classes. 

Because Quanser's core libraries are located in vendor directories that cannot be pushed or directly altered upstream, any project-wide architectural fixes and extensions are maintained here as drop-in replacements.

---

## 1. `scope.py` — High-Performance Real-Time Visualization

### The Problems in Upstream `pal.utilities.scope`

1. **`PlotDataItem` Object Explosion (The Chunking Bug):**
   - In upstream `Signal.add_chunk(x, y)`, every refresh tick (30 Hz) created a **brand-new `pg.PlotDataItem`** object and added it to the plot (`self.plot.addItem(pdi)`).
   - In a standard lab setup (e.g., 6 subplots with 21 total signals) and `maxChunks = 1024`, the Qt scene graph rapidly accumulated:
     $$21 \text{ signals} \times 1024 \text{ chunks} = \mathbf{21,504 \text{ separate QGraphicsItems!}}$$
   - **Why lag hit at $t = 5\text{ s}$:** During the first 5 seconds ($t < \text{timeWindow}$), the visible X-range was static at $[0, 5]$. Once $t \ge 5\text{ s}$, `setXRange(t - 5, t)` updated on **every single frame**. This forced Qt GraphicsView to compute coordinate transformations, bounding boxes, and clipping checks for all 21,504 individual items 30 times a second.
   - **No Off-Screen Pruning:** Chunks that scrolled off the left edge of the visible window were retained in memory and scene graph until 1024 chunks accumulated (~34 seconds).

2. **Nested Event Loop Recursion (`processEvents`):**
   - Upstream `MultiScope.refreshAll()` unconditionally called `QtWidgets.QApplication.processEvents()` on every flush.
   - When running modern PyQt applications with `QTimer` and `app.exec()`, calling `processEvents()` inside the timer callback forces Qt to re-enter the event loop recursively. If a frame took slightly longer than 33 ms, pending timer ticks interrupted active paint events, leading to queue backlog and complete UI freezing.

### The Solutions in `modified_library.scope`

- **Single Persistent Curve per Signal (`_fast_add_chunk`):**
  Instead of creating new graphics items, each `Signal` uses a single persistent `pg.PlotDataItem` (reusing the curve already registered with the plot legend: `self.curve = self._pDI4Legend`).
  - Total graphics items in scene: **Exactly 21** (down from 21,504).
- **Rolling Buffer with Fast Pruning:**
  Timestamps and values are maintained in fast `collections.deque` buffers. On each flush, points older than `timeWindow + 0.5s` are discarded, and the remaining points are passed directly to the curve via `curve.setData(np.fromiter(...))`.
- **Safe Event Loop Processing:**
  `MultiScope.refreshAll()` now checks:
  ```python
  if flush:
      if QThread.currentThread().loopLevel() == 0:
          QtWidgets.QApplication.processEvents()
  ```
  - In simple standalone scripts (`while True: ... refreshAll()`), `loopLevel() == 0`, so `processEvents()` is called to repaint the window.
  - In PyQt GUI applications using `app.exec()`, `loopLevel() >= 1`, so `processEvents()` is safely bypassed, letting Qt's event loop handle repainting without recursion.

### Benchmark Results
- **Upstream Chunking:** $\sim 12.7\text{ FPS}$ (23.7s for 300 frames, degrades over time).
- **Modified Rolling Buffer:** $\mathbf{132.5\text{ FPS}}$ (2.2s for 300 frames, **>10x faster**, constant $<1\%$ CPU).

---

## 2. `qarm.py` — Enhanced QArm with PWM Mode Support

### The Problems in Upstream `pal.products.qarm`

1. **PWM Mode Not Implemented:**
   - Upstream `QArm` hardcodes `j0_mode=0; j1_mode=0; j2_mode=0; j3_mode=0;` in `boardSpecificOptions`, with docstrings noting `(PWM mode not supported)`.
   - `read_write_std` only supported position commands sent to other channels `[1000:1003]` and gripper `[1004]`.
2. **Missing HIL Channel Mapping for Open-Loop PWM:**
   - In Quanser Simulink models (e.g. `BasicIO_pwm_mode.slx`), joint PWM commands are written to channels `[11000:11003]` as normalized duty cycles ($-1.0$ to $1.0$). These channels were not defined in upstream PAL.

### The Solutions in `modified_library.qarm`

- **Subclasses `pal.products.qarm.QArm`:** Retains 100% backward compatibility with existing code.
- **Configurable Control Mode (`jointMode`):**
  ```python
  myArm = QArm(hardware=0, readMode=0, jointMode=1)
  # jointMode=0: Position Mode
  # jointMode=1: PWM Mode (open-loop voltage control, -12V to +12V)
  ```
- **Added `read_write_pwm(pwmCMD, gprCMD, baseLED)`:**
  Writes to the correct Quanser HIL channel layout:
  - Channels `1000:1003`: `Phi cmd` (zeros in PWM mode)
  - Channel `1004`: `Gripper cmd` ($0.1$ open to $0.9$ closed)
  - Channels `11000:11003`: `PWM cmd` ($-1.0$ to $+1.0$ duty cycle)
  - Channels `11005:11007`: `Base RGB LED` ($[0, 1, 0]$ green)
  Then automatically calls `read_std()` to update measurement buffers (`measJointPosition`, `measJointSpeed`, `measJointCurrent`, `measJointPWM`, `measJointTemperature`).

---

## Usage in Lab Scripts

Import directly from `modified_library`:

```python
import os
import sys

# Ensure modified_library is discoverable from any directory
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from modified_library.scope import Scope, XYScope, MultiScope, TYAxis
from modified_library.qarm import QArm
```
