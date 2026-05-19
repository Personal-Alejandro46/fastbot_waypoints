# fastbot_waypoints

ROS 2 C++ port of the `tortoisebot_action_server.py` (ROS 1) using a custom
`WaypointAction` action message. The action server navigates the FastBot to a
target `[X, Y]` position and aligns its final yaw to the heading from the
robot's start position to the target.

---

## Package structure

```
fastbot_msgs/
fastbot_waypoints/
├── src/
│   └── fastbot_action_server.cpp   # Action server node
├── test/
│   └── waypoints_test.cpp          # GTest node-level tests
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Prerequisites — build both packages

```bash
cd ~/ros2_ws
colcon build --packages-select fastbot_msgs fastbot_waypoints
source install/setup.bash
```

---

## Running the tests

### Step 1 — Terminal 1: launch the simulation

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
```
---

### Step 2 — Terminal 2: launch the action server

```bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
```

---

### Step 3 — Switch between passing and failing conditions

Open `test/waypoints_test.cpp` and find this block near the top of the file:

```cpp
// ===== ALTERE AQUI PARA PASSAR OU FALHAR =====
// true  -> conecta ao servidor real (passing conditions)
// false -> conecta a servidor inexistente (failing conditions)
constexpr bool USE_REAL_SERVER = true;
// =============================================
```
---

### Passing conditions

Keep (or set) `USE_REAL_SERVER = true`.

The test connects to the real `fastbot_waypoint_as` action server (must be
running in Terminal 2). The robot navigates to the target and the assertions
verify that the final position and yaw are within the allowed tolerance (±0.1).

**Terminal 3:**

```bash
cd ~/ros2_ws && colcon build --packages-select fastbot_waypoints && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

**Expected output:**

```
Summary: 2 tests, 0 errors, 0 failures, 0 skipped
```

---

### Failing conditions

Set `USE_REAL_SERVER = false`.

The test tries to connect to a server named `nonexistent_server`. The
`ASSERT_TRUE` inside `SetUp()` fails after the 10-second timeout because no
such server exists. The simulation and action server do **not** need to be
running for this mode.

**Terminal 3:**

```bash
cd ~/ros2_ws && colcon build --packages-select fastbot_waypoints && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

**Expected output:**

```
Summary: 2 tests, 1 errors, 0 failures, 1 skipped
```

---

## What the tests verify

| Test | What is checked | Tolerance |
|---|---|---|
| `TestEndPosition` | Final `[X, Y]` position matches the goal | ±0.1 m |
| `TestEndYaw` | Final yaw matches the heading from the robot's start position to the goal | ±0.1 rad |

> `TestEndYaw` captures the robot's position at the start of the test (in
> `SetUp()`) and computes `expected_yaw = atan2(TARGET_Y - init_y, TARGET_X - init_x)`.
> This matches the `nav_yaw` the action server uses internally, so the test
> is correct regardless of where the robot spawns in the simulation.

---

## Cleaning stale test results

If previous test runs left old XML files, clear them before running again:

```bash
rm -rf ~/ros2_ws/build/fastbot_waypoints/test_results
```