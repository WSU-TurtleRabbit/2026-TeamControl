# SSL Game States — BT Implementation Summary

All game-phase handling lives in `src/TeamControl/bt/coordinator.py`.
The `GamePhase` enum and `RefereeState` are defined in `src/TeamControl/bt/contracts/snapshot.py`.
Referee data flows: GC → `gcfsm_runner` → `gc_q` → `WMWorker` → `WorldModel` → `adapter.py` → `Snapshot`.

---

## Field Constants (Div B — 9 m × 6 m)

| Constant | Value | Note |
|---|---|---|
| `STOP_BALL_CLEARANCE` | 0.55 m | Safety buffer used in code; legal rule threshold is 0.50 m |
| `LEGAL_BALL_CLEARANCE` | 0.50 m | Actual SSL rule minimum (§5.4) |
| `PENALTY_SPOT` | (3.5, 0.0) | 1 m from goal line on 9 m field (§8.2.3) |
| `OWN_GOAL_LINE_X` | −4.5 | x-coordinate of our goal line |

When `us_positive=False` (we attack toward negative-x), all fixed position maps are mirrored on startup so robot positions are always correct.

> **⚠ `us_positive` is negated at the entry point.** In the current grSim + SSL-GC setup, `wm.us_positive()` reports a value opposite to the codebase convention (the codebase uses `us_positive=True ⇒ we attack +x ⇒ own goal at -x`). To keep the convention consistent across the whole BT, `run_bt_v2_process` negates `wm.us_positive()` once before handing it to the `Coordinator` and the four role trees. The trees and the coordinator's set-piece tables all then agree. If you later fix the upstream value (e.g. flip the YAML default or correct `gcfsm_runner.check_color_side`), **remove the negation in `run_bt_v2_process`** so the values stop double-inverting. The negation site is the single place to look — search the file for "us_positive INVERSION".

---

## States

### HALTED / HALF_TIME
**Rule:** All robots must stop immediately. No movement.

**Implementation:** Coordinator returns an empty intent list. The dispatcher lets existing commands expire and robots coast to zero velocity.

---

### STOPPED
**Rule (§5.4):** All robots must slow to < 1.5 m/s. All robots must keep ≥ 0.5 m from the ball.

**Implementation:**
- Every robot gets `IntentMove` with `max_speed=1.4 m/s` (enforces < 1.5 m/s).
- If a robot is within `STOP_BALL_CLEARANCE` (0.55 m) of the ball it is nudged away to the 0.55 m boundary. The 0.05 m buffer ensures no robot is ever right at the legal limit.
- Robots already clear of the ball hold their current position.

---

### PREPARE_KICKOFF
**Rule (§5.3.2):** All robots must move to their own half. One attacker may be positioned anywhere inside the centre circle (radius 0.5 m). Robots must not touch the ball.

**Implementation:** `_handle_fixed_positions()` moves every robot to `KICKOFF_POSITIONS`:

| Robot ID | Role | Position |
|---|---|---|
| 0 | Goalie | (−4.0, 0.0) — in front of own goal |
| 1 | Defender | (−2.0, −1.5) |
| 2 | Defender | (−2.0, 1.5) |
| 3 | Supporter | (−1.0, −1.0) |
| 4 | Supporter | (−1.0, 1.0) |
| 5 | Attacker | **(0.0, 0.0) — centre of circle; anywhere in circle is allowed (§5.3.2)** |

> Previously incorrectly placed at the edge of the circle. Corrected to (0, 0).

---

### KICKOFF
**Rule:** Attacker kicks off from centre. All non-attacker robots must remain in their own half until ball is touched.

**Implementation:**
- **Attacker (robot 5):** `IntentMove` to `snapshot.ball_position` (ball is at centre).
- **All others:** Hold their `KICKOFF_POSITIONS` (same as PREPARE_KICKOFF).

---

### FREE_KICK
**Rule:** Attacker takes a free kick from the designated ball position. Opponent robots must keep 0.5 m from ball; our robots can approach normally.

**Implementation:**
- **Attacker (robot 5):** `IntentMove` to `snapshot.ball_position`.
- **All others:** `IntentMove` to hold their current position (freeze in place).

---

### BALL_PLACEMENT
**Rule (§9):** One robot (the placer) must carry the ball to the `designated_position` reported by the GC. All other robots must keep ≥ 0.05 m from the ball and ≥ 0.05 m from the ball→target path. After the ball is placed the placer must move ≥ 0.5 m away.

**Implementation:**
- **Placer (robot 5 / attacker):**
  1. If ball is not yet within 0.15 m of target: `IntentDribble` to `ball_placement_pos`.
  2. Once ball is within 0.15 m of target (`ball_at_target`): backs away using `IntentMove` to `LEGAL_BALL_CLEARANCE` (0.50 m) from ball — satisfies the post-placement clearance rule.
- **All others:** Kept clear of both the ball (≥ `STOP_BALL_CLEARANCE`) and the ball→target line segment using `_nudge_away_from_segment()`.
- `ball_placement_pos` is forwarded from GC `designated_position` via `gcfsm_runner` → `WorldModel.ball_placement_pos` → `Snapshot.referee_state.ball_placement_pos`.

---

### PENALTY_SHOOT
**Rule (§8.2):** Our attacker shoots from penalty spot (1 m from opponent's goal line). All other robots must be ≥ 1 m behind the ball (i.e., x ≤ penalty_spot_x − 1.0 = 2.5 m) and in our own half.

**Implementation:** `_handle_fixed_positions()` moves every robot to `PENALTY_SHOOT_POSITIONS`:

| Robot ID | Role | Position | Note |
|---|---|---|---|
| 0 | Our goalie | (2.0, 0.5) | Behind ball; not defending (opponent keeper defends their goal) |
| 1 | Defender | (2.0, −1.5) | Behind ball |
| 2 | Defender | (2.0, 1.5) | Behind ball |
| 3 | Supporter | (2.0, −0.5) | Behind ball |
| 4 | Supporter | (2.0, 0.5) | Behind ball |
| 5 | Attacker | **(3.5, 0.0) — penalty spot** | Shoots |

> Penalty spot corrected from (3.6, 0) to **(3.5, 0)** — 1 m from goal line on a 9 m field.
> Robot 0 corrected: previously placed at a goalie/goal-line position; corrected to (2.0, 0.5) — our goalie is NOT the defending keeper during an opponent penalty, it simply waits behind the ball.

---

### PENALTY_DEFEND
**Rule (§8.2):** Opponent shoots a penalty. Our goalie defends on our goal line. All other robots stay behind the ball (in own half).

**Implementation:**
- **Goalie (robot 0):** Dynamically tracks ball y-coordinate on `OWN_GOAL_LINE_X` (−4.5). Target = (−4.5, `ball_y`). Uses `_handle_penalty_defend()`.
- **All others:** `IntentMove` to `PENALTY_DEFEND_POSITIONS`:

| Robot ID | Position |
|---|---|
| 1 | (−2.0, −1.0) |
| 2 | (−2.0, 1.0) |
| 3 | (−1.5, −0.5) |
| 4 | (−1.5, 0.5) |
| 5 | (−1.0, 0.0) |

---

### RUNNING
**Rule:** Normal play.

**Implementation:** Normal role-tree dispatch. Each robot's assigned `RoleType` tree runs via `_normal_tick()`.

Role assignment (fixed by robot ID):

| Robot ID | Role |
|---|---|
| 0 | GOALIE |
| 1 | DEFENDER |
| 2 | DEFENDER |
| 3 | SUPPORTER |
| 4 | SUPPORTER |
| 5 | ATTACKER |

---

## Data Flow for Game State

```
SSL Game Controller
      │
      ▼
gcfsm_runner.py  (GCfsm.check_state)
  - Translates referee packet to GameState enum
  - Forwards designated_position as (PacketType.BALL_PLACEMENT_POS, (x, y))
      │
      ▼
gc_q  (multiprocessing Queue)
      │
      ▼
WMWorker  →  WorldModel.update_gc_data()
  - WorldModel._state     = current GameState
  - WorldModel.ball_placement_pos = (x, y) | None
      │
      ▼
adapter.build_snapshot_from_world_model()
  - Reads wm.get_game_state() → GamePhase via _PHASE_MAP
  - Reads wm.get_ball_placement_pos() → Snapshot.referee_state.ball_placement_pos
      │
      ▼
Snapshot (frozen)
      │
      ▼
Coordinator.tick(snapshot, robot_ids)
      │
      ▼
list[Intent] → RobotCommand → dispatcher_q → grSim
```