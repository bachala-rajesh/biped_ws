# Walking Robot — Problems

**Date:** 2026-06-30
**Context:** Current bipedal walking robot is unstable. Each leg has only **three joints**.

---

## 1. The Core Problem

Each leg has **three joints only**:

| Joint | Actuated? |
|-------|-----------|
| Hip pitch | yes |
| Hip roll | yes |
| Knee | yes |
| **Ankle** | **NO motor** |

The **ankle has no motor** → the foot cannot push or correct.
This makes the robot **underactuated** → very hard to balance and walk.

---

## 2. Why Three Joints Is Unstable

| Issue | Why it happens |
|-------|----------------|
| No ankle torque | Foot cannot push ground to stay upright |
| Underactuated | Fewer motors than degrees of freedom to control |
| Falls easily | No way to correct forward/back tilt at the foot |
| Hard contact switching | Each step swaps support foot → jumpy dynamics |
| Walking policy fragile | RL policy must do all balance with hip + knee only |

---

## 3. Effect on the Project

- Blind walking policy is **fragile** and hard to tune.
- Robot **does not walk steadily** in MuJoCo.
- Real-robot deployment is risky with current joints.

---

## 4. Motor Knowledge Gap

- The user **does not know motor testing**.
- Motors were bought from a **local store with no datasheet**.
- Unknown values: peak torque, continuous torque, gear ratio, torque constant.
- Without these numbers, the user **cannot confirm** if the motors are strong
  enough for stable walking.

---

## 5. Open Questions

- Can hip + knee alone produce a stable gait?
- Are the current motors strong enough? (specs unknown)
- How to characterize the motors properly?
