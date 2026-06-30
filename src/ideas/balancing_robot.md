# Balancing Robot — Idea Note

**Date:** 2026-06-26
**Context:** Convert current bipedal robot (passive/no ankle motor → walking very hard) into a wheel-legged balancing robot.

---

## 1. The Pivot

Current biped has **no motor at ankle** → underactuated → blind walking policy fragile and hard.

**Plan:** convert to **wheel-legged balancing robot**.

Per leg motors:
- Hip
- Knee
- Ankle → acts as **wheel**

Balances like a **Wheeled Inverted Pendulum (WIP)**.

Real-world examples: ETH **Ascento**, Unitree **B2-W**, Boston Dynamics **Handle**.

---

## 2. Is the Pivot a Good Idea? → Yes

| | Walking biped (old) | Wheel-legged balance (new) |
|---|---|---|
| Balance physics | hybrid, contact-switching, hard | WIP, well-understood |
| Control | RL policy, fragile | PID/LQR works day 1 |
| Hardware change | none | add wheel at ankle |
| Stairs / rough terrain | strong | weaker (legs still help) |
| Research maturity | hard | mature, many refs |

Removes the hardest part (passive-ankle walking). Reuses existing hardware.

---

## 3. RL on the Balancing Robot?

- **Pure flat-ground balance → LQR/PID is enough. RL = overkill.**
  - WIP near upright is almost linear. LQR is provable, fast, cheap.
- **RL worth it only if** we want:
  - big push recovery (far from upright, nonlinear)
  - rough terrain / slopes
  - combine balance + leg motion + jump in one policy
  - fast driving + crouch + lean together

---

## 4. LQR vs RL

| Axis | LQR | RL |
|---|---|---|
| Model | needs linear model near upright | no derived model, learns in sim |
| Valid range | small angles only | full envelope, big angles |
| Nonlinearity | ignores | handles |
| Multi-skill | one task / one equilibrium | balance + drive + jump in one net |
| Stability proof | yes | no |
| Effort | hours | weeks + GPU + sim2real |
| Disturbance | small only | large, learned recovery |

LQR optimal **only** for linear system, quadratic cost, near one point.

---

## 5. Merging LQR + RL → Yes, possible & beneficial

| # | Method | How | Benefit |
|---|--------|-----|---------|
| 1 | **Residual RL** ⭐ | `u = u_LQR + α·u_RL` | LQR keeps stability; RL fixes nonlinear/terrain. Safe, fast. |
| 2 | LQR warm-start | init/reward RL near LQR | faster learning, fewer crashes |
| 3 | RL gain scheduling | RL picks LQR gains by state | adapts, stays near-LQR |
| 4 | LQR cost in reward | reward = `xᵀQx + uᵀRu` | good objective inherited |
| 5 | Safety filter | LQR/CBF overrides RL near falling | hardware-safe exploration |

**Core idea:** LQR = stable backbone. RL adds only what LQR cannot model → smaller RL job → faster, safer, less sim2real gap.

**Chosen approach: #1 Residual RL.**

```
u = u_LQR(x)  +  α · u_RL(obs)
```

Refs: "Residual Reinforcement Learning" (Johannink 2018), ETH wheel-legged work, Unitree wheel-leg controllers.

---

## 6. Is This Advanced? → Yes (upper-intermediate → advanced)

- mixes classical control (LQR) + modern RL — two mindsets
- real hardware + sim2real
- realtime ROS2 + iceoryx2 stack (already built)
- full pipeline: train → sim → controller → real robot

Not insane because: balancing easier than walking; LQR backbone keeps it stable; existing ROS2 + Isaac Lab skills reused.

Portfolio-grade / research-grade scope.

---

## 7. Build Order (staged — do NOT build all at once)

1. Convert hardware → wheel-legged.
2. WIP state-space model.
3. **LQR** → get it balancing. Prove it.
4. Freeze LQR. Train **residual RL** in Isaac Lab.
5. Blend: start `α` small → grow. RL never controls alone.
6. Sim → real robot.

---

## Open Next Steps (pick one)
- WIP + LQR math derivation
- Residual RL env design (Isaac Lab)
