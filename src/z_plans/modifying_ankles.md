# Claude Code Prompt — Plan: Add Wheels at the Ankles (new URDF)

> Paste this whole file into Claude Code as the task.
> **This is a PLANNING task. Do NOT write or edit any code or URDF yet.**
> Produce a plan only. I will read it and decide the next step myself.

---

## Goal

Create a **new wheeled variant** of the biped:
- Keep all current designs untouched (mujoco / isaacsim / gazebo / mock / real).
- Add a **new** variant where each **passive ankle is replaced by a powered wheel** (hub motor at the ankle).
- This is the "wheeled" robot for LQR / wheeled-inverted-pendulum balancing.

I want both robots to live in the same workspace. The current biped stays as-is. The wheeled one is an addition.

## Hard constraints (read before planning)

1. **Do not modify** any existing URDF, xacro, config, or controller. Add new files only.
2. Follow the existing naming pattern in `src/biped_description/urdf/` (one URDF per backend, all sharing `biped.urdf.xacro` with a different `<ros2_control>` block).
3. The new wheel joint is **continuous** (spinning), not the old fixed/passive ankle. It needs a velocity command interface, not position.
4. Keep the leg geometry the same as the current robot for now (hip → knee → shank). Only the ankle end changes: passive foot → wheel.
5. I have no experience in system architecture — explain the plan **component by component**, simplest level, with tables and a numbered file-by-file list.

## What I want the plan to cover

### Part A — Understand current state
1. List every file that defines the current robot description (xacro, per-backend URDFs, mesh/visual files, the MuJoCo `mujoco_xml/SF_biped.xml`).
2. Show how `biped.urdf.xacro` plugs in different `<ros2_control>` blocks per backend. Point to the exact lines.
3. Identify where the **ankle / foot link** is defined today and how it is attached (fixed joint? passive?). Quote the relevant URDF/xacro snippet.

### Part B — Decide the wheel design (give me OPTIONS, not a decision)
For each open question, give a short table: Option | How it works | Merits | Demerits. Topics:
1. Wheel as a **new link + continuous joint** vs reusing the foot link.
2. One wheel per leg vs a different arrangement.
3. Where the hub motor's mass/inertia goes (effect on the swing-inertia problem we already discussed).
4. Whether the wheel needs its own `<ros2_control>` velocity interface, and how that sits next to the existing hip/knee position interfaces.
5. MuJoCo: edit a copy of `SF_biped.xml` vs generate from xacro — which is realistic here.

### Part C — File-by-file plan (the deliverable)
A numbered list of **new files to create** (no edits to old ones), e.g.:
- new xacro macro for the wheel link + joint
- new `biped_wheeled_*_ros2_control.urdf`
- new MuJoCo xml copy with wheels
- new controller config yaml (velocity interface for wheels)
- new launch file (or argument) to bring up the wheeled variant

For each file: what it contains, what it copies from, what it changes. **Skeletons only — no full code.**

### Part D — Risks and check
1. What breaks silently if I get the joint order / interface type wrong (we know the controller maps order silently).
2. The smallest test that proves the new URDF loads and the wheel spins in sim, before any control logic.
3. What I should decide before building (open questions left for me).

## Output format
- Simple English. Short sentences.
- Tables, bullets, numbered steps.
- One concept at a time.
- End with: **"Open questions for you to decide"** — a short list.

## Do NOT
- Do not write or edit any file.
- Do not pick the design for me — show options, recommend, wait.
- Do not touch the existing robot variants.
