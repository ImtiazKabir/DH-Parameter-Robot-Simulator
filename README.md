# DH Robot Simulator

**Live demo: <https://imtiazkabir.github.io/DH-Parameter-Robot-Simulator/>**

Interactive 3D web simulator for the Denavit–Hartenberg (DH) parameters of a
serial robot (1–8 joints) with a gripper. Built with plain JavaScript and Three.js
(loaded from a CDN), so there is no build step.

## Run

ES modules need an http origin, so serve the folder:

```bash
python3 -m http.server 8000
```

Then open <http://localhost:8000>. An internet connection is needed to load Three.js.
Open a preset directly with a URL hash, e.g. `#planar2r`, `#twist`, `#ex-wrist` or `#scara`
(the keys are listed in `PRESETS` in `js/main.js`).

## Features

- For each joint, set θ, d, a, α with a slider or a number input. The model updates live.
- Add joints with **+ Add joint** (up to 8) and remove one with its **✕** button.
- **Presets**, in three groups. Each one shows a short "what to notice" note in the sidebar, with
  ◀ ▶ buttons to step through its group. The note is marked *(edited)* once you change the robot.
  - **Robots** (open by default): 1R Planar Manipulator (the default), 2R Planar Manipulator,
    3R Planar Manipulator, 3R Spatial Manipulator, SCARA RRP, Cylindrical RPP, Stanford RRP.
  - **Lessons** (collapsed) introduce one idea at a time. All are revolute, so ▶ Learn DH works
    on each: L1 1R link length a · L2 1R joint offset d · L3 2R planar · L4 3R planar ·
    L5 link twist α with intersecting axes · L6 link twist α with skew axes (common normal) ·
    L7 3R spatial elbow arm.
  - **Exercises** (collapsed) each target one tricky case: E1 negative twist · E2 anti-parallel
    axes (α = 180°) · E3 oblique twist (45°) · E4 collinear axes (x is free) · E5 offset on a later
    joint · E6 negative offset · E7 intersecting wrist axes · E8 tilted tool frame ·
    E9 general skew chain · E10 prismatic joint.

  All presets use drawing scale 10. **Reset** goes back to the 1R planar manipulator.
- Each joint can be **Revolute** (θ is the variable, drawn as an orange drum with a θ arc)
  or **Prismatic** (d is the variable, drawn as a purple sleeve with a sliding rod).
  The row for the joint variable is highlighted.
- **Rotation cue**: while you drag a θ slider (or edit a θ value), that joint and its link turn
  into a black-and-white checker pattern. The α slider does the same for what it turns about xᵢ:
  the link along xᵢ and the next joint's drum. This makes the turning easy to see, even for parts
  that spin about their own axis, like the base column. The pattern goes away about half a
  second after you stop.
- **Projection on the XY plane** (opt-in, under "Drawing & view"): a soft shadow of the robot cast
  straight down onto the ground, i.e. its orthographic projection onto the XY plane. The opacity
  slider sets how strong it is (default 25%).
- **Save image / Copy image** (canvas toolbar): exports what you see as a PNG, including the labels.
  In multi-view the image contains the panes with their titles. Copying needs a browser with
  clipboard image support (recent Chrome, Edge, Safari or Firefox) and an `https://` or `localhost`
  page.
- View options start minimal: only the ground grid is shown. Turn on the EE coordinates, DH
  dimensions, frame axes and frame labels under "Drawing & view".
- Frame axes are drawn on top of the robot, so a z axis running through a joint or an offset
  link stays visible. The y axes are opt-in ("Show y axes", under "Show frame axes").
- Frames {0}–{3} are drawn with the usual colors: **X red, Y green, Z blue**.
- **Drawing scale**: 1 drawn unit = S real units. This lets large values (e.g. d = 50)
  fit on screen; the slider ranges grow with S. **Auto** picks a scale that fits the robot.
  Values typed into the number boxes for d and a are not capped by the slider.
- **DH parameters on canvas** (on by default): each θ, d, a and α is drawn in place, like a
  dimensioned drawing. d and a are dimension lines along z₍ᵢ₋₁₎ and xᵢ. θ is an arc about
  z₍ᵢ₋₁₎ from x₍ᵢ₋₁₎ to xᵢ. α is an arc about xᵢ from z₍ᵢ₋₁₎ to zᵢ. Joint variables are drawn
  bold. Zero values have no geometry, so they are not drawn.
- **End-effector tag** on the canvas showing the EE coordinates (on by default; it can be
  turned off under "Drawing & view").
- Sidebar readouts: the end-effector position (origin of the last frame), every link
  transform T⁰₁, T¹₂, … (written with both indices on the right), the final
  T⁰ₙ = A₁A₂…Aₙ, and a DH table. All use real (unscaled) values.
- The sidebar can be collapsed with the ◀ button at the top-left of the canvas. **Fit view** frames the robot.
- **View cube** (top-right, like Blender's): click a face to go to the Front, Back, Left,
  Right, Top or Bottom view, with a smooth transition. The edges (e.g. Front-Right) and
  corners (e.g. Top-Front-Right) are clickable too, giving 26 views in all. Hovering an edge
  or corner highlights it on every face it touches. **Iso** returns to the isometric view.
  Front looks along +Y, Right along −X, and Top looks down −Z.
- **Projection**: the **Ortho / Persp** toggle switches between orthographic and perspective
  without changing the view direction or the on-screen size. The default is orthographic,
  isometric.
- Keyboard (Blender numpad): 1 Front, 3 Right, 7 Top (with Ctrl: Back, Left, Bottom),
  5 toggles the projection, 0 isometric.
- **View modes** (toggle at the top of the canvas):
  - **Single**: one view you can orbit, with the view cube, projection toggle and axis locks.
  - **Multi-view**: several panes, each with a fixed orthographic camera. Choose them with the
    chips: Top, Iso, Front, Right, Left, Back, Bottom (default: Top and Iso). In a
    pane, drag to pan, scroll to zoom toward the cursor, and double-click to fit. You can't
    orbit. Each pane keeps its own pan and zoom; **Fit view** refits them all.
- Mouse: drag to rotate (turntable), right-drag or Shift-drag to pan, scroll to zoom.
- **Axis locks**: the Rotate and Pan rows under the view cube lock dragging to a world axis.
  - Rotate X/Y/Z turns the view about that axis through the orbit center (the dark dot).
    Drag across the axis; when the axis points at you, drag around the center to roll.
  - Pan X/Y/Z slides the scene only along that axis, following the mouse.
  - You can also hold X, Y or Z while dragging to lock temporarily.
  - Colored guide lines show the locked axes. A locked rotation may roll the view; the view
    cube or Iso brings it back level.

## Presentation mode: "Learn DH"

**▶ Learn DH** (canvas toolbar) starts a guided, animated walkthrough of how to read the DH
parameters off the loaded robot. It works with revolute-only robots; the button is disabled
otherwise. It runs in six steps, with one part per joint:

1. **Joint offsets dᵢ**: each joint turns back and forth, with its drum checkered so the turning
   shows. The part that twists in place about the joint's own axis is checkered too and then
   measured. If nothing twists, dᵢ = 0.
2. **Link lengths aᵢ**: the same, but highlighting the link that swings around the axis
   (its path is traced). If nothing swings, aᵢ = 0.
3. **z axes**: each joint's rotation axis gets its z arrow. Rule 1: along the offset if there is
   one. Rule 2: either way along the axis otherwise. The end-effector frame's z is a free choice.
4. **x axes**: Rule 1: along the link when aᵢ > 0. Rule 2: perpendicular to both zᵢ₋₁ and zᵢ
   when aᵢ = 0. Here zᵢ₋₁ slides to where it meets zᵢ, and right-angle marks show that both
   ±(zᵢ₋₁ × zᵢ) candidates are perpendicular to both axes.

   Wherever a rule allows more than one answer, every option is shown before one is chosen.
   Two directions are shown as ghost arrows ("option 1 / option 2"). For any direction around
   an axis (x₀, or collinear axes), a ghost sweeps around a dashed ring. Axes are drawn on top of
   the robot, so a z axis inside a joint or offset link stays visible. The y axes aren't shown.
5. **Link twists αᵢ**: zᵢ₋₁ slides along xᵢ to meet zᵢ, everything else fades, the camera looks
   straight down xᵢ, and an arc measures the angle.
6. **Joint angles θᵢ**: xᵢ₋₁ and xᵢ slide together on zᵢ₋₁, the camera looks straight down zᵢ₋₁,
   and an arc measures the angle.

A DH table fills in as you go; it can be hidden. The card shows the current step, the rule in
use and how it applies here. Controls: **Next ▶** (→), **◀ Back** (←), **⟲ Rewatch** (R),
**✕ Exit** (Esc). Click a step chip or a part dot to jump anywhere. You can still drag to look
around. Wherever a rule leaves a free choice, the walkthrough picks the model's direction, so the
finished table matches the sidebar's DH table. Exiting restores your view and settings.

## Convention

This uses the standard (distal) DH convention. Joint i moves about or along z₍ᵢ₋₁₎:

```
A_i = Rz(θi) · Tz(di) · Tx(ai) · Rx(αi)
T^0_n = A_1 · A_2 · … · A_n
```

The gripper is mounted at the last frame {n}, with its fingers pointing along xₙ.

## Files

- `js/dh.js`: the DH matrix and forward kinematics (no Three.js)
- `js/main.js`: the scene, robot drawing, UI and readouts
- `js/presentation.js`: the "Learn DH" presentation mode
- `index.html`, `style.css`: layout and styles
