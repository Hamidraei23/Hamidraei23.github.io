# PID game

The third control tab runs a ball-balancing challenge with a 0.2 × 0.2 m
plate and a solid sphere of radius 0.01 m. Gravity is 9.81 m/s². The ball
starts at (25, −20) mm on a level plate. The six gains control the plate's
roll and pitch; yaw and the plate's world position remain fixed.

The challenge starts at joint angles [−0.437, −1.002, 0.583, −2.362,
−0.958, 3.009, 0.764] rad. Joint 3 differs from the supplied screenshot
by −0.10 rad (about −5.7°) to allow the full tilt range.

The plate is grasped through a rigid fixture under one edge. Its center is
100 mm ahead of the fingertip point in the horizontal approach direction
and 18 mm above it. Each commanded tilt transforms both the gripper
position and orientation around that center. IK is solved with a 10 μm
position tolerance and a 0.0001 rad orientation tolerance. The gripper
therefore follows an arc while the plate center stays fixed. Joint,
Cartesian, demo, gripper and saved-trajectory controls cannot drive the arm
while this tab is active. Leaving the tab restores the previous arm pose.

## Desired targets and live joints

Controlled point mode accepts a click on the plate map or X/Y coordinates
in millimeters, anywhere from −100 to +100 mm. Center target sets both to
zero. With the map focused, arrows move the target 1 mm, Shift+arrow 10 mm,
and Home selects the center. A point can be changed while the game runs.

Drawn trajectory mode accepts a mouse or touch drag on the top-down plate.
The drawing is shown in purple on both the map and the 3D plate; the cyan
ring is the desired target and the gold ball is the measured position.
The duration is adjustable from 0.5 to 120 simulated seconds (default 10).
Pointer capture allows a drag to finish outside the map; coordinates are
clamped to the plate. Canceled drawings retain the previous path.

The path is a bounded polyline parameterized by arc length, independent of
pointer sampling speed. A quintic time law eases the reference to rest at
both ends. Its tangent supplies the reference velocity; sharp drawn
corners retain changes in direction. The ball tracks this moving reference
through the PID and dynamics, rather than being moved directly onto the
path. Timing or gains that exceed the plant's capabilities can cause
tracking error or a fall. After the duration, the target holds its last
point. Pause freezes both physics and path time. Restart path rewinds the
reference without resetting the ball. Reset and a fall restart the ball,
robot and path clock, retaining the drawing, duration and gains. Drawing,
changing mode/duration and clearing a path pause the challenge.

Seven compact joint meters show the actual IK joint angles relative to
their URDF limits. Their numbers follow the Degrees/Radians selector and
refresh once per rendered simulation frame, outside the physics substeps.

## Controller

With ball position x/y and desired position xd/yd in plate coordinates,
define ex = x−xd, ey = y−yd, dex = vx−vxd and dey = vy−vyd:

- Pitch command = −(Kp·ex + Ki·integral(ex) + Kd·dex).
- Roll command = +(Kp·ey + Ki·integral(ey) + Kd·dey).

The default desired position is the center, with zero desired velocity.

The signs differ because positive pitch accelerates the ball along +X,
whereas positive roll accelerates it along −Y. Gains use radians, meters
and seconds regardless of the joint display's angle unit.

Each command saturates at ±25°. Conditional integration prevents windup.
A critically damped angular servo (18 rad/s natural frequency), limited
to 0.8 rad/s and 4 rad/s², turns angle commands into continuous plate
motion. This servo is an ideal actuator model, not simulated robot motor
torques. Changing gains clears the integrals. A random push adds a planar
velocity vector at the selected push speed (0.1–0.4 m/s, in 0.01 m/s steps;
default 0.1 m/s) to the ball's existing velocity. The direction remains random,
and the push respects the rolling angular-velocity constraint.

## Rolling dynamics

The ball uses ideal rolling without slipping, with solid-sphere inertia
I = (2/5)·m·R². Mass cancels out. There is no artificial planar damping or
raised wall. The sphere's rotation is rendered with two visible bands.
Static friction is assumed sufficient to maintain no slip.

In the frame rotating with the plate, let r = (x,y,R), u = (vx,vy,0),
Ω be plate angular velocity, α its angular acceleration, n = (0,0,1),
and w the sphere's angular velocity expressed in this frame. With k = 2/5:

```text
w_tangent = Ω_tangent + (n × u) / R
w_dot_z   = −(Ω × w)_z
C         = 2 Ω × u + α × r + Ω × (Ω × r)
(1+k) u_dot_tangent = g_tangent − C_tangent
                     + k R [n × (α + Ω × w)]_tangent
N/m       = C_z − g_z
```

These equations follow from translation, torque about the sphere's
center, and the no-slip contact constraint. They include the plate's
angular acceleration, centrifugal and Coriolis terms, and sphere spin.
On a stationary incline they reduce to a = (5/7)·g·sin(θ), consistent with
[OpenStax's rolling-motion derivation](https://openstax.org/books/university-physics-volume-1/pages/11-1-rolling-motion).

Integration uses fixed 1/240 s steps, independent of the rendering rate,
with semi-implicit Euler updates. A frame contributes at most 50 ms;
hidden browser tabs suspend the simulation rather than accumulating
a catch-up burst. The PID game continues when its 3D canvas is offscreen,
so the top-down map remains playable on mobile; offscreen 3D rendering is skipped. No general robot collision dynamics are implemented.

Contact ends when the contact point leaves the square (|x| or |y| > 0.1 m)
or the required normal force becomes nonpositive. The game immediately
resets the robot, plate, ball velocity, ball spin and controller integrals.
It retains the six gains and increments the fall count. If IK cannot reach
a requested tilt, the dynamics do not advance and the challenge pauses
with a reset message.

## Checks

From the repository root:

```sh
node blog/franka-playground/tests/pid-physics.test.mjs
node blog/franka-playground/tests/pid-targets.test.mjs
node blog/franka-playground/tests/trajectory.test.mjs
```

The physics tests cover static inclines, no-slip contact velocity,
acceleration from plate rotation, impulse magnitude, convergence,
default-controller recovery, contact loss, actuator bounds, windup and
rejected IK steps. Browser checks additionally exercise the actual URDF,
fixed-center rotation, game controls, resets, tab transitions and mobile
layout. The viewer's existing bundled libraries are passed into
`assets/pid-game.js`; the physics kernel has no rendering dependencies.

The browser regression is `tests/pid-game.browser.cjs`. Serve the repository
with `python3 -m http.server 8765`, then run it with Node in an environment
where Playwright and its Chromium browser are installed. Optional variables:
`PID_GAME_URL` overrides the page URL, `PLAYWRIGHT_MODULE` selects an existing
Playwright installation, and `PLAYWRIGHT_CHROMIUM_EXECUTABLE` selects an existing
Chromium executable. Test-only inspection hooks are injected into the served
module; they are not exposed by the application. Desktop/mobile screenshots
are written to the system temporary directory.

Target drawing, timing, tracking and joint meters are checked by
`tests/pid-targets.browser.cjs`, using the same browser environment variables.
