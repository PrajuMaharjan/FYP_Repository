# Net, characters, teams and the rally demo

Everything is in metres. World axes (same as the physics): **X** = court length, **Y** = up, **Z** = court width.
The net is a wall standing at **X = 0** that runs along Z. Side A is X < 0, side B is X > 0.

Scope note: this is a physics demo, not the Sprint 2 rally state machine or the AI. Players run in straight lines,
there is no pass quality, no pathfinding and no scoring. Those are later sprints.

---

## Part A: Build the net

The net's top must be at the same height the physics uses (`CourtSpec.NetHeight`: **2.43 m** men, **2.24 m** women),
measured from the top surface of your floor. If it is not, a ball that the code says clears the net will look like it goes through it,
or a ball that "hit the net" will look like it passed over.

**You do not need a collider or a script on the net.** The ball is moved by script, so Unity colliders cannot stop it. The net is modelled
inside the physics code (`NetCollision`): a wall at X = 0 from the floor up to the tape height, reaching 0.5 m past each side line.
Your net object only has to look right and sit at X = 0 with its top tape at the right height.

### Dimensions

| Part | Primitive | Position (X, Y, Z) | Rotation | Scale |
|---|---|---|---|---|
| Net (empty parent) | Create Empty | 0, 0, 0 | 0, 0, 0 | 1, 1, 1 |
| Post A | Cylinder | 0, 1.275, -5 | 0, 0, 0 | 0.12, 1.275, 0.12 |
| Post B | Cylinder | 0, 1.275, 5 | 0, 0, 0 | 0.12, 1.275, 0.12 |
| Top tape | Cube | 0, 2.395, 0 | 0, 0, 0 | 0.03, 0.07, 10 |
| Bottom tape | Cube | 0, 1.455, 0 | 0, 0, 0 | 0.03, 0.05, 10 |
| Net mesh (front) | Quad | 0, 1.92, 0 | 0, 90, 0 | 10, 0.88, 1 |
| Net mesh (back) | Quad | 0, 1.92, 0 | 0, -90, 0 | 10, 0.88, 1 |
| Antenna A (optional) | Cylinder | 0, 2.83, -4.5 | 0, 0, 0 | 0.01, 0.4, 0.01 |
| Antenna B (optional) | Cylinder | 0, 2.83, 4.5 | 0, 0, 0 | 0.01, 0.4, 0.01 |

These heights assume the floor surface is at Y = 0. If your floor top is higher, add that height to every Y value above.

Why these numbers: a Unity Cylinder is 2 m tall at scale 1, so scale Y 1.275 gives a 2.55 m post standing on the floor.
The top tape is 7 cm tall and centred at 2.395 m, so its top edge is 2.395 + 0.035 = **2.43 m**. The bottom tape is 5 cm tall and the net
mesh fills the gap between the tapes (1.48 m to 2.36 m). The posts stand 0.5 m outside the 4.5 m half-width of the court. The antennae mark the
side boundaries and are decorative for now (the antenna rule is not modelled).

**Women's net (2.24 m):** subtract 0.19 from the Y position of the tapes, net meshes and antennae (not the posts), and tick
*Women Net* on the demo.

### Steps

1. **Floor (if you don't have one):** GameObject > 3D Object > Plane, position 0, 0, 0, scale **1.8, 1, 0.9**. A Plane is 10 x 10 m at scale 1, so this is the 18 x 9 m court.
   The plane must be centred at X = 0, Z = 0 and not rotated, because the code's court is centred on the origin.
   If you use a **Cube** as the floor instead, it can be any thickness and at any height: you will drag it into the demo's **Floor** field (Part C) and the ball will land on its top surface. For a cube, the top is at Position Y + Scale Y / 2.
2. **Parent:** GameObject > Create Empty, rename it `Net`, position 0, 0, 0.
3. **Posts, tapes, antennae:** create each primitive from GameObject > 3D Object, enter the values from the table, and drag each one onto `Net` in the Hierarchy so it becomes a child.
4. **Remove colliders.** Every primitive comes with a collider. Remove them from every net part: the ball is not moved by Unity physics and the players are moved by script, so the colliders do nothing useful.
5. **Net texture.** The project already contains `Assets/Art/Net/NetTexture.png` (a transparent texture with dark cords, one cell = 10 cm). Select it in the Project window and in the Inspector set:
   - Alpha Source: **Input Texture Alpha**, **Alpha Is Transparency** ticked
   - Wrap Mode: **Repeat**
   - click **Apply**
6. **Net material.** Assets > Create > Material, name it `NetMaterial`.
   - **If your project uses URP** (Edit > Project Settings > Graphics shows a Scriptable Render Pipeline asset): Shader *Universal Render Pipeline/Lit*, Surface Type **Opaque**, Render Face **Both**, tick **Alpha Clipping** (threshold 0.5), Base Map = `NetTexture`, Smoothness 0.
   - **If it uses the Built-in pipeline:** Shader *Standard*, Rendering Mode **Cutout**, Albedo = `NetTexture`, Smoothness 0. This shader only draws one side of a Quad, which is why the table has a front and a back Quad.
   - Either way set **Tiling X = 100, Y = 8.8** so each cell is 10 cm.
7. **Apply:** drag `NetMaterial` onto both net-mesh Quads. Give the tapes a plain white material and the posts a grey one.
8. **Check it with the rally demo (Part C):** with **Net Stops Ball** ticked and a short Return Flight Time Min (about 1 s), some returns should visibly stop at the net and drop on the hitter's side. The single-shot `BallPhysicsDriver` does not model the net: a 0.6 s flat shot passes straight through it there.
9. **Optional:** drag the `Net` object into the Project window to make a prefab.

---

## Part B: Set up the characters

1. **Rig.** Select a Low Poly Bros character's model file in the Project window, open the **Rig** tab, set Animation Type to **Humanoid**, click **Apply**. The Starter Assets animations are humanoid, so they only play on humanoid rigs. If Unity shows an error or a red avatar mapping, stop here and tell me what it says.
2. **Materials look pink?** Starter Assets needs URP and the Low Poly Bros materials are made for the Built-in pipeline. Select the character materials and use **Edit > Rendering > Materials > Convert Selected Built-in Materials to URP** (in newer Unity: Window > Rendering > Render Pipeline Converter, Built-in to URP, Material Upgrade).

3. **Place them.** Drag each character into the scene. Suggested layout for six per side (two rows of three, spread out so there is a clearly closest player):
   - Side A: (-3, y, -3), (-3, y, 0), (-3, y, 3), (-6, y, -3), (-6, y, 0), (-6, y, 3)
   - Side B: ( 3, y, -3), ( 3, y, 0), ( 3, y, 3), ( 6, y, -3), ( 6, y, 0), ( 6, y, 3)

   Only X and Z matter: the demo puts every character on the floor surface itself. The side is read from the starting X position, so keep every character clearly on its own side (at least 0.5 m from the net).
4. **Animator.** On each character's **Animator** component:
   - **Controller:** `StarterAssetsThirdPerson` (type that name in the Project window search box to find it)
   - **Avatar:** the character's own avatar (normally filled in automatically)
   - **Apply Root Motion:** unticked
5. **Remove competing components** if present: `ThirdPersonController`, `StarterAssetsInputs`, `PlayerInput`, `CharacterController`, `Rigidbody`. They would fight the script over the character's position. A plain Low Poly Bros model normally has none of them.
6. **Add the script:** select the character, Add Component > **Player Runner**. Its Animator field fills itself in.

### Per-character values (on Player Runner)

| Field | Default | Meaning |
|---|---|---|
| Max Speed | 4.5 | top running speed, m/s |
| Reaction Time | 0.3 | seconds before the player starts moving |
| Reach Radius | 0.6 | how far the ball may be (horizontally) and still be played |
| Foot Offset | 0 | height of the character's pivot above the floor. 0 if the pivot is at the feet. Raise it if the feet sink into the floor, lower it if they float. |

These are placeholders to tune by feel, not measured values. Characters that are inactive in the Hierarchy are not picked up.

### Teams

Make two empty objects, for example `Team 1` and `Team 2`, and put each team's characters underneath. In the demo, drag them into **Team One** and **Team Two** (Part C).
A team is **one side of the net**: every character under Team 1 must start at X < 0 and every character under Team 2 at X > 0 (or the other way round). The demo checks this when it starts and tells you which character is on the wrong side. Parent objects can be anywhere; the code uses each character's **world** position.

---

## Part C: Run the demo

1. Use your sphere as the ball, starting at (-10, 3, 0) above the floor (the ball's start position is where the first serve is hit from). **Set the sphere's Scale to 0.21 on every axis.** A default Unity sphere is 1 m wide; the physics uses a 0.21 m ball, so a default sphere would look like it sinks 0.4 m into the floor.
2. **Untick `BallPhysicsDriver`** (and `ShotChecker` if you added it). They also move the ball.
3. GameObject > Create Empty, rename it `RallyDemo`, Add Component > **Chase Return Demo**.
4. Fill in the fields:

| Field | Meaning |
|---|---|
| Ball | your sphere |
| Floor | your floor object (cube or plane). The ball lands on, and characters stand on, the top of what you can see |
| Players | every character, both teams. **Only needed if you leave the Team fields empty** (ignored, with a warning, if the Teams are set) |
| Serve Target | optional. Where the serve lands. Empty = a seeded random spot on the far side |
| Team One / Team Two | the two parent objects holding each team's characters (see Teams above) |
| Tint Teams, Team One Color, Team Two Color | optional. Colours each team (multiplies the materials' colour; can look odd on textured models) |
| Serve Flight Time | seconds the serve is in the air (5 = a very high, slow arc of about 30 m) |
| Return Flight Time Min / Max | each return takes a random time between these, so arcs differ. Short times are flat and fast and may hit the net. Set **Max to 0** for normal arcs of random height instead |
| Auto Start | start the rally on Play |
| Randomize Seed | ticked = a new random rally every Play (the seed is printed in the Console). Untick to use **Seed** |
| Seed | same seed = same rally, every time, at any frame rate. To replay a rally you liked, copy the seed from the Console |
| Touches Per Side | 1 to 3. 3 = pass, set, attack. 1 = the receiver returns it straight away |
| Net Stops Ball | ticked = a ball that does not clear the net hits it and drops on the hitter's side |
| Chase Until It Lands | ticked = if nobody can reach the ball, the closest player keeps running at it until it hits the floor. Unticked = everyone stops |
| Max Contacts | the rally stops after this many touches (200) |
| Women Net | use the 2.24 m net |

5. Press **Play**. The ball is served; the receiving team passes, sets and attacks it over the net; the other team does the same.

A line of text at the top-left of the Game view shows the latest touch (with the team name) or why the rally ended. The Console logs every contact, net hit and the reason the rally ended.
Right-click the component header > **Start Rally** to restart.

**Camera tip for 5-second flights:** the ball peaks around 30 m, so back the camera right off. Try position 0, 15, -55, rotation 8, 0, 0 and Field of View 60, then adjust. A Trail Renderer on the ball makes the arc easy to see. Or use the free camera (Part D).

### What to expect

- **Only one player runs per ball:** the one standing closest to the spot where it will be played, among those who can get there in time. Everyone else stands still. Ties go to the lower-numbered player.
- **Touches:** touch 1 is a high pass to a spot 2 to 4.5 m from the net, touch 2 a lower set to a spot 1 to 2 m from the net, touch 3 the attack over the net. **Nobody touches twice in a row.** Pass and set targets are chosen so someone else can reach them.
- **Net:** a return, or a serve, whose arc is below the tape when it reaches X = 0 hits the net. Nobody plays it, and the rally ends when it lands on the hitter's side.
- **Unreachable ball:** the Console says where the ball comes down (inside or outside the lines), how far the best-placed player was and how far they could cover. With Chase Until It Lands ticked the closest player runs at the landing spot and arrives too late; otherwise everyone stops. The rally ends when the ball hits the floor.
- Players are **not** sent back to their start spots after a touch, so a team drifts over a long rally. That is one reason rallies end.
- Returns are always aimed **inside the lines**. A ball that lands "out" by eye means the floor or net is not lined up with the code (court centred on the origin, net at X = 0, tape height above the floor surface).

### See the "everybody stops" behaviour

With 12 players and 5-second flights almost every ball is reachable (a player can run about 20 m in that time). To see an unreachable ball:

- **Untick Chase Until It Lands** (so players stop rather than chase), and
- put a Serve Target far outside the court, such as (15, 0, 0), or set Max Speed to **0.1** on every character, or set Return Flight Time Max to about **1.5** and Max Speed to **3**.

### See a net hit

Set **Return Flight Time Min** to about 0.8 and Max to 3, with **Net Stops Ball** ticked. A flat, fast return from close to the net will stop at the tape and drop.

### The floor

The demo reads your **Floor** object: the ball lands, and the characters stand, on the **top of what you can see** (its renderer; if it has none, its collider). The Console prints the height it found, for example *"floor surface is at world Y = 0.500"*.

The ball does not use Unity colliders: the code decides where it lands. So the Floor field matters. If it is empty the demo assumes the floor is at Y = 0, warns you, and a floor whose top is above Y = 0 will swallow the ball.

### Not included (on purpose)

No hit or dig animation (the ball just changes direction at the player's waist height), no jumping, no pass quality, no player avoidance, no return to position, no scoring, no out-of-bounds or touch-count rules. Those belong to later sprints.

---

## Part D: Free-flying camera

Add `Assets/Scripts/Camera/FreeFlyCamera.cs` to the **Main Camera** (Add Component > Free Fly Camera). Click inside the Game view first so it has focus.

| Input | Action |
|---|---|
| Hold right mouse button | look around (cursor hidden and locked while held) |
| W / S | fly forward / back |
| A / D | strafe left / right |
| E / Q | up / down |
| Left Shift | boost (x4) |
| Mouse wheel | faster / slower |
| R | back to where the camera started |

It works with the new Input System (what Starter Assets uses) or the old Input Manager. If the Main Camera also has a **Cinemachine Brain**, untick it or it will keep moving the camera back.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Console: "Animator has no Controller assigned" | Step B4: assign `StarterAssetsThirdPerson` |
| Characters are pink | Built-in materials under URP: convert them (Step B2) |
| Characters slide without a run animation | The Animator Controller has no float `Speed`. Select a character during Play, open Window > Animation > Animator and check the parameters. |
| Characters stand in a T-pose | The rig is not Humanoid or the Avatar is missing (B1, B4) |
| Characters jump to odd positions or drift | A `CharacterController`, `Rigidbody` or Starter Assets controller is still on the character (B5), or Apply Root Motion is ticked |
| Nothing moves | Ball or characters not assigned (Team One / Team Two, or Players), or `BallPhysicsDriver` is still enabled and moving the ball. Read the Console. |
| Console: "starts on the other side of the net" | A character in a Team object has the opposite sign of X from its teammates. Move it. |
| Console: "start on the same side of the net" | Both teams are at X < 0 or both at X > 0 |
| Everyone stops straight away | The ball is out of reach and Chase Until It Lands is off. Check Max Speed and the Console message. |
| Rally ends after a few touches | Read the Console: the ball hit the net, or the unreachable message says how far short the players fell. Raise Max Speed or Return Flight Time Min, or lower Return Flight Time Max. |
| Ball passes through the net | The net's top is not at the height the code uses (Part A), or the single-shot `BallPhysicsDriver` is running (it does not model the net) |
| Ball stops at an invisible net | The code's net is at X = 0 and 2.43 m (2.24 m women) above the floor surface. Line your net up with it. |
| Ball sinks into the floor | Check the Console. Most likely the **Floor** field is empty, or the ball's Scale is not 0.21 (the demo warns about both). |
| Ball floats above the floor | The floor object's renderer is taller than the visible surface (for example a parent object with a big bounding box), or the ball is smaller than 0.21. |
| Characters stand in the floor or hover | Adjust **Foot Offset** on their Player Runner |
| Every rally is different and you want one back | Copy the seed from the Console into **Seed** and untick **Randomize Seed** |
| Ball is tiny or off screen | 5-second flights peak around 30 m: move the camera back or use the free camera |