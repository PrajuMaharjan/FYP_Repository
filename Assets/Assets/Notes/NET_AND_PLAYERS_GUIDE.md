# Net, characters and the chase-and-return demo

Everything is in metres. World axes (same as the physics): **X** = court length, **Y** = up, **Z** = court width.
The net is a wall standing at **X = 0** that runs along Z. Side A is X < 0, side B is X > 0.

Scope note: this is a Sprint 1 physics demo. Only the player closest to where the ball will come down runs, in a straight
line, and returns the ball using the launch solver. There is no rally state machine, pass quality, pathfinding or AI yet; those are later sprints.

---

## Part A: Build the net

The net's top must be at the same height the physics uses (`CourtSpec.NetHeight`: **2.43 m** men, **2.24 m** women),
otherwise a ball that the code says clears the net will look like it goes through it.

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

Why these numbers: a Unity Cylinder is 2 m tall at scale 1, so scale Y 1.275 gives a 2.55 m post standing on the floor.
The top tape is 7 cm tall and centred at 2.395 m, so its top edge is 2.395 + 0.035 = **2.43 m**. The bottom tape is 5 cm tall and the net
mesh fills the gap between the tapes (1.48 m to 2.36 m). The posts stand 0.5 m outside the 4.5 m half-width of the court. The antennae mark the
side boundaries and are decorative for now (the antenna rule is not modelled).

**Women's net (2.24 m):** subtract 0.19 from the Y position of the tapes, net meshes and antennae (not the posts), and tick
*Women Net* on the demo.

### Steps

1. **Floor (if you don't have one):** GameObject > 3D Object > Plane, position 0, 0, 0, scale **1.8, 1, 0.9**. A Plane is 10 x 10 m at scale 1, so this is the 18 x 9 m court.
   If you use a **Cube** as the floor instead, it can be any thickness and at any height: you will drag it into the demo's **Floor** field (Part C) and the ball will land on its top surface. For a cube, the top is at Position Y + Scale Y / 2.
2. **Parent:** GameObject > Create Empty, rename it `Net`, position 0, 0, 0.
3. **Posts, tapes, antennae:** create each primitive from GameObject > 3D Object, enter the values from the table, and drag each one onto `Net` in the Hierarchy so it becomes a child.
4. **Remove colliders.** Every primitive comes with a collider (Capsule, Box or Mesh Collider). Remove them from every net part: the ball is not moved by Unity physics and the players are moved by script, so the colliders do nothing useful.
5. **Net texture.** The project already contains `Assets/Art/Net/NetTexture.png` (a transparent texture with dark cords, one cell = 10 cm). Select it in the Project window and in the Inspector set:
   - Alpha Source: **Input Texture Alpha**, **Alpha Is Transparency** ticked
   - Wrap Mode: **Repeat**
   - click **Apply**
6. **Net material.** Assets > Create > Material, name it `NetMaterial`.
   - **If your project uses URP** (Edit > Project Settings > Graphics shows a Scriptable Render Pipeline asset): Shader *Universal Render Pipeline/Lit*, Surface Type **Opaque**, Render Face **Both**, tick **Alpha Clipping** (threshold 0.5), Base Map = `NetTexture`, Smoothness 0.
   - **If it uses the Built-in pipeline:** Shader *Standard*, Rendering Mode **Cutout**, Albedo = `NetTexture`, Smoothness 0. This shader only draws one side of a Quad, which is why the table has a front and a back Quad.
   - Either way set **Tiling X = 100, Y = 8.8** so each cell is 10 cm.
7. **Apply:** drag `NetMaterial` onto both net-mesh Quads. Give the tapes a plain white material and the posts a grey one.
8. **Check it:** hit Play with the `BallPhysicsDriver` demo and flight time 1.2. The ball should pass clearly above the white tape. Set the flight time to 0.6 and it should visibly go through the net. That is the "illegal crossing" the tests describe.
9. **Optional:** drag the `Net` object into the Project window to make a prefab.

---

## Part B: Set up the characters

1. **Rig.** Select a Low Poly Bros character's model file in the Project window, open the **Rig** tab, set Animation Type to **Humanoid**, click **Apply**. The Starter Assets animations are humanoid, so they only play on humanoid rigs. If Unity shows an error or a red avatar mapping, stop here and tell me what it says.
2. **Place them.** Drag each character into the scene. Suggested layout for six per side (two rows of three, spread out so there is a clearly closest player):
   - Side A: (-3, y, -3), (-3, y, 0), (-3, y, 3), (-6, y, -3), (-6, y, 0), (-6, y, 3)
   - Side B: ( 3, y, -3), ( 3, y, 0), ( 3, y, 3), ( 6, y, -3), ( 6, y, 0), ( 6, y, 3)

   Only X and Z matter: the demo puts every character on the floor surface itself. The side is read from the starting X position, so keep every character clearly on its own side (at least 0.5 m from the net).
3. **Animator.** On each character's **Animator** component:
   - **Controller:** `StarterAssetsThirdPerson` (type that name in the Project window search box to find it)
   - **Avatar:** the character's own avatar (normally filled in automatically)
   - **Apply Root Motion:** unticked
4. **Remove competing components** if present: `ThirdPersonController`, `StarterAssetsInputs`, `PlayerInput`, `CharacterController`, `Rigidbody`. They would fight the script over the character's position. A plain Low Poly Bros model normally has none of them.
5. **Add the script:** select the character, Add Component > **Player Runner**. Its Animator field fills itself in.

### Per-character values (on Player Runner)

| Field | Default | Meaning |
|---|---|---|
| Max Speed | 4.5 | top running speed, m/s |
| Reaction Time | 0.3 | seconds before the player starts moving |
| Reach Radius | 0.6 | how far the ball may be (horizontally) and still be played |
| Foot Offset | 0 | height of the character's pivot above the floor. 0 if the pivot is at the feet. Raise it if the feet sink into the floor, lower it if they float. |

These are placeholders to tune by feel, not measured values.

---

## Part C: Run the demo

1. Use your sphere as the ball, starting at (-10, 3, 0) above the floor (the ball's start position is where the first serve is hit from). **Set the sphere's Scale to 0.21 on every axis.** A default Unity sphere is 1 m wide; the physics uses a 0.21 m ball, so a default sphere would look like it sinks 0.4 m into the floor.
2. **Untick `BallPhysicsDriver`** (and `ShotChecker` if you added it). They also move the ball.
3. GameObject > Create Empty, rename it `RallyDemo`, Add Component > **Chase Return Demo**.
4. Drag the ball into **Ball**, your floor object (the cube or plane) into **Floor**, and set **Players** size to 12 and drag all twelve characters in.
5. Flight times: **Serve Flight Time** and **Return Flight Time** are both 5 seconds by default. Set Return Flight Time to 0 to get normal-looking arcs of random height instead.
6. Optional: drag a Target object into **Serve Target** to choose where the serve lands. Leave it empty for a seeded random spot.
7. Press **Play**. The ball is served, the closest player on the other side runs under it and returns it, then the other team does the same.

A line of text at the top-left of the Game view shows the latest return, or why the rally ended. The Console logs each return.
Right-click the component header > **Start Rally** to restart.

**Camera tip for 5-second flights:** the ball peaks around 30 m, so back the camera right off. Try position 0, 15, -55, rotation 8, 0, 0 and Field of View 60, then adjust. A Trail Renderer on the ball makes the arc easy to see.

### What to expect

- **Only the closest player runs.** Closest means standing nearest (straight-line, on the floor) to the spot where the ball will be played, among the players who can get there in time. Everyone else stands still. Ties go to the lower-numbered player in the list.
- **If nobody can get under the ball, every player stops** and stays where they are, and the rally ends when the ball lands.
- Same **Seed** = same rally every time, at any frame rate. Change the seed for a different rally.
- The rally ends after **Max Contacts** returns (default 200).

### See the "everybody stops" behaviour

With 12 players and 5-second flights almost every ball is reachable (a player can run about 20 m in that time), so the stop is rare. To see it:

- Put a Serve Target far outside the court, such as (15, 0, 0), or
- Set Max Speed to **0.1** on every character (nobody can get anywhere in time), or
- Set Return Flight Time to **1.5** and Max Speed to **3**: rallies run for a few returns, then end when a ball is out of reach.

### The floor

The demo reads your **Floor** object: the ball lands, and the characters stand, on the **top of what you can see** (its renderer; if it has none, its collider). The Console prints the height it found, for example *"floor surface is at world Y = 0.500"*.

The ball does not use Unity colliders: the code decides where it lands. So the Floor field matters. If it is empty the demo assumes the floor is at Y = 0, warns you, and a floor whose top is above Y = 0 will swallow the ball.

### Not included (on purpose)

No hit or dig animation (the ball just changes direction at the player's waist height), no jumping, no ball-net collision,
players do not avoid each other, no scoring, no out-of-bounds rule. Those belong to later sprints.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Console: "Animator has no Controller assigned" | Step B3: assign `StarterAssetsThirdPerson` |
| Characters slide without a run animation | The Animator Controller has no float `Speed`. Select a character during Play, open Window > Animation > Animator and check the parameters. Tell me what you see. |
| Characters stand in a T-pose | The rig is not Humanoid or the Avatar is missing (B1, B3) |
| Characters jump to odd positions or drift | A `CharacterController`, `Rigidbody` or Starter Assets controller is still on the character (B4), or Apply Root Motion is ticked |
| Nothing moves | Ball or Players not assigned, or `BallPhysicsDriver` is still enabled and moving the ball. Read the Console. |
| Everyone stops straight away | The serve target is out of reach: check Max Speed and where the target is |
| Ball passes through the net | Expected if the shot is too flat. The ball does not collide with the net. |
| Ball sinks into the floor | Check the Console. Most likely the **Floor** field is empty, or the ball's Scale is not 0.21 (the demo warns about both). |
| Ball floats above the floor | The floor object's renderer is taller than the visible surface (for example a parent object with a big bounding box), or the ball is smaller than 0.21. |
| Characters stand in the floor or hover | Adjust **Foot Offset** on their Player Runner |
| Ball is tiny or off screen | 5-second flights peak around 30 m: move the camera back (see Part C) |
