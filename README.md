# Flip Lander

ComputerCraft autopilot for a Create Aeronautics craft. It flies to a target, flips upside down above it, descends inverted, and fires 4 redstone outputs together when the Optical Sensor reads the trigger distance. Works with regular Create Propulsion thrusters (`setPower`) and vector thrusters, mixed freely.

Thruster roles (you pick them in `flip setup`):

- **Bottom (B)**: lift and altitude while upright. 4 at the corners works well.
- **Side (S)**: steering and leveling. On a tall craft, mount them **near the top** so they can turn the craft over.
- **Top / nose (T)**: point the other way (exhaust out the top while upright). Once the craft is upside down they hold it up and brake the descent. Without them it flips and just falls, which lands far off target on a tall craft, so the charges won't fire.
- **Optical Sensor**: on the **nose**, pointing straight out the top, so it faces the ground once the craft is flipped.
- **CC: Sable** is needed for the craft's position and rotation.

## Install

```
wget https://raw.githubusercontent.com/levisnakes/cc-flip-lander/main/flip.lua flip.lua
```

Then run `flip setup`. Each thruster puffs at low power (or a vector nozzle wiggles) in turn, and you press **B**, **S**, **T** or **I** (ignore). For each side thruster it also asks which way its flame points (**N/E/S/W**, see F3). Right after lift-off it puffs one side thruster and checks those answers against what it measures (if they were typed backwards, it flips them). After that the side thrusters hold the craft level.

## Fly

```
flip <x> <y> <z>
```

Or just `flip` and type the target. Y is the ground height at the target. Hold **Ctrl+T** to abort; every output turns off.

1. **Calibrate** (first flight only, a few seconds): right after lift-off it puffs one side thruster to learn how hard the side thrusters turn the craft and which way the setup directions run. It works out the rest from those directions and the hover power, so the leveling never has to switch off for long. Nothing levels it before that first puff, so **the bottom thrusters must sit evenly around the center of mass**, or it tips on take-off. The result is saved to `flip.cal`. Run `flip recal` after rebuilding the craft.
2. **Climb** 20 blocks.
3. **Cruise** to the target.
4. **Approach**: sinks to `FLIP_HEIGHT` (160) above the target and holds still.
5. **Pop-up and flip**: climbs fast, then rolls over. A 12-block craft takes about 7 seconds and drops around 60 blocks doing it, which is why `FLIP_HEIGHT` is high. With no nose engines it then falls upside down and can't steer, and the flip throws it sideways the same way every time. Each drop saves where it landed (`flip.aim`), and the next flight flips that far short of the target, so it lands closer every flight. Expect the first one or two drops to miss and not fire (`DEPLOY_RADIUS`).
6. **Inverted descent**: the nose engines brake and steer it back over the target, slowing to `FINAL_SPEED` near the ground.
7. **Deploy**: when the sensor reads `DEPLOY_DISTANCE` (2.5), all 4 `CHARGES` switch on in the same tick.

A tall craft steers like a rocket: it leans so the main engines push it where it should go, and the side thrusters just hold the lean. It picks this mode automatically from the calibration.

## Weight and thrust

- **Bottom engines together need about 1.3× the craft's weight.** With 4 × 480 pN = 1,920 pN, keep the craft at or under ~1,500 pN *with fuel and nose engines on board*. If it's heavier, it refuses to fly after calibration ("Too heavy") and lands.
- **Nose engines together need at least 1.15× the weight**, or it lands upright instead of flipping. 4 × 480 pN is enough for ~1,500 pN.
- Check the craft's total weight again after adding the nose engines and fuel tanks.

## Safety

- Upright and past `MAX_TILT` (35°), every thruster cuts out and the flight stops, so a tipped craft can't fly off sideways.
- It only fires upside down and within `DEPLOY_RADIUS` (6 blocks) of the target. Otherwise it lands without firing.
- `ARMED = false` lands upside down without firing (or upright, if there are no nose engines).
- If it can't turn over, it gives up and lands upright, charges off.

## Settings

Settings are at the top of `flip.lua`. To keep your own values across updates, put them in `/config.lua`, for example:

```
CRUISE_Y = 250
DEPLOY_DISTANCE = 3
CHARGES = {
  { relay = "redstone_relay_0", side = "front" },
  { relay = "redstone_relay_0", side = "back" },
  { relay = "redstone_relay_0", side = "left" },
  { relay = "redstone_relay_0", side = "right" },
}
```

`CHARGE_THRUSTERS` sets the thrusters at the moment of firing: `"cut"` (all off, default), `"hold"` (keep braking) or `"full"`.

If it wobbles, raise `ATT_D`. If it leans too much while steering, lower `MAX_LEAN`.

## Updates and logs

It checks GitHub on start and every 15 minutes (never mid-flight), keeps your settings, and restarts. Turn off with `AUTO_UPDATE = false`.

Every flight writes `flip.log` (calibration results, then position, tilt, sensor and every thruster 5 times a second). Share it with `pastebin put flip.log`.

## One-time copies

For drones that fly a single delivery: set up and train one master, then copy its settings to the rest.

1. **Train the master.** Fly test deliveries with the cargo weight you'll really carry, until a drop lands on target and fires (about 3 flights; each drop corrects the aim). Keep the target the same way round from the drone every time, e.g. always off the same side.
2. **Export:** `flip export`, then `pastebin put flip.pack`.
3. **On each copy:** `pastebin get <code> flip.pack`, then `flip clone`. Each thruster puffs and the screen says what it should be (bottom, or side with its flame direction worked out from the master's real data). Press Y, or N and say what it actually is. If the answers don't add up to the master's layout, nothing is saved.
4. **Park it facing the same way relative to its target.** The learned aim fits deliveries where the target lies in the same direction from the drone as during training (tested within about 45 degrees, at any heading and distance). Before take-off it checks this, and if the target is more than 50 degrees off it says how far to turn the drone and asks before flying.
