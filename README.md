# Flip Lander

ComputerCraft autopilot for a Create Aeronautics craft with 8 vector thrusters (Create Propulsion). It flies to a target, flips upside down above it, descends inverted, and fires 4 redstone outputs together when the Optical Sensor reads the trigger distance.

- **4 bottom vector thrusters**: lift, altitude, descent while upright.
- **4 side vector thrusters**: steering, leveling, the flip, and braking the fall while upside down (their nozzles tilt to push up).
- **Optical Sensor**: on the craft's **top** face, pointing up. Once the craft is upside down it looks at the ground.
- **CC: Sable** is needed for the craft's rotation. Without it the program won't fly.

## Install

```
wget https://raw.githubusercontent.com/levisnakes/cc-flip-lander/main/flip.lua flip.lua
```

Then `flip setup`: each thruster's nozzle wiggles in turn and you press **B** (bottom), **S** (side) or **I** (ignore).

## Fly

```
flip <x> <y> <z>
```

Or just `flip` and type the target. Y is the ground height at the target. Hold **Ctrl+T** to abort; every output turns off.

1. **Climb** 20 blocks off the pad.
2. **Calibrate** (first flight only, about 2 minutes): it pulses every thruster and nozzle to learn how each one pushes and turns the craft. The result is saved to `flip.cal`, so later flights skip this. Run `flip recal` after you rebuild the craft.
3. **Cruise** at `CRUISE_Y` to the target.
4. **Approach**: sinks to `FLIP_HEIGHT` (40) above the target and stops.
5. **Flip**: bottom thrusters fade out, side nozzles roll the craft over in about `FLIP_SECONDS`.
6. **Inverted descent** at `DESCENT_SPEED`, slowing to `FINAL_SPEED` near the ground.
7. **Deploy**: when the sensor reads `DEPLOY_DISTANCE` (2.5), all 4 `CHARGES` switch on in the same tick for `CHARGE_SECONDS`.

## Safety

- It only fires upside down (over 120° tilt), and only within `DEPLOY_RADIUS` (6 blocks) of the target. Outside that it lands without firing.
- `ARMED = false` lands upside down without firing.
- If the side thrusters can't turn it over, it gives up after 4× `FLIP_SECONDS`, lands upright and doesn't fire.

## Build notes

- Upside down, only the side nozzles hold the craft up, and a nozzle tilts at most 30°, so only half of each side thruster's thrust points up. **Together the 4 side thrusters need about 2× the craft's weight.** If not, the flip fails safely.
- Put the bottom thrusters around the center of mass, one per corner works best.
- Thrusters must be on the computer's network (touching it or on wired modems).

## Settings

Settings are at the top of `flip.lua`. To keep your own values across updates, put them in `/config.lua`, for example:

```
CRUISE_Y = 150
DEPLOY_DISTANCE = 3
CHARGES = {
  { relay = "redstone_relay_0", side = "front" },
  { relay = "redstone_relay_0", side = "back" },
  { relay = "redstone_relay_0", side = "left" },
  { relay = "redstone_relay_0", side = "right" },
}
```

`CHARGE_THRUSTERS` sets the thrusters at the moment of firing: `"cut"` (all off, default), `"hold"` (keep braking) or `"full"` (all full power, nozzles centered).

If it wobbles, raise `ATT_D`. If it's slow to follow the flip, raise `ATT_P`.

## Updates

It checks GitHub on start and every 15 minutes (never mid-flight), keeps your settings, and restarts. Turn off with `AUTO_UPDATE = false`.

## Flight log

Every flight writes `flip.log`: calibration results, then position, tilt, sensor and every thruster's output 5 times a second. Share it with `pastebin put flip.log`.
