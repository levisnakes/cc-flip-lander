-- Flip Lander: flies a Create Aeronautics craft to a target, flips it
-- upside down on the way in and fires 4 outputs when the Optical Sensor
-- reads the trigger distance. Works with regular thrusters (setPower) and
-- vector thrusters (setThrust + nozzle aim), mixed freely.
--
--   bottom thrusters   lift and altitude while upright
--   side thrusters     steering and leveling; mounted away from the center
--                      of mass (e.g. near the top) they also do the flip
--   top (nose) engines optional; point the other way, so once the craft is
--                      upside down they hold it up and brake the descent.
--                      Without them it flips and drops onto the target.
--
-- Usage:  flip <x> <y> <z>    fly to a target (or just  flip  and type it)
--         flip setup          say which thruster is which
--         flip recal          forget the calibration and redo it next flight
-- Hold Ctrl+T to abort: every output switches off.
--
-- Flight plan:
--   CLIMB      bottom thrusters lift it CAL_HEIGHT off the pad
--   CALIBRATE  (first flight only) pulses every thruster to learn how each
--              one pushes and turns the craft; saved to flip.cal
--   CLIMB      to CRUISE_Y      CRUISE  to the target X/Z
--   APPROACH   down to FLIP_HEIGHT above the target, stops and hovers
--   FLIP       climbs fast (FLIP_POPUP), then rolls it over (180 deg)
--   INVERTED   nose engines brake the descent  (DROP without them: it falls)
--   DEPLOY     sensor reads DEPLOY_DISTANCE: all 4 charge outputs fire at once

VERSION = "2.0.0"

-- ======================== SETTINGS ===========================
-- To keep your own values across updates, put them in /config.lua
-- (same lines, e.g.  CRUISE_Y = 150 ).

-- Thruster names. Leave empty and run  flip setup  to pick them in-game.
BOTTOM_THRUSTERS = {}
SIDE_THRUSTERS = {}
TOP_THRUSTERS = {}

CRUISE_Y = 180          -- altitude to fly at
CRUISE_SPEED = 12       -- top horizontal speed (blocks/s)
CLIMB_SPEED = 6         -- top climb/sink speed while upright (blocks/s)
ARRIVE_RADIUS = 2       -- blocks from the target X/Z counted as "over it"

-- The flip. It hovers FLIP_HEIGHT above the target Y, fades out the bottom
-- thrusters and rolls over. It drops a little while doing it.
FLIP_HEIGHT = 160        -- a 12-block craft drops ~60 blocks flipping and needs room to brake
FLIP_SECONDS = 2.5      -- shortest flip; slower if the thrusters can't turn it that fast
-- Tall crafts (side thrusters far from the center of mass) climb at this
-- speed (blocks/s) right before flipping, so the craft is still rising while
-- it's on its side and nothing holds it up. 0 = off.
FLIP_POPUP = 8
FLIP_AXIS = "auto"      -- "auto", "x" or "z": ship axis to roll around

-- Inverted descent (nose engines brake the fall)
DESCENT_SPEED = 5       -- blocks/s
FINAL_SPEED = 1.5       -- blocks/s for the last SLOW_DISTANCE blocks
SLOW_DISTANCE = 10

-- Optical Sensor(s): put one on the craft's TOP face pointing up, so it
-- looks at the ground once the craft is upside down. All sensors found are
-- read; the closest hit counts.
DEPLOY_DISTANCE = 2.5   -- fire when the sensor reads this close (blocks)
ARMED = true            -- false: land upside down without firing
-- Never fires further than this from the target X/Z (blocks). If it comes
-- down outside it, it lands upside down without firing.
DEPLOY_RADIUS = 6

-- Charge outputs, all switched on in the same tick. relay = nil means the
-- computer's own side; otherwise a Redstone Relay's network name.
CHARGES = {
  { relay = nil, side = "front" },
  { relay = nil, side = "back" },
  { relay = nil, side = "left" },
  { relay = nil, side = "right" },
}
CHARGE_STRENGTH = 15
CHARGE_SECONDS = 1      -- how long the pulse stays on
-- Thrusters at the moment of firing: "cut" (all off), "hold" (keep braking)
-- or "full" (every thruster full power, nozzles centered).
CHARGE_THRUSTERS = "cut"

-- Calibration (first flight; delete flip.cal or run  flip recal  to redo)
CAL_HEIGHT = 20         -- climb this far off the pad first
HOVER_GUESS = 0.6       -- starting bottom throttle (0..1); it learns the real one
CAL_SIDE_THROTTLE = 0.5 -- vector side thruster throttle while its nozzle is tested
CAL_STEP = 0.2          -- throttle change per test pulse
CAL_AIM = 0.5           -- nozzle aim per test pulse (-1..1)
CAL_PULSE = 0.6         -- seconds per pulse

-- Controller tuning
POS_GAIN = 0.6          -- position error -> wanted speed
VEL_GAIN = 1.8          -- speed error -> wanted acceleration
VEL_I = 0.4             -- learns steady pushes (wind, off-center weight)
MAX_ACCEL = 8           -- blocks/s^2, horizontal
MAX_LEAN = 20           -- most it leans to steer (degrees), tall crafts
ATT_P = 5               -- attitude stiffness; raise if it lags the flip
ATT_D = 4               -- attitude damping; raise if it wobbles
SIDE_MIN = 0.15         -- vector side thrusters never go below this in flight,
                        -- so the nozzles always have something to steer

AUTO_UPDATE = true

-- ===================== END OF SETTINGS =======================

if fs and fs.exists("/config.lua") then
  local f, err = loadfile("/config.lua", nil, _ENV)
  if not f then error("/config.lua: " .. err, 0) end
  f()
end

local CFG_FILE, CAL_FILE, LOG_FILE = "flip.cfg", "flip.cal", "flip.log"
local VECTOR_TYPES = { vector_thruster = true, creative_vector_thruster = true, liquid_vector_thruster = true }
local REGULAR_TYPES = { thruster = true, creative_thruster = true, solid_fuel_thruster = true, ion_thruster = true }

-- ============================================================ helpers

local function now() return os.epoch("utc") / 1000 end
local function clamp(v, lo, hi) if v < lo then return lo elseif v > hi then return hi end return v end
local function v3(x, y, z) return { x = x or 0, y = y or 0, z = z or 0 } end
local function add(a, b) return v3(a.x + b.x, a.y + b.y, a.z + b.z) end
local function sub(a, b) return v3(a.x - b.x, a.y - b.y, a.z - b.z) end
local function mul(a, s) return v3(a.x * s, a.y * s, a.z * s) end
local function dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
local function cross(a, b) return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x) end
local function len(a) return math.sqrt(dot(a, a)) end
local UP = v3(0, 1, 0)

-- Rotate v by quaternion q (ship -> world), or by its inverse (world -> ship).
local function rotate(q, v, inverse)
  local qx, qy, qz, qw = q.x, q.y, q.z, q.w
  if inverse then qx, qy, qz = -qx, -qy, -qz end
  local tx = 2 * (qy * v.z - qz * v.y)
  local ty = 2 * (qz * v.x - qx * v.z)
  local tz = 2 * (qx * v.y - qy * v.x)
  return v3(v.x + qw * tx + (qy * tz - qz * ty),
            v.y + qw * ty + (qz * tx - qx * tz),
            v.z + qw * tz + (qx * ty - qy * tx))
end

local function qmul(a, b)
  return { w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
           x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
           y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
           z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w }
end

local function readFile(path)
  if not fs.exists(path) then return nil end
  local f = fs.open(path, "r")
  local t = f.readAll()
  f.close()
  return t
end

local function writeFile(path, text)
  local f = fs.open(path, "w")
  f.write(text)
  f.close()
end

local logFile, logStart = nil, now()
local function log(fmt, ...)
  if not logFile then return end
  logFile.writeLine(string.format("%7.2f ", now() - logStart) .. string.format(fmt, ...))
  logFile.flush()
end

-- Every peripheral once (two modems on one network list each name twice).
local function peripheralNames()
  local seen, out = {}, {}
  for _, name in ipairs(peripheral.getNames()) do
    if not seen[name] then seen[name] = true; out[#out + 1] = name end
  end
  return out
end

-- ============================================================ state

local S = {
  mode = "READY", status = "",
  pos = nil, vel = v3(), lastFix = 0,
  orient = { x = 0, y = 0, z = 0, w = 1 },
  omega = v3(),          -- angular velocity, ship frame (rad/s)
  gravity = 9.81,
  target = nil, pad = nil,
  ground = nil,          -- latest optical sensor distance (nil = no hit)
  hover = HOVER_GUESS,   -- bottom throttle that holds altitude (uncalibrated)
  velI = v3(),
  navSource = "-",
}

local thrusters = {}     -- list of { name, role = "bottom"/"side", t, ax, ay, out = {...}, cal = {...} }
local sensors = {}
local cal = nil          -- name -> { F0, T0, FX, TX, FY, TY } ship-frame responses
local onSable = false

-- ============================================================ thruster output

-- Power only takes 16 steps and the nozzle 1/15 steps. Each tick's
-- rounding error carries into the next so in-between values average out.
local function dither(th, key, v, steps, lo)
  th.err = th.err or {}
  local want = v + (th.err[key] or 0)
  local q = clamp(math.floor(want * steps + 0.5) / steps, lo, 1)
  th.err[key] = clamp(want - q, -1 / steps, 1 / steps)
  return q
end

-- Sends every thruster's t/ax/ay. Calls run side by side, so it costs one
-- tick even if the mod runs them on the main thread, and unchanged values
-- aren't resent.
local function flush()
  local jobs = {}
  for _, th in ipairs(thrusters) do
    local thrust = math.floor(dither(th, "t", clamp(th.t, 0, 1), 15, 0) * 15 + 0.5)
    local ax = dither(th, "x", clamp(th.ax, -1, 1), 15, -1)
    local ay = dither(th, "y", clamp(th.ay, -1, 1), 15, -1)
    if th.sentT ~= thrust then
      th.sentT = thrust
      local method = th.vector and "setThrust" or "setPower"
      jobs[#jobs + 1] = function() pcall(peripheral.call, th.name, method, thrust) end
    end
    if th.vector and (th.sentX ~= ax or th.sentY ~= ay) then
      th.sentX, th.sentY = ax, ay
      jobs[#jobs + 1] = function() pcall(peripheral.call, th.name, "setVector", ax, ay) end
    end
  end
  if #jobs > 0 then parallel.waitForAll(table.unpack(jobs)) end
end

local function setAll(t)
  for _, th in ipairs(thrusters) do th.t, th.ax, th.ay = t, 0, 0 end
end

local function chargesOff()
  for _, c in ipairs(CHARGES) do
    if c.relay then pcall(peripheral.call, c.relay, "setAnalogOutput", c.side, 0)
    else pcall(rs.setAnalogOutput, c.side, 0) end
  end
end

local function allOff()
  for _, th in ipairs(thrusters) do
    if th.vector then
      pcall(peripheral.call, th.name, "setThrust", 0)
      pcall(peripheral.call, th.name, "setVector", 0, 0)
    else
      pcall(peripheral.call, th.name, "setPower", 0)
    end
    th.t, th.ax, th.ay, th.sentT, th.sentX, th.sentY = 0, 0, 0, 0, 0, 0
  end
  chargesOff()
end

-- ============================================================ sensing

-- Pose every tick from CC: Sable: exact position and rotation.
local function sableLoop()
  local lastQ = nil
  while true do
    local ok, pose = pcall(sublevel.getLogicalPose)
    if ok and pose then
      local p, t = pose.position, now()
      local o = pose.orientation
      local q
      if o.v then q = { x = o.v.x, y = o.v.y, z = o.v.z, w = o.a }
      else q = { x = o.x, y = o.y, z = o.z, w = o.w } end
      if S.pos and t > S.lastFix then
        local dt, a = t - S.lastFix, 0.5
        S.vel = add(mul(S.vel, 1 - a), mul(sub(v3(p.x, p.y, p.z), S.pos), a / dt))
        -- ship-frame angular velocity from the change in rotation
        local inv = { x = -lastQ.x, y = -lastQ.y, z = -lastQ.z, w = lastQ.w }
        local d = qmul(inv, q)
        if d.w < 0 then d = { x = -d.x, y = -d.y, z = -d.z, w = -d.w } end
        S.omega = add(mul(S.omega, 1 - a), mul(v3(d.x, d.y, d.z), 2 * a / dt))
      end
      S.pos, S.orient, S.lastFix, lastQ = v3(p.x, p.y, p.z), q, t, q
      os.queueEvent("fix")
    else
      S.status = "Lost the ship pose"
    end
    sleep(0)
  end
end

-- Without CC: Sable: position from GPS. Attitude can't be measured then,
-- so the craft can't level or flip under control.
local function gpsLoop()
  while true do
    local x, y, z = gps.locate(0.5)
    if x then
      local t = now()
      if S.pos and t > S.lastFix then S.vel = mul(sub(v3(x, y, z), S.pos), 1 / (t - S.lastFix)) end
      S.pos, S.lastFix = v3(x, y, z), t
      os.queueEvent("fix")
    else
      S.status = "NO GPS FIX"
      sleep(0.2)
    end
  end
end

-- Closest optical sensor hit, read continuously.
local function sensorLoop()
  while true do
    local best = nil
    for _, name in ipairs(sensors) do
      local ok, hit = pcall(peripheral.call, name, "hasHit")
      if ok and hit then
        local ok2, d = pcall(peripheral.call, name, "getDistance")
        if ok2 and type(d) == "number" and (not best or d < best) then best = d end
      end
    end
    S.ground = best
    os.queueEvent("sensor")
    sleep(0)
  end
end

local function waitFix() os.pullEvent("fix") end
local function waitSeconds(s)
  local untilT = now() + s
  while now() < untilT do waitFix() end
end

local function bodyUp() return rotate(S.orient, UP) end
local function tiltDeg() return math.deg(math.acos(clamp(bodyUp().y, -1, 1))) end

-- ============================================================ allocation
-- Each thruster is modelled from calibration as
--   force  = t * (F0 + ax*FX + ay*FY)     torque = t * (T0 + ax*TX + ay*TY)
-- (ship frame, as acceleration). Each tick this picks every thruster's
-- throttle and two nozzle angles to get as close as it can to the wanted
-- force and torque, by exact one-variable steps in turn (coordinate descent),
-- starting from last tick's answer.

local W_FORCE, W_TORQUE, AIM_COST, POWER_COST = 1, 3, 0.02, 0.01

local function contribution(th, c)
  local g = add(add(c.F0, mul(c.FX, th.ax)), mul(c.FY, th.ay))
  local h = add(add(c.T0, mul(c.TX, th.ax)), mul(c.TY, th.ay))
  return g, h
end

-- limits: role -> { lo, hi } throttle bounds; missing role = off
-- wForce: how much getting the force right matters next to the torque,
-- per ship axis ({x=, y=, z=}); default W_FORCE on all three
local function allocate(wantF, wantT, limits, wForce)
  local W = wForce or v3(W_FORCE, W_FORCE, W_FORCE)
  local function dotF(a, b) return W.x * a.x * b.x + W.y * a.y * b.y + W.z * a.z * b.z end
  local F, T = v3(), v3()
  local act = {}
  for _, th in ipairs(thrusters) do
    local lim = limits[th.role]
    local c = cal[th.name]
    if lim and c then
      local lo = lim[1]
      if th.vector and th.role == "side" then lo = math.min(math.max(lo, SIDE_MIN), lim[2]) end
      if not th.vector then th.ax, th.ay = 0, 0 end
      th.t = clamp(th.t, lo, lim[2])
      act[#act + 1] = { th = th, c = c, lo = lo, hi = lim[2] }
      local g, h = contribution(th, c)
      F, T = add(F, mul(g, th.t)), add(T, mul(h, th.t))
    else
      th.t, th.ax, th.ay = 0, 0, 0
    end
  end
  for _ = 1, 6 do
    for _, a in ipairs(act) do
      local th, c = a.th, a.c
      -- nozzle axes (only matter while the thruster is running)
      for _, k in ipairs(th.vector and { { "ax", "FX", "TX" }, { "ay", "FY", "TY" } } or {}) do
        local fa, ta = mul(c[k[2]], th.t), mul(c[k[3]], th.t)
        F, T = sub(F, mul(fa, th[k[1]])), sub(T, mul(ta, th[k[1]]))
        local den = dotF(fa, fa) + W_TORQUE * dot(ta, ta) + AIM_COST
        local num = dotF(sub(wantF, F), fa) + W_TORQUE * dot(sub(wantT, T), ta)
        th[k[1]] = clamp(num / den, -1, 1)
        F, T = add(F, mul(fa, th[k[1]])), add(T, mul(ta, th[k[1]]))
      end
      -- throttle
      local g, h = contribution(th, c)
      F, T = sub(F, mul(g, th.t)), sub(T, mul(h, th.t))
      local den = dotF(g, g) + W_TORQUE * dot(h, h) + 1e-6
      local num = dotF(sub(wantF, F), g) + W_TORQUE * dot(sub(wantT, T), h) - POWER_COST / 2
      th.t = clamp(num / den, a.lo, a.hi)
      F, T = add(F, mul(g, th.t)), add(T, mul(h, th.t))
    end
  end
  return F, T
end

-- ============================================================ control

-- What the craft should point its top at (world), and an extra angular
-- acceleration to feed forward (world), set by the flip.
local att = { want = UP, ff = v3() }

-- Torque (ship frame) that turns the craft's top toward att.want.
local function attitudeTorque()
  local b = bodyUp()
  local axis = cross(b, att.want)
  local s, c = len(axis), clamp(dot(b, att.want), -1, 1)
  -- error capped at ~35 deg so a big lean doesn't take every bit of thrust
  -- away from holding height
  local err = v3()
  if s > 1e-6 then err = mul(axis, math.min(math.acos(c), 0.6) / s)
  elseif c < 0 then err = mul(rotate(S.orient, v3(1, 0, 0)), 0.6) end
  local errShip = rotate(S.orient, err, true)
  local ffShip = rotate(S.orient, att.ff, true)
  return add(sub(mul(errShip, ATT_P), mul(S.omega, ATT_D)), ffShip)
end

-- Wanted velocity -> wanted acceleration (world), with a slow integrator.
local lastCtl = nil
local function wantAccel(wantVel)
  local t = now()
  local dt = lastCtl and math.min(t - lastCtl, 0.5) or 0
  lastCtl = t
  local err = sub(wantVel, S.vel)
  -- Flipping or upside down the side thrusters are busy holding it up, so
  -- sideways control is gentler and doesn't wind up. A craft that steers by
  -- leaning takes a while to change its lean, so it steers gently too.
  local busy = S.ctl == "flip" or S.ctl == "inverted"
  local hI = busy and 0 or VEL_I
  local hGain = busy and 0.6 * VEL_GAIN or VEL_GAIN
  if S.leanSteer then hGain, hI = 0.35 * VEL_GAIN, 0.25 * hI end
  S.velI = add(S.velI, v3(err.x * hI * dt, err.y * VEL_I * dt, err.z * hI * dt))
  S.velI = v3(clamp(S.velI.x, -3, 3), clamp(S.velI.y, -6, 6), clamp(S.velI.z, -3, 3))
  local a = add(v3(err.x * hGain, err.y * VEL_GAIN, err.z * hGain), S.velI)
  local maxA = busy and 0.4 * MAX_ACCEL or MAX_ACCEL
  local h = math.sqrt(a.x * a.x + a.z * a.z)
  if h > maxA then a.x, a.z = a.x * maxA / h, a.z * maxA / h end
  return a
end

-- Speed toward a point that arrives at zero speed: sqrt braking profile.
local function approachSpeed(dist, top, brake)
  local gain = S.leanSteer and 0.4 * POS_GAIN or POS_GAIN
  return math.min(top, math.sqrt(2 * brake * math.max(dist, 0)), gain * dist + 0.5 * math.min(dist, 1))
end

local goal = { pos = nil, vy = nil }  -- hold/fly to goal.pos; goal.vy overrides vertical speed

local function wantVelocity()
  local v = v3()
  if goal.pos then
    local dx, dz = goal.pos.x - S.pos.x, goal.pos.z - S.pos.z
    local d = math.sqrt(dx * dx + dz * dz)
    if d > 0.05 then
      local brake = goal.brake or (S.leanSteer and 1.2 or 0.4 * MAX_ACCEL)
      local sp = approachSpeed(d, goal.maxSpeed or CRUISE_SPEED, brake)
      v.x, v.z = dx / d * sp, dz / d * sp
    end
    v.y = clamp(POS_GAIN * (goal.pos.y - S.pos.y), -CLIMB_SPEED, CLIMB_SPEED)
  end
  if goal.vy then v.y = goal.vy end
  return v
end

local LIMITS = {
  upright = { bottom = { 0, 1 }, side = { 0, 1 } },
  -- bottom thrusters help only while they point mostly up, nose engines
  -- only once they do (see controlLoop)
  flip = { bottom = { 0, 1 }, side = { 0, 1 }, top = { 0, 1 } },
  -- upside down the bottom thrusters would push it into the ground
  inverted = { side = { 0, 1 }, top = { 0, 1 } },
}

S.ctl = "off"   -- "off" | "manual" (calibration drives outputs) | "lift" | a LIMITS key

-- Before and during calibration: all bottom thrusters together on one
-- power. Bottom thrusters whose throttle is already measured get nudged up
-- or down around it to hold the craft level, so leveling improves with every
-- thruster tested.
local function liftOnly()
  local t = now()
  local dt = lastCtl and math.min(t - lastCtl, 0.5) or 0
  lastCtl = t
  local err = wantVelocity().y - S.vel.y
  if bodyUp().y > 0.95 and not S.testing then S.hover = clamp(S.hover + 0.05 * err * dt, 0, 1) end
  local power = clamp(S.hover + 0.15 * err, 0, 1)
  -- held still during a nozzle test, or its reaction would be measured too
  if S.testing then power = S.testPower or power else S.testPower = power end
  local known = {}
  for _, th in ipairs(thrusters) do
    if th ~= S.testing then th.ax, th.ay = 0, 0 end
    if th.role == "bottom" then
      th.t, th.nudge = power, 0
      if cal and cal[th.name] then known[#known + 1] = th end
    elseif th ~= S.testing and th ~= S.partner then
      th.t = 0
    end
  end
  if #known == 0 then return end
  -- with every bottom thruster measured, lean (up to 20 deg) to hold
  -- position; during a test it just holds level
  local nb = 0
  for _, th in ipairs(thrusters) do if th.role == "bottom" then nb = nb + 1 end end
  att.want = UP
  if #known == nb and not S.testing then
    local a = wantAccel(wantVelocity())
    local h = v3(a.x / S.gravity, 0, a.z / S.gravity)
    local m = len(h)
    if m > 0.36 then h = mul(h, 0.36 / m) end
    att.want = add(UP, h)
    att.want = mul(att.want, 1 / len(att.want))
  end
  local want = attitudeTorque()
  want.y = 0  -- throttle can't usefully yaw
  local T = v3()
  for _ = 1, 4 do
    for _, th in ipairs(known) do
      local t0 = cal[th.name].T0
      local h = v3(t0.x, 0, t0.z)
      T = sub(T, mul(h, th.nudge))
      th.nudge = clamp(dot(sub(want, T), h) / (dot(h, h) + 0.05), -0.3, 0.3)
      T = add(T, mul(h, th.nudge))
    end
  end
  -- no net change in lift, only tilt (once every bottom thruster is known)
  local mean = 0
  if #known == nb then
    for _, th in ipairs(known) do mean = mean + th.nudge / #known end
  end
  for _, th in ipairs(known) do th.t = clamp(power + th.nudge - mean, 0, 1) end
  S.liftPower = power
end

-- Torque and force (ship frame) the leveling nudges are adding right now,
-- so a test can subtract them from what it measured.
local function nudgeEffect()
  local F, T = v3(), v3()
  if S.ctl ~= "lift" or not S.liftPower then return F, T end
  for _, th in ipairs(thrusters) do
    local c = cal and cal[th.name]
    if th.role == "bottom" and c then
      local d = th.t - S.liftPower
      F, T = add(F, mul(c.F0, d)), add(T, mul(c.T0, d))
    end
  end
  return F, T
end

local function controlLoop()
  while true do
    waitFix()
    if S.ctl == "lift" then
      liftOnly()
    elseif LIMITS[S.ctl] and cal and onSable then
      local a = wantAccel(wantVelocity())
      local wantF = rotate(S.orient, add(a, v3(0, S.gravity, 0)), true)
      local wF = nil
      if S.leanSteer and S.ctl ~= "flip" then
        -- Steer like a rocket: lean so the main engines push it where it
        -- should go, and ask only for push along the craft's length.
        -- Side thrusters far from the center of mass then just hold the lean.
        local h = v3(a.x / S.gravity, 0, a.z / S.gravity)
        local m = len(h)
        local maxLean = math.tan(math.rad(MAX_LEAN))
        if m > maxLean then h = mul(h, maxLean / m) end
        local dir = add(UP, h)
        dir = mul(dir, 1 / len(dir))
        if S.ctl == "inverted" then
          -- the nose engines push along -top: top points down, leaning away
          att.want = mul(dir, -1)
          if not S.hasNose then att.want = v3(0, -1, 0) end
        else
          att.want = dir
        end
        local up = bodyUp().y
        local axial = (a.y + S.gravity) / (up >= 0 and math.max(up, 0.5) or math.min(up, -0.5))
        -- lift matters, sideways push hardly (the lean does the steering)
        wantF, wF = v3(0, axial, 0), v3(0.05, 1, 0.05)
      end
      local lim = LIMITS[S.ctl]
      if S.ctl == "flip" then
        -- fade out with the lean and over the flip
        local fade = math.max(0, 1 - (now() - (S.flipStart or now())) / (S.flipTime or FLIP_SECONDS))
        local up = bodyUp().y
        lim = { side = lim.side, bottom = { 0, clamp((up - 0.3) / 0.5, 0, 1) * fade },
          top = { 0, clamp((-up - 0.3) / 0.5, 0, 1) } }
      end
      -- While flipping, turning matters most: side thrusters near the top
      -- can't turn it without also pushing it sideways. Only the push along
      -- the craft's own length is asked for (the "stop falling" wish would
      -- otherwise turn into a big sideways one as it tips).
      if S.ctl == "flip" and S.leanSteer then
        wantF = rotate(S.orient, v3(0, a.y + S.gravity, 0), true)
        wantF, wF = v3(0, wantF.y, 0), v3(0.02, 0.1, 0.02)
      end
      allocate(wantF, attitudeTorque(), lim, wF)
    end
    if S.ctl ~= "off" then flush() end
  end
end

-- ============================================================ calibration

-- Waits until done(elapsed, spinChange, driftChange) and returns the average
-- thrust acceleration in the ship frame (velocity change plus gravity,
-- turned into the ship frame tick by tick, so tilting doesn't pollute it),
-- the change in spin, and how long it took.
local function measure(done)
  local t0, w0 = now(), S.omega
  local lastT, lastV, sum = t0, S.vel, v3()
  local nF, nT = v3(), v3()   -- what the leveling added, integrated
  repeat
    local f, tq = nudgeEffect()
    waitFix()
    local t = now()
    local dt = t - lastT
    local dv = add(sub(S.vel, lastV), v3(0, S.gravity * dt, 0))
    sum = add(sum, rotate(S.orient, dv, true))
    nF, nT = add(nF, mul(f, dt)), add(nT, mul(tq, dt))
    lastT, lastV = t, S.vel
  until done(t - t0, len(sub(S.omega, w0)))
  local dur = math.max(now() - t0, 0.05)
  return mul(sub(sum, nF), 1 / dur), sub(sub(S.omega, w0), nT), dur
end

-- One test: push +delta, then -delta for just as long, then back. The push
-- stops early once the craft starts turning (CAL_SPIN rad/s), so a strong
-- thruster can't tip it over. Returns the response per unit of
-- delta, ship frame: linear and angular acceleration. Pushing both ways
-- cancels most of the spin it causes.
local CAL_SPIN, CAL_MIN_PULSE = 0.35, 0.25
local function doublet(apply, delta, spinLimit)
  spinLimit = spinLimit or CAL_SPIN
  apply(delta)
  local fPlus, dwPlus, dur = measure(function(t, spin)
    return t >= CAL_PULSE or (spin > spinLimit and t >= CAL_MIN_PULSE)
  end)
  apply(-delta)
  local fMinus, dwMinus, d2 = measure(function(t) return t >= dur end)
  apply(0)
  log("  doublet %.2fs/%.2fs", dur, d2)
  local F = mul(sub(fPlus, fMinus), 1 / (2 * delta))
  local T = mul(sub(mul(dwPlus, 1 / dur), mul(dwMinus, 1 / d2)), 1 / (2 * delta))
  return F, T
end

local function fmtV(v) return string.format("(%.2f %.2f %.2f)", v.x, v.y, v.z) end

-- part "throttle": F0/T0;  part "aim": FX/TX/FY/TY (measured at throttle
-- `base`, stored per unit throttle).
-- The raw nozzle numbers are noisy (the craft turns, the nozzle swings
-- slowly). Physics fixes them: a nozzle only turns the thrust (up to 30 deg),
-- so each aim axis pushes at right angles to the main thrust with
-- |F0| * pi/6 per unit aim, and every torque is (lever arm) x (force) for one
-- lever arm per thruster, fitted to all three tests together.
local NOZZLE_RATE = math.pi / 6
local function solve3(m, b)
  local function det(a)
    return a[1][1] * (a[2][2] * a[3][3] - a[2][3] * a[3][2]) - a[1][2] * (a[2][1] * a[3][3] - a[2][3] * a[3][1])
      + a[1][3] * (a[2][1] * a[3][2] - a[2][2] * a[3][1])
  end
  local d = det(m)
  if math.abs(d) < 1e-9 then return nil end
  local out = {}
  for col = 1, 3 do
    local a = { { m[1][1], m[1][2], m[1][3] }, { m[2][1], m[2][2], m[2][3] }, { m[3][1], m[3][2], m[3][3] } }
    for row = 1, 3 do a[row][col] = b[row] end
    out[col] = det(a) / d
  end
  return v3(out[1], out[2], out[3])
end

-- Blocks on a ship sit on its grid, so a thruster pushes along one ship
-- axis and its nozzle turns that push toward the two other axes. Snap to
-- the nearest axis; the tests then only have to say which way.
local AXES = { v3(1, 0, 0), v3(-1, 0, 0), v3(0, 1, 0), v3(0, -1, 0), v3(0, 0, 1), v3(0, 0, -1) }
local function snap(v, avoid)
  local best, bd = nil, -math.huge
  for _, a in ipairs(AXES) do
    local ok = true
    for _, x in ipairs(avoid or {}) do if math.abs(dot(a, x)) > 0.5 then ok = false end end
    if ok and dot(v, a) > bd then best, bd = a, dot(v, a) end
  end
  return best
end

-- Lever arm (over inertia) rho with  T = rho x F, fitted to several tests.
local function fitLever(pairs)
  local m = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } }
  local b = v3()
  for _, pair in ipairs(pairs) do
    local F, T = pair[1], pair[2]
    local f2 = dot(F, F)
    local comp = { F.x, F.y, F.z }
    for i = 1, 3 do
      for j = 1, 3 do m[i][j] = m[i][j] + (i == j and f2 or 0) - comp[i] * comp[j] end
    end
    b = add(b, cross(F, T))
  end
  return solve3(m, { b.x, b.y, b.z })
end

local function sq(v) return dot(v, v) end

-- Tries every way the two nozzle axes could point and keeps the one that
-- best explains both how the craft was pushed and how it turned.
local function cleanResponses(c)
  local A = len(c.F0)
  if A < 1e-6 then return c end
  local d = snap(c.F0)
  local want = A * NOZZLE_RATE
  local F0 = mul(d, A)
  local tScale = math.max(sq(c.T0), sq(c.TX), sq(c.TY), 1e-6)
  local best, bestScore = nil, math.huge
  for _, ax in ipairs(AXES) do
    if math.abs(dot(ax, d)) < 0.5 then
      for _, ay in ipairs(AXES) do
        if math.abs(dot(ay, d)) < 0.5 and math.abs(dot(ay, ax)) < 0.5 then
          local fx, fy = mul(ax, want), mul(ay, want)
          local rho = fitLever({ { F0, c.T0 }, { fx, c.TX }, { fy, c.TY } })
          if rho then
            local score = (sq(sub(c.FX, fx)) + sq(sub(c.FY, fy))) / (want * want)
              + (sq(sub(c.T0, cross(rho, F0))) + sq(sub(c.TX, cross(rho, fx))) + sq(sub(c.TY, cross(rho, fy)))) / tScale
            if score < bestScore then
              bestScore = score
              best = { F0 = F0, FX = fx, FY = fy, T0 = cross(rho, F0), TX = cross(rho, fx), TY = cross(rho, fy) }
            end
          end
        end
      end
    end
  end
  return best or c
end

-- Nozzle tests keep the bottom thrusters leveling (their effect is known by
-- then and subtracted); throttle tests hold everything still.
local function testThruster(th, base, part, c, partner, partnerT)
  c = c or { F0 = v3(), T0 = v3(), FX = v3(), TX = v3(), FY = v3(), TY = v3() }
  th.ax, th.ay = 0, 0
  if part == "throttle" and th.role == "bottom" then
    S.ctl = "manual"
    th.t = base
    flush(); waitSeconds(0.3)
    c.F0, c.T0 = doublet(function(d) th.t = base + d; flush() end, CAL_STEP)
  elseif part == "throttle" and not th.vector then
    -- From off: 2 x CAL_STEP, then off, leveling paused, stopped early if it
    -- starts to turn (side thrusters near the top turn it hard).
    S.ctl = "manual"
    -- It can't push the other way, so the pulse leaves the craft turning;
    -- stop at a small turn rate the thrusters measured so far can undo.
    c.F0, c.T0 = doublet(function(d) th.t = d > 0 and 2 * d or 0; flush() end, CAL_STEP, 0.1)
    S.ctl = "lift"
  elseif part == "throttle" then
    S.testing, S.ctl = th, "lift"
    th.t = base
    waitSeconds(0.3)
    c.F0, c.T0 = doublet(function(d) th.t = base + d end, CAL_STEP)
    S.testing = nil
  else
    -- leveling runs until the test starts, then everything holds still
    S.testing, S.ctl = th, "lift"
    if th.role == "side" then th.t = base end
    S.partner = partner
    if partner then partner.t = partnerT end
    waitSeconds(0.4)
    S.ctl = "manual"
    base = math.max(th.t, 0.1)
    c.FX, c.TX = doublet(function(d) th.ax = d; flush() end, CAL_AIM)
    S.ctl = "lift"; waitSeconds(0.6); S.ctl = "manual"
    c.FY, c.TY = doublet(function(d) th.ay = d; flush() end, CAL_AIM)
    th.ax, th.ay = 0, 0
    S.ctl = "lift"
    S.testing, S.partner = nil, nil
    for _, k in ipairs({ "FX", "TX", "FY", "TY" }) do c[k] = mul(c[k], (S.calScale or 1) / base) end
    c = cleanResponses(c)
    c.scaled = true
  end
  if part == "throttle" and not th.vector and len(c.F0) > 1e-6 then
    -- a regular thruster pushes straight along one ship axis and can't turn
    -- the craft around that axis
    local d = snap(c.F0)
    c.F0 = mul(d, len(c.F0))
    c.T0 = sub(c.T0, mul(d, dot(c.T0, d)))
  end
  if S.calScale and not c.scaled then
    for _, k in ipairs({ "F0", "T0" }) do c[k] = mul(c[k], S.calScale) end
    c.scaled = true
  end
  log("cal %s %s %s F0%s T0%s FX%s TX%s FY%s TY%s", part, th.role, th.name, fmtV(c.F0), fmtV(c.T0),
    fmtV(c.FX), fmtV(c.TX), fmtV(c.FY), fmtV(c.TY))
  return c
end

-- Side thrusters far from the center of mass turn the craft more than they
-- move it. Then it steers by leaning, like a rocket.
local function chooseSteering()
  local turn, push = 0, 0
  for _, th in ipairs(thrusters) do
    local c = cal and cal[th.name]
    if th.role == "side" and c then turn, push = turn + len(c.T0), push + len(c.F0) end
  end
  S.leanSteer = push > 0 and turn / push > 0.08
  for _, th in ipairs(thrusters) do if th.role == "top" then S.hasNose = true end end
  log("steering: %s (side turn/push %.2f)", S.leanSteer and "lean" or "direct", push > 0 and turn / push or 0)
end

local function calibrate()
  S.mode = "CALIBRATE"
  cal = {}
  local hoverAt = v3(S.pos.x, S.pos.y, S.pos.z)
  -- leaning on the bottom thrusters only gives a few blocks/s^2
  goal.pos, goal.vy, goal.maxSpeed, goal.brake = hoverAt, nil, (S.leanSteer and 0 or 4), 1.5
  -- Thrusters take a moment to spin up, so the short tests read low. The
  -- hover throttle is known exactly, so scale every reading until the bottom
  -- thrusters at hover throttle hold up the craft's weight.
  local function setScale()
    local lift = 0
    for _, th in ipairs(thrusters) do
      if th.role == "bottom" then lift = lift + cal[th.name].F0.y * S.hover end
    end
    S.calScale = lift > 0.1 and clamp(S.gravity / lift, 0.5, 3) or 1
    log("thrust scale %.2f (hover %.2f)", S.calScale, S.hover)
    for _, th in ipairs(thrusters) do
      local c = cal[th.name]
      if c then c.F0, c.T0, c.scaled = mul(c.F0, S.calScale), mul(c.T0, S.calScale), true end
    end
  end
  -- Between tests: once the bottom thrusters are measured, every measured
  -- thruster helps hold it steady (on a tall craft the side thrusters near
  -- the top level it far better than the bottom ones can).
  local function settle(s)
    if S.calScale then chooseSteering() end
    S.ctl = S.calScale and "upright" or "lift"; waitSeconds(s)
    local t0 = now()
    while (tiltDeg() > 3 or len(S.omega) > 0.05) and now() - t0 < 8 do waitFix() end
  end
  -- 1. bottom throttles, while the others hold the hover power. After
  -- this the bottom thrusters can level the craft between the other tests.
  local hover = S.hover
  for i, th in ipairs(thrusters) do
    if th.role == "bottom" then
      S.status = "Testing bottom thruster " .. i
      settle(1.5)
      hover = S.hover
      cal[th.name] = testThruster(th, hover, "throttle")
    end
  end
  setScale()
  -- Too heavy: at this hover power there's nothing left to steer with.
  if S.hover > 0.8 then
    cal = nil
    return false
  end
  -- 2. bottom nozzles (vector thrusters only)
  for i, th in ipairs(thrusters) do
    if th.role == "bottom" and th.vector then
      S.status = "Testing bottom nozzle " .. i
      settle(1.5)
      cal[th.name] = testThruster(th, th.t, "aim", cal[th.name])
    end
  end
  -- 3. side and nose throttles, one at a time
  local sides = {}
  for i, th in ipairs(thrusters) do
    if th.role ~= "bottom" then
      S.status = string.format("Testing %s thruster %d", th.role == "top" and "nose" or "side", i)
      settle(2)
      cal[th.name] = testThruster(th, CAL_SIDE_THROTTLE, "throttle")
      th.t = 0
      if th.role == "side" and th.vector then sides[#sides + 1] = th end
    end
  end
  -- 4. vector side nozzles. The side thruster pushing the most the other way
  -- runs too (nozzle centered), so the craft isn't shoved sideways.
  for i, th in ipairs(sides) do
    local c = cal[th.name]
    local partner, best = nil, 0
    for _, o in ipairs(sides) do
      local oc = cal[o.name]
      local opp = -dot(c.F0, oc.F0) / math.max(len(c.F0) * len(oc.F0), 1e-6)
      if o ~= th and opp > best then partner, best = o, opp end
    end
    local partnerT = 0
    if partner and best > 0.7 then
      partnerT = clamp(CAL_SIDE_THROTTLE * len(c.F0) / math.max(len(cal[partner.name].F0), 1e-6), 0, 1)
    else
      partner = nil
    end
    S.status = "Testing side nozzle " .. i
    settle(2)
    cal[th.name] = testThruster(th, CAL_SIDE_THROTTLE, "aim", c, partner, partnerT)
    th.t, th.ax, th.ay = 0, 0, 0
    if partner then partner.t = 0 end
  end
  settle(2)
  S.calScale = nil
  local out = {}
  for name, c in pairs(cal) do
    out[name] = { F0 = c.F0, T0 = c.T0, FX = c.FX, TX = c.TX, FY = c.FY, TY = c.TY }
  end
  goal.maxSpeed, goal.brake = nil, nil
  writeFile(CAL_FILE, textutils.serialize({ responses = out, hover = S.hover }))
  S.status = "Calibration saved to " .. CAL_FILE
end

-- Ship axis (x or z) the side thrusters turn the craft around most strongly.
local function flipAxisShip()
  if FLIP_AXIS == "x" then return v3(1, 0, 0) elseif FLIP_AXIS == "z" then return v3(0, 0, 1) end
  local sx, sz = 0, 0
  for _, th in ipairs(thrusters) do
    local c = cal[th.name]
    if th.role == "side" and c then
      sx = sx + math.abs(c.T0.x) + math.abs(c.TX.x) + math.abs(c.TY.x)
      sz = sz + math.abs(c.T0.z) + math.abs(c.TX.z) + math.abs(c.TY.z)
    end
  end
  if sx >= sz then return v3(1, 0, 0) end
  return v3(0, 0, 1)
end

-- ============================================================ flight

local function horizontalDistance(p)
  local dx, dz = p.x - S.pos.x, p.z - S.pos.z
  return math.sqrt(dx * dx + dz * dz)
end

local function waitUntil(cond, status)
  while not cond() do
    if status then S.status = status() end
    waitFix()
  end
end

local function fireCharges()
  S.mode = "DEPLOY"
  -- thrusters first, then every output in the same tick
  if CHARGE_THRUSTERS == "cut" then
    S.ctl = "manual"; setAll(0); flush()
  elseif CHARGE_THRUSTERS == "full" then
    S.ctl = "manual"; setAll(1); flush()
  end
  local jobs = {}
  for _, c in ipairs(CHARGES) do
    if c.relay then
      jobs[#jobs + 1] = function() pcall(peripheral.call, c.relay, "setAnalogOutput", c.side, CHARGE_STRENGTH) end
    else
      jobs[#jobs + 1] = function() pcall(rs.setAnalogOutput, c.side, CHARGE_STRENGTH) end
    end
  end
  parallel.waitForAll(table.unpack(jobs))
  log("DEPLOY at sensor %.2f  pos %s  vel %s", S.ground or -1, fmtV(S.pos), fmtV(S.vel))
  S.status = string.format("Charges fired at %.1f blocks", S.ground or -1)
  sleep(CHARGE_SECONDS)
  chargesOff()
  S.ctl = "off"
  allOff()
end

-- Sets down upright on the target with the charges off.
local function landUpright(target, why)
  att.want, att.ff = UP, v3()
  S.mode, S.ctl = "ABORT", "upright"
  goal.pos, goal.vy = v3(target.x, target.y + 2, target.z), nil
  local t1 = now()
  while now() - t1 < 4 or S.pos.y > target.y + 3 do
    S.status = why
    goal.vy = -math.min(CLIMB_SPEED, math.max(1, 0.3 * (S.pos.y - target.y)))
    if S.ground and S.ground < 1.5 then break end
    waitFix()
  end
  S.ctl = "off"
  allOff()
  S.mode = "LANDED"
  S.status = why
end

local function missionLoop()
  waitFix()
  local target = S.target
  S.pad = v3(S.pos.x, S.pos.y, S.pos.z)
  S.velI = v3()
  if cal then chooseSteering() end

  -- 1. climb off the pad
  S.mode = "CLIMB"
  local calY = math.min(S.pos.y + CAL_HEIGHT, CRUISE_Y)
  goal.pos, goal.vy = v3(S.pad.x, calY, S.pad.z), nil
  S.ctl = cal and "upright" or "lift"
  waitUntil(function() return S.pos.y > calY - 1.5 and math.abs(S.vel.y) < 1.5 end,
    function() return string.format("Climbing to %.0f", calY) end)

  -- 2. learn the thrusters (first flight only)
  if not cal and calibrate() == false or S.hover > 0.8 then
    landUpright(v3(S.pad.x, S.pad.y - 1, S.pad.z), string.format(
      "Too heavy: hovers at %.0f%% power. Add bottom thrusters or lighten it.", S.hover * 100))
    return
  end
  S.velI = v3()

  -- 3. cruise altitude, then the target
  S.mode = "CLIMB"
  S.ctl = "upright"
  local cruiseY = math.max(CRUISE_Y, target.y + FLIP_HEIGHT)
  goal.pos = v3(S.pad.x, cruiseY, S.pad.z)
  waitUntil(function() return S.pos.y > cruiseY - 3 end,
    function() return string.format("Climbing to %.0f", cruiseY) end)
  S.mode = "CRUISE"
  goal.pos = v3(target.x, cruiseY, target.z)
  waitUntil(function() return horizontalDistance(target) < ARRIVE_RADIUS end,
    function() return string.format("%.0f blocks to go", horizontalDistance(target)) end)

  -- 4. down to the flip height and stop there
  S.mode = "APPROACH"
  local flipY = target.y + FLIP_HEIGHT
  goal.pos = v3(target.x, flipY, target.z)
  waitUntil(function()
    return math.abs(S.pos.y - flipY) < 1.5 and len(S.vel) < 1 and horizontalDistance(target) < ARRIVE_RADIUS
  end, function() return string.format("Descending to flip height %.0f", flipY) end)

  -- Without nose engines nothing holds it up once it's flipped, so it would
  -- crash. Disarmed, it just lands upright instead.
  local nose = 0
  for _, th in ipairs(thrusters) do if th.role == "top" then nose = nose + 1 end end
  if nose == 0 and not ARMED then
    landUpright(target, "Disarmed and no nose engines - landed upright")
    return
  end
  -- Upside down only the nose engines hold it up; they need some spare.
  if nose > 0 then
    local lift = 0
    for _, th in ipairs(thrusters) do
      if th.role == "top" then lift = lift + len(cal[th.name].F0) end
    end
    if lift < 1.15 * S.gravity then
      landUpright(target, string.format(
        "Nose engines too weak to hold it upside down (%.0f%% of weight) - landed upright", lift / S.gravity * 100))
      return
    end
  end

  -- 5. flip: rotate the craft's top from up to down around a horizontal axis.
  -- As fast as the side thrusters can turn it: the smooth 0..180 deg path
  -- needs a peak turn rate change of 6*pi/T^2.
  local axisShip = flipAxisShip()
  local turn = 0
  for _, th in ipairs(thrusters) do
    local c = cal[th.name]
    if th.role == "side" and c then
      turn = turn + math.max(math.abs(dot(c.T0, axisShip)), math.abs(dot(c.TX, axisShip)) + math.abs(dot(c.TY, axisShip)))
    end
  end
  -- opposite side thrusters turn it opposite ways: only half help at once
  turn = turn / 2
  local flipTime = FLIP_SECONDS
  if turn > 1e-3 then flipTime = math.max(FLIP_SECONDS, math.sqrt(6 * math.pi / (0.6 * turn))) end
  S.flipTime = flipTime
  log("flip time %.1fs (side thrusters turn it at %.2f rad/s^2)", flipTime, turn)

  if FLIP_POPUP > 0 and S.leanSteer then
    S.mode = "POP-UP"
    goal.vy = FLIP_POPUP
    waitUntil(function() return S.vel.y >= 0.8 * FLIP_POPUP end,
      function() return string.format("Pop-up %.1f blocks/s", S.vel.y) end)
  end

  S.mode = "FLIP"
  S.ctl = "flip"
  goal.pos, goal.vy = v3(target.x, S.pos.y, target.z), 0
  local axis = rotate(S.orient, axisShip)
  axis.y = 0
  axis = mul(axis, 1 / math.max(len(axis), 1e-6))
  local side = cross(axis, UP)
  local t0 = now()
  S.flipStart = t0
  log("FLIP start at y=%.1f axis %s", S.pos.y, fmtV(axis))
  repeat
    local s = clamp((now() - t0) / flipTime, 0, 1)
    local th = math.pi * (3 * s * s - 2 * s * s * s)        -- smooth 0 -> pi
    local acc = s < 1 and math.pi * (6 - 12 * s) / (flipTime * flipTime) or 0
    att.want = add(mul(UP, math.cos(th)), mul(side, math.sin(th)))
    att.ff = mul(axis, acc)
    S.status = string.format("Flipping  %.0f deg", tiltDeg())
    waitFix()
    if now() - t0 > 3 * flipTime and tiltDeg() < 120 then
      -- the side thrusters can't turn it over: give up and land upright
      log("FLIP failed: tilt %.0f after %.1fs", tiltDeg(), now() - t0)
      S.flipFailed = true
      break
    end
  until s >= 1 and tiltDeg() > 150
  if S.flipFailed then
    landUpright(target, "Couldn't flip (side thrusters too weak?) - landing upright, charges safe")
    return
  end
  att.want, att.ff = v3(0, -1, 0), v3()
  log("FLIP done at y=%.1f after %.1fs", S.pos.y, now() - t0)

  -- 6. upside-down descent; the sensor now looks at the ground. Without nose
  -- engines it just falls, the side thrusters keeping it pointed down.
  S.mode = nose > 0 and "INVERTED" or "DROP"
  S.ctl = "inverted"
  S.velI = v3(0, S.velI.y, 0)
  while true do
    local h = S.ground or (S.pos.y - target.y)
    local over = h - DEPLOY_DISTANCE
    -- brake gently (1.5 blocks/s^2) down to FINAL_SPEED for the last SLOW_DISTANCE
    local speed = clamp(math.sqrt(2 * 1.5 * math.max(over - SLOW_DISTANCE, 0)) + FINAL_SPEED, FINAL_SPEED, DESCENT_SPEED)
    goal.pos = v3(target.x, S.pos.y, target.z)
    goal.vy = -speed
    S.status = string.format("%s  ground %s", nose > 0 and "Inverted descent" or "Dropping",
      S.ground and string.format("%.1f", S.ground) or "--")
    -- falling fast it covers a block or more per tick, so look a tick ahead
    local lead = math.max(-S.vel.y, 0) * 0.05
    -- trigger: armed, upside down, and the sensor (or, with none, the target Y) says we're there
    local reached
    if #sensors > 0 then reached = S.ground ~= nil and S.ground - lead <= DEPLOY_DISTANCE
    else reached = S.pos.y - target.y - lead <= DEPLOY_DISTANCE end
    if reached and tiltDeg() > 120 then
      local onTarget = horizontalDistance(target) <= DEPLOY_RADIUS
      if not onTarget then log("landed %.1f blocks off target: not firing", horizontalDistance(target)) end
      if ARMED and onTarget then
        fireCharges()
      else
        S.mode = "LANDED"
        goal.vy = -FINAL_SPEED
        waitSeconds(2)
        S.ctl = "off"
        allOff()
        S.status = ARMED and string.format("Landed %.0f blocks off target - charges NOT fired", horizontalDistance(target))
          or "Landed upside down (disarmed)"
      end
      return
    end
    os.pullEvent()
  end
end

-- ============================================================ screen

local function screenLoop()
  while true do
    term.clear()
    term.setCursorPos(1, 1)
    print("Flip Lander v" .. VERSION .. "  [" .. S.mode .. "]")
    print("")
    local t = S.target
    if t then print(string.format("Target: %.0f %.0f %.0f", t.x, t.y, t.z)) end
    if S.pos then
      print(string.format("Pos:    %.1f %.1f %.1f  (%s)", S.pos.x, S.pos.y, S.pos.z, S.navSource))
      if t then print(string.format("Dist:   %.1f   height over target %.1f", horizontalDistance(t), S.pos.y - t.y)) end
    end
    print(string.format("Ground: %s   (fires at %.1f%s)", S.ground and string.format("%.1f", S.ground) or "--",
      DEPLOY_DISTANCE, ARMED and "" or ", DISARMED"))
    print(string.format("Tilt:   %.0f deg", tiltDeg()))
    local out = { bottom = {}, side = {}, top = {} }
    for _, th in ipairs(thrusters) do
      table.insert(out[th.role], string.format("%2d", th.sentT or 0))
    end
    print("Bottom: " .. table.concat(out.bottom, " ") .. "  Side: " .. table.concat(out.side, " ")
      .. (#out.top > 0 and ("  Nose: " .. table.concat(out.top, " ")) or ""))
    print("")
    print(S.status)
    print("")
    print("Ctrl+T to abort")
    sleep(0.25)
  end
end

local function logLoop()
  while true do
    sleep(0.2)
    if S.pos then
      local parts = {}
      for _, th in ipairs(thrusters) do
        parts[#parts + 1] = string.format("%s%d/%.2f/%.2f", th.role:sub(1, 1):upper(),
          th.sentT or 0, th.sentX or 0, th.sentY or 0)
      end
      log("%-9s y=%.1f pos %s vel %s tilt %.0f gnd %s | %s", S.mode, S.pos.y, fmtV(S.pos), fmtV(S.vel),
        tiltDeg(), S.ground and string.format("%.1f", S.ground) or "-", table.concat(parts, " "))
    end
  end
end

-- ============================================================ setup

local function allThrusters()
  local out = {}
  for _, name in ipairs(peripheralNames()) do
    local t = peripheral.getType(name)
    if VECTOR_TYPES[t] or REGULAR_TYPES[t] then out[#out + 1] = name end
  end
  table.sort(out)
  return out
end

-- Shows each thruster in turn (a vector nozzle wiggles, a regular thruster
-- puffs at low power) and asks what it is.
local function setup()
  local names = allThrusters()
  if #names == 0 then error("No thrusters found. Connect them with wired modems.", 0) end
  local cfg = { bottom = {}, side = {}, top = {} }
  print(string.format("Found %d thrusters. Each one will puff or wiggle.", #names))
  print("  B = bottom (lift)   S = side   T = top/nose (points the")
  print("  other way, holds it up when flipped)   I = ignore")
  for _, name in ipairs(names) do
    local vector = VECTOR_TYPES[peripheral.getType(name)]
    print("")
    write(name .. "... [B/S/T/I] ")
    local answer = nil
    parallel.waitForAny(function()
      while true do
        if vector then
          for _, a in ipairs({ { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } }) do
            pcall(peripheral.call, name, "setVector", a[1], a[2])
            sleep(0.4)
          end
        else
          pcall(peripheral.call, name, "setPower", 3)
          sleep(0.4)
          pcall(peripheral.call, name, "setPower", 0)
          sleep(0.8)
        end
      end
    end, function()
      while not answer do
        local _, ch = os.pullEvent("char")
        ch = ch:lower()
        if ch == "b" or ch == "s" or ch == "t" or ch == "i" then answer = ch end
      end
    end)
    if vector then pcall(peripheral.call, name, "setVector", 0, 0) end
    pcall(peripheral.call, name, vector and "setThrust" or "setPower", 0)
    print(answer:upper())
    if answer == "b" then table.insert(cfg.bottom, name)
    elseif answer == "s" then table.insert(cfg.side, name)
    elseif answer == "t" then table.insert(cfg.top, name) end
  end
  writeFile(CFG_FILE, textutils.serialize(cfg))
  if fs.exists(CAL_FILE) then fs.delete(CAL_FILE) end
  print("")
  print(string.format("Saved: %d bottom, %d side, %d nose. Calibration runs on the next flight.",
    #cfg.bottom, #cfg.side, #cfg.top))
end

local function loadThrusters()
  local bottom, side, top = BOTTOM_THRUSTERS, SIDE_THRUSTERS, TOP_THRUSTERS
  if #bottom == 0 and #side == 0 then
    local saved = readFile(CFG_FILE)
    if not saved then return false end
    local cfg = textutils.unserialize(saved)
    bottom, side, top = cfg.bottom, cfg.side, cfg.top or {}
  end
  thrusters = {}
  for _, list in ipairs({ { bottom, "bottom" }, { side, "side" }, { top, "top" } }) do
    for _, name in ipairs(list[1]) do
      if not peripheral.isPresent(name) then error("Thruster " .. name .. " is missing. Run: flip setup", 0) end
      local vector = VECTOR_TYPES[peripheral.getType(name)] and true or false
      thrusters[#thrusters + 1] = { name = name, role = list[2], vector = vector, t = 0, ax = 0, ay = 0 }
    end
  end
  return true
end

local function loadCalibration()
  local saved = readFile(CAL_FILE)
  if not saved then return end
  local c = textutils.unserialize(saved)
  if not c or not c.responses then return end
  for _, th in ipairs(thrusters) do
    if not c.responses[th.name] then return end  -- thrusters changed: recalibrate
  end
  cal, S.hover = c.responses, c.hover or S.hover
end

-- "x y z", "x,y,z" or three separate answers
local function parseTarget(text)
  local n = {}
  for num in text:gmatch("-?%d+%.?%d*") do n[#n + 1] = tonumber(num) end
  if #n == 3 then return v3(n[1], n[2], n[3]) end
  return nil
end

local function askTarget()
  while true do
    write("Target X Y Z: ")
    local t = parseTarget(read())
    if t then return t end
    print("Type three numbers, e.g.  120 64 -340")
  end
end

-- ============================================================ auto-update

-- Every UPDATE_CHECK_MINUTES this checks GitHub for a newer version. A new
-- version is downloaded, checked that it loads, given your current settings
-- (the UPPERCASE lines at the top of this file), saved, and the computer
-- reboots into it. Set AUTO_UPDATE = false at the top to turn this off.
local readyToUpdate, checkForUpdate, updateLoop, bootUpdate
do
local UPDATE_REPO = "levisnakes/cc-flip-lander"
local UPDATE_FILE = "flip.lua"
local UPDATE_CHECK_MINUTES = 15
local PROGRAM_PATH = (shell and shell.getRunningProgram()) or "startup.lua"
local SHA_FILE = "/.version_" .. UPDATE_FILE

-- never mid-flight
readyToUpdate = function() return S.mode == "READY" end

local function splitSettings(src)
  local settings, rest = {}, {}
  local inSettings = true
  for line in (src .. "\n"):gmatch("(.-)\n") do
    if inSettings and (line:match("^local ") or line:match("^%-%- ====")) then inSettings = false end
    table.insert(inSettings and settings or rest, line)
  end
  return settings, rest
end

local function braceDelta(line)
  local code = line:gsub('"[^"]*"', ""):gsub("%-%-.*$", "")
  local _, opens = code:gsub("{", "")
  local _, closes = code:gsub("}", "")
  return opens - closes
end

local function statements(lines)
  local out = {}
  local i = 1
  while i <= #lines do
    local name = lines[i]:match("^([%u_][%u%d_]*)%s*=")
    if name then
      local block = { lines[i] }
      local depth = braceDelta(lines[i])
      while depth > 0 and i < #lines do
        i = i + 1
        table.insert(block, lines[i])
        depth = depth + braceDelta(lines[i])
      end
      out[name] = table.concat(block, "\n")
    end
    i = i + 1
  end
  return out
end

local function mergeSettings(oldSrc, newSrc)
  local oldSettings = statements((splitSettings(oldSrc)))
  local newSettings, rest = splitSettings(newSrc)
  local out = {}
  local i = 1
  while i <= #newSettings do
    local line = newSettings[i]
    local name = line:match("^([%u_][%u%d_]*)%s*=")
    -- VERSION always comes from the new file
    if name and name ~= "VERSION" and oldSettings[name] then
      table.insert(out, oldSettings[name])
      local depth = braceDelta(line)
      while depth > 0 and i < #newSettings do
        i = i + 1
        depth = depth + braceDelta(newSettings[i])
      end
    else
      table.insert(out, line)
    end
    i = i + 1
  end
  for _, line in ipairs(rest) do table.insert(out, line) end
  local merged = table.concat(out, "\n")
  if newSrc:sub(-1) == "\n" and merged:sub(-1) ~= "\n" then merged = merged .. "\n" end
  return (merged:gsub("\n\n$", "\n"))
end

local function fetch(url, headers)
  local ok, h = pcall(http.get, url, headers)
  if not ok or not h then return nil end
  local body = h.readAll()
  h.close()
  return body
end

checkForUpdate = function(atBoot)
  if AUTO_UPDATE == false or not http then return end
  local sha = fetch("https://api.github.com/repos/" .. UPDATE_REPO .. "/commits/main",
    { ["Accept"] = "application/vnd.github.sha", ["User-Agent"] = "cc-auto-update" })
  if not sha then return end
  sha = sha:match("^%s*(%x+)%s*$")
  if not sha or #sha < 40 or readFile(SHA_FILE) == sha then return end
  local newSrc = fetch("https://raw.githubusercontent.com/" .. UPDATE_REPO .. "/" .. sha .. "/" .. UPDATE_FILE)
  if not newSrc or not load(newSrc, UPDATE_FILE) then return end
  local oldSrc = readFile(PROGRAM_PATH)
  if not oldSrc then return end
  local merged = mergeSettings(oldSrc, newSrc)
  if not load(merged, UPDATE_FILE) then return end
  if merged == oldSrc then
    writeFile(SHA_FILE, sha)
    return
  end
  while not atBoot and not readyToUpdate() do sleep(5) end
  writeFile(PROGRAM_PATH .. ".old", oldSrc)
  writeFile(PROGRAM_PATH, merged)
  writeFile(SHA_FILE, sha)
  term.clear()
  term.setCursorPos(1, 1)
  print("Updated to " .. sha:sub(1, 7) .. ", restarting...")
  if not atBoot then sleep(1) end
  os.reboot()
end

updateLoop = function()
  while true do
    sleep(UPDATE_CHECK_MINUTES * 60)
    pcall(checkForUpdate)
  end
end

bootUpdate = function()
  if AUTO_UPDATE == false or not http then return end
  term.clear()
  term.setCursorPos(1, 1)
  print("Checking for updates...")
  pcall(checkForUpdate, true)
end
end

-- ============================================================ main

local args = { ... }
term.clear()
term.setCursorPos(1, 1)

if args[1] == "setup" then
  setup()
  return
end
if args[1] == "recal" then
  if fs.exists(CAL_FILE) then fs.delete(CAL_FILE) end
  print("Calibration cleared; it runs again on the next flight.")
  return
end

bootUpdate()
term.clear()
term.setCursorPos(1, 1)

if not loadThrusters() then
  print("Thrusters not set up yet.")
  setup()
  print("")
end
local count = { bottom = 0, side = 0, top = 0 }
for _, th in ipairs(thrusters) do count[th.role] = count[th.role] + 1 end
if count.bottom == 0 or count.side == 0 then error("Need at least one bottom and one side thruster. Run: flip setup", 0) end
loadCalibration()

for _, name in ipairs(peripheralNames()) do
  if peripheral.getType(name) == "optical_sensor" then sensors[#sensors + 1] = name end
end

onSable = sublevel ~= nil and sublevel.isInPlotGrid()
if onSable then
  S.navSource = "Sable"
  if aero then
    local ok, g = pcall(aero.getGravity)
    local m = ok and type(g) == "table" and g.length and g:length()
    if m and m > 0.5 and m < 100 then S.gravity = m end
  end
else
  S.navSource = "GPS"
  if not peripheral.find("modem", function(_, m) return m.isWireless() end) then
    error("Not on a Sable ship and no wireless modem for GPS", 0)
  end
  print("Not on a Sable ship: no attitude data, so it can't level or flip.")
  print("Install CC: Sable and put the computer on the ship.")
  return
end

print(string.format("Flip Lander v%s  %d bottom, %d side, %d nose thrusters, %d optical sensor(s)",
  VERSION, count.bottom, count.side, count.top, #sensors))
if count.top == 0 then print("No nose engines: after the flip it drops onto the target.") end
print(cal and "Calibration loaded." or "Not calibrated: it will calibrate after take-off.")
if #sensors == 0 then print("No Optical Sensor: it fires at the target Y instead.") end
print("")

local x, y, z = tonumber(args[1]), tonumber(args[2]), tonumber(args[3])
if x and y and z then S.target = v3(x, y, z) else S.target = askTarget() end

if fs then logFile = fs.open(LOG_FILE, "w") end
logStart = now()
log("Flip Lander v%s target %s gravity %.2f calibrated %s", VERSION, fmtV(S.target), S.gravity, tostring(cal ~= nil))

allOff()
local navLoop = onSable and sableLoop or gpsLoop
local ok, err = pcall(parallel.waitForAny, navLoop, sensorLoop, controlLoop, missionLoop, screenLoop, logLoop)
log("stopped: %s", ok and S.mode or tostring(err))
if logFile then logFile.close() end
allOff()
S.mode = "READY"
term.clear()
term.setCursorPos(1, 1)
if not ok then print("Stopped: " .. tostring(err))
else print(S.status) end
