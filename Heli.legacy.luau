--SERVICES--
local Players = game:GetService("Players")
local RunService = game:GetService("RunService")

--REFERENCES--
local heli = script.Parent
local core = heli:WaitForChild("Core")
local seat = heli:WaitForChild("VehicleSeat")
local remote = heli:WaitForChild("HeliControl")
local prompt = seat.ProximityPrompt

--SOUNDS--
local engineLoop = core:WaitForChild("EngineLoop")

--PHYSICS INSTANCES--
local att = Instance.new("Attachment")
att.Name = "HeliAttachment"
att.Parent = core

-- RelativeTo = World ensures the force vector is always axis-aligned in world space,
-- so thrust direction never changes when the helicopter body tilts.
local force = Instance.new("VectorForce")
force.Name = "HeliForce"
force.Attachment0 = att
force.RelativeTo = Enum.ActuatorRelativeTo.World
force.Force = Vector3.zero
force.Parent = core

-- OneAttachment mode lets us supply a free target CFrame each frame instead of
-- aligning to a second attachment. The constraint applies internal torque to match it.
local gyro = Instance.new("AlignOrientation")
gyro.Name = "HeliGyro"
gyro.Mode = Enum.OrientationAlignmentMode.OneAttachment
gyro.Attachment0 = att
gyro.Responsiveness = 8
gyro.MaxTorque = 250000
gyro.Parent = core

--ROTORS--
local rotorMain = heli:WaitForChild("Rotor"):WaitForChild("RotorCenter")
local rotorTail = heli:WaitForChild("TailRotors"):WaitForChild("TailRotorCenter")
local hingeMain = rotorMain:WaitForChild("HingeConstraint")
local hingeTail = rotorTail:WaitForChild("HingeConstraint")

--COLLISION--
local function setupCollisionGroups()
    for _, part in ipairs(heli:GetDescendants()) do
        if part:IsA("BasePart") then
            part.CollisionGroup = "Helicopter"
        end
    end
end
setupCollisionGroups()

--CONSTANTS--
local MAX_SPEED_FORWARD = 70
local MAX_SPEED_STRAFE = 95
local HORIZONTAL_ACCEL = 0.4
local HORIZONTAL_DRAG = 3

local YAW_ACCEL = 1.2
local YAW_DAMP = 6
local BANK_TURN_RATE = 2.5

local VERTICAL_SPEED = 40
local MAX_DESCENT_SPEED = 20
local VERTICAL_KP = 6
local ALTITUDE_KP = 8
local ALTITUDE_KD = 3

local MAX_TILT_PITCH = math.rad(35)
local MAX_TILT_ROLL = math.rad(60)
local TILT_SMOOTH = 10
local COORDINATED_PITCH = 0.35

local ROTOR_SPEED_MAX = 25
local ROTOR_SPOOL_UP = 1.0
local ROTOR_SPOOL_DOWN = 0.3
local SOUND_PITCH_MIN = 0.6
local SOUND_PITCH_MAX = 1.2

--VARIABLES--
local pilot = nil
local engineOn = false
local currentRotorSpeed = 0

local input = {
    w = false, s = false,
    a = false, d = false,
    q = false, e = false,
    up = false, down = false,
}

local yaw = 0
local yawVelocity = 0
local horizontalVelocity = Vector3.zero
local currentPitch = 0
local currentRoll = 0
local smoothVertTarget = 0
local targetAltitude = nil

local rayParams = RaycastParams.new()
rayParams.FilterDescendantsInstances = { heli }
rayParams.FilterType = Enum.RaycastFilterType.Exclude

--FUNCTIONS--
local function resetState()
    yawVelocity = 0
    horizontalVelocity = Vector3.zero
    smoothVertTarget = 0
    targetAltitude = nil
    currentPitch = 0
    currentRoll = 0
    force.Force = Vector3.zero
    input = { w=false, s=false, a=false, d=false, q=false, e=false, up=false, down=false }
end

local function updateRotors(ratio)
    hingeMain.AngularVelocity = currentRotorSpeed
    hingeMain.MotorMaxTorque = 100000
    hingeTail.AngularVelocity = currentRotorSpeed * 0.6
    hingeTail.MotorMaxTorque = 100000
    engineLoop.PlaybackSpeed = SOUND_PITCH_MIN + (SOUND_PITCH_MAX - SOUND_PITCH_MIN) * ratio
    engineLoop.Volume = ratio
end

local function updateYaw(dt)
    local yawInput = (input.q and 1 or 0) - (input.e and 1 or 0)
    yawVelocity = yawVelocity + ((yawInput * YAW_ACCEL) - yawVelocity) * math.clamp(dt * YAW_DAMP, 0, 1)
    yaw += yawVelocity * dt
end

local function updateHorizontal(dt, forward, right)
    local targetHorizontal = Vector3.zero
    if input.w then targetHorizontal += forward * MAX_SPEED_FORWARD end
    if input.s then targetHorizontal -= forward * MAX_SPEED_FORWARD end
    if input.d then targetHorizontal += right * MAX_SPEED_STRAFE end
    if input.a then targetHorizontal -= right * MAX_SPEED_STRAFE end

    horizontalVelocity = horizontalVelocity:Lerp(targetHorizontal, math.clamp(dt * HORIZONTAL_ACCEL, 0, 1))

    if not (input.w or input.s or input.a or input.d) then
        horizontalVelocity *= math.max(0, 1 - HORIZONTAL_DRAG * dt)
    end

    -- P-controller for horizontal velocity: error between desired and actual velocity,
    -- scaled by mass, produces the corrective force (F = m * a).
    local currentHorizontal = Vector3.new(core.AssemblyLinearVelocity.X, 0, core.AssemblyLinearVelocity.Z)
    return (horizontalVelocity - currentHorizontal) * core.AssemblyMass * 6
end

local function updateTilt(dt, yawCF)
    local localVel = yawCF:VectorToObjectSpace(
        Vector3.new(horizontalVelocity.X, 0, horizontalVelocity.Z)
    )

    local targetPitch, targetRoll = 0, 0
    if localVel.Magnitude > 0.01 then
        local dir = Vector3.new(localVel.X, 0, localVel.Z).Unit
        local pitchRatio = math.clamp(math.abs(localVel.Z) / MAX_SPEED_FORWARD, 0, 1)
        local rollRatio = math.clamp(math.abs(localVel.X) / MAX_SPEED_STRAFE, 0, 1)
        -- Extra nose dip during lateral movement: strafing at high speed causes a realistic
        -- forward lean proportional to how much of the roll axis is being used.
        local extraNoseDip = rollRatio * MAX_TILT_ROLL * 0.6
        targetPitch = (dir.Z * pitchRatio * MAX_TILT_PITCH) - extraNoseDip * math.abs(dir.X)
        targetRoll = -dir.X * rollRatio * MAX_TILT_ROLL
    end

    currentPitch = currentPitch + (targetPitch - currentPitch) * math.clamp(dt * TILT_SMOOTH, 0, 1)
    currentRoll = currentRoll + (targetRoll - currentRoll) * math.clamp(dt * TILT_SMOOTH, 0, 1)

    return localVel
end

local function updateVertical(dt, mass, gravity, currentVY, groundDist, isLanded)
    if isLanded then
        targetAltitude = nil
        smoothVertTarget = 0
        local vel = core.AssemblyLinearVelocity
        core.AssemblyLinearVelocity = Vector3.new(vel.X, 0, vel.Z)
        return Vector3.zero
    end

    if input.up or input.down then
        targetAltitude = nil
        local rawTarget
        if input.up then
            rawTarget = VERTICAL_SPEED
        else
            -- Proximity brake: squaring brakeFactor makes deceleration more aggressive
            -- in the final few studs above the ground, softening the landing automatically.
            local brakeFactor = math.clamp(groundDist / 25, 0, 1)
            rawTarget = -MAX_DESCENT_SPEED * (brakeFactor * brakeFactor)
        end
        smoothVertTarget = smoothVertTarget + (rawTarget - smoothVertTarget) * math.clamp(dt * 2.5, 0, 1)
        local liftForce = Vector3.new(0, mass * gravity + (smoothVertTarget - currentVY) * mass * VERTICAL_KP, 0)
        if groundDist < 8 then
            liftForce = Vector3.new(0, liftForce.Y + (-currentVY * mass * 2.5), 0)
        end
        return liftForce
    end

    -- Altitude hold: lock a target Y when vertical keys are released.
    -- The momentum offset (currentVY * 0.4) lets the helicopter coast to a natural stop
    -- rather than snapping to the exact Y position the key was released at.
    if targetAltitude == nil then
        targetAltitude = core.Position.Y + math.clamp(currentVY * 0.4, -2, 6)
    end
    smoothVertTarget *= math.max(0, 1 - dt * 4)

    -- PD controller: proportional term corrects altitude error; derivative term
    -- (-currentVY * ALTITUDE_KD) damps oscillation so the helicopter doesn't bob.
    local altError = targetAltitude - core.Position.Y
    local correction = altError * ALTITUDE_KP - currentVY * ALTITUDE_KD
    return Vector3.new(0, math.max(mass * gravity + correction * mass, mass * gravity * 0.3), 0)
end

--EVENTS--
remote.OnServerEvent:Connect(function(plr, data)
    if plr ~= pilot then return end
    input.w = data.w == true
    input.s = data.s == true
    input.a = data.a == true
    input.d = data.d == true
    input.q = data.q == true
    input.e = data.e == true
    input.up = data.up == true
    input.down = data.down == true
end)

seat:GetPropertyChangedSignal("Occupant"):Connect(function()
    local hum = seat.Occupant
    if hum then
        pilot = Players:GetPlayerFromCharacter(hum.Parent)
        engineOn = true
        -- Transfer physics ownership to the pilot's client to eliminate input latency.
        core:SetNetworkOwner(pilot)
        -- Capture current heading so the helicopter doesn't snap to a different yaw on the first frame.
        local _, currentYaw, _ = core.CFrame:ToEulerAnglesYXZ()
        yaw = currentYaw
        resetState()
        engineLoop.Looped = true
        engineLoop:Play()
    else
        pilot = nil
        engineOn = false
        core:SetNetworkOwner(nil)
        resetState()
    end
    prompt.Enabled = not seat.Occupant
end)

--HEARTBEAT--
RunService.Heartbeat:Connect(function(dt)
    if not engineOn then
        -- ENGINE OFF: exponential decay toward zero rotor speed.
        -- The formula (speed * (1 - rate * dt)) produces a natural wind-down curve
        -- that is framerate-independent and asymptotically approaches zero.
        currentRotorSpeed = currentRotorSpeed * math.max(0, 1 - ROTOR_SPOOL_DOWN * dt * 3)
        local rotorRatio = math.clamp(currentRotorSpeed / ROTOR_SPEED_MAX, 0, 1)
        updateRotors(rotorRatio)
        if currentRotorSpeed < 0.05 then
            currentRotorSpeed = 0
            updateRotors(0)
            engineLoop:Stop()
        end
        return
    end

    local mass = core.AssemblyMass
    local gravity = workspace.Gravity
    local currentVY = core.AssemblyLinearVelocity.Y
    local groundHit = workspace:Raycast(core.Position, Vector3.new(0, -1, 0) * 200, rayParams)
    local groundDist = groundHit and groundHit.Distance or 200
    local isLanded = groundDist < 3 and math.abs(currentVY) < 1

    updateYaw(dt)

    local yawCF = CFrame.Angles(0, yaw, 0)
    local horizontalForce = updateHorizontal(dt, yawCF.LookVector, yawCF.RightVector)
    local localVel = updateTilt(dt, yawCF)

    -- Banking feeds back into yaw: roll at forward speed curves the flight path,
    -- replicating the aerodynamic behavior of a real coordinated helicopter turn.
    local forwardRatio = math.clamp(math.abs(localVel.Z) / MAX_SPEED_FORWARD, 0, 1)
    yaw += currentRoll * forwardRatio * BANK_TURN_RATE * dt

    -- Compose the gyro target: heading yaw, flight tilt, and a small coordinated pitch
    -- offset that dips the nose slightly in the direction of the bank.
    local coordinatedPitch = -currentRoll * forwardRatio * COORDINATED_PITCH
    gyro.CFrame = CFrame.Angles(0, yaw, 0) * CFrame.Angles(currentPitch + coordinatedPitch, 0, currentRoll)

    local liftForce = updateVertical(dt, mass, gravity, currentVY, groundDist, isLanded)
    force.Force = liftForce + horizontalForce

    currentRotorSpeed = currentRotorSpeed + (ROTOR_SPEED_MAX - currentRotorSpeed) * math.clamp(dt * ROTOR_SPOOL_UP, 0, 1)
    updateRotors(math.clamp(currentRotorSpeed / ROTOR_SPEED_MAX, 0, 1))
end)