local FINGER_OPEN_X   = 0.010
local FINGER_CLOSED_X = 0.005
local FINGER_Z_OFFSET = 0.030
local GRIP_RANGE      = 0.080
local MOTION_STEPS    = 40
local CONVERGE_STEPS  = 20

function lookup(path)
    local ok, h = pcall(sim.getObject, path)
    if ok then return h else return -1 end
end

function find_tip(base)
    local ok, h = pcall(sim.getObject, '/Mirobot/tip')
    if ok and h and h ~= -1 then return h end
    local children = sim.getObjectsInTree(base, sim.handle_all, 0)
    for _, obj in ipairs(children) do
        if sim.getObjectAlias(obj) == 'tip' then
            return obj
        end
    end
    error('Could not find "tip" dummy under /Mirobot')
end

function sysCall_init()
    print('=== MIROBOT CONTROLLER v6-SAFE ===')

    initOK  = false
    initErr = ''
    _diagShown = false

    local ok, err = pcall(function()
        if type(simIK) ~= 'table' then
            simIK = require('simIK')
        end

        base   = sim.getObject('/Mirobot')
        tip    = find_tip(base)
        target = sim.getObject('/Mirobot/target')

        jointHandles = {}
        for i = 1, 6 do
            jointHandles[i] = sim.getObject('/Mirobot/joint' .. i)
        end

        for i = 1, 6 do
            pcall(sim.setJointMode, jointHandles[i],
                sim.jointmode_dynamic, 0)
        end

        local j2_min = -110 * math.pi / 180
        local j2_max =   75 * math.pi / 180
        sim.setJointInterval(jointHandles[2], false,
            {j2_min, j2_max - j2_min})
        print(string.format(
            '[init] joint2 range widened to [%+.3f, %+.3f] rad '
            .. '(%+d°, %+d°) per Wlkata Mirobot manufacturer spec',
            j2_min, j2_max, -110, 75))

        local curProp = sim.getModelProperty(base)
        if (curProp & sim.modelproperty_not_dynamic) == 0 then
            sim.setModelProperty(base,
                curProp | sim.modelproperty_not_dynamic)
            print('[init] Mirobot model flagged NOT_DYNAMIC '
                  .. '(was dynamic, now frozen in physics)')
        else
            print('[init] Mirobot model already NOT_DYNAMIC')
        end

        ikEnv   = simIK.createEnvironment()
        ikGroup = simIK.createGroup(ikEnv)
        simIK.setGroupCalculation(ikEnv, ikGroup,
            simIK.method_damped_least_squares, 0.3, 99)

        local ikConstraint = simIK.constraint_position
                           + simIK.constraint_alpha_beta

        local ret = {simIK.addElementFromScene(
            ikEnv, ikGroup, base, tip, target, ikConstraint)}
        local simToIkMap = ret[2]

        ikJoints = {}
        for i = 1, 6 do
            local h = -1
            if type(simToIkMap) == 'table' then
                h = simToIkMap[jointHandles[i]] or -1
            end
            if h == -1 then
                local alias = sim.getObjectAlias(jointHandles[i])
                local okh, ikh = pcall(
                    simIK.getObjectHandle, ikEnv, alias)
                if okh and ikh and ikh ~= -1 then h = ikh end
            end
            ikJoints[i] = h
        end

        print('[init] Scene joint intervals:')
        for i = 1, 6 do
            local cyclic, interval = sim.getJointInterval(jointHandles[i])
            local pos = sim.getJointPosition(jointHandles[i])
            local range_str
            if cyclic then
                range_str = 'cyclic'
            else
                range_str = string.format(
                    '[%+.3f, %+.3f]', interval[1], interval[1]+interval[2])
            end
            print(string.format(
                '[init]   joint%d  cur=%+7.3f rad  range=%s',
                i, pos, range_str))
        end

        local tipPos = sim.getObjectPosition(tip, -1)
        sim.setObjectPosition(target, tipPos, -1)
        sim.setObjectOrientation(target, {math.pi, 0, 0}, -1)

        local tmat = sim.getObjectMatrix(target, -1)
        local zx, zy, zz = tmat[3], tmat[7], tmat[11]
        print(string.format(
            '[init] target +Z axis in world = (%.3f, %.3f, %.3f)  '
            .. '(expect 0, 0, -1 for world-down)',
            zx, zy, zz))

        leftFinger  = lookup('/Mirobot/gripper2f_leftFinger')
        rightFinger = lookup('/Mirobot/gripper2f_rightFinger')
        gripperBase = lookup('/Mirobot/gripper2f_base')

        heldBlock = -1
        open_fingers()
    end)

    if ok then
        initOK = true
        print('[Mirobot controller] ready')
    else
        initErr = tostring(err)
        print('[Mirobot controller] INIT FAILED: ' .. initErr)
    end
end

function open_fingers()
    if leftFinger and leftFinger ~= -1 then
        sim.setObjectPosition(leftFinger,
            {-FINGER_OPEN_X, 0, FINGER_Z_OFFSET}, tip)
    end
    if rightFinger and rightFinger ~= -1 then
        sim.setObjectPosition(rightFinger,
            { FINGER_OPEN_X, 0, FINGER_Z_OFFSET}, tip)
    end
end

function close_fingers()
    if leftFinger and leftFinger ~= -1 then
        sim.setObjectPosition(leftFinger,
            {-FINGER_CLOSED_X, 0, FINGER_Z_OFFSET}, tip)
    end
    if rightFinger and rightFinger ~= -1 then
        sim.setObjectPosition(rightFinger,
            { FINGER_CLOSED_X, 0, FINGER_Z_OFFSET}, tip)
    end
end

function drive_joints_to_ik()
    for i = 1, 6 do
        if ikJoints[i] ~= -1 then
            local okq, q = pcall(
                simIK.getJointPosition, ikEnv, ikJoints[i])
            if okq and type(q) == 'number' then
                sim.setJointPosition(jointHandles[i], q)
            end
        end
    end
end

function ping()
    if initOK then return 'ready' end
    return 'init_failed: ' .. initErr
end

local IK_OPTS = {syncWorlds = true, allowError = true}

function goto_pose(x, y, z)
    if not initOK then return {0, 0, 0} end
    local ok, result = pcall(function()
        local startPos = sim.getObjectPosition(target, -1)
        local lastRes, lastFlags, lastPrec = nil, nil, nil

        for i = 1, MOTION_STEPS do
            local t  = i / MOTION_STEPS
            local te = t * t * (3 - 2 * t)
            local pos = {
                startPos[1] + te * (x - startPos[1]),
                startPos[2] + te * (y - startPos[2]),
                startPos[3] + te * (z - startPos[3]),
            }
            sim.setObjectPosition(target, pos, -1)
            lastRes, lastFlags, lastPrec = simIK.handleGroup(
                ikEnv, ikGroup, IK_OPTS)
            drive_joints_to_ik()
        end
        for i = 1, CONVERGE_STEPS do
            lastRes, lastFlags, lastPrec = simIK.handleGroup(
                ikEnv, ikGroup, IK_OPTS)
            drive_joints_to_ik()
        end

        if not _diagShown then
            _diagShown = true
            local p_lin, p_ang = 0, 0
            if type(lastPrec) == 'table' then
                p_lin = lastPrec[1] or 0
                p_ang = lastPrec[2] or 0
            end
            print(string.format(
                '[goto_pose DIAG] first-call IK: result=%s flags=%s '
                .. 'precision=(lin=%.4f, ang=%.4f)',
                tostring(lastRes), tostring(lastFlags),
                p_lin, p_ang))

            local sceneTip = sim.getObjectPosition(tip, -1)
            print(string.format(
                '[goto_pose DIAG] scene    tip pos   = (%.3f, %.3f, %.3f)',
                sceneTip[1], sceneTip[2], sceneTip[3]))
            print(string.format(
                '[goto_pose DIAG] commanded target  = (%.3f, %.3f, %.3f)',
                x, y, z))

            print('[goto_pose DIAG] joint positions (rad):')
            for i = 1, 6 do
                local qScene = sim.getJointPosition(jointHandles[i])
                local qIk = qScene
                if ikJoints[i] ~= -1 then
                    local okq, qv = pcall(
                        simIK.getJointPosition, ikEnv, ikJoints[i])
                    if okq and type(qv) == 'number' then qIk = qv end
                end
                print(string.format(
                    '[goto_pose DIAG]   joint%d  scene=%+7.3f  ik=%+7.3f  diff=%+7.3f',
                    i, qScene, qIk, qIk - qScene))
            end
        end

        local p = sim.getObjectPosition(tip, -1)
        local dx, dy, dz = p[1]-x, p[2]-y, p[3]-z
        local err = math.sqrt(dx*dx + dy*dy + dz*dz)
        if err > 0.020 then
            print(string.format(
                '[goto_pose] WARN: cmd (%.3f, %.3f, %.3f) tip (%.3f, %.3f, %.3f) err=%.3f',
                x, y, z, p[1], p[2], p[3], err))
        end
        return {p[1], p[2], p[3]}
    end)
    if ok then return result end
    print('[goto_pose] ERROR: ' .. tostring(result))
    return {-1, -1, -1}
end

function get_tip_pos()
    if not initOK then return {0, 0, 0} end
    local p = sim.getObjectPosition(tip, -1)
    return {p[1], p[2], p[3]}
end

function grip()
    if not initOK then return 0 end
    local ok, result = pcall(function()
        close_fingers()
        if gripperBase == -1 then
            print('[grip] gripperBase handle not set')
            return 0
        end
        local gPos = sim.getObjectPosition(gripperBase, -1)
        local candidates = sim.getObjectsInTree(
            sim.handle_scene, sim.object_shape_type, 0)
        local nearest, nearestD = -1, GRIP_RANGE
        local nearestAlias = ''
        for _, h in ipairs(candidates) do
            local alias = sim.getObjectAlias(h) or ''
            if alias:sub(1, 5) == 'Block' then
                local bp = sim.getObjectPosition(h, -1)
                local d = math.sqrt(
                    (bp[1]-gPos[1])^2 +
                    (bp[2]-gPos[2])^2 +
                    (bp[3]-gPos[3])^2)
                if d < nearestD then
                    nearest, nearestD = h, d
                    nearestAlias = alias
                end
            end
        end
        if nearest == -1 then
            print(string.format(
                '[grip] no block within %.3fm of gripper at (%.3f, %.3f, %.3f)',
                GRIP_RANGE, gPos[1], gPos[2], gPos[3]))
            return 0
        end
        print(string.format(
            '[grip] grabbed %s (d=%.3fm)', nearestAlias, nearestD))
        sim.setObjectInt32Param(nearest, sim.shapeintparam_static, 1)
        sim.setObjectParent(nearest, gripperBase, true)
        heldBlock = nearest
        return 1
    end)
    if ok then return result end
    print('[grip] ERROR: ' .. tostring(result))
    return 0
end

function release()
    if not initOK then return 0 end
    local ok, result = pcall(function()
        if heldBlock ~= -1 then
            sim.setObjectParent(heldBlock, -1, true)
            heldBlock = -1
        end
        open_fingers()
        return 1
    end)
    if ok then return result end
    print('[release] ERROR: ' .. tostring(result))
    return 0
end

function home()
    if not initOK then return 0 end
    local ok, result = pcall(function()

        goto_pose(0.14, 0.00, 0.20)
        return 1
    end)
    if ok then return result end
    print('[home] ERROR: ' .. tostring(result))
    return 0
end

function sysCall_cleanup()
    if ikEnv then
        pcall(simIK.eraseEnvironment, ikEnv)
    end
end
