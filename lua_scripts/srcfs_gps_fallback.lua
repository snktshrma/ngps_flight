local PARAM_TABLE_KEY = 73
local PARAM_TABLE_PREFIX = 'SRCFS_'
local MAV_SEVERITY = {CRITICAL = 2, WARNING = 4, INFO = 6}
local PERIOD_MS = 100

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'srcfs: could not add param table')
local function bind_add_param(name, idx, default)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default), string.format('srcfs: could not add %s%s', PARAM_TABLE_PREFIX, name))
  return Parameter(PARAM_TABLE_PREFIX .. name)
end
local SRCFS_ENABLE = bind_add_param('ENABLE', 1, 1)
local SRCFS_DIST_M = bind_add_param('DIST_M', 2, 20)
local SRCFS_VO_TO_S = bind_add_param('VO_TO_S', 3, 0.5)
local SRCFS_GPS_FIX = bind_add_param('GPS_FIX', 4, 3)

local vo_bad_since_ms = nil
local warned_no_gps = false

local function gps_ok()
  return gps:num_sensors() > 0 and gps:status(0) >= SRCFS_GPS_FIX:get()
end

local function reason_to_fall_back(now_ms)
  if vehicle:has_ekf_failsafed() then
    return 'EKF failsafe'
  end
  if not rc:has_valid_input() then
    return 'RC lost'
  end
  if battery:has_failsafed() then
    return 'battery failsafe'
  end
  if visual_odom and not visual_odom:healthy() then
    vo_bad_since_ms = vo_bad_since_ms or now_ms
    if (now_ms - vo_bad_since_ms) >= SRCFS_VO_TO_S:get() * 1000 then
      return 'VisOdom unhealthy'
    end
  else
    vo_bad_since_ms = nil
  end
  local dist_max = SRCFS_DIST_M:get()
  if dist_max > 0 and gps_ok() then
    local ekf_loc = ahrs:get_location()
    local gps_loc = gps:location(0)
    if ekf_loc and gps_loc then
      local d = ekf_loc:get_distance(gps_loc)
      if d > dist_max then
        return string.format('vision %.0fm off GPS', d)
      end
    end
  end
  return nil
end

local function update()
  if SRCFS_ENABLE:get() <= 0 then
    return update, 1000
  end
  local now_ms = millis():toint()
  if ahrs:get_posvelyaw_source_set() == 0 then
    vo_bad_since_ms = nil
    warned_no_gps = false
    return update, PERIOD_MS
  end
  local reso = reason_to_fall_back(now_ms)
  if reso then
    if gps_ok() then
      ahrs:set_posvelyaw_source_set(0)
      gcs:send_text(MAV_SEVERITY.CRITICAL, 'SRCFS: ' .. reso .. ' -> src1 GPS')
      vo_bad_since_ms = nil
      warned_no_gps = false
    elseif not warned_no_gps then
      gcs:send_text(MAV_SEVERITY.CRITICAL, 'SRCFS: ' .. reso .. ', no GPS: stay')
      warned_no_gps = true
    end
  end
  return update, PERIOD_MS
end

gcs:send_text(MAV_SEVERITY.INFO, 'SRCFS: GPS fallback loaded')
return update, 1000
