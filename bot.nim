# A robot on the Clunkyard core's Nim binding. The core owns tracking,
# geometry and protocol handling; these callbacks choose the tactics.
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range)
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
import clunkyard

template get(query:untyped;robot:pointer):float64=
  block:
    var value:cdouble
    check(robot,query(robot,addr value))
    value
proc steer(towards,current:float64):float64=
  var delta,value:cdouble
  check(nil,cyAnglesAngleDiff(towards,current,addr delta))
  check(nil,cyAnglesClamp(delta/45,-1,1,addr value))
  value
proc drive(robot:pointer;throttle,turn,pitch:float64)=
  check(robot,cyDriveThrottle(robot,throttle));check(robot,cyDriveTurn(robot,turn));check(robot,cyDrivePitchTurn(robot,pitch))
proc scan(robot:pointer;heading,arc,pitch:float64)=
  check(robot,cyScannerHeading(robot,heading));check(robot,cyScannerArc(robot,arc));check(robot,cyScannerPitch(robot,pitch))
proc show(robot:pointer;text:string)=check(robot,cyStatusText(robot,text.cstring,text.len.cuint))
proc show(robot:pointer;text:string;value:float64)=show(robot,text);check(robot,cyStatusValue(robot,value))
var freeFlight=false
proc start(robot:pointer)=
  var gravity:CyVec3
  var is3d:cint
  check(robot,cyArenaGravity(robot,addr gravity));check(robot,cyMatchIs3d(robot,addr is3d))
  freeFlight=is3d!=0 and gravity.z==0
proc turn(robot:pointer)=
  var position:CyVec3
  var target:CyContact
  var cue:CyCue
  var last:CyScan
  check(robot,cyMePosition(robot,addr position));check(robot,cyScannerLast(robot,addr last))
  let heading=get(cyMeHeading,robot)
  let pitch=get(cyMePitch,robot)
  var tick:cint
  check(robot,cyClockTick(robot,addr tick));check(robot,cyDriveBrake(robot,0.2))
  if cySensorsNearest(robot,addr target)==CY_OK:
    let throttle=if target.range>350:0.7 elif target.range<200: -0.4 else:0.2
    drive(robot,throttle,steer(target.bearing,heading),steer(target.elevation,pitch));scan(robot,target.bearing,20,target.elevation)
    var power:cdouble
    var aim:CySolution
    check(robot,cyGunPowerForFlightTime(robot,target.range,1,addr power))
    if cyGunLead(robot,addr target,power,addr aim)!=CY_OK:aim.heading=target.bearing;aim.pitch=target.elevation
    check(robot,cyGunAimHeading(robot,aim.heading));check(robot,cyGunAimPitch(robot,aim.pitch))
    var now:CyDirection
    var delta:cdouble
    var canFire:cint
    check(robot,cyGunAimNow(robot,addr now));check(robot,cyAnglesAngleDiff(aim.heading,now.heading,addr delta));check(robot,cyGunCanFire(robot,power,10,addr canFire))
    if abs(delta)<3 and abs(aim.pitch-now.pitch)<3 and canFire!=0:check(robot,cyGunFire(robot));check(robot,cyGunPower(robot,power))
    show(robot,"TRACK",target.range)
  elif cyTrackerCue(robot,addr cue)==CY_OK:
    var bearing,distance,elevation,arc:cdouble
    check(robot,cyAnglesBearingTo(addr position,addr cue.position,addr bearing));check(robot,cyAnglesDistance(addr position,addr cue.position,addr distance))
    if freeFlight:check(robot,cyAnglesElevationTo(addr position,addr cue.position,addr elevation))
    let error=min(cue.bearingError,30.0)
    let across=float64(tick mod 5-2)/2*error
    let up=if freeFlight:float64((tick div 5)mod 3-1)*error/2 else:0.0
    drive(robot,0.7,steer(bearing,heading),steer(elevation,pitch));check(robot,cyScannerArcForReach(robot,distance+100,addr arc))
    scan(robot,bearing+across,arc,elevation+up);show(robot,"CUE",distance)
  else:
    var sweep:cdouble
    check(robot,cyAnglesNormalize(last.heading+36,addr sweep))
    let up=if freeFlight:float64((tick div 10)mod 3-1)*35 else:0.0
    drive(robot,0.5,0.33,steer(0,pitch));scan(robot,sweep,40,up);show(robot,"SEARCH")
  if get(cyBatteryCharge,robot)<get(cyBatteryCapacity,robot)/4:check(robot,cyDriveThrottle(robot,get(cyDriveGetThrottle,robot)*0.5))
  var storm:CyStorm
  var margin:cdouble
  if cyNavStorm(robot,addr storm)==CY_OK and cyNavStormMargin(robot,addr position,addr margin)==CY_OK and margin> -60:
    var away,delta,control:cdouble
    check(robot,cyAnglesBearingTo(addr storm.center,addr position,addr away));check(robot,cyAnglesAngleDiff(away,heading,addr delta))
    check(robot,cyAnglesClamp(delta/30,-1,1,addr control));check(robot,cyDriveThrottle(robot,0.8));check(robot,cyDriveTurn(robot,control))
  var zone:CyZone
  if cyNavZone(robot,addr zone)==CY_OK and cyNavZoneMargin(robot,addr position,addr margin)==CY_OK and margin<60:
    var bearing,climb,delta,turning,pitching:cdouble
    check(robot,cyAnglesBearingTo(addr position,addr zone.center,addr bearing))
    if zone.shape==CY_SHAPE_SPHERE:check(robot,cyAnglesElevationTo(addr position,addr zone.center,addr climb))
    check(robot,cyAnglesAngleDiff(bearing,heading,addr delta));check(robot,cyAnglesClamp(delta/30,-1,1,addr turning))
    check(robot,cyAnglesClamp((climb-pitch)/30,-1,1,addr pitching));drive(robot,0.8,turning,pitching)
run(turn,start)
