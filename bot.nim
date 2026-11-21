# A robot on the Clunkyard core's Nim binding. The core owns tracking,
# geometry and protocol handling; these callbacks choose the tactics.
# It searches with a broad scan, narrows the beam on a contact, and leads
# shots from a line through its recent sightings.
# Clunkyard robot API 2.1.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude wall_contact hurt applied hull_mounts
#   to_mount(heading pitch) from_mount(heading pitch) mount_toward(point)
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   history(slot) last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) wall_margin() pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# drone: has ready in_flight launches_left report sightings
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) to_chassis(heading pitch attitude)
#   from_chassis(heading pitch attitude) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
#   drone.launch(heading pitch param) drone.brief(index value)
#   drone.program(language source) status.set(text value) debug.log(line)
#   raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
import clunkyard,std/math

# Bearing is earned: a scan contact's bearing is off by up to its bearingError
# (degrees, a fixed share of the arc), so a wide beam finds a robot and a narrow
# one aims at it. Contacts within 120 units are exact.
const searchArc=40.0   # degrees: sweeps the circle in half a second
const holdWidth=60.0   # units across at the target: it stays in the beam from tick to tick
const minArc=6.0       # degrees: the narrowest the beam holds
const window=12        # ticks of sightings the aim averages
const sightings=8      # narrow sightings in the window before a shot
const lookEvery=40     # ticks between looks around the target, for a nearer robot
const reacquire=10     # ticks to look again where a lost target was

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

# A straight line through the narrowest of the last `window` ticks' sightings:
# where the robot is now and how fast it moves. Each sighting is off by its
# bearingError; the line averages that out.
proc fit(robot:pointer;slot,tick:int;position,velocity:var CyVec3;count:var int):bool=
  var all:array[32,CySighting]
  var total:cuint
  var rate:cint
  if cyTrackerHistory(robot,cint slot,addr all[0],32,addr total)!=CY_OK:return false
  check(robot,cyMatchTickRate(robot,addr rate))
  var best=Inf
  for s in all[0..<int(total)]:
    if s.tick>tick-window:best=min(best,s.bearingError)
  var seen:seq[CySighting]
  for s in all[0..<int(total)]:
    if s.tick>tick-window and s.bearingError<=1.5*best:seen.add s
  if seen.len<2:return false
  var t0=0.0
  for s in seen:t0+=float64(s.tick)
  t0/=float64(seen.len)
  var spread=0.0
  for s in seen:spread+=(float64(s.tick)-t0)^2
  # Least squares on one axis: its fitted position now, and its slope per tick.
  proc line(seen:seq[CySighting];pick:proc(s:CySighting):float64;t0,spread:float64;tick:int):(float64,float64)=
    var mean=0.0
    for s in seen:mean+=pick(s)
    mean/=float64(seen.len)
    var along=0.0
    for s in seen:along+=(float64(s.tick)-t0)*(pick(s)-mean)
    let slope=along/spread
    (mean+slope*(float64(tick)-t0),slope)
  let (px,vx)=line(seen,proc(s:CySighting):float64=s.position.x,t0,spread,tick)
  let (py,vy)=line(seen,proc(s:CySighting):float64=s.position.y,t0,spread,tick)
  let (pz,vz)=line(seen,proc(s:CySighting):float64=s.position.z,t0,spread,tick)
  position=CyVec3(x:px,y:py,z:pz);velocity=CyVec3(x:vx*float64(rate),y:vy*float64(rate),z:vz*float64(rate))
  count=seen.len
  true
var freeFlight=false
var dodgeTicks,dodges,slot=0
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
  var lastSeen:cint=cint(-1000)
  if slot!=0 and cyTrackerLastSeen(robot,cint slot,addr lastSeen)!=CY_OK:lastSeen= -1000
  if cySensorsNearest(robot,addr target)==CY_OK:
    slot=int target.slot
    let throttle=if target.range>350:0.7 elif target.range<200: -0.4 else:0.2
    drive(robot,throttle,steer(target.bearing,heading),steer(target.elevation,pitch))
    # Halve the beam each tick until it is holdWidth across at the target:
    # narrow enough for a precise bearing, wide enough to keep it in.
    let hold=max(minArc,2*arctan2(holdWidth/2,target.range)*180/PI)
    # In a crowd a narrow beam sees one robot. Every lookEvery ticks, widen it for a tick.
    let arc=if tick mod lookEvery==0:searchArc else:max(hold,min(searchArc,last.arc/2))
    scan(robot,target.bearing,arc,target.elevation)
    var power:cdouble
    var aim:CySolution
    var aimed,ready=true
    check(robot,cyGunPowerForFlightTime(robot,target.range,1,addr power))
    if target.bearingError>0:
      # A noisy contact: lead the fitted line, and fire only once it rests on enough sightings.
      var position,velocity:CyVec3
      var count:int
      aimed=fit(robot,int target.slot,int tick,position,velocity,count) and cyGunLeadPoint(robot,addr position,addr velocity,power,addr aim)==CY_OK
      ready=aimed and count>=sightings
    else:
      # An exact contact: cyGunLead is the intercept if the target holds course.
      aimed=cyGunLead(robot,addr target,power,addr aim)==CY_OK
    if not aimed:aim.heading=target.bearing;aim.pitch=target.elevation
    check(robot,cyGunAimHeading(robot,aim.heading));check(robot,cyGunAimPitch(robot,aim.pitch))
    var now:CyDirection
    var delta:cdouble
    var canFire:cint
    check(robot,cyGunAimNow(robot,addr now));check(robot,cyAnglesAngleDiff(aim.heading,now.heading,addr delta));check(robot,cyGunCanFire(robot,power,10,addr canFire))
    if ready and abs(delta)<3 and abs(aim.pitch-now.pitch)<3 and canFire!=0:check(robot,cyGunFire(robot));check(robot,cyGunPower(robot,power))
    show(robot,(if ready:"TRACK" else:"NARROW"),target.range)
  elif slot!=0 and int(tick)-int(lastSeen)<=reacquire:
    # Lost it: look again where it was last seen, widening the beam, before searching.
    var where:CyVec3
    var bearing,elevation:cdouble
    check(robot,cyTrackerPosition(robot,cint slot,addr where));check(robot,cyAnglesBearingTo(addr position,addr where,addr bearing))
    if freeFlight:check(robot,cyAnglesElevationTo(addr position,addr where,addr elevation))
    drive(robot,0.5,steer(bearing,heading),steer(elevation,pitch));scan(robot,bearing,min(searchArc,last.arc*2),elevation);show(robot,"REACQUIRE")
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
    drive(robot,0.5,0.33,steer(0,pitch));scan(robot,sweep,searchArc,up);show(robot,"SEARCH")
  # A simple dodge: vary it, or predict the shooter.
  var cues:array[32,CyContact]
  var count:cuint
  var fired=false
  if cySensorsCues(robot,addr cues[0],cues.len.cuint,addr count)==CY_OK:
    for heard in cues[0..<int(count)]:fired=fired or heard.source==CY_SOURCE_MUZZLE
  if dodgeTicks==0 and fired:
    inc dodges;dodgeTicks=10
  if dodgeTicks>0:
    # Odd dodges go left, even ones right; turn +1 and strafe -1 go left.
    let left=dodges mod 2==1
    let side=if left:1.0 else: -1.0
    var control:cint
    check(robot,cyMatchControl(robot,addr control));dec dodgeTicks
    if control==CY_CONTROL_FORCES:check(robot,cyDriveThrottle(robot,0.5));check(robot,cyDriveStrafe(robot,-side))
    else:check(robot,cyDriveThrottle(robot,1));check(robot,cyDriveTurn(robot,side))
    show(robot,(if left:"dodge:left" else:"dodge:right"),float64(dodges))
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
