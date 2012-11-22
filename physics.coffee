# vim: set expandtab ts=2 sw=2:

dt = 1/960  #update frequency in hz
#if we want a ~960hz simulation but only have <= 60 fps from the browser
#we need to call step 16 times with 60*16 = 960 updates/s
steps_per_frame = 16

b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef
b2Vec2 = Box2D.Common.Math.b2Vec2
b2Color = Box2D.Common.b2Color
b2AABB = Box2D.Collision.b2AABB
b2Transform = Box2D.Common.Math.b2Transform
b2BodyDef = Box2D.Dynamics.b2BodyDef
b2Body = Box2D.Dynamics.b2Body
b2FixtureDef = Box2D.Dynamics.b2FixtureDef
b2Fixture = Box2D.Dynamics.b2Fixture
b2Shape = Box2D.Collision.Shapes.b2Shape
b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
b2World = Box2D.Dynamics.b2World
b2MassData = Box2D.Collision.Shapes.b2MassData
b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
b2DebugDraw = Box2D.Dynamics.b2DebugDraw

class physics
  constructor: ->
    @world = new b2World(
      new b2Vec2(0, 9.81),    #gravity
      false                  #allow sleep
    )
      
    fixDef = new b2FixtureDef
    fixDef.density = 10
    fixDef.friction = 0.5
    fixDef.restitution = 0.1
    @fixDef = fixDef
    
    #create ground
    @ground_height = 0.03
    @ground_width = 3
    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_staticBody
    bodyDef.position.x = 1
    bodyDef.position.y = 1.1
    bodyDef.linearDamping = 50
    
    fixDef.shape = new b2PolygonShape
    fixDef.shape.SetAsBox @ground_width, @ground_height

    @ground = @world.CreateBody(bodyDef)
    @ground_bodyDef = bodyDef
    @ground.CreateFixture fixDef
  
    @lower_joint = null
    @upper_joint = null

    #setup debug draw
    @debugDraw = new b2DebugDraw()
    @debugDraw.SetSprite document.getElementById("canvas").getContext("2d")
    @debugDraw.SetDrawScale 260
    @debugDraw.SetFillAlpha 0.3
    @debugDraw.SetLineThickness 1.0
    @debugDraw.AppendFlags b2DebugDraw.e_shapeBit
    @world.SetDebugDraw @debugDraw
    
    #global flags
    @run = true           #if we are running continuously
    @step = false         #if we display the next step
    @pend_style = 0       #what pendulum / robot / etc. model is currently simulated
    @recordPhase = false  #if we log state data for saving later
    @startLog = true      #if we start a new log
    @logged_data = []     #where we log our data
    @beta = 0             #friction coefficient

    @abc = new abc()
  
  ##### methods to create bodies #####

  createBox: =>
    fixDef = new b2FixtureDef
    fixDef.density = 1
    fixDef.friction = 0.3
    fixDef.restitution = 0.2
    @fixDef = fixDef
    
    #create a box
    fixDef.shape = new b2PolygonShape
    fixDef.shape.SetAsBox 0.1, 0.1

    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_staticBody
    bodyDef.position.Set 1.3, 0.8
    box = @world.CreateBody bodyDef
    box.CreateFixture fixDef
  
  createCircle: =>
    fixDef = new b2FixtureDef
    fixDef.density = 1
    fixDef.friction = 0.3
    fixDef.restitution = 0.2
    @fixDef = fixDef
    
    fixDef.shape = new b2CircleShape
    fixDef.shape.m_radius = 0.12

    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_staticBody
    bodyDef.position.Set 1.3, 0.8
    box = @world.CreateBody bodyDef
    box.CreateFixture fixDef

  createPendulum: =>
    #create pendulum line
    pend_length = 0.400
    mass_size = 0.03
    damping = 0   #custom friction is used instead (further down)

    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_dynamicBody
    @fixDef.density = 35
    @fixDef.shape = new b2PolygonShape
    pend_vertices = new Array(
      #don't "touch" the ground so we don't get collisions while rotating
      new b2Vec2(@ground_bodyDef.position.x, @ground_bodyDef.position.y - @ground_height - 0.005),
      new b2Vec2(@ground_bodyDef.position.x, @ground_bodyDef.position.y - @ground_height - pend_length)
    )
    @fixDef.shape.SetAsArray pend_vertices, 2
    bodyDef.linearDamping = damping
    bodyDef.angularDamping = damping
    @body = @world.CreateBody(bodyDef)
    line = @body.CreateFixture(@fixDef)
    
    #initialise friction 
    @body.z2 = 0

    #create mass circle
    @fixDef.shape = new b2CircleShape(mass_size)
    @fixDef.shape.m_p = pend_vertices[1]
    mass = @body.CreateFixture(@fixDef)
    
    #rotating joint
    jointDef = new b2RevoluteJointDef()
    jointDef.Initialize @ground, @body, pend_vertices[0]
    jointDef.collideConnected = true
    jointDef.maxMotorTorque = @beta
    jointDef.motorSpeed = 0.0
    jointDef.enableMotor = true
    @lower_joint = @world.CreateJoint(jointDef)

    #initialize
    @lower_joint.angle_speed = 0
    @lower_joint.csl_active = false
    @lower_joint.bounce_active = false
    @lower_joint.joint_name = 'lower'
    @lower_joint.csl_sign = 1
    @lower_joint.gain = 1
    @lower_joint.gb = 0
    @lower_joint.motor_torque = 0
    @lower_joint.motor_control = 0
    @lower_joint.I_tm1 = 0
    
  createDoublePendulum: =>
    #create pendulum line
    pend_length = 0.5
    mass_size = 0.04
    damping = 0

    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_dynamicBody
    @fixDef.density = 10
    @fixDef.shape = new b2PolygonShape
    pend_vertices = new Array(
      #0.005 -> don't "touch" the ground so we don't get collisions while rotating
      new b2Vec2(@ground_bodyDef.position.x, @ground_bodyDef.position.y - @ground_height - 0.005),
      new b2Vec2(@ground_bodyDef.position.x, @ground_bodyDef.position.y - @ground_height - pend_length)
    )
    @fixDef.shape.SetAsArray pend_vertices, 2
    bodyDef.linearDamping = damping
    bodyDef.angularDamping = damping
    @body = @world.CreateBody(bodyDef)
    line = @body.CreateFixture(@fixDef)
    
    #initialise friction
    @body.z2 = 0
    
    #lower rotating joint
    jointDef = new b2RevoluteJointDef()
    jointDef.Initialize @body, @ground, pend_vertices[0]
    jointDef.collideConnected = true
    @lower_joint = @world.CreateJoint(jointDef)

    #initialize
    @lower_joint.angle_speed = 0
    @lower_joint.csl_active = false
    @lower_joint.bounce_active = false
    @lower_joint.joint_name = 'lower'
    @lower_joint.csl_sign = 1
    @lower_joint.gain = 1
    @lower_joint.gb = 0
    @lower_joint.motor_torque = 0
    @lower_joint.motor_control = 0
    @lower_joint.I_t = 0
    
    #create mass circle
    @fixDef.shape = new b2CircleShape(mass_size)
    @fixDef.shape.m_p = pend_vertices[1]
    mass = @body.CreateFixture(@fixDef)

    #second line
    pend_vertices = new Array(
      new b2Vec2(@ground_bodyDef.position.x, @ground_bodyDef.position.y - @ground_height - pend_length - 0.005),
      new b2Vec2(@ground_bodyDef.position.x, @ground_bodyDef.position.y - @ground_height - (2*pend_length))
    )
    @fixDef.shape = new b2PolygonShape
    @fixDef.shape.SetAsArray pend_vertices, 2
    bodyDef.linearDamping = damping
    bodyDef.angularDamping = damping
    @body2 = @world.CreateBody(bodyDef)
    line2 = @body2.CreateFixture(@fixDef)

    #initialise friction and motor state
    @body2.z2 = 0

    #upper rotating joint
    jointDef = new b2RevoluteJointDef()
    jointDef.Initialize @body2, @body, pend_vertices[0]
    jointDef.collideConnected = false
    @upper_joint = @world.CreateJoint(jointDef)
    
    @upper_joint.joint_name = 'upper'
    @upper_joint.angle_speed = 0
    @upper_joint.csl_active = false
    @upper_joint.csl_sign = 1
    @upper_joint.gain = 1
    @upper_joint.gb = 0
    @upper_joint.motor_torque = 0
    @upper_joint.bounce_active = false

    #create upper mass circle
    @fixDef.shape = new b2CircleShape(mass_size)
    @fixDef.shape.m_p = pend_vertices[1]
    mass = @body2.CreateFixture(@fixDef)

  ###
  ellipse2polygon: (r, b, x0, y0) =>
    points = new Array()
    step = 2*Math.PI/40
    for theta in [2*Math.PI..0] by -step
      x = + b*r*Math.cos(theta)
      y = - r*Math.sin(theta)
      points.push(new b2Vec2(x,y))
    return points[0...points.length-1]
  ###

  createSemni: (x0=1,y0=0.5) =>
    #semni overall weight: 432 g
    #body: 120 g
    #weight of one dynamixel: 72g
    #arm at body: 135 g (63+72)
    #second arm: 177 g (105+72) 
    #min/max angle of lower arm: -3.2421 and 1.90816 
    
    bodyDensity = 0.96  #0.99
    bodyFriction = 0.25
    bodyRestitution = 0.1

    upperArmDensity = 4.2 #=135g   #2.06 #=63g   #4.35
    upperArmFriction =  0.25
    upperArmRestitution = 0.1

    lowerArmDensity = 11.35 #7 #=105g #11.35 #=185g #10.9 #=177g  
    lowerArmFriction = 0.25
    lowerArmRestitution = 0.2
    
    #create body
    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_dynamicBody
    bodyDef.position.Set(x0,y0)
    @body = @world.CreateBody(bodyDef)

    @fixDef = new b2FixtureDef
    @fixDef.density = bodyDensity
    @fixDef.friction = bodyFriction
    @fixDef.restitution = bodyRestitution
    @fixDef.filter.groupIndex = -1  #negative groups never collide with each other
    @fixDef.shape = new b2PolygonShape
    #@fixDef.shape.SetAsArray contour, contour.length
    for fixture in contour
      @fixDef.shape.SetAsArray(fixture, fixture.length)
      @body.CreateFixture(@fixDef)

    #add head (hole)
    @fixDef.density = 0.00001
    @fixDef.shape = new b2CircleShape
    @fixDef.shape.m_p.Set(head[0].x, head[0].y)
    @fixDef.shape.m_radius = head[1]
    @fixDef.filter.groupIndex = 1
    @body.CreateFixture(@fixDef)

    #set center of mass, but keep the rest of the data
    md = new b2MassData()
    @body.GetMassData(md)
    md.center.Set(contourCenter.x, contourCenter.y)
    md.I = @body.GetInertia() + md.mass * (md.center.x * md.center.x + md.center.y * md.center.y)
    @body.SetMassData(md)
    
    #show center of mass
    #@fixDef.density = 0.00001
    #@fixDef.shape = new b2CircleShape
    #@fixDef.shape.m_p.Set(contourCenter.x, contourCenter.y)
    #@fixDef.shape.m_radius = 0.01
    #@fixDef.filter.groupIndex = -1
    #@body.CreateFixture(@fixDef)

    #############

    #upper arm (connected to body)
    bodyDef2 = new b2BodyDef
    bodyDef2.type = b2Body.b2_dynamicBody
    @body2 = @world.CreateBody(bodyDef2)

    @fixDef2 = new b2FixtureDef
    @fixDef2.density = upperArmDensity
    @fixDef2.friction = upperArmFriction
    @fixDef2.restitution = upperArmRestitution
    @fixDef2.filter.groupIndex = -1
    @fixDef2.shape = new b2PolygonShape
    for fixture in arm1ContourConvex
      @fixDef2.shape.SetAsArray(fixture, fixture.length)
      @body2.CreateFixture(@fixDef2)
 
    #set center of mass
    md = new b2MassData()
    @body2.GetMassData(md)
    md.center.Set(arm1Center.x, arm1Center.y)
    md.I = @body2.GetInertia() + md.mass * (md.center.x * md.center.x + md.center.y * md.center.y)
    @body2.SetMassData(md)
    @body2.SetPositionAndAngle(new b2Vec2(arm1Center.x, arm1Center.y), 0)
    
    #initialise friction and motor state
    @body2.z2 = 0
    
    #add motor mass separately to imitate moment of inertia different from only COM based
    ###
    @fixDef2.density = 14.2
    @fixDef2.shape = new b2CircleShape
    @fixDef2.shape.m_p.Set(arm1Center.x+0.03, arm1Center.y-0.015)
    @fixDef2.shape.m_radius = 0.04
    @fixDef2.filter.groupIndex = -1
    @body2.CreateFixture(@fixDef2)
    ###

    #connect body and arm with rotating joint
    jointDef = new b2RevoluteJointDef()
    #jointDef.Initialize @body, @body2, vertices2[0]
    jointDef.bodyA = @body
    jointDef.bodyB = @body2
    jointDef.localAnchorA.Set(arm1JointAnchor.x, arm1JointAnchor.y) #point on arm that is attached to body
    jointDef.localAnchorB.Set(arm1JointAnchor.x, arm1JointAnchor.y) #point on the body that arm is attached to 
    #jointDef.anchor = new b2Vec2(vertices[3].x, vertices[3].y)
    jointDef.collideConnected = true

    #simple friction with box2d possiblities (dry + stiction)
    jointDef.maxMotorTorque = @beta
    jointDef.motorSpeed = 0.0
    jointDef.enableMotor = true
    @upper_joint = @world.CreateJoint(jointDef)
    
    #initialize
    @upper_joint.joint_name = 'upper'
    @upper_joint.motor_control = 0
    @upper_joint.I_t = 0
    @upper_joint.angle_speed = 0
    @upper_joint.csl_active = false
    @upper_joint.csl_sign = 1
    @upper_joint.last_motor_torque = 0
    @upper_joint.motor_torque = 0
    @upper_joint.bounce_active = false
    @upper_joint.bounce_sign = 1
    @upper_joint.bounce_vel = 0.0003
    
    #####

    #lower arm (not at body)
    bodyDef3 = new b2BodyDef
    bodyDef3.type = b2Body.b2_dynamicBody
    #bodyDef3.position.Set(x0, y0 + 0.4)
    @body3 = @world.CreateBody(bodyDef3)
    
    @fixDef3 = new b2FixtureDef
    @fixDef3.density = lowerArmDensity
    @fixDef3.friction = lowerArmFriction
    @fixDef3.restitution = lowerArmRestitution
    @fixDef3.filter.groupIndex = -1
    @fixDef3.shape = new b2PolygonShape
    for fixture in arm2ContourConvex
      @fixDef3.shape.SetAsArray(fixture, fixture.length)
      @body3.CreateFixture(@fixDef3)

    #set center of mass
    md = new b2MassData()
    @body3.GetMassData(md)
    md.center.Set(arm2Center.x, arm2Center.y)
    md.I = @body3.GetInertia() + md.mass * (md.center.x * md.center.x + md.center.y * md.center.y)
    @body3.SetMassData(md)
    @body3.SetPositionAndAngle(new b2Vec2(arm1Center.x, arm1Center.y), 0)
    
    #add motor mass separately to imitate moment of inertia different from only COM based
    ###
    @fixDef3.density = 22.4  #=72g
    @fixDef3.shape = new b2CircleShape
    @fixDef3.shape.m_p.Set(arm2Center.x-0.04, arm2Center.y-0.01)
    @fixDef3.shape.m_radius = 0.03
    @fixDef3.filter.groupIndex = -1
    @body3.CreateFixture(@fixDef3)
    ###

    #show center of mass
    #@fixDef3.density = 0.00001
    #@fixDef3.shape = new b2CircleShape
    #@fixDef3.shape.m_p.Set(arm2Center.x, arm2Center.y)
    #@fixDef3.shape.m_radius = 0.01
    #@fixDef3.filter.groupIndex = -1
    #@body3.CreateFixture(@fixDef3)

    #initialise friction and motor state
    @body3.z2 = 0
    
    #lower rotating joint
    jointDef = new b2RevoluteJointDef()
    jointDef.bodyA = @body2
    jointDef.bodyB = @body3
    jointDef.localAnchorA.Set(arm2JointAnchor.x, arm2JointAnchor.y)
    jointDef.localAnchorB.Set(arm2JointAnchor.x, arm2JointAnchor.y)
    jointDef.collideConnected = false

    jointDef.maxMotorTorque = @beta
    jointDef.motorSpeed = 0.0
    jointDef.enableMotor = true
    
    jointDef.upperAngle = 1.90816
    jointDef.lowerAngle = -3.2421
    jointDef.enableLimit = true
    @lower_joint = @world.CreateJoint(jointDef)
    
    @lower_joint.joint_name = 'lower'
    @lower_joint.motor_control = 0
    @lower_joint.I_t = 0
    @lower_joint.angle_speed = 0
    @lower_joint.csl_active = false
    @lower_joint.csl_sign = 1
    @lower_joint.last_motor_torque = 0
    @lower_joint.motor_torque = 0
    @lower_joint.bounce_active = false
    @lower_joint.bounce_vel = 0.00047
    @lower_joint.bounce_sign = 1

  ##### stuff #####

  toggleCSL: (bodyJoint) =>
    if bodyJoint.bounce_active
      $("#toggle_bounce").click()
    bodyJoint.csl_active = not bodyJoint.csl_active
    if @lower_joint
      $("#set_csl_params_lower").trigger('click')
    if @upper_joint
      $("#set_csl_params_upper").trigger('click')
    unless bodyJoint.last_angle?
      bodyJoint.last_angle = bodyJoint.GetJointAngle()
    bodyJoint.motor_control = 0
    bodyJoint.last_integrated = 0

  toggleBounce: (bodyJoint) =>
    if bodyJoint.csl_active
      $("#toggle_csl").click()
    bodyJoint.bounce_active = not bodyJoint.bounce_active
    bodyJoint.motor_control = 0
    bodyJoint.last_integrated = 0
  
  getNoisyAngle: (bodyJoint) =>
    #sensor noise
    #rand = Math.random() / 1000
    #rand = rand - (rand/2)
    rand = 0
    -bodyJoint.GetJointAngle() * (1+rand)

  myon_precision: (number) =>
    Math.floor(number * 10000) / 10000
    
  logData: =>
    if @recordPhase
      if @startLog
        @logged_data = []
        @startLog = false
      
      @logged_data.push(-@body.GetAngle() + " " + -@upper_joint.GetJointAngle() + " " + -@lower_joint.GetJointAngle()+ " " + @body2.motor_control + " " + @body3.motor_control)

  ##### controllers #####

  CSL: (gi, gf, gb, angle_diff, gain=1, bodyJoint) =>
    unless bodyJoint.last_integrated?
      bodyJoint.last_integrated = 0
    #csl controller
    vel = gi * angle_diff
    sum = vel + bodyJoint.last_integrated
    bodyJoint.last_integrated = gf * sum
    return (sum * gain) + gb

  Bounce: (vs, angle_diff, bodyJoint) =>
    #turn around on stall
    if Math.abs(bodyJoint.motor_torque) > 0.9
      bodyJoint.bounce_sign = bodyJoint.bounce_sign * -1
      bodyJoint.last_integrated = 0

    bodyJoint.last_integrated += 35*(angle_diff-(vs*bodyJoint.bounce_sign))
    return bodyJoint.last_integrated

  updateController: (bodyJoint) =>
    bodyJoint.angle_diff_csl = bodyJoint.GetJointAngle() - bodyJoint.last_angle
    bodyJoint.last_angle = bodyJoint.GetJointAngle()

    if bodyJoint.csl_active
      bodyJoint.motor_control = @CSL(
        bodyJoint.gi, bodyJoint.gf, bodyJoint.gb,
        bodyJoint.angle_diff_csl,
        bodyJoint.gain,
        bodyJoint
      )
    else if bodyJoint.bounce_active
      bodyJoint.motor_control = @Bounce(bodyJoint.bounce_vel, bodyJoint.angle_diff_csl, bodyJoint)

  ##### motor calculation #####

  #motor constants
  km = 1.393        #(10.7 / 1000) * 194.57 * 0.625   #torque constant, km_RE-MAX = 0.0107
  kb = 2.563        #back emf, RE-MAX 889/(60*2*%pi*180) * 194.57 = 2.549
  R = 9.59        #R_RE-MAX = 8.3 ohm, R_65 = R_25 * (1+0.0039*(65-25)) (assume mean temp of 65°)
  R_inv = 1/R
  U_in = 0
  updateMotor: (bodyJoint) =>
    # motor model
    U_in = bodyJoint.motor_control
    bodyJoint.I_t = @clip(U_in - (kb*(-bodyJoint.GetJointSpeed())), 12)*(R_inv)   #limit to max battery voltage
    bodyJoint.motor_torque = km * bodyJoint.I_t
    bodyJoint.m_applyTorque += bodyJoint.motor_torque #* bodyJoint.csl_sign
    return

  ##### friction calculation #####
  
  clip: (value, cap=1) => Math.max(-cap, Math.min(cap, value))
  #sgn: (value) => if value > 0 then 1 else if value < 0 then -1 else 0
  v = fg = 0
  applyFriction: (bodyJoint) =>
    v = -bodyJoint.GetJointSpeed()
    #fluid/gliding friction
    fg = -v * @beta * 5

    #dry friction (box2d joint motor provides dry and sticky already)
    #fd = @sgn(-v) * (@beta)

    bodyJoint.m_applyTorque += fg #+ fd

  ##### continuous csl mode calculation experiment

  calcMode: (motor_torque, angle_speed) =>
    mc = if @w0_abs then Math.abs(motor_torque) else motor_torque
    as = if @w1_abs then Math.abs(angle_speed) else angle_speed
    mode = @w0 * mc + @w1 * as + @w2

  updateMode: (bodyJoint) =>
    #calc mode from current pendulum state

    #w0 = 70  #1.4 angle
    #w1 = 10 #-0.2 angle_speed
    #w2 = -0.2   #-1 bias
    #mode = @w0 * Math.abs(@angle) + @w1 * @angle_speed + @w2
  
    mode = @calcMode(bodyJoint.motor_torque, bodyJoint.angle_diff_csl)
    #mode = @clip(mode, 1.3)
    mode = @clip(mode, 3)
    @ui.map_mode bodyJoint, mode

  ###
  #try to play recorded data from a real semni as polygons in the background
  deltaPassed = Treal[0][14]
  j = 0
  s = null
  c = new b2Color(0.3,0.3,0.5)
  tracePlayer: =>
    if not p?
      px = 0.7
      py = 0.7
    if j < Treal.length
      deltaPassed -= (1/60)*1000
      if deltaPassed <= 0
        j++
        deltaPassed = 100

      #set shadow semni to positions of next trace frame        
      f = @body.GetFixtureList()
      while f
        bodyA = Math.PI+Math.atan2(Treal[j][7], Treal[j][6])
        s = f.GetShape()
        if s.m_type == b2Shape.e_polygonShape
          s.m_centroid.x = 0
          s.m_centroid.y = 0
        xf = new b2Transform()
        xf.position = new b2Vec2(px,py)
        xf.R.Set(bodyA)
        @world.DrawShape(s, xf, c)
        f = f.m_next
      f = @body2.GetFixtureList()
      while f
        s = f.GetShape()
        xf = new b2Transform()
        xf.position = new b2Vec2(px,py)
        xf.R.Set(bodyA + 2*Math.PI*Treal[j][10])
        @world.DrawShape(s, xf, c)
        f = f.m_next
      f = @body3.GetFixtureList()
      while f
        s = f.GetShape()
        xf = new b2Transform()
        xf.position = new b2Vec2(px,py)
        xf.R.Set(bodyA + 2*Math.PI*Treal[j][11])
        @world.DrawShape(s, xf, c)
        f = f.m_next
    ###

  ##### update simulation and display loop #####
  was_static = false
  update: =>
    ## update physics and display stuff (~60 Hz loop, depends on refresh rate)
    window.stats.begin()

    if (@run or @step) and @pend_style
      @step = false

      if isMouseDown and (not mouseJoint)
        body = window.getBodyAtMouse()
        if body
          md = new b2MouseJointDef()
          md.bodyA = @world.GetGroundBody()
          if body.GetType() is b2Body.b2_staticBody
            body.SetType b2Body.b2_dynamicBody
            was_static = true
          md.bodyB = body
          md.target.Set mouseX, mouseY
          md.collideConnected = false
          md.maxForce = 200.0 * body.GetMass()
          md.dampingRatio = 2
          md.frequencyHz = 20
          window.mouseJoint = @world.CreateJoint(md)
          body.SetAwake true
      if mouseJoint
        if isMouseDown
         mouseJoint.SetTarget new b2Vec2(mouseX, mouseY)
        else
          if was_static
            mouseJoint.m_bodyB.SetType b2Body.b2_staticBody
            was_static = false
            mouseJoint.m_bodyB.SetAwake false
          @world.DestroyJoint mouseJoint
          window.mouseJoint = null

      # recalc slow stuff, ~60 Hz
      # TODO: move this into ui or separate classes (mapping to mode and other
      # logic is not part of physics)
      if @pend_style is 1
        if @map_state_to_mode
          @updateMode @lower_joint
      else if @pend_style is 2
        if @map_state_to_mode
          @updateMode @lower_joint
          @updateMode @upper_joint
      else if @pend_style is 3   #semni
        @abc.update @body, @upper_joint, @lower_joint

      @logData()

      #recalc quick stuff, 60 Hz * 16 = 960 Hz loop
      i = steps_per_frame
      while i > 0
        if @pend_style is 3  #semni
          @updateController @upper_joint
          @updateController @lower_joint
          @updateMotor @upper_joint
          @updateMotor @lower_joint
          @applyFriction @upper_joint
          @applyFriction @lower_joint
        else if @pend_style is 1
          @updateController @lower_joint
          @updateMotor @lower_joint
          @applyFriction @lower_joint
        else if @pend_style is 2
          @updateController @lower_joint
          @updateController @upper_joint
          @updateMotor @lower_joint
          @updateMotor @upper_joint
          @applyFriction @lower_joint
          @applyFriction @upper_joint

        @world.Step(
          dt,              #timestep (advance timestep ms further in this step)
          10,              #velocity iterations
          10               #position iterations
        )
        i--

      @world.ClearForces()
      @ui.update()

      #@tracePlayer()

      draw_phase_space()
      draw_motor_torque()

    requestAnimFrame @update
    window.stats.end()


