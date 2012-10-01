# vim: set expandtab ts=2 sw=2:

dt = 1/960  #update frequency in hz
#we want a ~1000hz simulation but only have <= 60 fps from the browser
#so we need to call step 16 times with 60*16 = 960 updates/s
steps_per_frame = 16

b2Vec2 = Box2D.Common.Math.b2Vec2
b2AABB = Box2D.Collision.b2AABB
b2BodyDef = Box2D.Dynamics.b2BodyDef
b2Body = Box2D.Dynamics.b2Body
b2FixtureDef = Box2D.Dynamics.b2FixtureDef
b2Fixture = Box2D.Dynamics.b2Fixture
b2World = Box2D.Dynamics.b2World
b2MassData = Box2D.Collision.Shapes.b2MassData
b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef
b2DebugDraw = Box2D.Dynamics.b2DebugDraw

map_state_to_mode = false
w0 = 0
w1 = 0
w2 = 0
w0_abs = false
w1_abs = false

gamma = 0   #friction coefficient
beta = 0  #stiction coefficient
alpha = 0 #striction velocity threshold

mouseX = undefined
mouseY = undefined
mousePVec = undefined
isMouseDown = undefined
selectedBody = undefined
mouseJoint = undefined

window.requestAnimFrame = (->
  window.oRequestAnimationFrame or
  window.webkitRequestAnimationFrame or
  window.mozRequestAnimationFrame or
  window.oRequestAnimationFrame or
  window.msRequestAnimationFrame or
  (callback, element) ->
    window.setTimeout callback, 1000 / 60
)()

set_preset = (w0, w1, w2, w0_abs, w1_abs) ->
  $("#w0").val(w0).trigger("change")
  $("#w1").val(w1).trigger("change")
  $("#w2").val(w2).trigger("change")

  if w0_abs == true
    $("#w0_abs").attr("checked", "checked").trigger("change")
  else
    $("#w0_abs").attr("checked", null).trigger("change")

  if w1_abs == true
    $("#w1_abs").attr("checked", "checked").trigger("change")
  else
    $("#w1_abs").attr("checked", null).trigger("change")

  draw_state_to_mode_mapping()

map_mode_to_gi = (mode) ->
  #double pendulum
  #if mode < 0
  #  mode * 3
  #else
  #  mode * 0.7

  #single pendulum
  if mode < 0
    mode * 3
  else
    1.1 + (0.001 * mode)

map_mode_to_gf = (mode) ->
  #double pendulum
  #if mode > 1
  #  mode * 0.01 + 1
  #else if mode < 0
  #  0
  #else
  #  mode
  
  #single pendulum
  if mode > 1
    mode * 0.0006 + 1
  else if mode < 0
    0
  else
    mode


map_mode = (bodyJoint, mode, joint=bodyJoint.joint_name) ->
  #re-calc gi and gf from joints mode parameter and save in bodyJoint
  #bodyJoint should have a useful joint_name set (lower, upper, depends on dom elements)

  #support:      gi < 0, gf = 0
  #release:      gi > 0, 0 <= gf < 1
  #hold:         gi > 0, gf = 1
  #contraction:  gi > 0, gf > 1
  
  if not mode?
    mode = parseFloat($("#mode_param_#{joint}").val())

  gi = map_mode_to_gi(mode)
  gf = map_mode_to_gf(mode)

  $("#gi_param_#{joint}").val gi
  $("#gf_param_#{joint}").val gf
  $("#mode_param_#{joint}").val mode
  $("#mode_val_#{joint}").html mode.toFixed(2)

  if gi < 0 and gf is 0
    $("#csl_mode_name_#{joint}").html "support"
  else if gi > 0 and 0 <= gf < 1
    $("#csl_mode_name_#{joint}").html "release"
  else if gi > 0 and gf is 1
    $("#csl_mode_name_#{joint}").html "hold"
  else if gi > 0 and gf > 1
    $("#csl_mode_name_#{joint}").html "contraction"
  else
    $("#csl_mode_name_#{joint}").html "?"

  if bodyJoint
    bodyJoint.gi = gi
    bodyJoint.gf = gf


set_friction = (newBeta) ->
  beta = newBeta
  $("#friction_val").html beta.toFixed(3)
  if physics.pend_style is 3
    physics.upper_joint.m_maxMotorTorque = beta
    physics.lower_joint.m_maxMotorTorque = beta
  if physics.pend_style is 4
    physics.lower_joint.m_maxMotorTorque = beta

set_stiction = (newAlpha) ->
  alpha = newAlpha
  $("#stiction_val").html alpha.toFixed(3)
  
set_stiction_vel = (newGamma) ->
  gamma = newGamma
  $("#stiction_epsilon_val").html gamma.toFixed(3)

myon_precision = (number) ->
  Math.floor(number * 10000) / 10000

class physics
  constructor: ->
    ###
    #                               dens    frict  rest    
     Material DEFAULT= new Material(1.00f,  0.30f, 0.1f,  false, true,  true,  Color.GRAY);
     Material METAL  = new Material(7.85f,  0.20f, 0.2f,  false, false, false, Color.LIGHT_GRAY); // Heavy, inert.
     Material STONE  = new Material(2.40f,  0.50f, 0.1f,  false, false, false, Color.DARK_GRAY); // Heavy, inert.
     Material WOOD   = new Material(0.53f,  0.40f, 0.15f, false, true,  false, new Color(150, 98, 0)); // Medium weight, mostly inert.
     Material GLASS  = new Material(2.50f,  0.10f, 0.2f,  false, true,  true,  new Color(0, 0, 220, 128)); // Heavy, transparent.
     Material RUBBER = new Material(1.50f,  0.80f, 0.4f,  false, false, false, new Color(20, 20, 20)); // Medium weight, inert, bouncy.
     Material ICE    = new Material(0.92f,  0.01f, 0.1f,  false, true,  true,  new Color(0, 146, 220, 200)); // Medium weight, slippery surface.
    ###
   
    @world = new b2World(
      new b2Vec2(0, 9.81),    #gravity
      false                  #allow sleep
    )
      
    fixDef = new b2FixtureDef
    fixDef.density = 10
    fixDef.friction = 0.9
    fixDef.restitution = 0.01
    @fixDef = fixDef
    
    #create ground
    @ground_height = 0.03
    @ground_width = 1
    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_staticBody
    bodyDef.position.x = 1
    bodyDef.position.y = 1.1
    bodyDef.linearDamping = 30
    
    fixDef.shape = new b2PolygonShape
    fixDef.shape.SetAsBox @ground_width, @ground_height

    @ground = @world.CreateBody(bodyDef)
    @ground_bodyDef = bodyDef
    @ground.CreateFixture fixDef

    #setup debug draw
    debugDraw = new b2DebugDraw()
    debugDraw.SetSprite document.getElementById("canvas").getContext("2d")
    @drawScale = 300
    debugDraw.SetDrawScale @drawScale
    debugDraw.SetFillAlpha 0.5
    debugDraw.SetLineThickness 1.0
    debugDraw.SetFlags b2DebugDraw.e_shapeBit
    @world.SetDebugDraw debugDraw
    @run = true
    @step = false
    @pend_style = 0
  
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

  lower_joint: null
  createSimplePendulum: =>
    #create pendulum line
    pend_length = 0.5
    mass_size = 0.05
    damping = 0   #custom friction is used instead (further down)

    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_dynamicBody
    @fixDef.density = 25
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
    
    #initialise friction and motor state
    @body.z2 = 0
    @body.motor_control = 0
    @body.I_tm1 = 0
    @body.U_csl = 0

    #create mass circle
    @fixDef.shape = new b2CircleShape(mass_size)
    @fixDef.shape.m_p = pend_vertices[1]
    mass = @body.CreateFixture(@fixDef)
    
    #rotating joint
    jointDef = new b2RevoluteJointDef()
    jointDef.Initialize @body, @ground, pend_vertices[0]
    jointDef.collideConnected = true
    @lower_joint = @world.CreateJoint(jointDef)

    #initialize
    @lower_joint.angle_speed = 0
    @lower_joint.csl_active = false
    @lower_joint.joint_name = 'lower'
    @lower_joint.csl_sign = 1
    @lower_joint.gain = 1
    @lower_joint.gb = 0

  upper_joint: null
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
    
    #initialise friction and motor state
    @body.z2 = 0
    @body.motor_control = 0
    @body.I_tm1 = 0
    @body.U_csl = 0
    
    #lower rotating joint
    jointDef = new b2RevoluteJointDef()
    jointDef.Initialize @body, @ground, pend_vertices[0]
    jointDef.collideConnected = true
    @lower_joint = @world.CreateJoint(jointDef)

    #initialize
    @lower_joint.angle_speed = 0
    @lower_joint.csl_active = false
    @lower_joint.joint_name = 'lower'
    @lower_joint.csl_sign = 1
    @lower_joint.gain = 1
    @lower_joint.gb = 0
    
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
    @body2.motor_control = 0

    #upper rotating joint
    jointDef = new b2RevoluteJointDef()
    jointDef.Initialize @body2, @body, pend_vertices[0]
    jointDef.collideConnected = false
    @upper_joint = @world.CreateJoint(jointDef)
    
    @upper_joint.angle_speed = 0
    @upper_joint.csl_active = false
    @upper_joint.joint_name = 'upper'
    @upper_joint.csl_sign = 1
    @upper_joint.gain = 1
    @upper_joint.gb = 0

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

  upper_joint: null
  createSemni: =>
    #semni overall weight: 432 g
    #body: 120 g
    #arm at body: 135 g
    #second arm: 177 g 

    #create body
    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_dynamicBody
    x0 = @ground_bodyDef.position.x
    y0 = @ground_bodyDef.position.y - 0.2
    bodyDef.position.Set(x0,y0)
    @body = @world.CreateBody(bodyDef)

    @fixDef = new b2FixtureDef
    @fixDef.density = 2.8
    @fixDef.friction = 0.6
    @fixDef.restitution = 0.1
    @fixDef.filter.groupIndex = -1  #negative groups never collide with each other
    @fixDef.shape = new b2PolygonShape
    #@fixDef.shape.SetAsArray contour, contour.length
    for fixture in contour
      @fixDef.shape.SetAsArray(fixture, fixture.length)
      @body.CreateFixture(@fixDef)
    #bodyFix = @body.CreateFixture(@fixDef)

    #md = new b2MassData()
    #@body.GetMassData(md)
    #md.mass = 0.030
    #md.center.Set(0.05, 0.01)
    #@body.SetMassData(md)
    
    #initialise friction and motor state
    @body.z2 = 0
    @body.last_motor_control = 0
    @body.motor_control = 0
    @body.I_tm1 = 0
    @body.U_csl = 0

    ###################

    #upper arm (connected to body)
    bodyDef2 = new b2BodyDef
    bodyDef2.type = b2Body.b2_dynamicBody
    #bodyDef2.position.Set(x0, y0)
    @body2 = @world.CreateBody(bodyDef2)

    @fixDef2 = new b2FixtureDef
    @fixDef2.density = 12
    @fixDef2.friction = 0.6
    @fixDef2.restitution = 0.1
    @fixDef2.filter.groupIndex = -1
    @fixDef2.shape = new b2PolygonShape
    for fixture in arm1ContourConvex
      @fixDef2.shape.SetAsArray(fixture, fixture.length)
      @body2.CreateFixture(@fixDef2)
    #@fixDef2.shape.SetAsBox(0.02,0.05)
    #b2Separator.Separate(@body2, @fixDef2, arm2Contour, 1)
    
    md = new b2MassData()
    @body2.GetMassData(md)
    md.I = 0.02
    #md.center.Set()
    @body2.SetMassData(md)
    
    #initialise friction and motor state
    @body2.z2 = 0
    @body2.last_motor_control = 0
    @body2.motor_control = 0
    @body2.I_tm1 = 0
    @body2.U_csl = 0

    #connect body and arm with rotating joint
    jointDef = new b2RevoluteJointDef()
    #jointDef.Initialize @body, @body2, vertices2[0]
    jointDef.bodyA = @body2
    jointDef.bodyB = @body
    jointDef.localAnchorA.Set(672/ptm_ratio, 385/ptm_ratio)  #point on arm that is attached to body
    jointDef.localAnchorB.Set(672/ptm_ratio, 385/ptm_ratio)  #point on the body that arm is attached to 
    #jointDef.anchor = new b2Vec2(vertices[3].x, vertices[3].y)
    jointDef.collideConnected = false

    #simple friction with box2d possiblities (dry)
    jointDef.maxMotorTorque = 0.100
    jointDef.motorSpeed = 0.0
    jointDef.enableMotor = true

    jointDef.upperAngle = 0.157
    jointDef.lowerAngle = -1.487
    jointDef.enableLimit = true
    @upper_joint = @world.CreateJoint(jointDef)
    
    #initialize
    @upper_joint.angle_speed = 0
    @upper_joint.csl_active = false
    @upper_joint.joint_name = 'upper'
    @upper_joint.csl_sign = 1
    @upper_joint.gain = 1
    @upper_joint.gb = 0

    ########

    #lower arm (not at body)
    bodyDef3 = new b2BodyDef
    bodyDef3.type = b2Body.b2_dynamicBody
    #bodyDef3.position.Set(x0, y0 + 0.4)
    @body3 = @world.CreateBody(bodyDef3)
    
    @fixDef3 = new b2FixtureDef
    @fixDef3.density = 30
    @fixDef3.friction = 0.6
    @fixDef3.restitution = 0.2
    @fixDef3.filter.groupIndex = -1
    @fixDef3.shape = new b2PolygonShape
    for fixture in arm2ContourConvex
      @fixDef3.shape.SetAsArray(fixture, fixture.length)
      @body3.CreateFixture(@fixDef3)

    
    md = new b2MassData()
    @body3.GetMassData(md)
    #md.mass = 0.05
    #md.center.x = ...
    md.I = 0.01
    @body3.SetMassData(md)

    #initialise friction and motor state
    @body3.z2 = 0
    @body3.last_motor_control = 0
    @body3.motor_control = 0
    @body3.I_tm1 = 0
    @body3.U_csl = 0
    
    
    #lower rotating joint
    jointDef = new b2RevoluteJointDef()
    jointDef.bodyA = @body3
    jointDef.bodyB = @body2
    jointDef.localAnchorA.Set(430/ptm_ratio, 504/ptm_ratio)
    jointDef.localAnchorB.Set(430/ptm_ratio, 504/ptm_ratio)
    jointDef.collideConnected = false
    jointDef.maxMotorTorque = 0.02
    jointDef.motorSpeed = 0.0
    jointDef.enableMotor = true

    jointDef.upperAngle = 9.27
    jointDef.lowerAngle = 4.57
    jointDef.enableLimit = true
    @lower_joint = @world.CreateJoint(jointDef)
    
    @lower_joint.angle_speed = 0
    @lower_joint.csl_active = false
    @lower_joint.joint_name = 'lower'
    @lower_joint.csl_sign = 1
    @lower_joint.gain = 1
    @lower_joint.gb = 0
    
  createTestBoxes: =>
    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_dynamicBody
    bodyDef.position.Set(0.5, 0.5)
    @body = @world.CreateBody(bodyDef)

    @fixDef = new b2FixtureDef
    @fixDef.density = 1
    @fixDef.friction = 0.5
    @fixDef.restitution = 0.1
    @fixDef.filter.groupIndex = -1
    @fixDef.shape = new b2PolygonShape
    @fixDef.shape.SetAsBox(0.1,0.2)
    @body.CreateFixture(@fixDef)
    #####
    bodyDef = new b2BodyDef
    bodyDef.type = b2Body.b2_dynamicBody
    @body2 = @world.CreateBody(bodyDef)

    @fixDef = new b2FixtureDef
    @fixDef.density = 1
    @fixDef.friction = 0.5
    @fixDef.restitution = 0.1
    @fixDef.filter.groupIndex = -1
    @fixDef.shape = new b2PolygonShape
    @fixDef.shape.SetAsBox(0.1,0.2)
    @body2.CreateFixture(@fixDef)
    #####

    jointDef = new b2RevoluteJointDef()
    jointDef.bodyA = @body2
    jointDef.bodyB = @body
    jointDef.localAnchorA.Set(0.04, 0.09)  #point on arm that is attached to body
    jointDef.localAnchorB.Set(0.04, 0.09)  #point on the body that arm is attached to 
    jointDef.collideConnected = false

    #simple friction with box2d possiblities (dry)
    jointDef.maxMotorTorque = beta #0.100
    jointDef.motorSpeed = 0.0
    jointDef.enableMotor = true

    jointDef.upperAngle = 0.157
    jointDef.lowerAngle = -1.487
    jointDef.enableLimit = true
    @lower_joint = @world.CreateJoint(jointDef)
    
    #initialize
    @lower_joint.angle_speed = 0
    @lower_joint.csl_active = false
    @lower_joint.joint_name = 'lower'
    @lower_joint.csl_sign = 1
    @lower_joint.gain = 1
    @lower_joint.gb = 0

  getNoisyAngle: (bodyJoint) =>
    #sensor noise
    #rand = Math.random() / 1000
    #rand = rand - (rand/2)
    rand = 0
    -bodyJoint.GetJointAngle() * (1+rand)

  toggleCSL: (bodyObject, bodyJoint) =>
    bodyJoint.csl_active = not bodyJoint.csl_active
    if @lower_joint
      $("#set_csl_params_lower").trigger('click')
    if @upper_joint
      $("#set_csl_params_upper").trigger('click')

  CSL: (gi, gf, gb, angle_speed, gain=1, bodyObject) =>
    #csl controller
    vel = gi * angle_speed
    sum = vel + bodyObject.last_integrated
    bodyObject.last_integrated = gf * sum
    if @pend_style is 3
      limit = 2.8
    else
      limit = 3
    return @clip((sum * gain) + gb, limit)

  updateCSL: (bodyObject, bodyJoint) =>
    if not bodyObject.last_integrated? or not bodyJoint.csl_active
      bodyObject.last_integrated = 0
      bodyObject.U_csl = 0
      bodyJoint.last_angle = bodyJoint.GetJointAngle()

    bodyJoint.angle_speed_csl = bodyJoint.GetJointAngle() - bodyJoint.last_angle
    if bodyJoint.csl_active
      bodyObject.U_csl = @CSL(
        bodyJoint.gi, bodyJoint.gf, bodyJoint.gb,
        bodyJoint.angle_speed_csl,
        bodyJoint.gain,
        bodyObject
      )
      draw_phase_space() if not isMouseDown or not mouseJoint
    bodyJoint.last_angle = bodyJoint.GetJointAngle()
    
    #calm down if contraction goes out of bounds
    #if Math.abs(bodyObject.motor_control) > 0.5
    #  bodyJoint.csl_sign = if bodyJoint.csl_sign then 0 else 1
    #  bodyObject.last_integrated = 0
    #else
    #  bodyJoint.csl_sign = 1

    #TODO: detect stable poses
    # 1 compare current to last angle value and set could-be stable flag if we have a small interval
    # 2a keep first value compare all following values to it and count how many steps have passed
    # 2b if any new value is out of the allowed interval, set could-be stable flag to false again
    # 4 if enough steps have passed and and could-be stable flag is still true, set stable flag
    # 5 unset stable and could-be stable flag if any new value is too far away from initial position    
    
  #motor constants
  km = 10.7 * 193 * 0.4 / 1000   #torque constant, km_RX28 = 0.0107, includes ratio * efficiency, M=km*I  
  #kb = km            #back emf constant, kn in maxon Documents
  kb = 5
  L = 0.208 / 1000   #L_RX28 = 0.208 mH; 
  L_inv = 1000       #L_inv = 5.004807, SciCos variant: 4854
  R = 8.3            #R_RX28 = 8.3 ohm
  updateMotor: (bodyObject, bodyJoint) =>
    # motor model
    U_csl = bodyObject.U_csl
    I_t = (U_csl - (kb*(-bodyJoint.angle_speed_csl)))*(1/R)     #U_csl-(R*I_tm1)-(kb*bodyJoint.angle_speed)
    bodyObject.motor_control = km * I_t
    if bodyObject.motor_control
      bodyJoint.m_applyTorque = bodyObject.motor_control #* bodyJoint.csl_sign

  clip: (value, cap=1) => Math.max(-cap, Math.min(cap, value))
  sgn: (value) => if value > 0 then 1 else if value < 0 then -1 else 0
  ### 
  G = 0.0     #gaussian y offset (>0 results in dry friction component)
  gaussian: (v) => Math.min(1, 1.1 * Math.exp(- Math.pow( (v/gamma), 2 ))+G)
  applyFriction: (bodyObject, bodyJoint) =>
    #csl hold friction model, crap because of time dependent behaviour
    v = -bodyObject.GetAngularVelocity()
    u = @clip(v+bodyObject.z2)
    bodyObject.z2 = u * @gaussian(v)

    #sticky_damping = 30*alpha*@gaussian(-v)*v
    t_sticky = alpha * bodyObject.z2 #+ sticky_damping
    t_fluid = beta * v
    
    bodyObject.ApplyTorque(t_fluid + t_sticky)
  ###
  
  
  #close_to_zero: (value) => if Math.abs(value) < 1e-4 then true else false
  applyFriction: (bodyObject, bodyJoint) =>
    #friction model 
    v = -bodyJoint.GetJointSpeed()

    #sticky friction
    #if @close_to_zero(bodyJoint.m_impulse.x)
    #  bodyJoint.m_impulse.x = 0
    #  bodyObject.SetAngularVelocity(0)
    #else
    #fluid/gliding friction
    fg = -v * beta / 2

    #dry friction
    fd = @sgn(-v) * (beta / 10)
    #fd = 0

    if not isMouseDown
      bodyObject.ApplyTorque(fg + fd)

  calcMode: (motor_control, angle_speed) =>
    mc = if w0_abs then Math.abs(motor_control) else motor_control
    as = if w1_abs then Math.abs(angle_speed) else angle_speed
    mode = w0 * mc + w1 * as + w2

  updateMode: (bodyObject, bodyJoint) =>
    #calc mode from current pendulum state

    #w0 = 70  #1.4 angle
    #w1 = 10 #-0.2 angle_speed
    #w2 = -0.2   #-1 bias
    #mode = w0 * Math.abs(@angle) + w1 * @angle_speed + w2
  
    mode = @calcMode(bodyObject.motor_control, bodyJoint.angle_speed_csl)
    #mode = @clip(mode, 1.3)
    mode = @clip(mode, 3)
    map_mode bodyJoint, mode

  #update simulation and display loop
  was_static = false
  update: =>
    ## update physics and display stuff (~60 Hz loop)
    window.stats.begin()

    if (@run or @step) and @pend_style
      @step = false

      if isMouseDown and (not mouseJoint)
        body = @getBodyAtMouse()
        if body
          md = new b2MouseJointDef()
          md.bodyA = @world.GetGroundBody()
          if body.GetType() is b2Body.b2_staticBody
            body.SetType b2Body.b2_dynamicBody
            was_static = true
          md.bodyB = body
          md.target.Set mouseX, mouseY
          md.collideConnected = false
          md.maxForce = 100.0 * body.GetMass()
          mouseJoint = @world.CreateJoint(md)
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
          mouseJoint = null

      # recalc csl, 60 Hz
      if @pend_style is 1
        if map_state_to_mode
          @updateMode @body, @lower_joint

      if @pend_style is 2
        if map_state_to_mode
          @updateMode @body, @lower_joint
          @updateMode @body2, @upper_joint

      #1000 Hz loop
      i = 0
      while i < steps_per_frame
        if @pend_style is 1
          @updateCSL @body, @lower_joint
          @updateMotor @body, @lower_joint
          @applyFriction @body, @lower_joint
        if @pend_style is 2
          @updateCSL @body, @lower_joint
          @updateCSL @body2, @upper_joint
          @updateMotor @body, @lower_joint
          @updateMotor @body2, @upper_joint
          @applyFriction @body, @lower_joint
          @applyFriction @body2, @upper_joint
        if @pend_style is 3
          @updateCSL @body2, @lower_joint
          @updateCSL @body3, @upper_joint
          @updateMotor @body2, @lower_joint
          @updateMotor @body3, @upper_joint
        if @pend_style is 4
          @updateCSL @body, @lower_joint
          @updateMotor @body, @lower_joint

        @world.Step(
          dt,             #timestep (1 / fps)
          10,             #velocity iterations
          10              #position iterations
        )
        i++

      @world.ClearForces()
      @world.DrawDebugData()
      draw_motor_torque()

    requestAnimFrame @update
    window.stats.end()

$ ->
  #get initial friction params
  alpha = $("#stiction_param").val()
  beta = $("#friction_param").val()
  gamma = $("#stiction_epsilon").val()

  #set up map to mode checkbox and get values
  $("#map_state_to_mode").click =>
    map_state_to_mode = !map_state_to_mode
    $('#map_state_to_mode').attr('checked', map_state_to_mode)

  $("#w0").change =>
    $("#w0_val").html("="+$("#w0").val())
    w0 = parseFloat $("#w0").val()
  $("#w1").change =>
    $("#w1_val").html("="+$("#w1").val())
    w1 = parseFloat $("#w1").val()
  $("#w2").change =>
    $("#w2_val").html("="+$("#w2").val())
    w2 = parseFloat $("#w2").val()
  w0 = parseFloat $("#w0").val()
  w1 = parseFloat $("#w1").val()
  w2 = parseFloat $("#w2").val()

  #get wether U and phi should be used as absolute or signed values from checkboxed
  $("#w0_abs").change =>
    w0_abs = ($("#w0_abs").attr("checked") isnt undefined)
    draw_state_to_mode_mapping()
  $("#w1_abs").change =>
    w1_abs = ($("#w1_abs").attr("checked") isnt undefined)
    draw_state_to_mode_mapping()
  #initial values  
  w0_abs = ($("#w0_abs").attr("checked") isnt undefined)
  w1_abs = ($("#w1_abs").attr("checked") isnt undefined)

  physics = new physics()
  #make physics global for external access
  window.physics = physics

  #request first frame from browser to start update cycle
  requestAnimFrame physics.update

  #fps counter
  window.stats = new Stats()
  window.stats.setMode 0
  # align top-left
  window.stats.domElement.style.position = "absolute"
  window.stats.domElement.style.left = "0px"
  window.stats.domElement.style.top = "0px"
  document.body.appendChild window.stats.domElement

  # stuff to handle mouse manipulation
  getElementPosition = (element) ->
    elem = element
    tagname = ""
    x = 0
    y = 0
    while (typeof (elem) is "object") and (typeof (elem.tagName) isnt "undefined")
      y += elem.offsetTop
      x += elem.offsetLeft
      tagname = elem.tagName.toUpperCase()
      elem = 0  if tagname is "BODY"
      elem = elem.offsetParent  if typeof (elem.offsetParent) is "object"  if typeof (elem) is "object"
    x: x
    y: y

  canvasPosition = getElementPosition(document.getElementById("canvas"))
  document.addEventListener "mousedown", ((e) ->
    isMouseDown = true
    handleMouseMove e
    document.addEventListener "mousemove", handleMouseMove, true
  ), true
  document.addEventListener "mouseup", (->
    document.removeEventListener "mousemove", handleMouseMove, true
    isMouseDown = false
    mouseX = `undefined`
    mouseY = `undefined`
  ), true

  handleMouseMove = (e) ->
    mouseX = (e.clientX - canvasPosition.x) / physics.drawScale
    mouseY = (e.clientY - canvasPosition.y) / physics.drawScale

  physics.getBodyAtMouse = ->
    mousePVec = new b2Vec2(mouseX, mouseY)
    aabb = new b2AABB()
    aabb.lowerBound.Set mouseX - 0.001, mouseY - 0.001
    aabb.upperBound.Set mouseX + 0.001, mouseY + 0.001
    
    # Query the world for overlapping shapes.
    selectedBody = null
    physics.world.QueryAABB getBodyCB, aabb
    selectedBody

  getBodyCB = (fixture) ->
    #if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
    if fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)
      selectedBody = fixture.GetBody()
      return false
    #}
    true
