// Generated by CoffeeScript 1.3.3
var alpha, b2AABB, b2Body, b2BodyDef, b2CircleShape, b2DebugDraw, b2Fixture, b2FixtureDef, b2MassData, b2MouseJointDef, b2PolygonShape, b2RevoluteJointDef, b2Vec2, b2World, beta, dt, gamma, isMouseDown, map_mode, map_mode_to_gf, map_mode_to_gi, map_state_to_mode, mouseJoint, mousePVec, mouseX, mouseY, myon_precision, physics, selectedBody, set_friction, set_preset, set_stiction, set_stiction_vel, steps_per_frame, w0, w0_abs, w1, w1_abs, w2,
  __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

dt = 1 / 960;

steps_per_frame = 16;

b2Vec2 = Box2D.Common.Math.b2Vec2;

b2AABB = Box2D.Collision.b2AABB;

b2BodyDef = Box2D.Dynamics.b2BodyDef;

b2Body = Box2D.Dynamics.b2Body;

b2FixtureDef = Box2D.Dynamics.b2FixtureDef;

b2Fixture = Box2D.Dynamics.b2Fixture;

b2World = Box2D.Dynamics.b2World;

b2MassData = Box2D.Collision.Shapes.b2MassData;

b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;

b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;

b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef;

b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef;

b2DebugDraw = Box2D.Dynamics.b2DebugDraw;

map_state_to_mode = false;

w0 = 0;

w1 = 0;

w2 = 0;

w0_abs = false;

w1_abs = false;

gamma = 0;

beta = 0;

alpha = 0;

mouseX = void 0;

mouseY = void 0;

mousePVec = void 0;

isMouseDown = void 0;

selectedBody = void 0;

mouseJoint = void 0;

window.requestAnimFrame = (function() {
  return window.oRequestAnimationFrame || window.webkitRequestAnimationFrame || window.mozRequestAnimationFrame || window.oRequestAnimationFrame || window.msRequestAnimationFrame || function(callback, element) {
    return window.setTimeout(callback, 1000 / 60);
  };
})();

set_preset = function(w0, w1, w2, w0_abs, w1_abs) {
  $("#w0").val(w0).trigger("change");
  $("#w1").val(w1).trigger("change");
  $("#w2").val(w2).trigger("change");
  if (w0_abs === true) {
    $("#w0_abs").attr("checked", "checked").trigger("change");
  } else {
    $("#w0_abs").attr("checked", null).trigger("change");
  }
  if (w1_abs === true) {
    $("#w1_abs").attr("checked", "checked").trigger("change");
  } else {
    $("#w1_abs").attr("checked", null).trigger("change");
  }
  return draw_state_to_mode_mapping();
};

map_mode_to_gi = function(mode) {
  if (mode < 0) {
    return mode * 3;
  } else {
    return mode * 0.35;
  }
};

map_mode_to_gf = function(mode) {
  if (mode > 1) {
    return mode * 0.03 + 1;
  } else if (mode < 0) {
    return 0;
  } else {
    return mode;
  }
};

map_mode = function(bodyJoint, mode, joint) {
  var gf, gi;
  if (joint == null) {
    joint = bodyJoint.joint_name;
  }
  if (!(mode != null)) {
    mode = parseFloat($("#mode_param_" + joint).val());
  }
  gi = map_mode_to_gi(mode);
  gf = map_mode_to_gf(mode);
  $("#gi_param_" + joint).val(gi);
  $("#gf_param_" + joint).val(gf);
  $("#mode_param_" + joint).val(mode);
  $("#mode_val_" + joint).html(mode.toFixed(2));
  if (gi < 0 && gf === 0) {
    $("#csl_mode_name_" + joint).html("support");
  } else if (gi > 0 && (0 <= gf && gf < 1)) {
    $("#csl_mode_name_" + joint).html("release");
  } else if (gi > 0 && gf === 1) {
    $("#csl_mode_name_" + joint).html("hold");
  } else if (gi > 0 && gf > 1) {
    $("#csl_mode_name_" + joint).html("contraction");
  } else {
    $("#csl_mode_name_" + joint).html("?");
  }
  if (bodyJoint) {
    bodyJoint.gi = gi;
    return bodyJoint.gf = gf;
  }
};

set_friction = function(newBeta) {
  beta = newBeta;
  return $("#friction_val").html(beta.toFixed(3));
};

set_stiction = function(newAlpha) {
  alpha = newAlpha;
  return $("#stiction_val").html(alpha.toFixed(3));
};

set_stiction_vel = function(newGamma) {
  gamma = newGamma;
  return $("#stiction_epsilon_val").html(gamma.toFixed(3));
};

myon_precision = function(number) {
  return Math.floor(number * 10000) / 10000;
};

physics = (function() {
  var L, L_inv, R, kb, km, was_static;

  function physics() {
    this.update = __bind(this.update, this);

    this.updateMode = __bind(this.updateMode, this);

    this.calcMode = __bind(this.calcMode, this);

    this.applyFriction = __bind(this.applyFriction, this);

    this.close_to_zero = __bind(this.close_to_zero, this);

    this.sgn = __bind(this.sgn, this);

    this.clip = __bind(this.clip, this);

    this.updateMotor = __bind(this.updateMotor, this);

    this.updateCSL = __bind(this.updateCSL, this);

    this.CSL = __bind(this.CSL, this);

    this.toggleCSL = __bind(this.toggleCSL, this);

    this.getCurrentAngle = __bind(this.getCurrentAngle, this);

    this.createSemni = __bind(this.createSemni, this);

    this.ellipse2polygon = __bind(this.ellipse2polygon, this);

    this.createDoublePendulum = __bind(this.createDoublePendulum, this);

    this.createSimplePendulum = __bind(this.createSimplePendulum, this);

    this.createBox = __bind(this.createBox, this);

    /*
        #                               dens    frict  rest    
         Material DEFAULT= new Material(1.00f,  0.30f, 0.1f,  false, true,  true,  Color.GRAY);
         Material METAL  = new Material(7.85f,  0.20f, 0.2f,  false, false, false, Color.LIGHT_GRAY); // Heavy, inert.
         Material STONE  = new Material(2.40f,  0.50f, 0.1f,  false, false, false, Color.DARK_GRAY); // Heavy, inert.
         Material WOOD   = new Material(0.53f,  0.40f, 0.15f, false, true,  false, new Color(150, 98, 0)); // Medium weight, mostly inert.
         Material GLASS  = new Material(2.50f,  0.10f, 0.2f,  false, true,  true,  new Color(0, 0, 220, 128)); // Heavy, transparent.
         Material RUBBER = new Material(1.50f,  0.80f, 0.4f,  false, false, false, new Color(20, 20, 20)); // Medium weight, inert, bouncy.
         Material ICE    = new Material(0.92f,  0.01f, 0.1f,  false, true,  true,  new Color(0, 146, 220, 200)); // Medium weight, slippery surface.
    */

    var bodyDef, debugDraw, fixDef;
    this.world = new b2World(new b2Vec2(0, 9.81), false);
    fixDef = new b2FixtureDef;
    fixDef.density = 0.53;
    fixDef.friction = 0.4;
    fixDef.restitution = 0.15;
    this.fixDef = fixDef;
    this.ground_height = 0.03;
    this.ground_width = 1;
    bodyDef = new b2BodyDef;
    bodyDef.type = b2Body.b2_staticBody;
    bodyDef.position.x = 1;
    bodyDef.position.y = 1.1;
    bodyDef.linearDamping = 30;
    fixDef.shape = new b2PolygonShape;
    fixDef.shape.SetAsBox(this.ground_width, this.ground_height);
    this.ground = this.world.CreateBody(bodyDef);
    this.ground_bodyDef = bodyDef;
    this.ground.CreateFixture(fixDef);
    debugDraw = new b2DebugDraw();
    debugDraw.SetSprite(document.getElementById("canvas").getContext("2d"));
    this.drawScale = 300;
    debugDraw.SetDrawScale(this.drawScale);
    debugDraw.SetFillAlpha(0.5);
    debugDraw.SetLineThickness(1.0);
    debugDraw.SetFlags(b2DebugDraw.e_shapeBit);
    this.world.SetDebugDraw(debugDraw);
    this.run = true;
    this.step = false;
    this.pend_style = 0;
  }

  physics.prototype.createBox = function() {
    var bodyDef, box, fixDef;
    fixDef = new b2FixtureDef;
    fixDef.density = 1;
    fixDef.friction = 0.3;
    fixDef.restitution = 0.2;
    this.fixDef = fixDef;
    fixDef.shape = new b2PolygonShape;
    fixDef.shape.SetAsBox(0.1, 0.1);
    bodyDef = new b2BodyDef;
    bodyDef.type = b2Body.b2_staticBody;
    bodyDef.position.Set(1.3, 0.8);
    box = this.world.CreateBody(bodyDef);
    return box.CreateFixture(fixDef);
  };

  physics.prototype.lower_joint = null;

  physics.prototype.createSimplePendulum = function() {
    var bodyDef, damping, jointDef, line, mass, mass_size, pend_length, pend_vertices;
    pend_length = 0.5;
    mass_size = 0.05;
    damping = 0;
    bodyDef = new b2BodyDef;
    bodyDef.type = b2Body.b2_dynamicBody;
    this.fixDef.density = 25;
    this.fixDef.shape = new b2PolygonShape;
    pend_vertices = new Array(new b2Vec2(this.ground_bodyDef.position.x, this.ground_bodyDef.position.y - this.ground_height - 0.005), new b2Vec2(this.ground_bodyDef.position.x, this.ground_bodyDef.position.y - this.ground_height - pend_length));
    this.fixDef.shape.SetAsArray(pend_vertices, 2);
    bodyDef.linearDamping = damping;
    bodyDef.angularDamping = damping;
    this.body = this.world.CreateBody(bodyDef);
    line = this.body.CreateFixture(this.fixDef);
    this.body.z2 = 0;
    this.body.motor_control = 0;
    this.body.I_tm1 = 0;
    this.body.U_csl = 0;
    this.fixDef.shape = new b2CircleShape(mass_size);
    this.fixDef.shape.m_p = pend_vertices[1];
    mass = this.body.CreateFixture(this.fixDef);
    jointDef = new b2RevoluteJointDef();
    jointDef.Initialize(this.body, this.ground, pend_vertices[0]);
    jointDef.collideConnected = true;
    this.lower_joint = this.world.CreateJoint(jointDef);
    this.lower_joint.angle_speed = 0;
    this.lower_joint.csl_active = false;
    this.lower_joint.joint_name = 'lower';
    this.lower_joint.csl_sign = 1;
    this.lower_joint.gain = 1;
    return this.lower_joint.gb = 0;
  };

  physics.prototype.upper_joint = null;

  physics.prototype.createDoublePendulum = function() {
    var bodyDef, damping, jointDef, line, line2, mass, mass_size, pend_length, pend_vertices;
    pend_length = 0.5;
    mass_size = 0.04;
    damping = 0;
    bodyDef = new b2BodyDef;
    bodyDef.type = b2Body.b2_dynamicBody;
    this.fixDef.density = 10;
    this.fixDef.shape = new b2PolygonShape;
    pend_vertices = new Array(new b2Vec2(this.ground_bodyDef.position.x, this.ground_bodyDef.position.y - this.ground_height - 0.005), new b2Vec2(this.ground_bodyDef.position.x, this.ground_bodyDef.position.y - this.ground_height - pend_length));
    this.fixDef.shape.SetAsArray(pend_vertices, 2);
    bodyDef.linearDamping = damping;
    bodyDef.angularDamping = damping;
    this.body = this.world.CreateBody(bodyDef);
    line = this.body.CreateFixture(this.fixDef);
    this.body.z2 = 0;
    this.body.motor_control = 0;
    this.body.I_tm1 = 0;
    this.body.U_csl = 0;
    jointDef = new b2RevoluteJointDef();
    jointDef.Initialize(this.body, this.ground, pend_vertices[0]);
    jointDef.collideConnected = true;
    this.lower_joint = this.world.CreateJoint(jointDef);
    this.lower_joint.angle_speed = 0;
    this.lower_joint.csl_active = false;
    this.lower_joint.joint_name = 'lower';
    this.lower_joint.csl_sign = 1;
    this.lower_joint.gain = 1;
    this.lower_joint.gb = 0;
    this.fixDef.shape = new b2CircleShape(mass_size);
    this.fixDef.shape.m_p = pend_vertices[1];
    mass = this.body.CreateFixture(this.fixDef);
    pend_vertices = new Array(new b2Vec2(this.ground_bodyDef.position.x, this.ground_bodyDef.position.y - this.ground_height - pend_length - 0.005), new b2Vec2(this.ground_bodyDef.position.x, this.ground_bodyDef.position.y - this.ground_height - (2 * pend_length)));
    this.fixDef.shape = new b2PolygonShape;
    this.fixDef.shape.SetAsArray(pend_vertices, 2);
    bodyDef.linearDamping = damping;
    bodyDef.angularDamping = damping;
    this.body2 = this.world.CreateBody(bodyDef);
    line2 = this.body2.CreateFixture(this.fixDef);
    this.body2.z2 = 0;
    this.body2.motor_control = 0;
    jointDef = new b2RevoluteJointDef();
    jointDef.Initialize(this.body2, this.body, pend_vertices[0]);
    jointDef.collideConnected = false;
    this.upper_joint = this.world.CreateJoint(jointDef);
    this.upper_joint.angle_speed = 0;
    this.upper_joint.csl_active = false;
    this.upper_joint.joint_name = 'upper';
    this.upper_joint.csl_sign = 1;
    this.upper_joint.gain = 1;
    this.upper_joint.gb = 0;
    this.fixDef.shape = new b2CircleShape(mass_size);
    this.fixDef.shape.m_p = pend_vertices[1];
    return mass = this.body2.CreateFixture(this.fixDef);
  };

  physics.prototype.ellipse2polygon = function(r, a, x0, y0) {
    var points, step, theta, x, y, _i, _ref;
    points = new Array();
    step = 2 * Math.PI / 40;
    for (theta = _i = _ref = 2 * Math.PI; _ref <= 0 ? _i <= 0 : _i >= 0; theta = _i += -step) {
      x = x0 + r * Math.cos(theta);
      y = y0 - a * r * Math.sin(theta);
      points.push(new b2Vec2(x, y));
    }
    return points.slice(0, points.length - 1);
  };

  physics.prototype.upper_joint = null;

  physics.prototype.createSemni = function() {
    var a, arm_length, bodyDef, bodyFix, jointDef, md, r, upper_arm, v, vertices, vertices2, x0, y0;
    r = 0.2;
    a = 1.5;
    x0 = this.ground_bodyDef.position.x;
    y0 = this.ground_bodyDef.position.y - 2 * r;
    vertices = this.ellipse2polygon(r, a, x0, y0);
    this.fixDef = new b2FixtureDef;
    this.fixDef.density = 2;
    this.fixDef.friction = 0.4;
    this.fixDef.restitution = 0.2;
    this.fixDef.shape = new b2PolygonShape;
    this.fixDef.shape.SetAsArray(vertices, vertices.length);
    bodyDef = new b2BodyDef;
    bodyDef.type = b2Body.b2_dynamicBody;
    this.body = this.world.CreateBody(bodyDef);
    bodyFix = this.body.CreateFixture(this.fixDef);
    this.body.z2 = 0;
    this.body.motor_control = 0;
    this.body.I_tm1 = 0;
    this.body.U_csl = 0;
    arm_length = 0.1;
    this.fixDef = new b2FixtureDef;
    this.fixDef.density = 1;
    this.fixDef.friction = 0.4;
    this.fixDef.restitution = 0.2;
    this.fixDef.shape = new b2PolygonShape;
    v = vertices[3];
    vertices2 = new Array(v, new b2Vec2(v.x + arm_length, v.y), new b2Vec2(v.x + arm_length, v.y - 0.02), new b2Vec2(v.x, v.y - 0.02));
    this.fixDef.shape.SetAsArray(vertices, vertices2.length);
    bodyDef = new b2BodyDef;
    bodyDef.type = b2Body.b2_dynamicBody;
    this.body2 = this.world.CreateBody(bodyDef);
    upper_arm = this.body2.CreateFixture(this.fixDef);
    md = new b2MassData();
    this.body2.GetMassData(md);
    md.mass = 0.05;
    this.body2.SetMassData(md);
    jointDef = new b2RevoluteJointDef();
    jointDef.Initialize(this.body, this.body2, vertices[0]);
    jointDef.collideConnected = false;
    this.lower_joint = this.world.CreateJoint(jointDef);
    this.lower_joint.angle_speed = 0;
    this.lower_joint.csl_active = false;
    this.lower_joint.joint_name = 'lower';
    this.lower_joint.csl_sign = 1;
    this.lower_joint.gain = 1;
    return this.lower_joint.gb = 0;
    /*
        #upper rotating joint
        jointDef = new b2RevoluteJointDef()
        jointDef.Initialize @body2, @body3, vertices[0]
        jointDef.collideConnected = false
        @upper_joint = @world.CreateJoint(jointDef)
        
        @upper_joint.angle_speed = 0
        @upper_joint.csl_active = false
        @upper_joint.joint_name = 'upper'
        @upper_joint.csl_sign = 1
        @upper_joint.gain = 1
        @upper_joint.gb = 0
    */

  };

  physics.prototype.getCurrentAngle = function(bodyJoint) {
    var rand;
    rand = 0;
    return -bodyJoint.GetJointAngle() * (1 + rand);
  };

  physics.prototype.toggleCSL = function(bodyObject, bodyJoint) {
    bodyJoint.csl_active = !bodyJoint.csl_active;
    bodyObject.last_integrated = 0;
    return bodyObject.U_csl = 0;
  };

  physics.prototype.CSL = function(gi, gf, gb, angle_speed, gain, bodyObject) {
    var sum, vel;
    if (gain == null) {
      gain = 1;
    }
    vel = gi * angle_speed;
    sum = vel + bodyObject.last_integrated;
    bodyObject.last_integrated = gf * sum;
    return (sum * gain) + gb;
  };

  physics.prototype.updateCSL = function(bodyObject, bodyJoint) {
    bodyJoint.angle_speed_csl = bodyJoint.GetJointAngle() - bodyJoint.last_angle;
    if (bodyJoint.csl_active) {
      bodyObject.U_csl = this.CSL(bodyJoint.gi, bodyJoint.gf, bodyJoint.gb, bodyJoint.angle_speed_csl, bodyJoint.gain, bodyObject);
      if (!isMouseDown || !mouseJoint) {
        draw_phase_space();
      }
    }
    return bodyJoint.last_angle = bodyJoint.GetJointAngle();
  };

  km = 10.7 * 193 * 0.4 / 1000;

  km = 5;

  kb = 20;

  L = 0.208 / 1000;

  L_inv = 1000;

  R = 8.3;

  physics.prototype.updateMotor = function(bodyObject, bodyJoint) {
    var I_t, I_tm1, U_csl, U_t;
    bodyJoint.angle_speed = bodyJoint.GetJointSpeed();
    I_tm1 = bodyObject.I_tm1;
    U_csl = bodyObject.U_csl;
    U_t = U_csl - (R * I_tm1) - (kb * bodyJoint.angle_speed);
    I_t = (U_t * L_inv) + I_tm1;
    bodyObject.motor_control = U_csl;
    bodyObject.I_tm1 = I_t;
    if (bodyObject.motor_control) {
      return bodyObject.ApplyTorque(bodyObject.motor_control);
    }
  };

  physics.prototype.clip = function(value, cap) {
    if (cap == null) {
      cap = 1;
    }
    return Math.max(-cap, Math.min(cap, value));
  };

  physics.prototype.sgn = function(value) {
    if (value > 0) {
      return 1;
    } else if (value < 0) {
      return -1;
    } else {
      return 0;
    }
  };

  /* 
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
  */


  physics.prototype.close_to_zero = function(value) {
    if (Math.abs(value) < 1e-4) {
      return true;
    } else {
      return false;
    }
  };

  physics.prototype.applyFriction = function(bodyObject, bodyJoint) {
    var fd, fg, v;
    v = -bodyJoint.GetJointSpeed();
    fg = -v * beta;
    fd = this.sgn(-v) * (beta / 10);
    if (!isMouseDown) {
      return bodyObject.ApplyTorque(fg + fd);
    }
  };

  physics.prototype.calcMode = function(motor_control, angle_speed) {
    var as, mc, mode;
    mc = w0_abs ? Math.abs(motor_control) : motor_control;
    as = w1_abs ? Math.abs(angle_speed) : angle_speed;
    return mode = w0 * mc + w1 * as + w2;
  };

  physics.prototype.updateMode = function(bodyObject, bodyJoint) {
    var mode;
    mode = this.calcMode(bodyObject.motor_control, bodyJoint.angle_speed_csl);
    mode = this.clip(mode, 3);
    map_mode(bodyJoint, mode);
    if (Math.abs(bodyObject.motor_control) > 0.3) {
      bodyJoint.csl_sign = bodyJoint.csl_sign ? 0 : 1;
      return bodyObject.last_integrated = 0;
    } else {
      return bodyJoint.csl_sign = 1;
    }
  };

  was_static = false;

  physics.prototype.update = function() {
    var body, i, md;
    window.stats.begin();
    if ((this.run || this.step) && this.pend_style) {
      this.step = false;
      if (isMouseDown && (!mouseJoint)) {
        body = this.getBodyAtMouse();
        if (body) {
          md = new b2MouseJointDef();
          md.bodyA = this.world.GetGroundBody();
          if (body.GetType() === b2Body.b2_staticBody) {
            body.SetType(b2Body.b2_dynamicBody);
            was_static = true;
          }
          md.bodyB = body;
          md.target.Set(mouseX, mouseY);
          md.collideConnected = false;
          md.maxForce = 100.0 * body.GetMass();
          mouseJoint = this.world.CreateJoint(md);
          body.SetAwake(true);
        }
      }
      if (mouseJoint) {
        if (isMouseDown) {
          mouseJoint.SetTarget(new b2Vec2(mouseX, mouseY));
        } else {
          if (was_static) {
            mouseJoint.m_bodyB.SetType(b2Body.b2_staticBody);
            was_static = false;
            mouseJoint.m_bodyB.SetAwake(false);
          }
          this.world.DestroyJoint(mouseJoint);
          mouseJoint = null;
        }
      }
      if (this.pend_style === 1) {
        if (map_state_to_mode) {
          this.updateMode(this.body, this.lower_joint);
        }
        this.updateCSL(this.body, this.lower_joint);
      }
      if (this.pend_style === 2) {
        if (map_state_to_mode) {
          this.updateMode(this.body, this.lower_joint);
          this.updateMode(this.body2, this.upper_joint);
        }
        this.updateCSL(this.body, this.lower_joint);
        this.updateCSL(this.body2, this.upper_joint);
      }
      i = 0;
      while (i < steps_per_frame) {
        if (this.pend_style === 1) {
          this.updateMotor(this.body, this.lower_joint);
          this.applyFriction(this.body, this.lower_joint);
        }
        if (this.pend_style === 2) {
          this.updateMotor(this.body, this.lower_joint);
          this.updateMotor(this.body2, this.upper_joint);
          this.applyFriction(this.body, this.lower_joint);
          this.applyFriction(this.body2, this.upper_joint);
        }
        this.world.Step(dt, 10, 10);
        i++;
      }
      this.world.ClearForces();
      this.world.DrawDebugData();
      draw_motor_torque();
    }
    requestAnimFrame(this.update);
    return window.stats.end();
  };

  return physics;

})();

$(function() {
  var canvasPosition, getBodyCB, getElementPosition, handleMouseMove,
    _this = this;
  alpha = $("#stiction_param").val();
  beta = $("#friction_param").val();
  gamma = $("#stiction_epsilon").val();
  $("#map_state_to_mode").click(function() {
    map_state_to_mode = !map_state_to_mode;
    return $('#map_state_to_mode').attr('checked', map_state_to_mode);
  });
  $("#w0").change(function() {
    $("#w0_val").html("=" + $("#w0").val());
    return w0 = parseFloat($("#w0").val());
  });
  $("#w1").change(function() {
    $("#w1_val").html("=" + $("#w1").val());
    return w1 = parseFloat($("#w1").val());
  });
  $("#w2").change(function() {
    $("#w2_val").html("=" + $("#w2").val());
    return w2 = parseFloat($("#w2").val());
  });
  w0 = parseFloat($("#w0").val());
  w1 = parseFloat($("#w1").val());
  w2 = parseFloat($("#w2").val());
  $("#w0_abs").change(function() {
    w0_abs = $("#w0_abs").attr("checked") !== void 0;
    return draw_state_to_mode_mapping();
  });
  $("#w1_abs").change(function() {
    w1_abs = $("#w1_abs").attr("checked") !== void 0;
    return draw_state_to_mode_mapping();
  });
  w0_abs = $("#w0_abs").attr("checked") !== void 0;
  w1_abs = $("#w1_abs").attr("checked") !== void 0;
  physics = new physics();
  window.physics = physics;
  requestAnimFrame(physics.update);
  window.stats = new Stats();
  window.stats.setMode(0);
  window.stats.domElement.style.position = "absolute";
  window.stats.domElement.style.left = "0px";
  window.stats.domElement.style.top = "0px";
  document.body.appendChild(window.stats.domElement);
  getElementPosition = function(element) {
    var elem, tagname, x, y;
    elem = element;
    tagname = "";
    x = 0;
    y = 0;
    while ((typeof elem === "object") && (typeof elem.tagName !== "undefined")) {
      y += elem.offsetTop;
      x += elem.offsetLeft;
      tagname = elem.tagName.toUpperCase();
      if (tagname === "BODY") {
        elem = 0;
      }
      if (typeof elem === "object" ? typeof elem.offsetParent === "object" : void 0) {
        elem = elem.offsetParent;
      }
    }
    return {
      x: x,
      y: y
    };
  };
  canvasPosition = getElementPosition(document.getElementById("canvas"));
  document.addEventListener("mousedown", (function(e) {
    isMouseDown = true;
    handleMouseMove(e);
    return document.addEventListener("mousemove", handleMouseMove, true);
  }), true);
  document.addEventListener("mouseup", (function() {
    document.removeEventListener("mousemove", handleMouseMove, true);
    isMouseDown = false;
    mouseX = undefined;
    return mouseY = undefined;
  }), true);
  handleMouseMove = function(e) {
    mouseX = (e.clientX - canvasPosition.x) / physics.drawScale;
    return mouseY = (e.clientY - canvasPosition.y) / physics.drawScale;
  };
  physics.getBodyAtMouse = function() {
    var aabb;
    mousePVec = new b2Vec2(mouseX, mouseY);
    aabb = new b2AABB();
    aabb.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
    aabb.upperBound.Set(mouseX + 0.001, mouseY + 0.001);
    selectedBody = null;
    physics.world.QueryAABB(getBodyCB, aabb);
    return selectedBody;
  };
  getBodyCB = function(fixture) {
    if (fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
      selectedBody = fixture.GetBody();
      return false;
    }
    return true;
  };
});
