(function() {
  "use strict";
  var b2Body, b2BodyDef, b2CircleShape, b2Color, b2DebugDraw, b2Fixture, b2FixtureDef, b2MassData, b2MouseJointDef, b2PolygonShape, b2RevoluteJointDef, b2Shape, b2Transform, b2Vec2, b2World, dt, physics, steps_per_frame,
    __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

  dt = 1 / 960;

  steps_per_frame = 16;

  b2MouseJointDef = Box2D.Dynamics.Joints.b2MouseJointDef;

  b2Vec2 = Box2D.Common.Math.b2Vec2;

  b2Color = Box2D.Common.b2Color;

  b2Transform = Box2D.Common.Math.b2Transform;

  b2BodyDef = Box2D.Dynamics.b2BodyDef;

  b2Body = Box2D.Dynamics.b2Body;

  b2FixtureDef = Box2D.Dynamics.b2FixtureDef;

  b2Fixture = Box2D.Dynamics.b2Fixture;

  b2Shape = Box2D.Collision.Shapes.b2Shape;

  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;

  b2World = Box2D.Dynamics.b2World;

  b2MassData = Box2D.Collision.Shapes.b2MassData;

  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape;

  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape;

  b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef;

  b2DebugDraw = Box2D.Dynamics.b2DebugDraw;

  physics = (function() {
    var R, R_inv, U_in, angle, fg, kb, km, pos_i, pos_p, v, was_static;

    function physics() {
      this.update = __bind(this.update, this);
      this.updateMode = __bind(this.updateMode, this);
      this.calcMode = __bind(this.calcMode, this);
      this.applyFriction = __bind(this.applyFriction, this);
      this.clip = __bind(this.clip, this);
      this.updateMotor = __bind(this.updateMotor, this);
      this.updateController = __bind(this.updateController, this);
      this.Position = __bind(this.Position, this);
      this.Bounce = __bind(this.Bounce, this);
      this.CSL = __bind(this.CSL, this);
      this.logData = __bind(this.logData, this);
      this.myon_precision = __bind(this.myon_precision, this);
      this.getNoisyAngle = __bind(this.getNoisyAngle, this);
      this.togglePositionController = __bind(this.togglePositionController, this);
      this.toggleBounce = __bind(this.toggleBounce, this);
      this.toggleCSL = __bind(this.toggleCSL, this);
      this.createSemni = __bind(this.createSemni, this);
      this.createDoublePendulum = __bind(this.createDoublePendulum, this);
      this.createPendulum = __bind(this.createPendulum, this);
      this.createCircle = __bind(this.createCircle, this);
      this.createBox = __bind(this.createBox, this);
      var bodyDef, fixDef;

      this.world = new b2World(new b2Vec2(0, 9.81), false);
      fixDef = new b2FixtureDef;
      fixDef.density = 10;
      fixDef.friction = 0.5;
      fixDef.restitution = 0.1;
      this.fixDef = fixDef;
      this.ground_height = 0.03;
      this.ground_width = 50;
      bodyDef = new b2BodyDef;
      bodyDef.type = b2Body.b2_staticBody;
      bodyDef.position.x = 0;
      bodyDef.position.y = 1.1;
      bodyDef.linearDamping = 50;
      fixDef.shape = new b2PolygonShape;
      fixDef.shape.SetAsBox(this.ground_width, this.ground_height);
      this.ground = this.world.CreateBody(bodyDef);
      this.ground_bodyDef = bodyDef;
      this.ground.CreateFixture(fixDef);
      this.lower_joint = null;
      this.upper_joint = null;
      this.debugDraw = new b2DebugDraw();
      this.debugDraw.SetSprite($("#simulation canvas")[0].getContext("2d"));
      this.debugDraw.SetDrawScale(260);
      this.debugDraw.SetFillAlpha(0);
      this.debugDraw.SetLineThickness(1.0);
      this.debugDraw.AppendFlags(b2DebugDraw.e_shapeBit);
      this.world.SetDebugDraw(this.debugDraw);
      this.run = true;
      this.step = false;
      this.pend_style = 0;
      this.recordPhase = false;
      this.startLog = true;
      this.logged_data = [];
      this.beta = 0;
      this.abc = new simni.Abc();
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
      bodyDef.position.Set(2, 0.8);
      box = this.world.CreateBody(bodyDef);
      return box.CreateFixture(fixDef);
    };

    physics.prototype.createCircle = function() {
      var bodyDef, box, fixDef;

      fixDef = new b2FixtureDef;
      fixDef.density = 1;
      fixDef.friction = 0.3;
      fixDef.restitution = 0.2;
      this.fixDef = fixDef;
      fixDef.shape = new b2CircleShape;
      fixDef.shape.m_radius = 0.12;
      bodyDef = new b2BodyDef;
      bodyDef.type = b2Body.b2_staticBody;
      bodyDef.position.Set(2, 0.8);
      box = this.world.CreateBody(bodyDef);
      return box.CreateFixture(fixDef);
    };

    physics.prototype.createPendulum = function() {
      var bodyDef, damping, jointDef, line, mass, mass_size, pend_length, pend_vertices;

      pend_length = 0.400;
      mass_size = 0.03;
      damping = 0;
      bodyDef = new b2BodyDef;
      bodyDef.type = b2Body.b2_dynamicBody;
      this.fixDef.density = 35;
      this.fixDef.shape = new b2PolygonShape;
      pend_vertices = new Array(new b2Vec2(this.ground_bodyDef.position.x + (this.ground_width / 2) + 1.4, this.ground_bodyDef.position.y - this.ground_height - 0.005), new b2Vec2(this.ground_bodyDef.position.x + (this.ground_width / 2) + 1.4, this.ground_bodyDef.position.y - this.ground_height - pend_length));
      this.fixDef.shape.SetAsArray(pend_vertices, 2);
      bodyDef.linearDamping = damping;
      bodyDef.angularDamping = damping;
      this.body = this.world.CreateBody(bodyDef);
      line = this.body.CreateFixture(this.fixDef);
      this.body.z2 = 0;
      this.fixDef.shape = new b2CircleShape(mass_size);
      this.fixDef.shape.m_p = pend_vertices[1];
      mass = this.body.CreateFixture(this.fixDef);
      jointDef = new b2RevoluteJointDef();
      jointDef.Initialize(this.ground, this.body, pend_vertices[0]);
      jointDef.collideConnected = true;
      jointDef.maxMotorTorque = this.beta;
      jointDef.motorSpeed = 0.0;
      jointDef.enableMotor = true;
      this.lower_joint = this.world.CreateJoint(jointDef);
      this.lower_joint.angle_speed = 0;
      this.lower_joint.csl_active = false;
      this.lower_joint.bounce_active = false;
      this.lower_joint.position_controller_active = false;
      this.lower_joint.joint_name = 'lower';
      this.lower_joint.csl_sign = 1;
      this.lower_joint.gain = 1;
      this.lower_joint.gb = 0;
      this.lower_joint.motor_torque = 0;
      this.lower_joint.motor_control = 0;
      return this.lower_joint.I_tm1 = 0;
    };

    physics.prototype.createDoublePendulum = function() {
      var bodyDef, damping, jointDef, line, line2, mass, mass_size, pend_length, pend_vertices;

      pend_length = 0.5;
      mass_size = 0.04;
      damping = 0;
      bodyDef = new b2BodyDef;
      bodyDef.type = b2Body.b2_dynamicBody;
      this.fixDef.density = 10;
      this.fixDef.shape = new b2PolygonShape;
      pend_vertices = new Array(new b2Vec2(this.ground_bodyDef.position.x + (this.ground_width / 2) + 1.4, this.ground_bodyDef.position.y - this.ground_height - 0.005), new b2Vec2(this.ground_bodyDef.position.x + (this.ground_width / 2) + 1.4, this.ground_bodyDef.position.y - this.ground_height - pend_length));
      this.fixDef.shape.SetAsArray(pend_vertices, 2);
      bodyDef.linearDamping = damping;
      bodyDef.angularDamping = damping;
      this.body = this.world.CreateBody(bodyDef);
      line = this.body.CreateFixture(this.fixDef);
      this.body.z2 = 0;
      jointDef = new b2RevoluteJointDef();
      jointDef.Initialize(this.body, this.ground, pend_vertices[0]);
      jointDef.collideConnected = true;
      this.lower_joint = this.world.CreateJoint(jointDef);
      this.lower_joint.angle_speed = 0;
      this.lower_joint.csl_active = false;
      this.lower_joint.bounce_active = false;
      this.lower_joint.position_controller_active = false;
      this.lower_joint.joint_name = 'lower';
      this.lower_joint.csl_sign = 1;
      this.lower_joint.gain = 1;
      this.lower_joint.gb = 0;
      this.lower_joint.motor_torque = 0;
      this.lower_joint.motor_control = 0;
      this.lower_joint.I_t = 0;
      this.fixDef.shape = new b2CircleShape(mass_size);
      this.fixDef.shape.m_p = pend_vertices[1];
      mass = this.body.CreateFixture(this.fixDef);
      pend_vertices = new Array(new b2Vec2(this.ground_bodyDef.position.x + (this.ground_width / 2) + 1.4, this.ground_bodyDef.position.y - this.ground_height - pend_length - 0.005), new b2Vec2(this.ground_bodyDef.position.x + (this.ground_width / 2) + 1.4, this.ground_bodyDef.position.y - this.ground_height - (2 * pend_length)));
      this.fixDef.shape = new b2PolygonShape;
      this.fixDef.shape.SetAsArray(pend_vertices, 2);
      bodyDef.linearDamping = damping;
      bodyDef.angularDamping = damping;
      this.body2 = this.world.CreateBody(bodyDef);
      line2 = this.body2.CreateFixture(this.fixDef);
      this.body2.z2 = 0;
      jointDef = new b2RevoluteJointDef();
      jointDef.Initialize(this.body2, this.body, pend_vertices[0]);
      jointDef.collideConnected = false;
      this.upper_joint = this.world.CreateJoint(jointDef);
      this.upper_joint.joint_name = 'upper';
      this.upper_joint.angle_speed = 0;
      this.upper_joint.csl_active = false;
      this.upper_joint.csl_sign = 1;
      this.upper_joint.gain = 1;
      this.upper_joint.gb = 0;
      this.upper_joint.motor_torque = 0;
      this.upper_joint.bounce_active = false;
      this.upper_joint.position_controller_active = false;
      this.fixDef.shape = new b2CircleShape(mass_size);
      this.fixDef.shape.m_p = pend_vertices[1];
      return mass = this.body2.CreateFixture(this.fixDef);
    };

    /*
    ellipse2polygon: (r, b, x0, y0) =>
      points = new Array()
      step = 2*Math.PI/40
      for theta in [2*Math.PI..0] by -step
        x = + b*r*Math.cos(theta)
        y = - r*Math.sin(theta)
        points.push(new b2Vec2(x,y))
      return points[0...points.length-1]
    */


    physics.prototype.createSemni = function(x0, y0) {
      var bodyDef, bodyDef2, bodyDef3, bodyDensity, bodyFriction, bodyRestitution, fixture, jointDef, lowerArmDensity, lowerArmFriction, lowerArmRestitution, md, upperArmDensity, upperArmFriction, upperArmRestitution, _i, _j, _len, _len1, _ref, _ref1;

      if (x0 == null) {
        x0 = 1;
      }
      if (y0 == null) {
        y0 = 0.5;
      }
      bodyDensity = 0.96;
      bodyFriction = 0.25;
      bodyRestitution = 0.1;
      upperArmDensity = 4.2;
      upperArmFriction = 0.25;
      upperArmRestitution = 0.1;
      lowerArmDensity = 11.35;
      lowerArmFriction = 0.25;
      lowerArmRestitution = 0.2;
      bodyDef = new b2BodyDef;
      bodyDef.type = b2Body.b2_dynamicBody;
      bodyDef.position.Set(x0, y0);
      this.body = this.world.CreateBody(bodyDef);
      this.fixDef = new b2FixtureDef;
      this.fixDef.density = bodyDensity;
      this.fixDef.friction = bodyFriction;
      this.fixDef.restitution = bodyRestitution;
      this.fixDef.filter.groupIndex = -1;
      this.fixDef.shape = new b2PolygonShape;
      b2Separator.Separate(this.body, this.fixDef, simni.contour_original_low_detail, 1000, 0.177, 0.192);
      /*
      #else
        console.log "can't import contour, validator error " + e
        console.log """
                0 if the vertices can be properly processed.
                1 If there are overlapping lines.
                2 if the points are not in counter-clockwise order.
                3 if there are overlapping lines and the points are not in counter-clockwise order.
        """
      */

      this.fixDef.density = 0.00001;
      this.fixDef.shape = new b2CircleShape;
      this.fixDef.shape.m_p.Set(simni.head[0].x, simni.head[0].y);
      this.fixDef.shape.m_radius = simni.head[1];
      this.fixDef.filter.groupIndex = 1;
      this.body.CreateFixture(this.fixDef);
      md = new b2MassData();
      this.body.GetMassData(md);
      md.center.Set(simni.contourCenter.x, simni.contourCenter.y);
      md.I = this.body.GetInertia() + md.mass * (md.center.x * md.center.x + md.center.y * md.center.y);
      this.body.SetMassData(md);
      bodyDef2 = new b2BodyDef;
      bodyDef2.type = b2Body.b2_dynamicBody;
      this.body2 = this.world.CreateBody(bodyDef2);
      this.fixDef2 = new b2FixtureDef;
      this.fixDef2.density = upperArmDensity;
      this.fixDef2.friction = upperArmFriction;
      this.fixDef2.restitution = upperArmRestitution;
      this.fixDef2.filter.groupIndex = -1;
      this.fixDef2.shape = new b2PolygonShape;
      _ref = simni.arm1ContourConvex;
      for (_i = 0, _len = _ref.length; _i < _len; _i++) {
        fixture = _ref[_i];
        this.fixDef2.shape.SetAsArray(fixture, fixture.length);
        this.body2.CreateFixture(this.fixDef2);
      }
      md = new b2MassData();
      this.body2.GetMassData(md);
      md.center.Set(simni.arm1Center.x, simni.arm1Center.y);
      md.I = this.body2.GetInertia() + md.mass * (md.center.x * md.center.x + md.center.y * md.center.y);
      this.body2.SetMassData(md);
      this.body2.SetPositionAndAngle(new b2Vec2(simni.arm1Center.x, simni.arm1Center.y), 0);
      this.body2.z2 = 0;
      /*
      @fixDef2.density = 14.2
      @fixDef2.shape = new b2CircleShape
      @fixDef2.shape.m_p.Set(simni.arm1Center.x+0.03, simni.arm1Center.y-0.015)
      @fixDef2.shape.m_radius = 0.04
      @fixDef2.filter.groupIndex = -1
      @body2.CreateFixture(@fixDef2)
      */

      jointDef = new b2RevoluteJointDef();
      jointDef.bodyA = this.body;
      jointDef.bodyB = this.body2;
      jointDef.localAnchorA.Set(simni.arm1JointAnchor.x, simni.arm1JointAnchor.y);
      jointDef.localAnchorB.Set(simni.arm1JointAnchor.x, simni.arm1JointAnchor.y);
      jointDef.collideConnected = true;
      jointDef.maxMotorTorque = this.beta;
      jointDef.motorSpeed = 0.0;
      jointDef.enableMotor = true;
      this.upper_joint = this.world.CreateJoint(jointDef);
      this.upper_joint.joint_name = 'upper';
      this.upper_joint.motor_control = 0;
      this.upper_joint.I_t = 0;
      this.upper_joint.angle_speed = 0;
      this.upper_joint.csl_active = false;
      this.upper_joint.csl_sign = 1;
      this.upper_joint.last_motor_torque = 0;
      this.upper_joint.motor_torque = 0;
      this.upper_joint.bounce_active = false;
      this.upper_joint.bounce_sign = 1;
      this.upper_joint.bounce_vel = 0.0003;
      this.upper_joint.position_controller_active = false;
      bodyDef3 = new b2BodyDef;
      bodyDef3.type = b2Body.b2_dynamicBody;
      this.body3 = this.world.CreateBody(bodyDef3);
      this.fixDef3 = new b2FixtureDef;
      this.fixDef3.density = lowerArmDensity;
      this.fixDef3.friction = lowerArmFriction;
      this.fixDef3.restitution = lowerArmRestitution;
      this.fixDef3.filter.groupIndex = -1;
      this.fixDef3.shape = new b2PolygonShape;
      _ref1 = simni.arm2ContourConvex;
      for (_j = 0, _len1 = _ref1.length; _j < _len1; _j++) {
        fixture = _ref1[_j];
        this.fixDef3.shape.SetAsArray(fixture, fixture.length);
        this.body3.CreateFixture(this.fixDef3);
      }
      md = new b2MassData();
      this.body3.GetMassData(md);
      md.center.Set(simni.arm2Center.x, simni.arm2Center.y);
      md.I = this.body3.GetInertia() + md.mass * (md.center.x * md.center.x + md.center.y * md.center.y);
      this.body3.SetMassData(md);
      this.body3.SetPositionAndAngle(new b2Vec2(simni.arm1Center.x, simni.arm1Center.y), 0);
      /*
      @fixDef3.density = 22.4  #=72g
      @fixDef3.shape = new b2CircleShape
      @fixDef3.shape.m_p.Set(simni.arm2Center.x-0.04, simni.arm2Center.y-0.01)
      @fixDef3.shape.m_radius = 0.03
      @fixDef3.filter.groupIndex = -1
      @body3.CreateFixture(@fixDef3)
      */

      this.body3.z2 = 0;
      jointDef = new b2RevoluteJointDef();
      jointDef.bodyA = this.body2;
      jointDef.bodyB = this.body3;
      jointDef.localAnchorA.Set(simni.arm2JointAnchor.x, simni.arm2JointAnchor.y);
      jointDef.localAnchorB.Set(simni.arm2JointAnchor.x, simni.arm2JointAnchor.y);
      jointDef.collideConnected = false;
      jointDef.maxMotorTorque = this.beta;
      jointDef.motorSpeed = 0.0;
      jointDef.enableMotor = true;
      jointDef.upperAngle = 1.90816;
      jointDef.lowerAngle = -3.2421;
      jointDef.enableLimit = true;
      this.lower_joint = this.world.CreateJoint(jointDef);
      this.lower_joint.joint_name = 'lower';
      this.lower_joint.motor_control = 0;
      this.lower_joint.I_t = 0;
      this.lower_joint.angle_speed = 0;
      this.lower_joint.csl_active = false;
      this.lower_joint.csl_sign = 1;
      this.lower_joint.last_motor_torque = 0;
      this.lower_joint.motor_torque = 0;
      this.lower_joint.bounce_active = false;
      this.lower_joint.bounce_vel = 0.00047;
      this.lower_joint.bounce_sign = 1;
      return this.lower_joint.position_controller_active = false;
    };

    physics.prototype.toggleCSL = function(bodyJoint) {
      if (bodyJoint.bounce_active) {
        $("#toggle_bounce").click();
      }
      if (bodyJoint.position_controller_active) {
        bodyJoint.position_controller_active = false;
      }
      bodyJoint.csl_active = !bodyJoint.csl_active;
      if (this.lower_joint) {
        $("#set_csl_params_lower").trigger('click');
      }
      if (this.upper_joint) {
        $("#set_csl_params_upper").trigger('click');
      }
      if (bodyJoint.last_angle == null) {
        bodyJoint.last_angle = bodyJoint.GetJointAngle();
      }
      bodyJoint.motor_control = 0;
      return bodyJoint.last_integrated = 0;
    };

    physics.prototype.toggleBounce = function(bodyJoint) {
      if (bodyJoint.csl_active) {
        $("#toggle_csl").click();
      }
      if (bodyJoint.position_controller_active) {
        bodyJoint.position_controller_active = false;
      }
      bodyJoint.bounce_active = !bodyJoint.bounce_active;
      bodyJoint.motor_control = 0;
      return bodyJoint.last_integrated = 0;
    };

    physics.prototype.togglePositionController = function(bodyJoint) {
      if (bodyJoint.csl_active) {
        $("#toggle_csl").click();
      }
      if (bodyJoint.position_controller_active) {
        bodyJoint.position_controller_active = false;
      } else {
        bodyJoint.position_controller_active = true;
      }
      bodyJoint.motor_control = 0;
      return bodyJoint.last_integrated = 0;
    };

    physics.prototype.getNoisyAngle = function(bodyJoint) {
      var rand;

      rand = Math.random() / 1000;
      rand = rand - (0.5 / 1000);
      return bodyJoint.GetJointAngle() + rand;
    };

    physics.prototype.myon_precision = function(number) {
      return Math.floor(number * 10000) / 10000;
    };

    physics.prototype.logData = function() {
      if (this.recordPhase) {
        if (this.startLog) {
          this.logged_data = [];
          this.startLog = false;
        }
        return this.logged_data.push(-this.body.GetAngle() + " " + -this.upper_joint.GetJointAngle() + " " + -this.lower_joint.GetJointAngle() + " " + this.body2.motor_control + " " + this.body3.motor_control);
      }
    };

    physics.prototype.CSL = function(gi, gf, gb, angle_diff, gain, bodyJoint) {
      var sum, vel;

      if (gain == null) {
        gain = 1;
      }
      vel = gi * angle_diff;
      sum = vel + bodyJoint.last_integrated;
      bodyJoint.last_integrated = gf * sum;
      return (sum * gain) + gb;
    };

    physics.prototype.Bounce = function(vs, angle_diff, bodyJoint) {
      if (Math.abs(bodyJoint.motor_torque) > 0.9) {
        bodyJoint.bounce_sign = -bodyJoint.bounce_sign;
        bodyJoint.last_integrated = 0;
      }
      bodyJoint.last_integrated += 35 * (angle_diff - (vs * bodyJoint.bounce_sign));
      return bodyJoint.last_integrated;
    };

    pos_p = 5;

    pos_i = 0.001;

    physics.prototype.Position = function(set_position, bodyJoint) {
      var offset;

      offset = bodyJoint.GetJointAngle() - set_position;
      bodyJoint.last_integrated += offset * pos_i;
      return offset * pos_p + bodyJoint.last_integrated;
    };

    angle = 0;

    physics.prototype.updateController = function(bodyJoint) {
      angle = this.getNoisyAngle(bodyJoint);
      bodyJoint.angle_diff = angle - bodyJoint.last_angle;
      bodyJoint.last_angle = angle;
      if (bodyJoint.last_integrated == null) {
        bodyJoint.last_integrated = 0;
      }
      if (bodyJoint.csl_active) {
        return bodyJoint.motor_control = this.CSL(bodyJoint.gi, bodyJoint.gf, bodyJoint.gb, bodyJoint.angle_diff, bodyJoint.gain, bodyJoint);
      } else if (bodyJoint.bounce_active) {
        return bodyJoint.motor_control = this.Bounce(bodyJoint.bounce_vel, bodyJoint.angle_diff, bodyJoint);
      } else if (bodyJoint.position_controller_active) {
        return bodyJoint.motor_control = this.Position(bodyJoint.set_position, bodyJoint);
      }
    };

    km = 1.393;

    kb = 2.563;

    R = 9.59;

    R_inv = 1 / R;

    U_in = 0;

    physics.prototype.updateMotor = function(bodyJoint) {
      U_in = bodyJoint.motor_control;
      bodyJoint.I_t = this.clip(U_in - (kb * (-bodyJoint.GetJointSpeed())), 12) * R_inv;
      bodyJoint.motor_torque = km * bodyJoint.I_t;
      bodyJoint.m_applyTorque += bodyJoint.motor_torque;
    };

    physics.prototype.clip = function(value, cap) {
      if (cap == null) {
        cap = 1;
      }
      return Math.max(-cap, Math.min(cap, value));
    };

    v = fg = 0;

    physics.prototype.applyFriction = function(bodyJoint) {
      v = -bodyJoint.GetJointSpeed();
      fg = -v * this.beta * 5;
      return bodyJoint.m_applyTorque += fg;
    };

    physics.prototype.calcMode = function(motor_torque, angle_speed) {
      var as, mc, mode;

      mc = this.w0_abs ? Math.abs(motor_torque) : motor_torque;
      as = this.w1_abs ? Math.abs(angle_speed) : angle_speed;
      return mode = this.w0 * mc + this.w1 * as + this.w2;
    };

    physics.prototype.updateMode = function(bodyJoint) {
      var mode;

      mode = this.calcMode(bodyJoint.motor_torque, bodyJoint.angle_diff_csl);
      mode = this.clip(mode, 3);
      return this.ui.map_mode(bodyJoint, mode);
    };

    /*
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
    */


    was_static = false;

    physics.prototype.update = function() {
      var body, i, md;

      if (!this.run && !this.step) {
        return;
      }
      window.stats.begin();
      if ((this.run || this.step) && this.pend_style) {
        this.step = false;
        if (isMouseDown && (!mouseJoint)) {
          body = window.getBodyAtMouse();
          if (body && !(body === this.ground && this.pend_style === 3)) {
            md = new b2MouseJointDef();
            md.bodyA = this.world.GetGroundBody();
            if (body.GetType() === b2Body.b2_staticBody) {
              body.SetType(b2Body.b2_dynamicBody);
              was_static = true;
            }
            md.bodyB = body;
            md.target.Set(mouseX, mouseY);
            md.collideConnected = false;
            md.maxForce = 200.0 * body.GetMass();
            md.dampingRatio = 2;
            md.frequencyHz = 20;
            window.mouseJoint = this.world.CreateJoint(md);
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
            window.mouseJoint = null;
          }
        }
        if (this.pend_style === 1) {
          if (this.map_state_to_mode) {
            this.updateMode(this.lower_joint);
          }
        } else if (this.pend_style === 2) {
          if (this.map_state_to_mode) {
            this.updateMode(this.lower_joint);
            this.updateMode(this.upper_joint);
          }
        } else if (this.pend_style === 3) {
          this.abc.update(this.body, this.upper_joint, this.lower_joint);
        }
        this.logData();
        i = steps_per_frame;
        while (i > 0) {
          if (this.pend_style === 3) {
            this.updateController(this.upper_joint);
            this.updateController(this.lower_joint);
            this.updateMotor(this.upper_joint);
            this.updateMotor(this.lower_joint);
            this.applyFriction(this.upper_joint);
            this.applyFriction(this.lower_joint);
          } else if (this.pend_style === 1) {
            this.updateController(this.lower_joint);
            this.updateMotor(this.lower_joint);
            this.applyFriction(this.lower_joint);
          } else if (this.pend_style === 2) {
            this.updateController(this.lower_joint);
            this.updateController(this.upper_joint);
            this.updateMotor(this.lower_joint);
            this.updateMotor(this.upper_joint);
            this.applyFriction(this.lower_joint);
            this.applyFriction(this.upper_joint);
          }
          this.world.Step(dt, 10, 10);
          i--;
        }
        this.world.ClearForces();
        this.ui.update();
        draw_phase_space();
        draw_motor_torque();
      }
      window.stats.end();
      return requestAnimFrame(this.update);
    };

    return physics;

  })();

  window.simni.Physics = physics;

}).call(this);
