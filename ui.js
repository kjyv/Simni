// Generated by CoffeeScript 1.3.3
var ui,
  __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

ui = (function() {

  function ui(physics) {
    this.toggleRecorder = __bind(this.toggleRecorder, this);

    this.set_csl_modes = __bind(this.set_csl_modes, this);

    this.set_posture = __bind(this.set_posture, this);

    this.set_friction = __bind(this.set_friction, this);

    this.map_mode = __bind(this.map_mode, this);

    this.map_mode_to_gf = __bind(this.map_mode_to_gf, this);

    this.map_mode_to_gi = __bind(this.map_mode_to_gi, this);

    this.set_preset = __bind(this.set_preset, this);

    this.init = __bind(this.init, this);

    this.update = __bind(this.update, this);
    this.draw_graphics = true;
    this.physics = physics;
    this.init();
  }

  ui.prototype.update = function() {
    if (this.draw_graphics) {
      return this.physics.world.DrawDebugData();
    }
  };

  ui.prototype.init = function() {
    var canvasPosition, getBodyCB, getElementPosition, handleMouseMove,
      _this = this;
    this.physics.map_state_to_mode = false;
    $("#map_state_to_mode").click(function() {
      _this.physics.map_state_to_mode = !_this.physics.map_state_to_mode;
      return $('#map_state_to_mode').attr('checked', _this.physics.map_state_to_mode);
    });
    $("#w0").change(function() {
      $("#w0_val").html("=" + $("#w0").val());
      return _this.physics.w0 = parseFloat($("#w0").val());
    });
    $("#w1").change(function() {
      $("#w1_val").html("=" + $("#w1").val());
      return _this.physics.w1 = parseFloat($("#w1").val());
    });
    $("#w2").change(function() {
      $("#w2_val").html("=" + $("#w2").val());
      return _this.physics.w2 = parseFloat($("#w2").val());
    });
    this.physics.w0 = parseFloat($("#w0").val());
    this.physics.w1 = parseFloat($("#w1").val());
    this.physics.w2 = parseFloat($("#w2").val());
    $("#w0_abs").change(function() {
      _this.physics.w0_abs = $("#w0_abs").attr("checked") !== void 0;
      return draw_state_to_mode_mapping();
    });
    $("#w1_abs").change(function() {
      _this.physics.w1_abs = $("#w1_abs").attr("checked") !== void 0;
      return draw_state_to_mode_mapping();
    });
    this.physics.w0_abs = $("#w0_abs").attr("checked") !== void 0;
    this.physics.w1_abs = $("#w1_abs").attr("checked") !== void 0;
    requestAnimFrame(this.physics.update);
    window.stats = new Stats();
    window.stats.setMode(0);
    window.stats.domElement.style.position = "absolute";
    window.stats.domElement.style.left = "0px";
    window.stats.domElement.style.top = "0px";
    document.body.appendChild(window.stats.domElement);
    window.mouseX = void 0;
    window.mouseY = void 0;
    window.mousePVec = void 0;
    window.isMouseDown = void 0;
    window.selectedBody = void 0;
    window.mouseJoint = void 0;
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
      window.isMouseDown = true;
      handleMouseMove(e);
      return document.addEventListener("mousemove", handleMouseMove, true);
    }), true);
    document.addEventListener("mouseup", (function() {
      document.removeEventListener("mousemove", handleMouseMove, true);
      window.isMouseDown = false;
      window.mouseX = undefined;
      return window.mouseY = undefined;
    }), true);
    handleMouseMove = function(e) {
      window.mouseX = (e.clientX - canvasPosition.x) / physics.debugDraw.GetDrawScale();
      return window.mouseY = (e.clientY - canvasPosition.y) / physics.debugDraw.GetDrawScale();
    };
    window.getBodyAtMouse = function() {
      var aabb;
      window.mousePVec = new b2Vec2(mouseX, mouseY);
      aabb = new b2AABB();
      aabb.lowerBound.Set(mouseX - 0.001, mouseY - 0.001);
      aabb.upperBound.Set(mouseX + 0.001, mouseY + 0.001);
      window.selectedBody = null;
      this.physics.world.QueryAABB(getBodyCB, aabb);
      return window.selectedBody;
    };
    return getBodyCB = function(fixture) {
      if (fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), window.mousePVec)) {
        window.selectedBody = fixture.GetBody();
        return false;
      }
      return true;
    };
  };

  ui.prototype.set_preset = function(w0, w1, w2, w0_abs, w1_abs) {
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

  ui.prototype.map_mode_to_gi = function(mode) {
    if (mode < 0) {
      return mode * 3;
    } else {
      return 18 + (5 * mode);
    }
  };

  ui.prototype.map_mode_to_gf = function(mode) {
    if (mode > 1) {
      return mode * 0.0006 + 1;
    } else if (mode < 0) {
      return 0;
    } else {
      return mode;
    }
  };

  ui.prototype.map_mode = function(bodyJoint, mode, joint) {
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

  ui.prototype.set_friction = function(beta) {
    $("#friction_val").html(beta.toFixed(3));
    if (this.physics.pend_style === 3) {
      this.physics.upper_joint.m_maxMotorTorque = beta;
      this.physics.lower_joint.m_maxMotorTorque = beta;
    } else {
      this.physics.lower_joint.m_maxMotorTorque = beta;
    }
    return this.physics.beta = beta;
  };

  ui.prototype.set_posture = function(bodyAngle, hipAngle, kneeAngle, hipCsl, kneeCsl) {
    var p, x0, y0;
    p = this.physics;
    p.world.DestroyBody(p.body3);
    p.world.DestroyJoint(p.lower_joint);
    p.world.DestroyBody(p.body2);
    p.world.DestroyJoint(p.upper_joint);
    p.world.DestroyBody(p.body);
    x0 = 0.516;
    y0 = 0.76;
    return p.createSemni(x0, y0);
  };

  ui.prototype.set_csl_modes = function(hipCSL, kneeCSL) {
    var contract_gf_hip, contract_gf_knee, gb, gf, gi_hip, gi_knee, release_bias_hip, release_bias_knee, release_gf;
    release_bias_hip = 1;
    release_bias_knee = 0.8;
    release_gf = 0.99;
    contract_gf_hip = 1.0030;
    contract_gf_knee = 1.0020;
    gi_hip = 28;
    gi_knee = 28;
    if (hipCSL === "r+") {
      gf = release_gf;
      gb = release_bias_hip;
    } else if (hipCSL === "r-") {
      gf = release_gf;
      gb = -release_bias_hip;
    } else if (hipCSL === "c") {
      gf = contract_gf_hip;
      gb = 0;
    }
    $("#gi_param_upper").val(gi_hip);
    this.physics.upper_joint.gi = gi_hip;
    $("#gf_param_upper").val(gf);
    this.physics.upper_joint.gf = gf;
    $("#gb_param_upper").val(gb);
    this.physics.upper_joint.gb = gb;
    if (kneeCSL === "r+") {
      gf = release_gf;
      gb = release_bias_knee;
    } else if (kneeCSL === "r-") {
      gf = release_gf;
      gb = -release_bias_knee;
    } else if (kneeCSL === "c") {
      gf = contract_gf_knee;
      gb = 0;
    }
    $("#gi_param_lower").val(gi_knee);
    this.physics.lower_joint.gi = gi_knee;
    $("#gf_param_lower").val(gf);
    this.physics.lower_joint.gf = gf;
    $("#gb_param_lower").val(gb);
    return this.physics.lower_joint.gb = gb;
  };

  ui.prototype.toggleRecorder = function() {
    return this.physics.recordPhase = !this.physics.recordPhase;
  };

  return ui;

})();

window.requestAnimFrame = (function() {
  return window.oRequestAnimationFrame || window.webkitRequestAnimationFrame || window.mozRequestAnimationFrame || window.oRequestAnimationFrame || window.msRequestAnimationFrame || function(callback, element) {
    return window.setTimeout(callback, 1000 / 60);
  };
})();

$(function() {
  var p;
  p = new physics();
  window.physics = p;
  ui = new ui(p);
  p.ui = ui;
  return window.ui = ui;
});
