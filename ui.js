// Generated by CoffeeScript 1.3.3
var ui,
  __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

ui = (function() {

  function ui(physics) {
    this.getPostureGraphAsFile = __bind(this.getPostureGraphAsFile, this);

    this.setSemniTransformAsFile = __bind(this.setSemniTransformAsFile, this);

    this.setSemniTransformAsJSON = __bind(this.setSemniTransformAsJSON, this);

    this.getSemniTransformAsFile = __bind(this.getSemniTransformAsFile, this);

    this.getSemniTransformAsJSON = __bind(this.getSemniTransformAsJSON, this);

    this.getLogfile = __bind(this.getLogfile, this);

    this.toggleRecorder = __bind(this.toggleRecorder, this);

    this.set_csl_mode_lower = __bind(this.set_csl_mode_lower, this);

    this.set_csl_mode_upper = __bind(this.set_csl_mode_upper, this);

    this.set_posture = __bind(this.set_posture, this);

    this.set_friction = __bind(this.set_friction, this);

    this.map_mode = __bind(this.map_mode, this);

    this.map_mode_to_gf = __bind(this.map_mode_to_gf, this);

    this.map_mode_to_gi = __bind(this.map_mode_to_gi, this);

    this.set_preset = __bind(this.set_preset, this);

    this.init = __bind(this.init, this);

    this.getSemniOutlineSVG = __bind(this.getSemniOutlineSVG, this);

    this.rotate_point = __bind(this.rotate_point, this);

    this.update = __bind(this.update, this);
    this.draw_graphics = true;
    this.physics = physics;
    this.init();
    this.halftime = true;
    this.svg_scale = 100;
  }

  ui.prototype.update = function() {
    if (this.draw_graphics && this.halftime) {
      this.physics.world.DrawDebugData();
      /*
              @drawSemniOutlineSVG(
                @physics.body.GetPosition(),
                @physics.body2.GetPosition(),
                @physics.body3.GetPosition(),
                @physics.body.GetAngle(),
                @physics.body2.GetAngle(),
                @physics.body3.GetAngle()
              )
      */

    }
    return this.halftime = !this.halftime;
  };

  ui.prototype.rotate_point = function(cx, cy, angle, p) {
    var c, s, xnew, ynew;
    p.x -= cx;
    p.y -= cy;
    s = Math.sin(angle);
    c = Math.cos(angle);
    xnew = p.x * c - p.y * s;
    ynew = p.x * s + p.y * c;
    p.x = xnew + cx;
    p.y = ynew + cy;
    return p;
  };

  ui.prototype.getSemniOutlineSVG = function(body_pos, arm1_pos, arm2_pos, body_angle, arm1_angle, arm2_angle, container) {
    var arm1_joint, arm2_joint, b_x, b_y, d3line2, h_x, h_y, svg, svg_joint, svg_joint2, svg_semni_arm1, svg_semni_arm2, svg_semni_body, svg_semni_head;
    svg = container.append("svg:g");
    d3line2 = d3.svg.line().x(function(d) {
      return d.x * physics.ui.svg_scale;
    }).y(function(d) {
      return d.y * physics.ui.svg_scale;
    }).interpolate("linear");
    svg_semni_body = svg.append("svg:path").attr("d", d3line2(contour_original_lowest_detail)).style("stroke-width", 1).style("stroke", "gray").style("fill", "none");
    svg_semni_arm1 = svg.append("svg:path").attr("d", d3line2(arm1Contour)).style("stroke-width", 1).style("stroke", "gray").style("fill", "none");
    svg_joint = svg.append("svg:circle").attr("cx", 0).attr("cy", 0).attr("r", "1").style("stroke", "red");
    svg_semni_arm2 = svg.append("svg:path").attr("d", d3line2(arm2Contour)).style("stroke-width", 1).style("stroke", "gray").style("fill", "none");
    svg_joint2 = svg.append("svg:circle").attr("cx", 0).attr("cy", 0).attr("r", "1").style("stroke", "red");
    svg_semni_head = svg.append("svg:circle").attr("cx", 0).attr("cy", 0).attr("r", head2[1] * this.svg_scale).style("stroke-width", 1).style("stroke", "gray").style("fill", "none");
    b_x = body_pos.x * this.svg_scale;
    b_y = body_pos.y * this.svg_scale;
    svg_semni_body.attr("transform", "rotate(" + body_angle * 180 / Math.PI + ")");
    arm1_joint = new b2Vec2();
    arm1_joint.x = arm1JointAnchor2.x * this.svg_scale;
    arm1_joint.y = arm1JointAnchor2.y * this.svg_scale;
    arm1_joint = this.rotate_point(0, 0, body_angle, arm1_joint);
    svg_joint.attr("transform", "translate(" + arm1_joint.x + "," + arm1_joint.y + ")");
    svg_semni_arm1.attr("transform", "rotate(" + arm1_angle * 180 / Math.PI + "," + arm1_joint.x + "," + arm1_joint.y + ") translate(" + arm1_joint.x + "," + arm1_joint.y + ")");
    arm2_joint = new b2Vec2();
    arm2_joint.x = arm2JointAnchor2.x * this.svg_scale;
    arm2_joint.y = arm2JointAnchor2.y * this.svg_scale;
    arm2_joint = this.rotate_point(0, 0, body_angle, arm2_joint);
    arm2_joint = this.rotate_point(arm1_joint.x, arm1_joint.y, arm1_angle - body_angle - 0.85, arm2_joint);
    svg_joint2.attr("transform", "translate(" + arm2_joint.x + "," + arm2_joint.y + ")");
    svg_semni_arm2.attr("transform", "rotate(" + arm2_angle * 180 / Math.PI + "," + arm2_joint.x + "," + arm2_joint.y + ") translate(" + arm2_joint.x + "," + arm2_joint.y + ")");
    h_x = head2[0].x * this.svg_scale;
    h_y = head2[0].y * this.svg_scale;
    svg_semni_head.attr("transform", "rotate(" + body_angle * 180 / Math.PI + "," + 0 + "," + 0 + ") translate(" + h_x + "," + h_y + ")");
    return svg;
  };

  ui.prototype.init = function() {
    var canvas, canvasPosition, getBodyCB, getElementPosition, handleMouseMove,
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
    canvas = $('#simulation canvas')[0];
    canvasPosition = getElementPosition(canvas);
    canvas.addEventListener("mousedown", (function(e) {
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
      return 15 + (5 * mode);
    }
  };

  ui.prototype.map_mode_to_gf = function(mode) {
    if (mode > 1) {
      return mode * 0.00125 + 1;
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
    gi = this.map_mode_to_gi(mode);
    gf = this.map_mode_to_gf(mode);
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

  ui.prototype.set_csl_mode_upper = function(hipCSL, change_select) {
    var contract_gf_hip, gb, gf, gi_hip, release_bias_hip, release_gf, stall_gb;
    if (change_select == null) {
      change_select = true;
    }
    release_bias_hip = 0.6;
    release_gf = 0.3;
    contract_gf_hip = 1.0030;
    gi_hip = 30;
    stall_gb = 15;
    if (hipCSL === "r+") {
      gf = release_gf;
      gb = release_bias_hip;
      this.physics.upper_joint.csl_prefill = 0.5;
    } else if (hipCSL === "r-") {
      gf = release_gf;
      gb = -release_bias_hip;
      this.physics.upper_joint.csl_prefill = -0.5;
    } else if (hipCSL === "c") {
      gf = contract_gf_hip;
      gb = 0;
      this.physics.upper_joint.last_integrated = this.physics.upper_joint.csl_prefill;
    } else if (hipCSL === "s+") {
      gf = release_gf;
      gb = stall_gb;
    } else if (hipCSL === "s-") {
      gf = release_gf;
      gb = -stall_gb;
    }
    if (change_select) {
      $("#csl_mode_hip option[value='" + hipCSL + "']").attr("selected", true);
    }
    $("#gi_param_upper").val(gi_hip);
    this.physics.upper_joint.gi = gi_hip;
    $("#gf_param_upper").val(gf);
    this.physics.upper_joint.gf = gf;
    $("#gb_param_upper").val(gb);
    this.physics.upper_joint.gb = gb;
    return this.physics.upper_joint.csl_mode = hipCSL;
  };

  ui.prototype.set_csl_mode_lower = function(kneeCSL, change_select) {
    var contract_gf_knee, gb, gf, gi_knee, release_bias_knee, release_gf, stall_gb;
    if (change_select == null) {
      change_select = true;
    }
    release_bias_knee = 0.6;
    contract_gf_knee = 1.0020;
    release_gf = 0.3;
    gi_knee = 35;
    stall_gb = 15;
    if (kneeCSL === "r+") {
      gf = release_gf;
      gb = release_bias_knee;
      this.physics.lower_joint.csl_prefill = -0.5;
    } else if (kneeCSL === "r-") {
      gf = release_gf;
      gb = -release_bias_knee;
      this.physics.lower_joint.csl_prefill = 0.5;
    } else if (kneeCSL === "c") {
      gf = contract_gf_knee;
      gb = 0;
      this.physics.upper_joint.last_integrated = this.physics.upper_joint.csl_prefill;
    } else if (kneeCSL === "s+") {
      gf = release_gf;
      gb = stall_gb;
    } else if (kneeCSL === "s-") {
      gf = release_gf;
      gb = -stall_gb;
    }
    if (change_select) {
      $("#csl_mode_knee option[value='" + kneeCSL + "']").attr('selected', true);
    }
    $("#gi_param_lower").val(gi_knee);
    this.physics.lower_joint.gi = gi_knee;
    $("#gf_param_lower").val(gf);
    this.physics.lower_joint.gf = gf;
    $("#gb_param_lower").val(gb);
    this.physics.lower_joint.gb = gb;
    return this.physics.lower_joint.csl_mode = kneeCSL;
  };

  ui.prototype.toggleRecorder = function() {
    this.physics.startLog = true;
    return this.physics.recordPhase = !this.physics.recordPhase;
  };

  ui.prototype.getLogfile = function() {
    this.physics.recordPhase = false;
    location.href = 'data:text;charset=utf-8,' + encodeURI(Functional.reduce(function(x, y) {
      return x + y + "\n";
    }, "", this.physics.logged_data));
  };

  ui.prototype.getSemniTransformAsJSON = function() {
    var t, t2, t3;
    t = this.physics.body.GetTransform();
    t2 = this.physics.body2.GetTransform();
    t3 = this.physics.body3.GetTransform();
    return JSON.stringify({
      "body": t,
      "body2": t2,
      "body3": t3
    });
  };

  ui.prototype.getSemniTransformAsFile = function() {
    return location.href = 'data:text;charset=utf-8,' + encodeURI(this.getSemniTransformAsJSON());
  };

  ui.prototype.setSemniTransformAsJSON = function(tj) {
    var t;
    if (tj == null) {
      tj = null;
    }
    t = JSON.parse(tj);
    this.physics.body.SetTransform(new b2Transform(t.body.position, t.body.R));
    this.physics.body2.SetTransform(new b2Transform(t.body2.position, t.body2.R));
    return this.physics.body3.SetTransform(new b2Transform(t.body3.position, t.body3.R));
  };

  ui.prototype.setSemniTransformAsFile = function(files) {
    var readFile;
    readFile = function(file, callback) {
      var reader;
      reader = new FileReader();
      reader.onload = function(evt) {
        if (typeof callback === "function") {
          return callback(file, evt);
        }
      };
      return reader.readAsBinaryString(file);
    };
    if (files.length > 0) {
      return readFile(files[0], function(file, evt) {
        return window.ui.setSemniTransformAsJSON(evt.target.result);
      });
    }
  };

  ui.prototype.getPostureGraphAsFile = function() {
    var svg;
    svg = $("#viewport_svg").clone();
    svg.find("defs").append("<style>\n   line {\n      stroke-width: 1;\n      stroke: black;\n      fill: none;\n  }\n\n  text {\n    font-family: Verdana; sans-serif;\n    font-size: 7pt;\n    text-anchor: middle;\n    fill: #333333;\n  } \n</style>");
    return location.href = 'data:text;charset=utf-8,' + encodeURI('<?xml version="1.0" encoding="UTF-8" standalone="no"?>' + svg.html());
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
