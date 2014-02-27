// Generated by CoffeeScript 1.3.3
(function() {
  var ui,
    __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

  ui = (function() {

    function ui(physics) {
      this.set_draw_edge_labels = __bind(this.set_draw_edge_labels, this);

      this.set_save_periodically = __bind(this.set_save_periodically, this);

      this.set_render_manifold = __bind(this.set_render_manifold, this);

      this.set_pause_explore = __bind(this.set_pause_explore, this);

      this.set_pause_layouting = __bind(this.set_pause_layouting, this);

      this.set_pause_drawing = __bind(this.set_pause_drawing, this);

      this.set_draw_semni = __bind(this.set_draw_semni, this);

      this.set_activation = __bind(this.set_activation, this);

      this.set_color_activation = __bind(this.set_color_activation, this);

      this.set_graph_animated = __bind(this.set_graph_animated, this);

      this.set_realtime = __bind(this.set_realtime, this);

      this.getSubmanifoldColor = __bind(this.getSubmanifoldColor, this);

      this.activation2color = __bind(this.activation2color, this);

      this.getPostureGraphAsFile = __bind(this.getPostureGraphAsFile, this);

      this.setSemniTransformAsFile = __bind(this.setSemniTransformAsFile, this);

      this.setSemniTransformAsJSON = __bind(this.setSemniTransformAsJSON, this);

      this.getSemniTransformAsFile = __bind(this.getSemniTransformAsFile, this);

      this.getSemniTransformAsJSON = __bind(this.getSemniTransformAsJSON, this);

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
      this.svg_scale = 200;
      this.realtime = false;
      this.canvas_old_x = this.canvas_old_y = 0;
      this.canvas_trans_x = this.canvas_trans_y = 0;
    }

    ui.prototype.update = function() {
      var ctx;
      if (this.draw_graphics && this.halftime) {
        if (this.canvas_trans_x !== 0 || this.canvas_trans_y !== 0) {
          ctx = this.physics.debugDraw.m_ctx;
          ctx.save();
          this.physics.debugDraw.m_sprite.graphics.clear();
          ctx.translate(this.canvas_trans_x, this.canvas_trans_y);
          this.canvas_old_x = window.mousePixelX;
          this.canvas_old_y = window.mousePixelY;
          this.physics.world.DrawDebugData();
          ctx.scale(1, -1);
          ctx.restore();
        } else {
          this.physics.world.DrawDebugData();
        }
        /*
              #draw semni as svg
              container = "#semni_svg"
              $(container).html("")
              @svg = d3.select(container).append("svg:svg")
                      .attr("width", 300)
                      .attr("height", 300)
                      .attr("xmlns", "http://www.w3.org/2000/svg")
              @semni = @getSemniOutlineSVG(
                @physics.body.GetPosition(),
                @physics.body.GetAngle(),
                @physics.body2.GetAngle(),
                @physics.body3.GetAngle(),
                @svg
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

    ui.prototype.getSemniOutlineSVG = function(body_pos, body_angle, arm1_angle, arm2_angle, container) {
      var arm1_joint, arm2_joint, d3line2, h_x, h_y, svg, svg_joint, svg_joint2, svg_semni_arm1, svg_semni_arm2, svg_semni_body, svg_semni_head;
      svg = container.append("svg:g");
      d3line2 = d3.svg.line().x(function(d) {
        return d.x * physics.ui.svg_scale;
      }).y(function(d) {
        return d.y * physics.ui.svg_scale;
      }).interpolate("linear");
      svg_semni_body = svg.append("svg:path").attr("d", d3line2(simni.contour_original_lowest_detail)).style("stroke-width", 1).style("fill", "none");
      svg_semni_arm1 = svg.append("svg:path").attr("d", d3line2(simni.arm1Contour)).style("stroke-width", 1).style("fill", "none");
      svg_joint = svg.append("svg:circle").attr("cx", 0).attr("cy", 0).attr("r", "1").style("stroke", "red");
      svg_semni_arm2 = svg.append("svg:path").attr("d", d3line2(simni.arm2Contour)).style("stroke-width", 1).style("fill", "none");
      svg_joint2 = svg.append("svg:circle").attr("cx", 0).attr("cy", 0).attr("r", "1").style("stroke", "red");
      svg_semni_head = svg.append("svg:circle").attr("cx", 0).attr("cy", 0).attr("r", simni.head2[1] * this.svg_scale).style("stroke-width", 1).style("fill", "none");
      svg_semni_body.attr("transform", "rotate(" + body_angle * 180 / Math.PI + ")");
      arm1_joint = new b2Vec2();
      arm1_joint.x = simni.arm1JointAnchor2.x * this.svg_scale;
      arm1_joint.y = simni.arm1JointAnchor2.y * this.svg_scale;
      arm1_joint = this.rotate_point(0, 0, body_angle, arm1_joint);
      svg_joint.attr("transform", "translate(" + arm1_joint.x + "," + arm1_joint.y + ")");
      svg_semni_arm1.attr("transform", "rotate(" + (arm1_angle + body_angle) * 180 / Math.PI + "," + arm1_joint.x + "," + arm1_joint.y + ") translate(" + arm1_joint.x + "," + arm1_joint.y + ")");
      arm2_joint = new b2Vec2();
      arm2_joint.x = simni.arm2JointAnchor2.x * this.svg_scale;
      arm2_joint.y = simni.arm2JointAnchor2.y * this.svg_scale;
      arm2_joint = this.rotate_point(0, 0, body_angle, arm2_joint);
      arm2_joint = this.rotate_point(arm1_joint.x, arm1_joint.y, arm1_angle - 0.85, arm2_joint);
      svg_joint2.attr("transform", "translate(" + arm2_joint.x + "," + arm2_joint.y + ")");
      svg_semni_arm2.attr("transform", "rotate(" + (body_angle + arm1_angle + arm2_angle) * 180 / Math.PI + "," + arm2_joint.x + "," + arm2_joint.y + ") translate(" + arm2_joint.x + "," + arm2_joint.y + ")");
      h_x = simni.head2[0].x * this.svg_scale;
      h_y = simni.head2[0].y * this.svg_scale;
      svg_semni_head.attr("transform", "rotate(" + body_angle * 180 / Math.PI + "," + 0 + "," + 0 + ") translate(" + h_x + "," + h_y + ")");
      return svg;
    };

    ui.prototype.init = function() {
      var canvas, getBodyCB, getElementPosition, handleMouseMove,
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
      window.stats = new Stats();
      window.stats.setMode(0);
      window.stats.domElement.style.position = "absolute";
      window.stats.domElement.style.left = "0px";
      window.stats.domElement.style.top = "0px";
      document.body.appendChild(window.stats.domElement);
      this.set_realtime(this.realtime);
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
      window.canvasPosition = canvas.getBoundingClientRect();
      canvas.addEventListener("mousedown", (function(e) {
        window.isMouseDown = true;
        handleMouseMove(e);
        physics.ui.canvas_old_x = mousePixelX;
        physics.ui.canvas_old_y = mousePixelY;
        return document.addEventListener("mousemove", handleMouseMove, true);
      }), true);
      document.addEventListener("mouseup", (function() {
        document.removeEventListener("mousemove", handleMouseMove, true);
        window.isMouseDown = false;
        window.mouseX = void 0;
        return window.mouseY = void 0;
      }), true);
      handleMouseMove = function(e) {
        window.mouseX = (e.clientX - canvasPosition.left + window.scrollX - physics.ui.canvas_trans_x) / physics.debugDraw.GetDrawScale();
        window.mouseY = (e.clientY - canvasPosition.top + window.scrollY - physics.ui.canvas_trans_y) / physics.debugDraw.GetDrawScale();
        window.mousePixelX = e.clientX - canvasPosition.left;
        return window.mousePixelY = e.clientY - canvasPosition.top;
      };
      window.getBodyAtMouse = function() {
        var aabb;
        window.mousePVec = new b2Vec2(mouseX, mouseY);
        aabb = new Box2D.Collision.b2AABB();
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
        return mode * 0.1;
      } else {
        return 0.7 + (0.1 * mode);
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
        this.physics.lower_joint.m_maxMotorTorque = beta * 1.5;
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
      x0 = 0.6;
      y0 = 0.4;
      return p.createSemni(x0, y0);
    };

    ui.prototype.set_csl_mode_upper = function(hipCSL, change_select) {
      var contract_gf_hip, contract_gi, gb, gf, gi, release_bias_hip, release_gf, release_gi, stall_gb, stall_gf;
      if (change_select == null) {
        change_select = true;
      }
      release_bias_hip = 0.04;
      release_gf = 0;
      release_gi = 0;
      contract_gf_hip = 1.01;
      contract_gi = 8;
      stall_gb = 0.2;
      stall_gf = 0;
      if (hipCSL === "r+") {
        gf = release_gf;
        gb = release_bias_hip;
        gi = release_gi;
        this.physics.upper_joint.csl_prefill = 0.01;
      } else if (hipCSL === "r-") {
        gf = release_gf;
        gb = -release_bias_hip;
        gi = release_gi;
        this.physics.upper_joint.csl_prefill = -0.01;
      } else if (hipCSL === "c") {
        gf = contract_gf_hip;
        gb = 0;
        gi = contract_gi;
        this.physics.upper_joint.last_integrated = this.physics.upper_joint.csl_prefill;
      } else if (hipCSL === "s+") {
        gf = stall_gf;
        gb = stall_gb;
        gi = release_gi;
        this.physics.upper_joint.last_integrated = 0;
      } else if (hipCSL === "s-") {
        gf = stall_gf;
        gb = -stall_gb;
        gi = release_gi;
        this.physics.upper_joint.last_integrated = 0;
      }
      if (change_select) {
        $("#csl_mode_hip option[value='" + hipCSL + "']").attr("selected", true);
      }
      $("#gi_param_upper").val(gi);
      this.physics.upper_joint.gi = gi;
      $("#gf_param_upper").val(gf);
      this.physics.upper_joint.gf = gf;
      $("#gb_param_upper").val(gb);
      this.physics.upper_joint.gb = gb;
      this.physics.upper_joint.csl_mode = hipCSL;
      return this.physics.abc.manual_noop = false;
    };

    ui.prototype.set_csl_mode_lower = function(kneeCSL, change_select) {
      var contract_gf_knee, contract_gi, gb, gf, gi, release_bias_knee, release_gf, release_gi, stall_gb, stall_gf;
      if (change_select == null) {
        change_select = true;
      }
      release_bias_knee = 0.04;
      release_gf = 0;
      release_gi = 0;
      contract_gf_knee = 1.03;
      contract_gi = 32;
      stall_gb = 0.2;
      stall_gf = 0;
      if (kneeCSL === "r+") {
        gf = release_gf;
        gb = release_bias_knee;
        gi = release_gi;
        this.physics.lower_joint.csl_prefill = 0.01;
      } else if (kneeCSL === "r-") {
        gf = release_gf;
        gb = -release_bias_knee;
        gi = release_gi;
        this.physics.lower_joint.csl_prefill = -0.01;
      } else if (kneeCSL === "c") {
        gf = contract_gf_knee;
        gb = 0;
        gi = contract_gi;
        this.physics.lower_joint.last_integrated = this.physics.lower_joint.csl_prefill;
      } else if (kneeCSL === "s+") {
        gf = stall_gf;
        gb = stall_gb;
        gi = release_gi;
        this.physics.lower_joint.last_integrated = 0;
      } else if (kneeCSL === "s-") {
        gf = stall_gf;
        gb = -stall_gb;
        gi = release_gi;
        this.physics.lower_joint.last_integrated = 0;
      }
      if (change_select) {
        $("#csl_mode_knee option[value='" + kneeCSL + "']").attr('selected', true);
      }
      $("#gi_param_lower").val(gi);
      this.physics.lower_joint.gi = gi;
      $("#gf_param_lower").val(gf);
      this.physics.lower_joint.gf = gf;
      $("#gb_param_lower").val(gb);
      this.physics.lower_joint.gb = gb;
      this.physics.lower_joint.csl_mode = kneeCSL;
      if (physics.abc.mode_strategy === "manual") {
        physics.abc.trajectory = [];
      }
      return this.physics.abc.manual_noop = false;
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
      /*
          svg.find("defs").append """
          <style>
             line {
                stroke-width: 1;
                stroke: black;
                fill: none;
            }
          </style>
          """
      */

      return location.href = 'data:text;charset=utf-8,' + encodeURI('<?xml version="1.0" encoding="UTF-8" standalone="no"?>' + svg.html());
    };

    ui.prototype.activation2color = function(value) {
      var c, color, h, l;
      if (value > 1.0) {
        value = 1.0;
      }
      l = 80;
      c = 210;
      h = 40 + ((1 - value) * (140 - 40));
      color = new Color(l, c, h, ColorMode.CIELCh);
      return color.getHex();
    };

    ui.prototype.getSubmanifoldColor = function(id) {
      var c;
      switch (id) {
        case 1:
          c = [255, 150, 0];
          break;
        case 2:
          c = [0, 195, 80];
          break;
        case 3:
          c = [0, 173, 244];
          break;
        case 4:
          c = [197, 0, 169];
          break;
        default:
          c = [0, 0, 0];
      }
      return "rgb(" + c[0] + "," + c[1] + "," + c[2] + ")";
    };

    ui.prototype.set_realtime = function(value) {
      this.realtime = value;
      if (this.realtime) {
        clearInterval(this.realtime_timer);
        return physics.update();
      } else {
        return this.realtime_timer = setInterval(this.physics.update, 1);
      }
    };

    ui.prototype.set_graph_animated = function(value) {
      physics.abc.graph.renderer.draw_graph_animated = value;
      return physics.abc.graph.renderer.redraw();
    };

    ui.prototype.set_color_activation = function(value) {
      physics.abc.graph.renderer.draw_color_activation = value;
      return physics.abc.graph.renderer.redraw();
    };

    ui.prototype.set_activation = function(value) {
      physics.abc.graph.renderer.draw_activation = value;
      return physics.abc.graph.renderer.redraw();
    };

    ui.prototype.set_draw_semni = function(value) {
      physics.abc.graph.renderer.draw_semni = value;
      return physics.abc.graph.renderer.redraw();
    };

    ui.prototype.set_pause_drawing = function(value) {
      return physics.abc.graph.renderer.pause_drawing = value;
    };

    ui.prototype.set_pause_layouting = function(value) {
      physics.abc.graph.renderer.pause_layout = value;
      if (value) {
        return physics.abc.graph.stop();
      } else {
        return physics.abc.graph.start(true);
      }
    };

    ui.prototype.set_pause_explore = function() {
      physics.run = !physics.run;
      physics.update();
      if (physics.run) {
        return console.log("unpause explore at " + new Date);
      } else {
        return console.log("pause explore at " + new Date);
      }
    };

    ui.prototype.set_render_manifold = function(value) {
      manifoldRenderer.do_render = value;
      if (value) {
        return manifoldRenderer.animate();
      }
    };

    ui.prototype.set_save_periodically = function(value) {
      return physics.abc.save_periodically = value;
    };

    ui.prototype.set_draw_edge_labels = function(value) {
      physics.abc.graph.renderer.draw_edge_labels = value;
      return physics.abc.graph.renderer.redraw();
    };

    return ui;

  })();

  window.requestAnimFrame = (function() {
    return window.oRequestAnimationFrame || window.webkitRequestAnimationFrame || window.mozRequestAnimationFrame || window.oRequestAnimationFrame || window.msRequestAnimationFrame || function(callback, element) {
      return window.setTimeout(callback, 1000 / 60);
    };
  })();

  $(function() {
    var logging, p;
    p = new simni.Physics();
    simni.Ui = ui;
    ui = new ui(p);
    p.ui = ui;
    logging = new simni.Logging(p);
    window.physics = p;
    window.ui = ui;
    return window.logging = logging;
  });

}).call(this);
