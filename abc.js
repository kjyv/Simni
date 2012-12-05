// Generated by CoffeeScript 1.3.3
var abc, posture, postureGraph, transition,
  __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

posture = (function() {
  var eps;

  function posture(position, csl_mode, x_pos) {
    if (csl_mode == null) {
      csl_mode = [];
    }
    if (x_pos == null) {
      x_pos = 0;
    }
    this.isClose = __bind(this.isClose, this);

    this.getEdgeTo = __bind(this.getEdgeTo, this);

    this.isEqualTo = __bind(this.isEqualTo, this);

    this.position = position;
    this.csl_mode = csl_mode;
    this.edges_out = [];
    this.body_x = x_pos;
    this.length = 1;
    this.number = -1;
  }

  posture.prototype.isEqualTo = function(node) {
    return this.position[0] === node.position[0] && this.position[1] === node.position[1] && this.position[2] === node.position[2] && this.csl_mode[0] === node.csl_mode[0] && this.csl_mode[1] === node.csl_mode[1];
  };

  posture.prototype.getEdgeTo = function(target) {
    var edge, _i, _len, _ref;
    _ref = this.edges_out;
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      edge = _ref[_i];
      if (edge.target_node === target) {
        return edge;
      }
    }
  };

  eps = 0.6;

  posture.prototype.isClose = function(a, i, b, j) {
    return Math.abs(a.position[0] - b[j].position[0]) < eps && Math.abs(a.position[1] - b[j].position[1]) < eps && Math.abs(a.position[2] - b[j].position[2]) < eps && a.csl_mode[0] === b[j].csl_mode[0] && a.csl_mode[1] === b[j].csl_mode[1];
  };

  return posture;

})();

transition = (function() {

  function transition(start_node, target_node) {
    this.isInList = __bind(this.isInList, this);

    this.toString = __bind(this.toString, this);
    this.start_node = start_node;
    this.target_node = target_node;
    this.distance = 0;
  }

  transition.prototype.toString = function() {
    return this.start_node + "->" + this.target_node;
  };

  transition.prototype.isInList = function(list) {
    var t, _i, _len;
    for (_i = 0, _len = list.length; _i < _len; _i++) {
      t = list[_i];
      if (this.start_node === t.start_node && this.target_node === t.target_node) {
        return true;
      }
    }
    return false;
  };

  return transition;

})();

postureGraph = (function() {

  function postureGraph() {
    this.walkCircle = __bind(this.walkCircle, this);

    this.findElementaryCircles = __bind(this.findElementaryCircles, this);

    this.length = __bind(this.length, this);

    this.getNode = __bind(this.getNode, this);

    this.addNode = __bind(this.addNode, this);
    this.nodes = [];
    this.walk_circle_active = false;
  }

  postureGraph.prototype.addNode = function(node) {
    node.number = this.nodes.length;
    return this.nodes.push(node);
  };

  postureGraph.prototype.getNode = function(index) {
    return this.nodes[index];
  };

  postureGraph.prototype.length = function() {
    return this.nodes.length;
  };

  postureGraph.prototype.findElementaryCircles = function() {
    var A, a, backtrack, circles, edge, i, marked, marked_stack, node, num, parent, point_stack, s, u, _i, _j, _k, _l, _len, _m, _ref, _ref1, _ref2, _ref3, _ref4;
    A = [];
    for (a = _i = 0, _ref = this.nodes.length - 1; 0 <= _ref ? _i <= _ref : _i >= _ref; a = 0 <= _ref ? ++_i : --_i) {
      A.push([]);
    }
    for (num = _j = 0, _ref1 = this.nodes.length - 1; 0 <= _ref1 ? _j <= _ref1 : _j >= _ref1; num = 0 <= _ref1 ? ++_j : --_j) {
      node = this.nodes[num];
      if (!node) {
        break;
      }
      _ref2 = node.edges_out;
      for (_k = 0, _len = _ref2.length; _k < _len; _k++) {
        edge = _ref2[_k];
        A[num].push(edge.target_node.number);
      }
    }
    point_stack = [];
    marked = {};
    marked_stack = [];
    circles = [];
    parent = this;
    backtrack = function(v) {
      var d, f, m, n, path, u, w, _l, _len1, _m, _ref3, _ref4;
      f = false;
      point_stack.push(v);
      marked[v] = true;
      marked_stack.push(v);
      _ref3 = A[v];
      for (_l = 0, _len1 = _ref3.length; _l < _len1; _l++) {
        w = _ref3[_l];
        if (w < s) {
          A[w] = 0;
        } else if (w === s) {
          path = [];
          d = 0;
          for (n = _m = 1, _ref4 = point_stack.length; 1 <= _ref4 ? _m <= _ref4 : _m >= _ref4; n = 1 <= _ref4 ? ++_m : --_m) {
            if (n === point_stack.length) {
              m = n - 1;
              n = 0;
            } else {
              m = n - 1;
            }
            edge = parent.nodes[point_stack[m]].getEdgeTo(parent.nodes[point_stack[n]]);
            path.push(edge);
            d += edge.distance;
          }
          path = path.concat([d]);
          circles.push(path);
          f = true;
        } else if (!marked[w]) {
          f = backtrack(w) || f;
        }
      }
      if (f) {
        while (marked_stack.slice(-1)[0] !== v) {
          u = marked_stack.pop();
          marked[u] = false;
        }
        marked_stack.pop();
        marked[v] = false;
      }
      point_stack.pop();
      return f;
    };
    for (i = _l = 0, _ref3 = A.length - 1; 0 <= _ref3 ? _l <= _ref3 : _l >= _ref3; i = 0 <= _ref3 ? ++_l : --_l) {
      marked[i] = false;
    }
    for (s = _m = 0, _ref4 = A.length - 1; 0 <= _ref4 ? _m <= _ref4 : _m >= _ref4; s = 0 <= _ref4 ? ++_m : --_m) {
      backtrack(s);
      while (marked_stack.length) {
        u = marked_stack.pop();
        marked[u] = false;
      }
    }
    return this.circles = circles.sort(function(a, b) {
      if (a.slice(-1)[0] <= b.slice(-1)[0]) {
        return -1;
      } else {
        return 1;
      }
    });
  };

  postureGraph.prototype.walkCircle = function() {
    if (this.circles) {
      p.abc.explore_active = false;
      this.best_circle = this.circles.slice(-1)[0];
      this.walk_circle_active = true;
      return this.best_circle[0].active = true;
    }
  };

  return postureGraph;

})();

abc = (function() {
  var MAX_UNIX_TIME, time, trajectory;

  function abc() {
    this.update = __bind(this.update, this);

    this.limitCSL = __bind(this.limitCSL, this);

    this.newCSLMode = __bind(this.newCSLMode, this);

    this.compareModes = __bind(this.compareModes, this);

    this.savePosture = __bind(this.savePosture, this);

    this.detectAttractor = __bind(this.detectAttractor, this);

    this.searchSubarray = __bind(this.searchSubarray, this);

    this.toggleExplore = __bind(this.toggleExplore, this);
    this.posture_graph = new postureGraph();
    this.last_posture = null;
    this.explore_active = false;
    this.graph = arbor.ParticleSystem();
    this.graph.parameters({
      repulsion: 5000,
      stiffness: 100,
      friction: .5,
      gravity: true
    });
    this.graph.renderer = new Renderer("#viewport", this.graph);
  }

  abc.prototype.toggleExplore = function() {
    if (!physics.upper_joint.csl_active) {
      $("#toggle_csl").click();
    }
    return this.explore_active = !this.explore_active;
  };

  abc.prototype.searchSubarray = function(sub, array, cmp) {
    var found, i, j, _i, _j, _ref, _ref1;
    found = [];
    for (i = _i = 0, _ref = array.length - sub.length; _i <= _ref; i = _i += 1) {
      for (j = _j = 0, _ref1 = sub.length - 1; _j <= _ref1; j = _j += 1) {
        if (!cmp(sub, j, array, i + j)) {
          break;
        }
      }
      if (j === sub.length) {
        found.push(i);
        i = _i = i + sub.length;
      }
    }
    if (found.length === 0) {
      return false;
    } else {
      return found;
    }
  };

  MAX_UNIX_TIME = 1924988399;

  time = MAX_UNIX_TIME;

  trajectory = [];

  abc.prototype.detectAttractor = function(body, upper_joint, lower_joint, action) {
    var d, eps, last, p_body, p_hip, p_knee, position;
    if (!physics.run) {
      return;
    }
    p_body = Math.abs(body.GetAngle());
    p_hip = Math.abs(upper_joint.GetJointAngle());
    p_knee = Math.abs(lower_joint.GetJointAngle());
    if (trajectory.length === 3000) {
      trajectory.shift();
    }
    trajectory.push([p_body, p_hip, p_knee]);
    if (trajectory.length > 200 && (Date.now() - time) > 2000) {
      last = trajectory.slice(-40);
      eps = 0.025;
      d = this.searchSubarray(last, trajectory, function(a, i, b, j) {
        return Math.abs(a[i][0] - b[j][0]) < eps && Math.abs(a[i][1] - b[j][1]) < eps && Math.abs(a[i][2] - b[j][2]) < eps;
      });
      if (d.length > 4) {
        position = trajectory.pop();
        action(position, this);
        trajectory = [];
      }
      time = Date.now();
    }
    if ((Date.now() - this.graph.renderer.click_time) > 5000) {
      this.graph.renderer.click_time = Date.now();
      return this.graph.stop();
    }
  };

  abc.prototype.savePosture = function(position, body, upper_csl, lower_csl) {
    var addEdge, ctx, ctx2, f, found, i, imageData, newCanvas, p, parent, pix, range, x, y, _i, _ref;
    parent = this;
    addEdge = function(start_node, target_node, edge_list) {
      var current_node, distance, edge, n0, n1;
      if (edge_list == null) {
        edge_list = start_node.edges_out;
      }
      edge = new transition(start_node, target_node);
      if (!edge.isInList(edge_list) && parent.posture_graph.length() > 1 && !start_node.isEqualTo(target_node)) {
        console.log("adding edge from posture " + start_node.number + " to posture: " + target_node.number);
        distance = target_node.body_x - start_node.body_x;
        edge.distance = distance;
        edge_list.push(edge);
        n0 = start_node.position.toString();
        n1 = target_node.position.toString();
        parent.graph.addEdge(n0, n1, {
          "distance": distance.toFixed(3)
        });
        parent.graph.current_node = current_node = parent.graph.getNode(n1);
        current_node.data.label = target_node.csl_mode;
        current_node.data.number = target_node.number;
        parent.graph.start(true);
        return parent.graph.renderer.click_time = Date.now();
      }
    };
    p = new posture(position, [upper_csl.csl_mode, lower_csl.csl_mode], body.GetWorldCenter().x);
    found = this.searchSubarray(p, this.posture_graph.nodes, p.isClose);
    if (!found) {
      console.log("found new pose/attractor: " + p.position);
      this.posture_graph.addNode(p);
    } else {
      f = found[0];
      p = this.posture_graph.getNode(f);
    }
    if (this.last_posture && this.posture_graph.length() > 1) {
      p.body_x = body.GetWorldCenter().x;
      addEdge(this.last_posture, p);
      ctx = $("#simulation canvas")[0].getContext('2d');
      x = physics.body.GetWorldCenter().x * physics.debugDraw.GetDrawScale();
      y = physics.body.GetWorldCenter().y * physics.debugDraw.GetDrawScale();
      range = 120;
      imageData = ctx.getImageData(x - range, y - range, range * 2, range * 2);
      pix = imageData.data;
      for (i = _i = 0, _ref = pix.length - 4; 0 <= _ref ? _i <= _ref : _i >= _ref; i = 0 <= _ref ? ++_i : --_i) {
        if (pix[i] === 255 && pix[i + 1] === 255 && pix[i + 2] === 255) {
          pix[i + 4] = 0;
        }
      }
      newCanvas = $("<canvas>").attr("width", imageData.width).attr("height", imageData.height)[0];
      ctx = newCanvas.getContext("2d");
      ctx.putImageData(imageData, 0, 0);
      ctx2 = $("#tempimage")[0].getContext('2d');
      ctx2.clearRect(0, 0, ctx2.canvas.width, ctx2.canvas.height);
      ctx2.scale(0.5, 0.5);
      ctx2.drawImage(newCanvas, 0, 0);
      this.graph.getNode(p.position.toString()).data.imageData = ctx2.getImageData(0, 0, range * 2, range * 2);
      ctx2.scale(2, 2);
    }
    this.last_posture = p;
    return this.newCSLMode();
  };

  abc.prototype.compareModes = function(a, b) {
    return a[0] === b[0] && a[1] === b[1];
  };

  abc.prototype.newCSLMode = function() {
    var mode, which;
    which = Math.floor(Math.random() * 2);
    if (which) {
      while (true) {
        mode = ["r+", "r-", "c"][Math.floor(Math.random() * 2.99)];
        if (this.last_posture.csl_mode[0] !== mode) {
          break;
        }
      }
      return ui.set_csl_mode_upper(mode);
    } else {
      while (true) {
        mode = ["r+", "r-", "c"][Math.floor(Math.random() * 2.99)];
        if (this.last_posture.csl_mode[1] !== mode) {
          break;
        }
      }
      return ui.set_csl_mode_lower(["r+", "r-", "c"][Math.floor(Math.random() * 2.99)]);
    }
  };

  abc.prototype.limitCSL = function(upper_joint, lower_joint) {
    var limit, mc;
    if (upper_joint.csl_active && upper_joint.csl_mode === "c") {
      mc = upper_joint.motor_control;
      limit = 20;
      if (Math.abs(mc) > limit) {
        if (mc > limit) {
          ui.set_csl_mode_upper("r+", false);
          $("#gb_param_upper").val(limit);
          physics.upper_joint.gb = limit;
        } else if (mc < -limit) {
          ui.set_csl_mode_upper("r-", false);
          $("#gb_param_upper").val(-limit);
          physics.upper_joint.gb = -limit;
        }
        upper_joint.csl_mode = "c";
      }
    }
    if (lower_joint.csl_active && lower_joint.csl_mode === "c") {
      mc = lower_joint.motor_control;
      if (Math.abs(mc) > limit) {
        if (mc > limit) {
          ui.set_csl_mode_lower("r+", false);
          $("#gb_param_lower").val(limit);
          physics.lower_joint.gb = limit;
        } else if (mc < -limit) {
          ui.set_csl_mode_lower("r-", false);
          $("#gb_param_lower").val(-limit);
          physics.lower_joint.gb = -limit;
        }
        return lower_joint.csl_mode = "c";
      }
    }
  };

  abc.prototype.update = function(body, upper_joint, lower_joint) {
    this.limitCSL(upper_joint, lower_joint);
    if (this.explore_active) {
      this.detectAttractor(body, upper_joint, lower_joint, function(position, parent) {
        return parent.savePosture(position, body, upper_joint, lower_joint);
      });
    }
    if (this.posture_graph.walk_circle_active) {
      return this.detectAttractor(body, upper_joint, lower_joint, function(position, parent) {
        var csl_mode, current_posture, edge, _i, _len, _ref, _results;
        _ref = parent.posture_graph.best_circle;
        _results = [];
        for (_i = 0, _len = _ref.length; _i < _len; _i++) {
          edge = _ref[_i];
          current_posture = new posture(position, [physics.upper_joint.csl_mode, physics.lower_joint.csl_mode]);
          if (edge.start_node.isClose(current_posture, 0, [edge.start_node], 0)) {
            edge.active = true;
            parent.last_posture = edge.start_node;
          }
          if (edge.target_node.isClose(current_posture, 0, [edge.target_node], 0)) {
            edge.active = false;
          }
          if (edge.active) {
            csl_mode = edge.target_node.csl_mode;
            ui.set_csl_mode_upper(csl_mode[0]);
            ui.set_csl_mode_lower(csl_mode[1]);
            break;
          } else {
            _results.push(void 0);
          }
        }
        return _results;
      });
    }
  };

  return abc;

})();
