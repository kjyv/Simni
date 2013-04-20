// Generated by CoffeeScript 1.3.3
var abc, posture, postureGraph, transition,
  __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; },
  __indexOf = [].indexOf || function(item) { for (var i = 0, l = this.length; i < l; i++) { if (i in this && this[i] === item) return i; } return -1; };

if (typeof Array.prototype.clone !== 'function') {
  Array.prototype.clone = function() {
    var cloned, i, _i, _len;
    cloned = [];
    for (_i = 0, _len = this.length; _i < _len; _i++) {
      i = this[_i];
      cloned.push(i);
    }
    return cloned;
  };
}

if (typeof String.prototype.startsWith !== 'function') {
  String.prototype.startsWith = function(input) {
    return this.substring(0, input.length) === input;
  };
}

posture = (function() {
  var e;

  function posture(configuration, csl_mode, x_pos, timestamp) {
    if (csl_mode == null) {
      csl_mode = [];
    }
    if (x_pos == null) {
      x_pos = 0;
    }
    if (timestamp == null) {
      timestamp = Date.now();
    }
    this.isCloseExplore = __bind(this.isCloseExplore, this);

    this.isClose = __bind(this.isClose, this);

    this.isEqualTo = __bind(this.isEqualTo, this);

    this.getEdgeFrom = __bind(this.getEdgeFrom, this);

    this.getEdgeTo = __bind(this.getEdgeTo, this);

    this.asJSON = __bind(this.asJSON, this);

    this.name = -99;
    this.csl_mode = csl_mode;
    this.configuration = configuration;
    this.positions = [];
    this.world_angles = [];
    this.body_x = x_pos;
    this.timestamp = timestamp;
    this.edges_out = [];
    this.edges_in = [];
    this.exit_directions = [0, 0, 0, 0];
    this.length = 1;
    this.activation = 1;
  }

  posture.prototype.asJSON = function() {
    var replacer;
    replacer = function(edges) {
      var e, new_edges, _i, _len;
      new_edges = [];
      for (_i = 0, _len = edges.length; _i < _len; _i++) {
        e = edges[_i];
        new_edges.push(e.target_node.name);
      }
      return new_edges;
    };
    return JSON.stringify({
      "name": this.name,
      "csl_mode": this.csl_mode,
      "configuration": this.configuration,
      "positions": this.positions,
      "world_angles": this.world_angles,
      "body_x": this.body_x,
      "timestamp": this.timestamp,
      "exit_directions": this.exit_directions,
      "activation": this.activation,
      "edges_out": replacer(this.edges_out)
    }, null, 4);
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

  posture.prototype.getEdgeFrom = function(source) {
    var edge, _i, _len, _ref;
    _ref = source.edges_out;
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      edge = _ref[_i];
      if (edge.target_node === this) {
        return edge;
      }
    }
  };

  posture.prototype.isEqualTo = function(node) {
    return this.configuration[0] === node.configuration[0] && this.configuration[1] === node.configuration[1] && this.configuration[2] === node.configuration[2] && this.csl_mode[0] === node.csl_mode[0] && this.csl_mode[1] === node.csl_mode[1];
  };

  posture.prototype.isClose = function(a, b, eps) {
    if (b == null) {
      b = this;
    }
    if (eps == null) {
      eps = 0.25;
    }
    return Math.abs(a.configuration[0] - b.configuration[0]) < eps && Math.abs(a.configuration[1] - b.configuration[1]) < eps && Math.abs(a.configuration[2] - b.configuration[2]) < eps && a.csl_mode[0] === b.csl_mode[0] && a.csl_mode[1] === b.csl_mode[1];
  };

  e = 0.4;

  posture.prototype.isCloseExplore = function(a, i, b, j, eps) {
    if (eps == null) {
      eps = e;
    }
    return Math.abs(a.configuration[0] - b[j].configuration[0]) < eps && Math.abs(a.configuration[1] - b[j].configuration[1]) < eps && Math.abs(a.configuration[2] - b[j].configuration[2]) < eps && a.csl_mode[0] === b[j].csl_mode[0] && a.csl_mode[1] === b[j].csl_mode[1];
  };

  return posture;

})();

transition = (function() {

  function transition(start_node, target_node) {
    this.isInList = __bind(this.isInList, this);

    this.asJSON = __bind(this.asJSON, this);

    this.toString = __bind(this.toString, this);
    this.start_node = start_node;
    this.target_node = target_node;
    this.distance = 0;
    this.timedelta = 0;
    this.csl_mode = [];
  }

  transition.prototype.toString = function() {
    return this.start_node.name + "->" + this.target_node.name;
  };

  transition.prototype.asJSON = function() {
    return JSON.stringify({
      "name": this.toString(),
      "csl_mode": this.csl_mode,
      "start_node": this.start_node.name,
      "target_node": this.target_node.name,
      "distance": this.distance,
      "timedelta": this.timedelta
    }, null, 4);
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

  function postureGraph(arborGraph) {
    this.diffuseLearnProgress = __bind(this.diffuseLearnProgress, this);

    this.walkCircle = __bind(this.walkCircle, this);

    this.findElementaryCircles = __bind(this.findElementaryCircles, this);

    this.loadGraphFromFile = __bind(this.loadGraphFromFile, this);

    this.populateGraphFromJSON = __bind(this.populateGraphFromJSON, this);

    this.saveGaphToFile = __bind(this.saveGaphToFile, this);

    this.length = __bind(this.length, this);

    this.getNode = __bind(this.getNode, this);

    this.addNode = __bind(this.addNode, this);
    this.nodes = [];
    this.walk_circle_active = false;
    this.arborGraph = arborGraph;
  }

  postureGraph.prototype.addNode = function(node) {
    node.name = this.nodes.length + 1;
    this.nodes.push(node);
    return node.name;
  };

  postureGraph.prototype.getNode = function(index) {
    return this.nodes[index];
  };

  postureGraph.prototype.length = function() {
    return this.nodes.length;
  };

  postureGraph.prototype.saveGaphToFile = function() {
    var e, edges, edges_as_string, graph_as_string, n, _i, _j, _k, _len, _len1, _len2, _ref, _ref1;
    graph_as_string = "";
    edges = [];
    _ref = this.nodes;
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      n = _ref[_i];
      graph_as_string += "\n" + n.asJSON() + ",";
      _ref1 = n.edges_out;
      for (_j = 0, _len1 = _ref1.length; _j < _len1; _j++) {
        e = _ref1[_j];
        edges.push(e);
      }
    }
    edges_as_string = "";
    for (_k = 0, _len2 = edges.length; _k < _len2; _k++) {
      e = edges[_k];
      edges_as_string += "\n" + e.asJSON() + ",";
    }
    graph_as_string = graph_as_string.substring(0, graph_as_string.length - 1);
    edges_as_string = edges_as_string.substring(0, edges_as_string.length - 1);
    return location.href = 'data:text;charset=utf-8,' + encodeURI("{\n" + "\"nodes\": [" + graph_as_string + "],\n" + "\"edges\": [" + edges_as_string + "]" + "\n}");
  };

  postureGraph.prototype.populateGraphFromJSON = function(tj) {
    var ag, e, ee, n, nn, source_node, t, target_node, _i, _j, _k, _len, _len1, _len2, _ref, _ref1, _ref2, _results;
    if (tj == null) {
      tj = null;
    }
    tj = tj.replace(/(\r\n|\n|\r)/gm, "");
    t = JSON.parse(tj);
    this.nodes = [];
    _ref = t.nodes;
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      n = _ref[_i];
      nn = new posture(n.configuration, n.csl_mode, n.body_x, n.timestamp);
      nn.name = n.name;
      nn.activation = n.activation;
      nn.exit_directions = n.exit_directions;
      nn.positions = n.positions;
      nn.world_angles = n.world_angles;
      this.nodes.push(nn);
    }
    _ref1 = t.edges;
    for (_j = 0, _len1 = _ref1.length; _j < _len1; _j++) {
      e = _ref1[_j];
      n = this.getNode(e.start_node);
      nn = this.getNode(e.target_node);
      ee = new transition(n, nn);
      ee.csl_mode = e.csl_mode;
      ee.distance = e.distance;
      ee.timedelta = e.timedelta;
      n.edges_out.push(ee);
    }
    ag = this.arborGraph;
    this.arborGraph.prune();
    this.arborGraph.renderer.svg_nodes = {};
    this.arborGraph.renderer.svg_edges = {};
    $("#viewport_svg svg g").remove();
    $("#viewport_svg svg rect").remove();
    $("#viewport_svg svg text").remove();
    $("#viewport_svg svg line").remove();
    _ref2 = this.nodes;
    _results = [];
    for (_k = 0, _len2 = _ref2.length; _k < _len2; _k++) {
      n = _ref2[_k];
      _results.push((function() {
        var _l, _len3, _ref3, _results1;
        _ref3 = n.edges_out;
        _results1 = [];
        for (_l = 0, _len3 = _ref3.length; _l < _len3; _l++) {
          e = _ref3[_l];
          nn = e.target_node;
          ag.addEdge(n.name, nn.name, {
            "label": e.csl_mode,
            "distance": e.distance,
            "timedelta": e.timedelta
          });
          source_node = ag.getNode(n.name);
          target_node = ag.getNode(nn.name);
          source_node.data = {
            label: n.csl_mode,
            number: n.name,
            activation: n.activation,
            configuration: n.configuration,
            positions: n.positions,
            world_angles: n.world_angles
          };
          _results1.push(target_node.data = {
            label: nn.csl_mode,
            number: nn.name,
            activation: nn.activation,
            configuration: nn.configuration,
            positions: nn.positions,
            world_angles: nn.world_angles
          });
        }
        return _results1;
      })());
    }
    return _results;
  };

  postureGraph.prototype.loadGraphFromFile = function(files) {
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
      readFile(files[0], function(file, evt) {
        return physics.abc.posture_graph.populateGraphFromJSON(evt.target.result);
      });
    }
    this.arborGraph.renderer.pause_drawing = false;
    $("#graph_pause_drawing").attr('checked', false);
    this.arborGraph.start(true);
    this.arborGraph.renderer.click_time = Date.now();
    return this.arborGraph.renderer.redraw();
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
        A[num].push(edge.target_node.name);
      }
    }
    point_stack = [];
    marked = {};
    marked_stack = [];
    circles = [];
    parent = this;
    backtrack = function(v) {
      var d, f, m, n, path, t, u, w, _l, _len1, _m, _ref3, _ref4;
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
          t = 0;
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
            t += edge.timedelta;
          }
          path = path.concat([d, t, (d / t) * 1000]);
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
      if (this.walk_circle_active) {
        this.walk_circle_active = false;
        this.best_circle.length = 0;
        return this.best_circle = void 0;
      } else {
        physics.abc.explore_active = false;
        this.best_circle = this.circles.slice(-1)[0];
        this.walk_circle_active = true;
        this.best_circle[0].active = true;
        return physics.abc.graph.renderer.redraw();
      }
    }
  };

  postureGraph.prototype.diffuseLearnProgress = function() {
    var activation_in, divisor, e, node, _i, _j, _k, _len, _len1, _len2, _ref, _ref1, _ref2;
    if (!(this.nodes.length > 1)) {
      return;
    }
    _ref = this.nodes;
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      node = _ref[_i];
      if (__indexOf.call(node.csl_mode[0], "s") >= 0 && __indexOf.call(node.csl_mode[1], "s") >= 0) {
        divisor = 0.5;
      } else if (__indexOf.call(node.csl_mode[0], "s") >= 0 || __indexOf.call(node.csl_mode[1], "s") >= 0) {
        divisor = 1 / 3;
      } else {
        divisor = 0.25;
      }
      activation_in = 0;
      node.activation_self = divisor * node.exit_directions.reduce((function(x, y) {
        if (y === 0) {
          return x + 1;
        } else {
          return x;
        }
      }), 0);
      if (node.edges_out.length) {
        _ref1 = node.edges_out;
        for (_j = 0, _len1 = _ref1.length; _j < _len1; _j++) {
          e = _ref1[_j];
          activation_in += e.target_node.activation;
        }
        activation_in /= node.edges_out.length;
      }
      node.activation_tmp = node.activation_self * 0.7 + activation_in * 0.3;
    }
    _ref2 = this.nodes;
    for (_k = 0, _len2 = _ref2.length; _k < _len2; _k++) {
      node = _ref2[_k];
      node.activation = node.activation_tmp;
      this.arborGraph.getNode(node.name).data.activation = node.activation;
    }
    return this.arborGraph.renderer.redraw();
  };

  return postureGraph;

})();

abc = (function() {
  var MAX_UNIX_TIME, time;

  function abc() {
    this.update = __bind(this.update, this);

    this.limitCSL = __bind(this.limitCSL, this);

    this.newCSLMode = __bind(this.newCSLMode, this);

    this.set_strategy = __bind(this.set_strategy, this);

    this.compareModes = __bind(this.compareModes, this);

    this.savePosture = __bind(this.savePosture, this);

    this.detectAttractor = __bind(this.detectAttractor, this);

    this.wrapAngle = __bind(this.wrapAngle, this);

    this.searchSubarray = __bind(this.searchSubarray, this);

    this.toggleExplore = __bind(this.toggleExplore, this);
    this.graph = arbor.ParticleSystem();
    this.graph.parameters({
      repulsion: 1000,
      stiffness: 100,
      friction: .5,
      gravity: true
    });
    this.graph.renderer = new RendererSVG("#viewport_svg", this.graph, this);
    this.posture_graph = new postureGraph(this.graph);
    this.last_posture = null;
    this.previous_posture = null;
    this.trajectory = [];
    this.mode_strategy = "unseen";
    this.explore_active = false;
    this.save_periodically = false;
  }

  abc.prototype.toggleExplore = function() {
    if (!physics.upper_joint.csl_active) {
      $("#toggle_csl").click();
    }
    this.explore_active = !this.explore_active;
    if (this.explore_active) {
      return console.log("start explore run at " + new Date);
    }
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

  abc.prototype.wrapAngle = function(angle) {
    var twoPi;
    twoPi = 2 * Math.PI;
    return angle - twoPi * Math.floor(angle / twoPi);
  };

  MAX_UNIX_TIME = 1924988399;

  time = MAX_UNIX_TIME;

  abc.prototype.detectAttractor = function(body, upper_joint, lower_joint, action) {
    var configuration, d, eps, last, p_body, p_hip, p_knee;
    p_body = this.wrapAngle(body.GetAngle());
    p_hip = upper_joint.GetJointAngle();
    p_knee = lower_joint.GetJointAngle();
    if (this.trajectory.length === 10000) {
      this.trajectory.shift();
    }
    this.trajectory.push([p_body, p_hip, p_knee]);
    if (this.trajectory.length > 200 && (Date.now() - time) > 2000) {
      last = this.trajectory.slice(-50);
      eps = 0.025;
      d = this.searchSubarray(last, this.trajectory, function(a, i, b, j) {
        return Math.abs(a[i][0] - b[j][0]) < eps && Math.abs(a[i][1] - b[j][1]) < eps && Math.abs(a[i][2] - b[j][2]) < eps;
      });
      if (d.length > 3) {
        configuration = this.trajectory.pop();
        action(configuration, this);
        this.trajectory = [];
      }
      return time = Date.now();
    }
  };

  abc.prototype.savePosture = function(configuration, body, upper_csl, lower_csl) {
    var addEdge, f, found, n, new_p, node_id, p, parent;
    parent = this;
    addEdge = function(start_node, target_node, edge_list) {
      var current_node, distance, edge, init_node, n0, n1, offset, offset_x, offset_y, source_node, timedelta;
      if (edge_list == null) {
        edge_list = start_node.edges_out;
      }
      edge = new transition(start_node, target_node);
      if (!edge.isInList(edge_list) && parent.posture_graph.length() > 1 && !start_node.isEqualTo(target_node)) {
        console.log("adding edge from posture " + start_node.name + " to posture: " + target_node.name);
        distance = target_node.body_x - start_node.body_x;
        edge.distance = distance;
        timedelta = target_node.timestamp - start_node.timestamp;
        edge.timedelta = timedelta;
        edge.csl_mode = target_node.csl_mode;
        edge_list.push(edge);
        target_node.edges_in.push(edge);
        n0 = start_node.name;
        n1 = target_node.name;
        if (parent.posture_graph.length() > 2) {
          source_node = parent.graph.getNode(n0);
          offset = 0.3;
          offset_x = Math.floor(Math.random() / 0.5);
          if (offset_x) {
            offset_x = offset;
          } else {
            offset_x = -offset;
          }
          offset_y = Math.floor(Math.random() / 0.5);
          if (offset_y) {
            offset_y = offset;
          } else {
            offset_y = -offset;
          }
          parent.graph.getNode(n1) || parent.graph.addNode(n1, {
            'x': source_node.p.x + offset_x,
            'y': source_node.p.y + offset_y
          });
        }
        parent.graph.addEdge(n0, n1, {
          distance: distance.toFixed(3),
          timedelta: timedelta,
          label: parent.transition_mode
        });
        if (n0 === 1 && n1 === 2) {
          init_node = parent.graph.getNode(n0);
          init_node.data.label = start_node.csl_mode;
          init_node.data.number = start_node.name;
          init_node.data.activation = start_node.activation;
          init_node.data.positions = start_node.positions;
          init_node.data.world_angles = start_node.world_angles;
        }
        source_node = parent.graph.getNode(n0);
        parent.graph.current_node = current_node = parent.graph.getNode(n1);
        current_node.data.label = target_node.csl_mode;
        current_node.data.number = target_node.name;
        current_node.data.positions = target_node.positions;
        current_node.data.world_angles = target_node.world_angles;
        current_node.data.activation = target_node.activation;
        source_node.data.activation = start_node.activation;
        parent.graph.start(true);
        return parent.graph.renderer.click_time = Date.now();
      }
    };
    p = new posture(configuration, [upper_csl.csl_mode, lower_csl.csl_mode], body.GetWorldCenter().x);
    found = this.searchSubarray(p, this.posture_graph.nodes, p.isCloseExplore);
    if (!found) {
      if (this.previous_posture && this.previous_posture.exit_directions[this.last_dir_index]) {
        1;

      }
      console.log("found new posture: " + p.configuration);
      node_id = this.posture_graph.addNode(p);
      if (this.last_posture) {
        this.last_posture.exit_directions[this.last_dir_index] = node_id;
      }
    } else {
      f = found[0];
      new_p = p;
      p = this.posture_graph.getNode(f);
      p.configuration[0] = (new_p.configuration[0] + p.configuration[0]) / 2;
      p.configuration[1] = (new_p.configuration[1] + p.configuration[1]) / 2;
      p.configuration[2] = (new_p.configuration[2] + p.configuration[2]) / 2;
      n = this.graph.getNode(p.name);
      if (n.data.semni) {
        n.data.semni.remove();
        n.data.semni = void 0;
      }
    }
    p.positions = [physics.body.GetPosition(), physics.body2.GetPosition(), physics.body3.GetPosition()];
    p.world_angles = [physics.body.GetAngle(), physics.body2.GetAngle(), physics.body3.GetAngle()];
    p.body_x = body.GetWorldCenter().x;
    p.timestamp = Date.now();
    if (this.last_posture && this.posture_graph.length() > 1) {
      addEdge(this.last_posture, p);
      this.graph.current_node = this.graph.getNode(p.name);
      this.graph.renderer.redraw();
    }
    this.previous_posture = this.last_posture;
    this.last_posture = p;
    this.newCSLMode();
    this.posture_graph.diffuseLearnProgress();
    this.posture_graph.diffuseLearnProgress();
    if (this.save_periodically) {
      return this.posture_graph.saveGaphToFile();
    }
  };

  abc.prototype.compareModes = function(a, b) {
    if (!a || !b) {
      return false;
    }
    return a[0] === b[0] && a[1] === b[1];
  };

  abc.prototype.set_strategy = function(strategy) {
    return this.mode_strategy = strategy;
  };

  abc.prototype.newCSLMode = function() {
    /* helpers
    */

    var current_mode, dir, dir_index_for_dir_and_joint, dir_index_for_modes, direction, e, found_index, go_this_edge, joint, joint_from_dir_index, joint_index, next_dir_index, next_mode, next_mode_for_direction, previous_mode, set_random_mode, stall_index_for_mode, _i, _j, _k, _len, _len1, _len2, _ref, _ref1, _ref2;
    set_random_mode = function(current_mode) {
      var mode, which;
      which = Math.floor(Math.random() * 2);
      if (which) {
        while (true) {
          mode = ["r+", "r-", "c"][Math.floor(Math.random() * 2.99)];
          if (current_mode[0] !== mode) {
            break;
          }
        }
        return ui.set_csl_mode_upper(mode);
      } else {
        while (true) {
          mode = ["r+", "r-", "c"][Math.floor(Math.random() * 2.99)];
          if (current_mode[1] !== mode) {
            break;
          }
        }
        return ui.set_csl_mode_lower(mode);
      }
    };
    next_mode_for_direction = function(old_mode, direction) {
      if (direction === "+") {
        switch (old_mode) {
          case "r+":
            return "c";
          case "r-":
            return "r+";
          case "c":
            return "r+";
          case "s-":
            return "r+";
          case "s+":
            return "s+";
        }
      } else if (direction === "-") {
        switch (old_mode) {
          case "r+":
            return "r-";
          case "r-":
            return "c";
          case "c":
            return "r-";
          case "s+":
            return "r-";
          case "s-":
            return "s-";
        }
      }
    };
    dir_index_for_modes = function(start_mode, target_mode) {
      var a, b, d, i, s_h, s_k, t_h, t_k;
      s_h = start_mode[0], s_k = start_mode[1];
      t_h = target_mode[0], t_k = target_mode[1];
      if (s_h === t_h) {
        a = s_k;
        b = t_k;
        i = 2;
      } else {
        a = s_h;
        b = t_h;
        i = 0;
      }
      d = 0;
      if (a === "s+" || a === "s-") {
        a = "c";
      }
      if (b === "s+" || b === "s-") {
        b = "c";
      }
      if (a === "r+" && b === "c") {
        d = 0;
      } else if (a === "r+" && b === "r-") {
        d = 1;
      } else if (a === "r-" && b === "r+") {
        d = 0;
      } else if (a === "r-" && b === "c") {
        d = 1;
      } else if (a === "c" && b === "r-") {
        d = 1;
      } else if (a === "c" && b === "r+") {
        d = 0;
      }
      return i + d;
    };
    dir_index_for_dir_and_joint = function(dir, joint_index) {
      var dir_index;
      if (dir === "-") {
        dir_index = 1;
      } else {
        dir_index = 0;
      }
      return dir_index + 2 * joint_index;
    };
    stall_index_for_mode = function(mode, joint_index) {
      if (mode === "s+") {
        return 0 + joint_index * 2;
      } else if (mode === "s-") {
        return 1 + joint_index * 2;
      }
    };
    joint_from_dir_index = function(index) {
      return Math.ceil((index + 1) / 2) - 1;
    };
    /* end helpers
    */

    current_mode = this.last_posture.csl_mode;
    if (this.previous_posture) {
      previous_mode = this.previous_posture.csl_mode;
    } else {
      previous_mode = void 0;
    }
    _ref = [0, 1];
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      joint = _ref[_i];
      if (__indexOf.call(current_mode[joint], "s") >= 0) {
        this.last_posture.exit_directions[stall_index_for_mode(current_mode[joint], joint)] = -1;
      }
    }
    if (this.mode_strategy === "unseen") {
      /*
            if @last_dir and @last_joint_index in [0, 1]
              back_dir = if @last_dir is "+" then "-" else "+"         #reverse direction
              back_dir_offset = if @last_dir is "+" then 0 else 1      #offset for index
              next_dir_index = @last_joint_index+back_dir_offset            #get index for reverse direction
              if @last_posture.exit_directions[next_dir_index] is 0         #if we have not gone this direction from here, we go back
                next_mode = next_mode_for_direction current_mode[@last_joint_index], back_dir
                direction = back_dir
                joint_index = @last_joint_index
              else
                next_dir_index = undefined
      */

      if (!next_mode) {
        if (__indexOf.call(this.last_posture.exit_directions, 0) >= 0) {
          next_dir_index = this.last_posture.exit_directions.indexOf(0);
          found_index = next_dir_index;
          while (found_index > -1) {
            if (joint_from_dir_index(found_index) !== this.last_joint_index) {
              next_dir_index = found_index;
              if (Math.floor(Math.random() / 0.5)) {
                break;
              }
            }
            found_index = this.last_posture.exit_directions.indexOf(0, found_index + 1);
          }
        } else {
          console.log("following the activation");
          if (!(next_dir_index != null) || this.last_posture.exit_directions[next_dir_index] === -1) {
            go_this_edge = this.last_posture.edges_out[0];
            _ref1 = this.last_posture.edges_out;
            for (_j = 0, _len1 = _ref1.length; _j < _len1; _j++) {
              e = _ref1[_j];
              if (e.target_node.activation > go_this_edge.target_node.activation) {
                go_this_edge = e;
              }
            }
            if (!go_this_edge) {
              _ref2 = this.last_posture.exit_directions;
              for (_k = 0, _len2 = _ref2.length; _k < _len2; _k++) {
                dir = _ref2[_k];
                if (dir > -1) {
                  next_dir_index = dir;
                }
              }
            } else {
              next_dir_index = dir_index_for_modes(this.last_posture.csl_mode, go_this_edge.target_node.csl_mode);
              console.log("followed the edge " + go_this_edge.start_node.name + "->" + go_this_edge.target_node.name + " because of largest activation.");
            }
          }
        }
        joint_index = joint_from_dir_index(next_dir_index);
        if (next_dir_index % 2) {
          direction = "-";
        } else {
          direction = "+";
        }
        next_mode = next_mode_for_direction(current_mode[joint_index], direction);
      }
      if (joint_index === 0) {
        ui.set_csl_mode_upper(next_mode);
      } else {
        ui.set_csl_mode_lower(next_mode);
      }
      this.last_dir = direction;
      this.last_dir_index = next_dir_index;
      this.last_joint_index = joint_index;
      this.transition_mode = current_mode.clone();
      return this.transition_mode[joint_index] = next_mode;
    } else if (this.mode_strategy === "random") {
      return set_random_mode(current_mode);
    } else if (this.mode_strategy === "manual") {
      return 1;
    }
  };

  abc.prototype.limitCSL = function(upper_joint, lower_joint) {
    var limit, mc;
    limit = 15;
    if (upper_joint.csl_active && upper_joint.csl_mode === "c") {
      mc = upper_joint.motor_control;
      if (mc > limit) {
        ui.set_csl_mode_upper("s+");
      } else if (mc < -limit) {
        ui.set_csl_mode_upper("s-");
      }
    }
    if (lower_joint.csl_active && lower_joint.csl_mode === "c") {
      mc = lower_joint.motor_control;
      if (mc > limit) {
        return ui.set_csl_mode_lower("s+");
      } else if (mc < -limit) {
        return ui.set_csl_mode_lower("s-");
      }
    }
  };

  abc.prototype.update = function(body, upper_joint, lower_joint) {
    this.limitCSL(upper_joint, lower_joint);
    if (this.explore_active) {
      this.detectAttractor(body, upper_joint, lower_joint, function(configuration, parent) {
        return parent.savePosture(configuration, body, upper_joint, lower_joint);
      });
    }
    if (this.posture_graph.walk_circle_active) {
      return this.detectAttractor(body, upper_joint, lower_joint, function(configuration, parent) {
        var csl_mode, current_posture, edge, _i, _len, _ref, _results;
        current_posture = new posture(configuration, [physics.upper_joint.csl_mode, physics.lower_joint.csl_mode]);
        _ref = parent.posture_graph.best_circle;
        _results = [];
        for (_i = 0, _len = _ref.length; _i < _len; _i++) {
          edge = _ref[_i];
          if (edge.start_node.isClose(current_posture)) {
            edge.active = true;
            parent.graph.current_node = parent.graph.getNode(edge.start_node.name);
            parent.graph.renderer.redraw();
          }
          if (edge.target_node.isClose(current_posture)) {
            edge.active = false;
          }
          if (edge.active) {
            csl_mode = edge.target_node.csl_mode;
            csl_mode[0] = csl_mode[0].startsWith("s") ? "c" : csl_mode[0];
            csl_mode[1] = csl_mode[1].startsWith("s") ? "c" : csl_mode[1];
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
