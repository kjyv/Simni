// Generated by CoffeeScript 1.3.3
var Renderer,
  __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

Renderer = (function() {

  function Renderer(canvas, parent, abc) {
    this.initMouseHandling = __bind(this.initMouseHandling, this);

    this.init = __bind(this.init, this);

    var MAX_UNIX_TIME;
    this.canvas = $(canvas).get(0);
    this.ctx = this.canvas.getContext("2d");
    this.ctx2 = $("#tempimage")[0].getContext('2d');
    this.particleSystem = null;
    this.nodeBoxes = [];
    MAX_UNIX_TIME = 1924988399;
    this.click_time = MAX_UNIX_TIME;
    this.graph = parent;
    this.abc = abc;
  }

  Renderer.prototype.init = function(system) {
    this.particleSystem = system;
    this.particleSystem.screenSize(this.canvas.width, this.canvas.height);
    this.particleSystem.screenPadding(90);
    return this.initMouseHandling();
  };

  Renderer.prototype.intersect_line_line = function(p1, p2, p3, p4) {
    var denom, ua, ub;
    denom = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);
    if (denom === 0) {
      return false;
    }
    ua = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x)) / denom;
    ub = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x)) / denom;
    if (ua < 0 || ua > 1 || ub < 0 || ub > 1) {
      return false;
    }
    return arbor.Point(p1.x + ua * (p2.x - p1.x), p1.y + ua * (p2.y - p1.y));
  };

  Renderer.prototype.intersect_line_box = function(p1, p2, boxTuple) {
    var bl, br, h, p3, tl, tr, w;
    p3 = {
      x: boxTuple[0],
      y: boxTuple[1]
    };
    w = boxTuple[2];
    h = boxTuple[3];
    tl = {
      x: p3.x,
      y: p3.y
    };
    tr = {
      x: p3.x + w,
      y: p3.y
    };
    bl = {
      x: p3.x,
      y: p3.y + h
    };
    br = {
      x: p3.x + w,
      y: p3.y + h
    };
    return this.intersect_line_line(p1, p2, tl, tr) || this.intersect_line_line(p1, p2, tr, br) || this.intersect_line_line(p1, p2, br, bl) || this.intersect_line_line(p1, p2, bl, tl) || false;
  };

  Renderer.prototype.redraw = function() {
    var ctx, ctx2, graph, parent;
    this.ctx.fillStyle = "white";
    this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    parent = this;
    ctx = this.ctx;
    ctx2 = this.ctx2;
    graph = this.graph;
    this.particleSystem.eachNode(function(node, pt) {
      var c_h, c_w, canvas, image, label, number, transition, w, w2, _i, _len, _ref;
      label = node.data.label;
      number = node.data.number;
      image = node.data.imageData;
      if (label) {
        w = 26;
        w2 = 13;
      } else {
        w = 8;
        w2 = 4;
      }
      if (image) {
        canvas = ctx2.canvas;
        c_w = canvas.width;
        c_h = canvas.height;
        ctx2.clearRect(0, 0, c_w, c_h);
        ctx2.putImageData(image, 0, 0);
        ctx.drawImage(ctx2.canvas, pt.x - (c_w / 4), pt.y - (c_h / 4));
      }
      if (graph.current_node === node) {
        ctx.strokeStyle = "red";
      } else {
        ctx.strokeStyle = "black";
        if (parent.abc.posture_graph.best_circle) {
          _ref = parent.abc.posture_graph.best_circle.slice(0, -3);
          for (_i = 0, _len = _ref.length; _i < _len; _i++) {
            transition = _ref[_i];
            if (node.name === transition.start_node.name) {
              ctx.strokeStyle = "blue";
              break;
            }
          }
        }
      }
      ctx.strokeRect(pt.x - w2, pt.y - w2, w, w);
      ctx.lineWidth = 1;
      if (label) {
        ctx.font = "7px Verdana; sans-serif";
        ctx.textAlign = "center";
        ctx.fillStyle = node.data.color ? node.data.color : "#333333";
        ctx.fillText(number, pt.x, pt.y - 3);
        ctx.fillText(label || "", pt.x, pt.y + 4);
      }
      return parent.nodeBoxes[node.name] = [pt.x - w2, pt.y - w2, w, w];
    });
    return this.particleSystem.eachEdge(function(edge, pt1, pt2) {
      var angle, arrowLength, arrowWidth, color, corner, head, label, mid, tail, w, weight, wt, x, y;
      weight = edge.data.weight;
      color = edge.data.color;
      label = edge.data.distance;
      tail = parent.intersect_line_box(pt1, pt2, parent.nodeBoxes[edge.source.name]);
      head = parent.intersect_line_box(tail, pt2, parent.nodeBoxes[edge.target.name]);
      ctx.strokeStyle = ctx.fillStyle = color ? color : "rgba(0,0,0, .333)";
      ctx.lineWidth = 1;
      if (pt1.x === pt2.x && pt1.y === pt2.y) {
        corner = parent.nodeBoxes[edge.source.name];
        ctx.beginPath();
        x = corner[0];
        y = corner[1];
        w = corner[2];
        ctx.arc(x + w, y, w / 4, Math.PI, 0.5 * Math.PI, false);
        return ctx.stroke();
      } else {
        ctx.beginPath();
        ctx.moveTo(tail.x, tail.y);
        ctx.lineTo(head.x, head.y);
        ctx.stroke();
        if (label && Math.abs(label) > 0.05) {
          ctx.save();
          mid = {
            x: (pt1.x + pt2.x) / 2,
            y: (pt1.y + pt2.y) / 2
          };
          ctx.translate(mid.x, mid.y);
          angle = Math.atan2(pt2.y - pt1.y, pt2.x - pt1.x);
          ctx.rotate(angle);
          ctx.font = "7px Verdana; sans-serif";
          ctx.textAlign = "center";
          ctx.fillStyle = edge.data.color ? edge.data.color : "#333333";
          ctx.fillText(label || "", 0, -3);
          ctx.restore();
        }
        ctx.save();
        wt = ctx.lineWidth;
        arrowLength = 6 + wt;
        arrowWidth = 2 + wt;
        ctx.translate(head.x, head.y);
        ctx.rotate(Math.atan2(head.y - tail.y, head.x - tail.x));
        ctx.clearRect(-arrowLength / 2, -wt / 2, arrowLength / 2, wt);
        ctx.beginPath();
        ctx.moveTo(-arrowLength, arrowWidth);
        ctx.lineTo(0, 0);
        ctx.lineTo(-arrowLength, -arrowWidth);
        ctx.lineTo(-arrowLength * 0.8, -0);
        ctx.closePath();
        ctx.fill();
        return ctx.restore();
      }
    });
  };

  Renderer.prototype.initMouseHandling = function() {
    var Handler, dragged, parent;
    dragged = null;
    parent = this;
    Handler = (function() {

      function Handler() {}

      Handler.clicked = function(e) {
        var pos, _mouseP;
        parent.graph.start(true);
        pos = $(parent.canvas).offset();
        _mouseP = arbor.Point(e.pageX - pos.left, e.pageY - pos.top);
        dragged = parent.particleSystem.nearest(_mouseP);
        if (dragged && dragged.node !== null) {
          dragged.node.fixed = true;
        }
        $(parent.canvas).bind('mousemove', Handler.dragged);
        $(window).bind('mouseup', Handler.dropped);
        return false;
      };

      Handler.dragged = function(e) {
        var p, pos, s;
        parent.graph.start(true);
        pos = $(parent.canvas).offset();
        s = arbor.Point(e.pageX - pos.left, e.pageY - pos.top);
        if (dragged && dragged.node !== null) {
          p = parent.particleSystem.fromScreen(s);
          dragged.node.p = p;
        }
        return false;
      };

      Handler.dropped = function(e) {
        var _mouseP;
        if (dragged === null || dragged.node === void 0) {
          return;
        }
        parent.graph.start(true);
        if (dragged.node !== null) {
          dragged.node.fixed = false;
        }
        dragged.node.tempMass = 1000;
        dragged = null;
        $(parent.canvas).unbind('mousemove', Handler.dragged);
        $(window).unbind('mouseup', Handler.dropped);
        _mouseP = null;
        return false;
      };

      return Handler;

    })();
    return $(this.canvas).mousedown(Handler.clicked);
  };

  return Renderer;

})();
