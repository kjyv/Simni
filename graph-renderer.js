// Generated by CoffeeScript 1.3.3
var Renderer,
  __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

Renderer = (function() {

  function Renderer(canvas) {
    this.initMouseHandling = __bind(this.initMouseHandling, this);

    this.init = __bind(this.init, this);
    this.canvas = $(canvas).get(0);
    this.ctx = this.canvas.getContext("2d");
    this.particleSystem = null;
    this.nodeBoxes = [];
  }

  Renderer.prototype.init = function(system) {
    this.particleSystem = system;
    this.particleSystem.screenSize(this.canvas.width, this.canvas.height);
    this.particleSystem.screenPadding(80);
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
    var ctx, parent;
    this.ctx.fillStyle = "white";
    this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    parent = this;
    ctx = this.ctx;
    this.particleSystem.eachNode(function(node, pt) {
      var ctx2, image, label, w;
      label = node.data.label;
      image = node.data.imageData;
      if (label) {
        w = ctx.measureText("" + label).width + 8;
      } else {
        w = 8;
      }
      if (image) {
        ctx2 = $("#tempimage")[0].getContext('2d');
        ctx2.clearRect(0, 0, ctx2.canvas.width, ctx2.canvas.height);
        ctx2.putImageData(image, 0, 0);
        ctx.drawImage(ctx2.canvas, pt.x, pt.y);
      } else {

      }
      ctx.rect(pt.x - w / 2, pt.y - w / 2, w, w);
      ctx.strokeStyle = node.data.color ? node.data.color : "black";
      ctx.lineWidth = 1;
      ctx.stroke();
      if (label) {
        ctx.font = "7px Verdana; sans-serif";
        ctx.textAlign = "center";
        ctx.fillStyle = node.data.color ? node.data.color : "#333333";
        ctx.fillText(label || "", pt.x, pt.y + 4);
        ctx.fillText(label || "", pt.x, pt.y + 4);
      }
      return parent.nodeBoxes[node.name] = [pt.x - w / 2, pt.y - w / 2, w, w];
    });
    return this.particleSystem.eachEdge(function(edge, pt1, pt2) {
      var arrowLength, arrowWidth, color, corner, head, tail, w, weight, wt, x, y;
      weight = edge.data.weight;
      color = edge.data.color;
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
        ctx.save();
        wt = (!isNaN(weight) ? parseFloat(weight) : ctx.lineWidth);
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
