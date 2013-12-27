// Generated by CoffeeScript 1.3.3
var RendererSVG,
  __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

RendererSVG = (function() {

  function RendererSVG(container, parent, abc) {
    this.initMouseHandling = __bind(this.initMouseHandling, this);

    this.init = __bind(this.init, this);

    var MAX_UNIX_TIME, arrowLength, arrowWidth;
    this.width = 1400;
    this.height = 850;
    this.svg = d3.select(container).append("svg:svg").attr("width", this.width).attr("height", this.height).attr("xmlns", "http://www.w3.org/2000/svg");
    this.svg_nodes = {};
    this.svg_edges = {};
    this.particleSystem = null;
    this.nodeBoxes = [];
    MAX_UNIX_TIME = 1924988399;
    this.click_time = MAX_UNIX_TIME;
    this.graph = parent;
    this.abc = abc;
    $("canvas#viewport").hide();
    this.draw_graph_animated = true;
    this.draw_color_activation = true;
    this.draw_edge_labels = false;
    this.draw_activation = false;
    this.draw_semni = true;
    this.pause_drawing = true;
    this.previous_hover = null;
    arrowLength = 6 + 1;
    arrowWidth = 2 + 1;
    this.svg.append("svg:defs").append("svg:marker").attr("id", "arrowtip").attr("viewBox", "-10 -5 10 10").attr("refX", 0).attr("refY", 0).attr("markerWidth", 10).attr("markerHeight", 10).attr("orient", "auto").attr("stroke", "gray").append("svg:path").attr("d", "M-7,3L0,0L-7,-3L-5.6,0");
  }

  RendererSVG.prototype.init = function(system) {
    this.particleSystem = system;
    this.particleSystem.screenSize(this.width, this.height);
    this.particleSystem.screenPadding(90);
    return this.initMouseHandling();
  };

  RendererSVG.prototype.intersect_line_line = function(p1, p2, p3, p4) {
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

  RendererSVG.prototype.intersect_line_box = function(p1, p2, boxTuple) {
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

  RendererSVG.prototype.draw_once = function() {
    if (this.draw_graph_animated) {
      return this.redraw();
    } else if (!this.draw_graph_animated) {
      this.draw_graph_animated = true;
      this.redraw();
      return this.draw_graph_animated = false;
    }
  };

  RendererSVG.prototype.redraw = function() {
    var graph, parent;
    if (!this.draw_graph_animated) {
      return;
    }
    parent = this;
    graph = this.graph;
    this.particleSystem.eachNode(function(node, pt) {
      var a, activation, c, crect, hovered, image, label, number, positions, strokeStyle, strokeWidth, w, w2, world_angles;
      label = node.data.label;
      number = node.data.number;
      image = node.data.imageData;
      positions = node.data.positions;
      world_angles = node.data.world_angles;
      activation = node.data.activation;
      hovered = node.data.hovered;
      if (label) {
        w = 26;
        w2 = 13;
      } else {
        w = 8;
        w2 = 4;
      }
      if (!node.data.semni && positions && positions.length && world_angles) {
        node.data.semni = ui.getSemniOutlineSVG(positions[0], world_angles[0], world_angles[1], world_angles[2], parent.svg);
      }
      if (!parent.draw_semni && node.data.semni) {
        node.data.semni.remove();
        node.data.semni = void 0;
      }
      if (node.data.semni) {
        crect = node.data.semni[0][0].getBBox();
        node.data.semni.attr("transform", "translate(" + (pt.x - crect.width - crect.x + 20) + "," + (pt.y - crect.height - crect.y - 10) + ")");
        if (hovered) {
          node.data.semni.attr("stroke", "orange");
        } else {
          node.data.semni.attr("stroke", "gray");
        }
      }
      if (parent.svg_nodes[number] === void 0) {
        parent.svg_nodes[number] = parent.svg.append("svg:rect");
      }
      if (graph.current_node === node) {
        strokeStyle = "red";
        strokeWidth = "2px";
      } else if (hovered) {
        strokeStyle = "blue";
        strokeWidth = "2px";
      } else {
        strokeStyle = "black";
        strokeWidth = "1px";
      }
      parent.svg_nodes[number].attr("x", pt.x - w2).attr("y", pt.y - w2).attr("width", w).attr("height", w).style("fill", "none").style("stroke", strokeStyle).style("stroke-width", strokeWidth);
      /*
              if parent.abc.posture_graph.best_circle
                for transition in parent.abc.posture_graph.best_circle.slice(0,-3)   #leave out the extra data in each circle array
                  if node.name is transition.start_node.name
                    ctx.strokeStyle = "blue"
                    break
      */

      if (label) {
        if (activation != null) {
          a = activation.toFixed(1);
          if (parent.draw_color_activation) {
            c = physics.ui.powerColor(a);
            parent.svg_nodes[number].style("fill", "rgb(" + c[0] + "," + c[1] + "," + c[2] + ")");
          } else {
            parent.svg_nodes[number].style("fill", "none");
          }
        } else {
          a = "";
        }
        if (node.data.label_svg === void 0) {
          node.data.label_svg = parent.svg.append("svg:text");
          node.data.label_svg[0][0].textContent = number.toString();
          node.data.label_svg2 = parent.svg.append("svg:text");
          node.data.label_svg2[0][0].textContent = label || "";
          node.data.label_svg3 = parent.svg.append("svg:text");
        }
        node.data.label_svg.attr("x", pt.x).attr("y", pt.y - 3);
        node.data.label_svg2.attr("x", pt.x).attr("y", pt.y + 4);
        if (parent.draw_activation) {
          node.data.label_svg3.attr("x", pt.x).attr("y", pt.y + 11);
          node.data.label_svg3[0][0].textContent = "a:" + a;
        } else {
          node.data.label_svg3[0][0].textContent = "";
        }
      }
      return parent.nodeBoxes[node.name] = [pt.x - w2, pt.y - w2, w, w];
    });
    this.particleSystem.eachEdge(function(edge, pt1, pt2) {
      var angle, color, distance, head, label, mid, offset, tail;
      color = edge.data.color;
      distance = edge.data.distance;
      label = edge.data.label;
      if (pt1.x === pt2.x && pt1.y === pt2.y) {

      } else {
        tail = parent.intersect_line_box(pt1, pt2, parent.nodeBoxes[edge.source.name]);
        if (tail === false) {
          tail = pt1;
        }
        head = parent.intersect_line_box(tail, pt2, parent.nodeBoxes[edge.target.name]);
        if (head === false) {
          head = pt2;
        }
        if (edge.data.name === null || edge.data.name === void 0) {
          edge.data.name = edge.source.name + "-" + edge.target.name;
        }
        if (parent.svg_edges[edge.data.name] === void 0) {
          parent.svg_edges[edge.data.name] = parent.svg.append("svg:line");
        }
        parent.svg_edges[edge.data.name].attr("x1", tail.x).attr("y1", tail.y).attr("x2", head.x).attr("y2", head.y).attr("marker-end", "url(#arrowtip)");
        if (edge.source.data.hovered) {
          parent.svg_edges[edge.data.name].attr("stroke", "orange");
        } else if (edge.target.data.hovered) {
          parent.svg_edges[edge.data.name].attr("stroke", "blue");
        } else {
          parent.svg_edges[edge.data.name].attr("stroke", "gray");
        }
        if ((label != null) && parent.draw_edge_labels) {
          if (!edge.data.label_svg) {
            edge.data.label_svg = parent.svg.append("svg:text");
            edge.data.label_svg[0][0].textContent = label || "";
          }
          mid = {
            x: (pt1.x + pt2.x) / 2,
            y: (pt1.y + pt2.y) / 2
          };
          angle = Math.atan2(pt2.y - pt1.y, pt2.x - pt1.x);
          if (angle > Math.PI) {
            offset = 2;
          } else {
            offset = -2;
          }
          return edge.data.label_svg.attr("x", mid.x).attr("y", mid.y + offset).attr("transform", "rotate(" + angle / Math.PI * 180 + "," + mid.x + "," + mid.y + ")");
        } else {
          if (edge.data.label_svg) {
            edge.data.label_svg.remove();
            return edge.data.label_svg = void 0;
          }
        }
      }
    });
    if (this.pause_drawing && (Date.now() - this.click_time) > 5000) {
      this.click_time = Date.now();
      return this.graph.stop();
    }
  };

  RendererSVG.prototype.initMouseHandling = function() {
    var Handler, dragged, parent;
    dragged = null;
    parent = this;
    Handler = (function() {

      function Handler() {}

      Handler.clicked = function(e) {
        var pos, _mouseP;
        parent.graph.start(true);
        pos = $(parent.svg[0][0]).offset();
        _mouseP = arbor.Point(e.pageX - pos.left, e.pageY - pos.top);
        dragged = parent.particleSystem.nearest(_mouseP);
        if (dragged && dragged.node !== null) {
          dragged.node.fixed = true;
        }
        $(parent.svg[0][0]).bind('mousemove', Handler.dragged);
        $(window).bind('mouseup', Handler.dropped);
        return false;
      };

      Handler.dragged = function(e) {
        var p, pos, s;
        parent.graph.start(true);
        pos = $(parent.svg[0][0]).offset();
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
        $(parent.svg[0][0]).unbind('mousemove', Handler.dragged);
        $(window).unbind('mouseup', Handler.dropped);
        _mouseP = null;
        return false;
      };

      Handler.hover = function(e) {
        var hover, pos, redraw, _mouseP;
        if (dragged) {
          return;
        }
        pos = $(parent.svg[0][0]).offset();
        _mouseP = arbor.Point(e.pageX - pos.left, e.pageY - pos.top);
        hover = parent.particleSystem.nearest(_mouseP);
        redraw = false;
        if (hover && hover.distance < 25) {
          hover.node.data.hovered = true;
          if ((parent.previous_hover != null) && parent.previous_hover !== hover.node) {
            parent.previous_hover.data.hovered = false;
          }
          parent.previous_hover = hover.node;
          redraw = true;
        } else {
          if (parent.previous_hover && parent.previous_hover.data.hovered) {
            parent.previous_hover.data.hovered = false;
            redraw = true;
          }
          if (hover) {
            hover.node.data.hovered = false;
          }
        }
        if (redraw) {
          return parent.draw_once();
        }
      };

      return Handler;

    })();
    $(this.svg[0][0]).mousedown(Handler.clicked);
    return $(this.svg[0][0]).bind('mousemove', Handler.hover);
  };

  return RendererSVG;

})();

if (!(window.simni != null)) {
  window.simni = {};
}

window.simni.RendererSVG = RendererSVG;
