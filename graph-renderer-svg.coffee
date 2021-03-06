
class RendererSVG
  constructor: (container, parent, abc) ->
    @width = 1400
    @height = 850
    @svg = d3.select(container).append("svg:svg")
            .attr("width", @width)
            .attr("height", @height)
            .attr("xmlns", "http://www.w3.org/2000/svg")
    @svg_nodes = {}
    @svg_edges = {}
    @particleSystem = null
    @nodeBoxes = []
    MAX_UNIX_TIME = 1924988399 #31/12/2030 23:59:59
    @click_time = MAX_UNIX_TIME
    @graph = parent
    @abc = abc

    @draw_graph_animated = true
    @draw_color_activation = false
    @draw_color_visits = false
    @draw_edge_labels = false
    @draw_activation = false
    @draw_semni = true
    @pause_drawing = true
    @pause_layout = false

    @previous_hover = null

    #pre-create arrow tip marker (svg borrowed from inkscape)
    @svg.append("svg:defs").append("svg:marker")
        .attr("id", "Arrow2Mend")
        .style("overflow", "visible")
        .attr("refX", 0)
        .attr("refY", 0)
        .attr("orient", "auto")
        .attr("stroke", "gray")
      .append("svg:path")
        .attr("style", "font-size:12.0;fill-rule:evenodd;stroke-width:0.62500000;stroke-linejoin:round;")
        .attr("d", "M-7,3L0,0L-7,-3L-5.6,0")

    @initManifoldBar()

  initManifoldBar: =>
    @mt_width = 400
    @mt_height = 20
    @mt_x = 980
    @mt_y = 30
    @mt_count = 0
    @manifold_over_time = @svg.append("g")
    @manifold_over_time.append("rect")
      .attr("x", @mt_x)
      .attr("y", @mt_y)
      .attr("width", @mt_width)
      .attr("height", @mt_height)
      .style("stroke", "gray")
      .style("stroke-width", "1px")
      .style("fill", "none")
      .attr("rx", 2)
      .attr("ry", 2)

  initVisitCountLegend: =>
    @vc_width = 400
    @vc_height = 20
    @vc_x = 980
    @vc_y = 70
    @visitslegend = @svg.append("g")
    @visitslegend.append("rect")
      .attr("x", @vc_x)
      .attr("y", @vc_y)
      .attr("width", @vc_width)
      .attr("height", @vc_height)
      .style("stroke", "gray")
      .style("stroke-width", "1px")
      .style("fill", "none")
      .attr("rx", 2)
      .attr("ry", 2)

    max = 15
    for i in [0..max]
      @visitslegend.append("rect")
          .attr("width", @vc_width/max)
          .attr("height", @vc_height)
          .attr("x", @vc_x+((i*(@vc_width/max)-1)))
          .attr("y", @vc_y)
          .attr("fill", physics.ui.visits2color(i))

  init: (system) =>
    # the particle system will call the init function once, right before the
    # first frame is to be drawn. it's a good place to set up the canvas and
    # to pass the canvas size to the particle system
    #
    # save a reference to the particle system for use in the .redraw() loop
    @particleSystem = system

    # inform the system of the screen dimensions so it can map coords for us.
    # if the canvas is ever resized, screenSize should be called again with
    # the new dimensions
    @particleSystem.screenSize @width, @height
    @particleSystem.screenPadding 90 # leave extra whitespace per side

    # set up some event handlers to allow for node-dragging
    @initMouseHandling()

  # helpers for figuring out where to draw arrows (thanks springy.js)
  intersect_line_line: (p1, p2, p3, p4) ->
    denom = ((p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y))
    return false  if denom is 0 # lines are parallel
    ua = ((p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x)) / denom
    ub = ((p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x)) / denom
    return false  if ua < 0 or ua > 1 or ub < 0 or ub > 1
    arbor.Point p1.x + ua * (p2.x - p1.x), p1.y + ua * (p2.y - p1.y)

  intersect_line_box: (p1, p2, boxTuple) ->
    p3 =
      x: boxTuple[0]
      y: boxTuple[1]

    w = boxTuple[2]
    h = boxTuple[3]
    tl =
      x: p3.x
      y: p3.y

    tr =
      x: p3.x + w
      y: p3.y

    bl =
      x: p3.x
      y: p3.y + h

    br =
      x: p3.x + w
      y: p3.y + h

    @intersect_line_line(p1, p2, tl, tr) or @intersect_line_line(p1, p2, tr, br) or @intersect_line_line(p1, p2, br, bl) or @intersect_line_line(p1, p2, bl, tl) or false

  draw_once: ->
    #draw once without touching settings
    if @draw_graph_animated
      @redraw()
    else if not @draw_graph_animated
      @draw_graph_animated = true
      @redraw()
      @draw_graph_animated = false

  redraw: ->
    # redraw will be called repeatedly during the run whenever the node positions
    # change. the new positions for the nodes can be accessed by looking at the
    # .p attribute of a given node. however the p.x & p.y values are in the coordinates
    # of the particle system rather than the screen. you can either map them to
    # the screen yourself, or use the convenience iterators .eachNode (and .eachEdge)
    # which allow you to step through the actual node objects but also pass an
    # x,y point in the screen's coordinate system
    #

    if not @draw_graph_animated
      return

    parent = this
    graph = @graph

    @particleSystem.eachNode (node, pt) ->
      # node: {mass:#, p:{x,y}, name:"", data:{}}
      # pt:   {x:#, y:#}  node position in screen coords

      label = node.data.label
      number = node.data.number
      image = node.data.imageData
      positions = node.data.positions
      configuration = node.data.configuration
      activation = node.data.activation
      subManifoldId = node.data.subManifoldId
      visits = node.data.visits

      hovered = node.data.hovered

      if label
        w = 26
        w2 = 13
      else
        w = 8
        w2 = 4

      #draw semni contour and posture
      if not node.data.semni and positions and positions.length and configuration
        #put new svg elements with nodes posture if not existent
        node.data.semni = ui.getSemniOutlineSVG(positions[0], configuration[0], configuration[1], configuration[2], parent.svg)
        node.data.semni_crect = node.data.semni[0][0].getBBox() #getBoundingClientRect()

      if not parent.draw_semni and node.data.semni
        node.data.semni.remove()
        node.data.semni = undefined

      if node.data.semni
        #move to current nodes position
        node.data.semni.attr("transform", "translate(" + (pt.x - node.data.semni_crect.width - node.data.semni_crect.x + 20) + "," + (pt.y - node.data.semni_crect.height - node.data.semni_crect.y - 10) + ")")
        if hovered then node.data.semni.attr("stroke", "orange") else node.data.semni.attr("stroke", "gray")

      # draw a rectangle centered at pt
      if parent.svg_nodes[number] == undefined
        parent.svg_nodes[number] = parent.svg.append("svg:rect")
        parent.svg_nodes[number]
          .attr("width", w)
          .attr("height", w)
          .style("fill", "none")
          .attr("rx", 2)
          .attr("ry", 2)

      #draw last node highlit
      if graph.current_node is node
        strokeStyle = "red"
        strokeWidth = "2px"
      else if hovered
        strokeStyle = "blue"
        strokeWidth = "2px"
      else
        strokeStyle = "black"
        strokeWidth = "1px"

      parent.svg_nodes[number]
             .attr("x", pt.x - w2)
             .attr("y", pt.y - w2)
             .style("stroke", strokeStyle)
             .style("stroke-width", strokeWidth)

      ###
        if parent.abc.posture_graph.best_circle
          for transition in parent.abc.posture_graph.best_circle.slice(0,-3)   #leave out the extra data in each circle array
            if node.name is transition.start_node.name
              ctx.strokeStyle = "blue"
              break
      ###

      if label
        if activation?
          a = activation.toFixed(2)

          if parent.draw_color_activation
            c = physics.ui.activation2color a
            parent.svg_nodes[number].style("fill", c)
          else if parent.draw_color_visits
            c = physics.ui.visits2color visits
            parent.svg_nodes[number].style("fill", c)
          else
            c = physics.ui.getSubmanifoldColor(subManifoldId)
            parent.svg_nodes[number].style("fill", c)
        else
          a = ""
        if node.data.label_svg == undefined
          font_size = "7pt"
          font_family = "Verdana"
          text_anchor = "middle"
          #fill = "#333333"
          fill = '#000000'

          #id
          node.data.label_svg = parent.svg.append("svg:text")
          node.data.label_svg[0][0].textContent = number.toString()
          node.data.label_svg
                .style("font-size", font_size)
                .style("font-family", font_family)
                .style("text-anchor", text_anchor)
                .style("fill", fill)

          #mode
          node.data.label_svg2 = parent.svg.append("svg:text")
          node.data.label_svg2[0][0].textContent = label || ""
          node.data.label_svg2
                .style("font-size", font_size)
                .style("font-family", font_family)
                .style("text-anchor", text_anchor)
                .style("fill", fill)

          #activation
          node.data.label_svg3 = parent.svg.append("svg:text")
          node.data.label_svg3
                .style("font-size", font_size)
                .style("font-family", font_family)
                .style("text-anchor", text_anchor)
                .style("fill", fill)

        node.data.label_svg.attr("x",pt.x).attr("y",pt.y-3)
        node.data.label_svg2.attr("x",pt.x).attr("y",pt.y+4)

        if parent.draw_activation
          node.data.label_svg3.attr("x",pt.x).attr("y",pt.y+11.5)
          node.data.label_svg3[0][0].textContent = a
        else
          node.data.label_svg3[0][0].textContent = ""


      # save box coordinates
      parent.nodeBoxes[node.name] = [pt.x-w2, pt.y-w2, w, w]


    @particleSystem.eachEdge (edge, pt1, pt2) ->
      # edge: {source:Node, target:Node, length:#, data:{}}
      # pt1:  {x:#, y:#}  source position in screen coords
      # pt2:  {x:#, y:#}  target position in screen coords

      color = edge.data.color
      distance = edge.data.distance
      label = edge.data.label

      #ctx.strokeStyle = ctx.fillStyle = if color then color else "rgba(0,0,0, .333)"

      if pt1.x is pt2.x and pt1.y is pt2.y
        #loop edge, draw a circle line
        #corner = parent.nodeBoxes[edge.source.name]
        #ctx.beginPath()
        #x = corner[0]
        #y = corner[1]
        #w = corner[2]
        #ctx.arc(x+w, y, w/4, Math.PI, 0.5*Math.PI, false)
        #ctx.stroke()

        #TODO: draw self loops again and arrow
      else
        #normal edge, draw straight line

        #find the visible end points
        tail = parent.intersect_line_box(pt1, pt2, parent.nodeBoxes[edge.source.name])
        if tail is false
          tail = pt1
        head = parent.intersect_line_box(tail, pt2, parent.nodeBoxes[edge.target.name])
        if head is false
          head = pt2

        if edge.data.name == null or edge.data.name == undefined
          edge.data.name = edge.source.name+"-"+edge.target.name

        #add svg element if not existing
        if parent.svg_edges[edge.data.name] == undefined
          parent.svg_edges[edge.data.name] = parent.svg.append("svg:line")
          parent.svg_edges[edge.data.name]
            .attr("stroke-width", "1")
            .attr("stroke", "#808080")
            .attr("fill", "none")
            .attr("marker-end", "url(#Arrow2Mend)")

        #refresh coordinates
        parent.svg_edges[edge.data.name].attr("x1", tail.x).attr("y1", tail.y)
                                        .attr("x2", head.x).attr("y2", head.y)

        if edge.source.data.hovered
          parent.svg_edges[edge.data.name].attr("stroke", "orange")
        else if edge.target.data.hovered
          parent.svg_edges[edge.data.name].attr("stroke", "blue")
        else
          parent.svg_edges[edge.data.name].attr("stroke", "#808080")

        #draw a label
        if label? and parent.draw_edge_labels
          if not edge.data.label_svg
            edge.data.label_svg = parent.svg.append("svg:text")
            edge.data.label_svg[0][0].textContent = (label or "")

          mid =
            x: (pt1.x + pt2.x) / 2
            y: (pt1.y + pt2.y) / 2
          angle = Math.atan2(pt2.y - pt1.y, pt2.x - pt1.x)
          if angle > Math.PI then offset = 2 else offset = -2
          edge.data.label_svg.attr("x",mid.x).attr("y",mid.y+offset).attr("transform", "rotate(" +angle/Math.PI*180+ "," +mid.x+ "," +(mid.y)+ ")")
        else
          if edge.data.label_svg
            edge.data.label_svg.remove()
            edge.data.label_svg = undefined

    #timeout for drawing graph
    if @pause_drawing and (Date.now() - @click_time) > 5000
      @click_time = Date.now()
      @graph.stop()

  initMouseHandling: =>
    # no-nonsense drag and drop (thanks springy.js)
    dragged = null

    # set up a handler object that will initially listen for mousedowns then
    # for moves and mouseups while dragging
    parent = this
    class Handler
      @clicked: (e) ->
        if not parent.pause_layout
          parent.graph.start(true)
        pos = $(parent.svg[0][0]).offset()
        _mouseP = arbor.Point(e.pageX-pos.left, e.pageY-pos.top)
        dragged = parent.particleSystem.nearest(_mouseP)

        if dragged and dragged.node isnt null
          # while we're dragging, don't let physics move the node
          dragged.node.fixed = true

        $(parent.svg[0][0]).bind('mousemove', Handler.dragged)
        $(window).bind('mouseup', Handler.dropped)

        return false

      @dragged: (e) ->
        if not parent.pause_layout
          parent.graph.start(true)
        pos = $(parent.svg[0][0]).offset()
        s = arbor.Point(e.pageX-pos.left, e.pageY-pos.top)

        if dragged and dragged.node isnt null
          p = parent.particleSystem.fromScreen(s)
          dragged.node.p = p

        return false

      @dropped: (e) ->
        if (dragged is null or dragged.node is undefined)
          return
        if not parent.pause_layout
          parent.graph.start(true)
        if dragged.node isnt null
          dragged.node.fixed = false
        dragged.node.tempMass = 1000
        dragged = null
        $(parent.svg[0][0]).unbind('mousemove', Handler.dragged)
        $(window).unbind('mouseup', Handler.dropped)
        _mouseP = null
        return false

      @hover: (e) ->
        if dragged
          return
        pos = $(parent.svg[0][0]).offset()
        _mouseP = arbor.Point(e.pageX-pos.left, e.pageY-pos.top)
        hover = parent.particleSystem.nearest(_mouseP)
        redraw = false
        if hover and hover.distance < 25
          hover.node.data.hovered = true
          if parent.previous_hover? and parent.previous_hover isnt hover.node
            parent.previous_hover.data.hovered = false
          parent.previous_hover = hover.node
          redraw = true

          manifoldRenderer.updateCurrentState manifoldRenderer.internalToManifold hover.node.data.configuration
          window.stopStateUpdate = true
        else
          if parent.previous_hover and parent.previous_hover.data.hovered
            parent.previous_hover.data.hovered = false
            redraw = true
          if hover
            hover.node.data.hovered = false
          window.stopStateUpdate = false

        if redraw
          parent.draw_once()

    # start listening
    $(@svg[0][0]).mousedown(Handler.clicked)
    $(@svg[0][0]).bind('mousemove', Handler.hover)

if not window.simni?
  window.simni = {}

window.simni.RendererSVG = RendererSVG
