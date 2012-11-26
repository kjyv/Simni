
class Renderer
  constructor: (canvas) ->
    @canvas = $(canvas).get(0)
    @ctx = @canvas.getContext("2d")
    @ctx2 = $("#tempimage")[0].getContext('2d')
    @particleSystem = null
    @nodeBoxes = []
    @draw_graphics = false
    MAX_UNIX_TIME = 1924988399 #31/12/2030 23:59:59
    @click_time = MAX_UNIX_TIME

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
    @particleSystem.screenSize @canvas.width, @canvas.height
    @particleSystem.screenPadding 80 # leave an extra 80px of whitespace per side

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

  redraw: ->
    # redraw will be called repeatedly during the run whenever the node positions
    # change. the new positions for the nodes can be accessed by looking at the
    # .p attribute of a given node. however the p.x & p.y values are in the coordinates
    # of the particle system rather than the screen. you can either map them to
    # the screen yourself, or use the convenience iterators .eachNode (and .eachEdge)
    # which allow you to step through the actual node objects but also pass an
    # x,y point in the screen's coordinate system
    # 
    
    if @draw_graphics
      @ctx.fillStyle = "white"
      @ctx.fillRect 0, 0, @canvas.width, @canvas.height
      parent = this
      ctx = @ctx
      ctx2 = @ctx2

      @particleSystem.eachNode (node, pt) ->
        # node: {mass:#, p:{x,y}, name:"", data:{}}
        # pt:   {x:#, y:#}  node position in screen coords
       
        label = node.data.label
        image = node.data.imageData

        if label
          w = ctx.measureText(""+label).width + 8
        else
          w = 8

        if image
          canvas = ctx2.canvas
          c_w = canvas.width
          c_h = canvas.height
          ctx2.clearRect 0,0, c_w, c_h
          ctx2.putImageData image, 0, 0
          ctx.drawImage ctx2.canvas, pt.x - (c_w/4), pt.y - (c_h/4)

        # draw a rectangle centered at pt
        ctx.rect pt.x - w / 2, pt.y - w / 2, w, w
        ctx.strokeStyle =  if node.data.color then node.data.color else "black"
        ctx.lineWidth = 1
        ctx.stroke()
        
        if label
          ctx.font = "7px Verdana; sans-serif"
          ctx.textAlign = "center"
          ctx.fillStyle =  if node.data.color then node.data.color else "#333333"
          ctx.fillText(label||"", pt.x, pt.y+4)
          ctx.fillText(label||"", pt.x, pt.y+4)

        # save box coordinates
        parent.nodeBoxes[node.name] = [pt.x-w/2, pt.y-w/2, w,w]

      @particleSystem.eachEdge (edge, pt1, pt2) ->
        # edge: {source:Node, target:Node, length:#, data:{}}
        # pt1:  {x:#, y:#}  source position in screen coords
        # pt2:  {x:#, y:#}  target position in screen coords
       
        weight = edge.data.weight
        color = edge.data.color
        
        #find the end points
        tail = parent.intersect_line_box(pt1, pt2, parent.nodeBoxes[edge.source.name])
        head = parent.intersect_line_box(tail, pt2, parent.nodeBoxes[edge.target.name])

        ctx.strokeStyle = ctx.fillStyle = if color then color else "rgba(0,0,0, .333)"
        ctx.lineWidth = 1

        if pt1.x == pt2.x and pt1.y == pt2.y
          #draw a circle line
          corner = parent.nodeBoxes[edge.source.name]
          ctx.beginPath()
          x = corner[0]
          y = corner[1]
          w = corner[2]
          ctx.arc(x+w, y, w/4, Math.PI, 0.5*Math.PI, false)
          ctx.stroke()

          #TODO: draw arrow for circles
          #simply draw on top of the box, pointing down
          #might be better to not show two arrows in the same spot though
        else
          # draw a line from pt1 to pt2
          ctx.beginPath()
          ctx.moveTo tail.x, tail.y
          ctx.lineTo head.x, head.y
          ctx.stroke()
       
          # draw arrow
          ctx.save()

          # move to the head position of the edge we just drew
          wt = (if not isNaN(weight) then parseFloat(weight) else ctx.lineWidth)
          arrowLength = 6 + wt
          arrowWidth = 2 + wt
          ctx.translate head.x, head.y
          ctx.rotate Math.atan2(head.y - tail.y, head.x - tail.x)

          # delete some of the edge that's already there (so the point isn't hidden)
          ctx.clearRect -arrowLength / 2, -wt / 2, arrowLength / 2, wt

          # draw the chevron
          ctx.beginPath()
          ctx.moveTo -arrowLength, arrowWidth
          ctx.lineTo 0, 0
          ctx.lineTo -arrowLength, -arrowWidth
          ctx.lineTo -arrowLength * 0.8, -0
          ctx.closePath()
          ctx.fill()
          ctx.restore()
 
  initMouseHandling: =>
    # no-nonsense drag and drop (thanks springy.js)
    dragged = null

    # set up a handler object that will initially listen for mousedowns then
    # for moves and mouseups while dragging
    parent = this
    class Handler
      @clicked: (e) ->
        parent.draw_graphics = true
        pos = $(parent.canvas).offset()
        _mouseP = arbor.Point(e.pageX-pos.left, e.pageY-pos.top)
        dragged = parent.particleSystem.nearest(_mouseP)

        if dragged and dragged.node isnt null
          # while we're dragging, don't let physics move the node
          dragged.node.fixed = true

        $(parent.canvas).bind('mousemove', Handler.dragged)
        $(window).bind('mouseup', Handler.dropped)

        return false
      
      @dragged: (e) ->
        parent.draw_graphics = true
        pos = $(parent.canvas).offset()
        s = arbor.Point(e.pageX-pos.left, e.pageY-pos.top)

        if dragged and dragged.node isnt null
          p = parent.particleSystem.fromScreen(s)
          dragged.node.p = p

        return false

      @dropped: (e) ->
        if (dragged is null or dragged.node is undefined)
          return
        parent.draw_graphics = true
        if dragged.node isnt null
          dragged.node.fixed = false
        dragged.node.tempMass = 1000
        dragged = null
        $(parent.canvas).unbind('mousemove', Handler.dragged)
        $(window).unbind('mouseup', Handler.dropped)
        _mouseP = null
        return false
    
    # start listening
    $(@canvas).mousedown(Handler.clicked)
