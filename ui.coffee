## ui helper methods

class ui
  constructor: (physics) ->
    @draw_graphics = true
    @physics = physics
    @init()
    @halftime = true   #draw only every second frame of the physic bodies
    @svg_scale = 200
    @realtime = false

  update: =>
    if @draw_graphics and @halftime
      @physics.world.DrawDebugData()

      ###
      #draw semni as svg
      container = "#semni_svg"
      $(container).html("")
      @svg = d3.select(container).append("svg:svg")
              .attr("width", 300)
              .attr("height", 300)
              .attr("xmlns", "http://www.w3.org/2000/svg")
      @semni = @getSemniOutlineSVG(
        @physics.body.GetPosition(),
        @physics.body2.GetPosition(),
        @physics.body3.GetPosition(),
        @physics.body.GetAngle(),
        @physics.body2.GetAngle(),
        @physics.body3.GetAngle(),
        @svg
      )
      ###

    @halftime = not @halftime

  rotate_point: (cx, cy, angle, p) =>
    #translate point back to origin:origin
    p.x -= cx
    p.y -= cy

    #rotate point
    s = Math.sin(angle)
    c = Math.cos(angle)

    xnew = p.x * c - p.y * s
    ynew = p.x * s + p.y * c

    #translate point back
    p.x = xnew + cx
    p.y = ynew + cy

    return p

  getSemniOutlineSVG: (body_pos, arm1_pos, arm2_pos, body_angle, arm1_angle, arm2_angle, container) =>
    #draw semni with given angles and position values into given svg container, return
    #set up svg objects
    svg = container.append("svg:g")

    d3line2 = d3.svg.line().x((d) ->
      d.x*physics.ui.svg_scale
    ).y((d) ->
      d.y*physics.ui.svg_scale
    ).interpolate("linear")

    svg_semni_body = svg.append("svg:path").attr("d",d3line2(simni.contour_original_lowest_detail))
            .style("stroke-width", 1)
            #.style("stroke", "gray")
            .style("fill", "none")

    svg_semni_arm1 = svg.append("svg:path").attr("d",d3line2(simni.arm1Contour))
            .style("stroke-width", 1)
            #.style("stroke", "gray")
            .style("fill", "none")
    svg_joint = svg.append("svg:circle").attr("cx", 0).attr("cy", 0).attr("r", "1")
            .style("stroke", "red")
    svg_semni_arm2 = svg.append("svg:path").attr("d",d3line2(simni.arm2Contour))
            .style("stroke-width", 1)
            #.style("stroke", "gray")
            .style("fill", "none")
    svg_joint2 = svg.append("svg:circle").attr("cx", 0).attr("cy", 0).attr("r", "1")
            .style("stroke", "red")
    svg_semni_head = svg.append("svg:circle").attr("cx", 0).attr("cy", 0).attr("r", simni.head2[1]*@svg_scale)
            .style("stroke-width", 1)
            #.style("stroke", "gray")
            .style("fill", "none")

    #draw body at current position and angle
    b_x = body_pos.x*@svg_scale
    b_y = body_pos.y*@svg_scale
    #svg_semni_body.attr("transform", "translate(" +b_x+ "," +b_y+ ") rotate("+body_angle*180/Math.PI+")")
    svg_semni_body.attr("transform", "rotate("+body_angle*180/Math.PI+")")

    #draw upper arm at first joint that is moved and rotated with the body, then rotated by arm angle
    arm1_joint = new b2Vec2()
    arm1_joint.x = simni.arm1JointAnchor2.x*@svg_scale #+b_x
    arm1_joint.y = simni.arm1JointAnchor2.y*@svg_scale #+b_y
    #arm1_joint = @rotate_point(b_x, b_y, body_angle, arm1_joint)
    arm1_joint = @rotate_point(0, 0, body_angle, arm1_joint)
    svg_joint.attr("transform", "translate(" +arm1_joint.x+ "," +arm1_joint.y+ ")")
    svg_semni_arm1.attr("transform", "rotate(" +arm1_angle*180/Math.PI+ "," +arm1_joint.x+ "," +arm1_joint.y+ ") translate(" +arm1_joint.x+ "," +arm1_joint.y+ ")")

    #draw lower arm at second joint that is moved with body and rotated, then rotated by upper arm angle, then by own angle
    arm2_joint = new b2Vec2()
    arm2_joint.x = simni.arm2JointAnchor2.x*@svg_scale #+b_x
    arm2_joint.y = simni.arm2JointAnchor2.y*@svg_scale #+b_y

    #arm2_joint = @rotate_point(b_x, b_y, body_angle, arm2_joint)
    arm2_joint = @rotate_point(0, 0, body_angle, arm2_joint)
    arm2_joint = @rotate_point(arm1_joint.x, arm1_joint.y, arm1_angle-body_angle-0.85, arm2_joint) #confused why angle offset is necessary
    svg_joint2.attr("transform", "translate(" +arm2_joint.x+ "," +arm2_joint.y+ ")")
    svg_semni_arm2.attr("transform", "rotate(" +arm2_angle*180/Math.PI+ "," +arm2_joint.x+ "," +arm2_joint.y+ ") translate(" +arm2_joint.x+ "," +arm2_joint.y+ ")")

    #draw head
    h_x = simni.head2[0].x*@svg_scale #+b_x
    h_y = simni.head2[0].y*@svg_scale #+b_y
    #svg_semni_head.attr("transform", "rotate(" +body_angle*180/Math.PI+ "," +b_x+ "," +b_y+ ") translate(" +h_x+ "," +h_y+ ")")
    svg_semni_head.attr("transform", "rotate(" +body_angle*180/Math.PI+ "," +0+ "," +0+ ") translate(" +h_x+ "," +h_y+ ")")

    return svg

  init: =>
    #set up map to mode checkbox and get values
    @physics.map_state_to_mode = false
    $("#map_state_to_mode").click =>
      @physics.map_state_to_mode = !@physics.map_state_to_mode
      $('#map_state_to_mode').attr('checked', @physics.map_state_to_mode)

    $("#w0").change =>
      $("#w0_val").html("="+$("#w0").val())
      @physics.w0 = parseFloat $("#w0").val()
    $("#w1").change =>
      $("#w1_val").html("="+$("#w1").val())
      @physics.w1 = parseFloat $("#w1").val()
    $("#w2").change =>
      $("#w2_val").html("="+$("#w2").val())
      @physics.w2 = parseFloat $("#w2").val()
    @physics.w0 = parseFloat $("#w0").val()
    @physics.w1 = parseFloat $("#w1").val()
    @physics.w2 = parseFloat $("#w2").val()

    #get wether U and phi should be used as absolute or signed values from checkboxed
    $("#w0_abs").change =>
      @physics.w0_abs = ($("#w0_abs").attr("checked") isnt undefined)
      draw_state_to_mode_mapping()
    $("#w1_abs").change =>
      @physics.w1_abs = ($("#w1_abs").attr("checked") isnt undefined)
      draw_state_to_mode_mapping()
    #initial values
    @physics.w0_abs = ($("#w0_abs").attr("checked") isnt undefined)
    @physics.w1_abs = ($("#w1_abs").attr("checked") isnt undefined)

    #fps counter
    window.stats = new Stats()
    window.stats.setMode 0
    # align top-left
    window.stats.domElement.style.position = "absolute"
    window.stats.domElement.style.left = "0px"
    window.stats.domElement.style.top = "0px"
    document.body.appendChild window.stats.domElement

    #request first frame from browser to start update cycle (depends on realtime option)
    @set_realtime @realtime

    # stuff to handle mouse manipulation
    window.mouseX = undefined
    window.mouseY = undefined
    window.mousePVec = undefined
    window.isMouseDown = undefined
    window.selectedBody = undefined
    window.mouseJoint = undefined

    getElementPosition = (element) ->
      elem = element
      tagname = ""
      x = 0
      y = 0
      while (typeof (elem) is "object") and (typeof (elem.tagName) isnt "undefined")
        y += elem.offsetTop
        x += elem.offsetLeft
        tagname = elem.tagName.toUpperCase()
        elem = 0  if tagname is "BODY"
        elem = elem.offsetParent  if typeof (elem.offsetParent) is "object"  if typeof (elem) is "object"
      x: x
      y: y

    canvas = $('#simulation canvas')[0]
    canvasPosition = getElementPosition(canvas)
    canvas.addEventListener "mousedown", ((e) ->
      window.isMouseDown = true
      handleMouseMove e
      document.addEventListener "mousemove", handleMouseMove, true
    ), true
    document.addEventListener "mouseup", (->
      document.removeEventListener "mousemove", handleMouseMove, true
      window.isMouseDown = false
      window.mouseX = undefined
      window.mouseY = undefined
    ), true

    handleMouseMove = (e) ->
      window.mouseX = (e.clientX - canvasPosition.x) / physics.debugDraw.GetDrawScale()
      window.mouseY = (e.clientY - canvasPosition.y) / physics.debugDraw.GetDrawScale()

    window.getBodyAtMouse = ->
      window.mousePVec = new b2Vec2(mouseX, mouseY)
      aabb = new Box2D.Collision.b2AABB()
      aabb.lowerBound.Set mouseX - 0.001, mouseY - 0.001
      aabb.upperBound.Set mouseX + 0.001, mouseY + 0.001

      # Query the world for overlapping shapes.
      window.selectedBody = null
      @physics.world.QueryAABB getBodyCB, aabb
      window.selectedBody

    getBodyCB = (fixture) ->
      #if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
      if fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), window.mousePVec)
        window.selectedBody = fixture.GetBody()
        return false
      #}
      true

  set_preset: (w0, w1, w2, w0_abs, w1_abs) =>
    $("#w0").val(w0).trigger("change")
    $("#w1").val(w1).trigger("change")
    $("#w2").val(w2).trigger("change")

    if w0_abs == true
      $("#w0_abs").attr("checked", "checked").trigger("change")
    else
      $("#w0_abs").attr("checked", null).trigger("change")

    if w1_abs == true
      $("#w1_abs").attr("checked", "checked").trigger("change")
    else
      $("#w1_abs").attr("checked", null).trigger("change")

    draw_state_to_mode_mapping()

  map_mode_to_gi: (mode) =>
    #double pendulum
    #if mode < 0
    #  mode * 3
    #else
    #  mode * 0.7

    #single pendulum
    if mode < 0
      mode * 3
    else
      15 + (5 * mode)

  map_mode_to_gf: (mode) =>
    #double pendulum
    #if mode > 1
    #  mode * 0.01 + 1
    #else if mode < 0
    #  0
    #else
    #  mode

    #single pendulum
    if mode > 1
      mode * 0.00125 + 1
    else if mode < 0
      0
    else
      mode

  map_mode: (bodyJoint, mode, joint=bodyJoint.joint_name) =>
    #re-calc gi and gf from joints mode parameter and save in bodyJoint
    #bodyJoint should have a useful joint_name set (lower, upper, depends on dom elements)

    #support:      gi < 0, gf = 0
    #release:      gi > 0, 0 <= gf < 1
    #hold:         gi > 0, gf = 1
    #contraction:  gi > 0, gf > 1

    if not mode?
      mode = parseFloat($("#mode_param_#{joint}").val())

    gi = @map_mode_to_gi(mode)
    gf = @map_mode_to_gf(mode)

    $("#gi_param_#{joint}").val gi
    $("#gf_param_#{joint}").val gf
    $("#mode_param_#{joint}").val mode
    $("#mode_val_#{joint}").html mode.toFixed(2)

    if gi < 0 and gf is 0
      $("#csl_mode_name_#{joint}").html "support"
    else if gi > 0 and 0 <= gf < 1
      $("#csl_mode_name_#{joint}").html "release"
    else if gi > 0 and gf is 1
      $("#csl_mode_name_#{joint}").html "hold"
    else if gi > 0 and gf > 1
      $("#csl_mode_name_#{joint}").html "contraction"
    else
      $("#csl_mode_name_#{joint}").html "?"

    if bodyJoint
      bodyJoint.gi = gi
      bodyJoint.gf = gf

  set_friction: (beta) =>
    $("#friction_val").html beta.toFixed(3)
    if @physics.pend_style is 3
      @physics.upper_joint.m_maxMotorTorque = beta
      @physics.lower_joint.m_maxMotorTorque = beta*1.5
    else
      @physics.lower_joint.m_maxMotorTorque = beta
    @physics.beta = beta

  set_posture: (bodyAngle, hipAngle, kneeAngle, hipCsl, kneeCsl) =>
    #TODO, hm, how to properly set joint angles without recreating? sum angles going up from body
    #body.SetTransform could help
    p = @physics

    p.world.DestroyBody p.body3
    p.world.DestroyJoint p.lower_joint
    p.world.DestroyBody p.body2
    p.world.DestroyJoint p.upper_joint
    p.world.DestroyBody p.body

    x0 = 0.516
    y0 = 0.76

    #p.body.SetPositionAndAngle(new b2Vec2(x0,y0), 0)
    p.createSemni(x0,y0)

  set_csl_mode_upper: (hipCSL, change_select=true) =>
    #set ABC learning modes for exploration
    release_bias_hip = 0.015
    release_gf = 0
    release_gi = 0
    contract_gf_hip = 1.01 #1.0025 #1.006
    contract_gi = 7 #30 #27 #50
    stall_gb = 0.2
    stall_gf = 0 #0.8

    if hipCSL is "r+"
      gf = release_gf
      gb = release_bias_hip
      gi = release_gi
      @physics.upper_joint.csl_prefill = 0.01
    else if hipCSL is "r-"
      gf = release_gf
      gb = -release_bias_hip
      gi = release_gi
      @physics.upper_joint.csl_prefill = -0.01
    else if hipCSL is "c"
      gf = contract_gf_hip
      gb = 0
      gi = contract_gi
      #prefill integrator to pre-determine direction
      @physics.upper_joint.last_integrated = @physics.upper_joint.csl_prefill
    else if hipCSL is "s+"
      gf = stall_gf
      gb = stall_gb
      gi = release_gi
      @physics.upper_joint.last_integrated = 0
    else if hipCSL is "s-"
      gf = stall_gf
      gb = -stall_gb
      gi = release_gi
      @physics.upper_joint.last_integrated = 0

    if change_select
      #re-select select option in case we came from another function and not from select widget
      $("#csl_mode_hip option[value='" + hipCSL + "']").attr "selected", true

    $("#gi_param_upper").val(gi)
    @physics.upper_joint.gi = gi

    $("#gf_param_upper").val(gf)
    @physics.upper_joint.gf = gf

    $("#gb_param_upper").val(gb)
    @physics.upper_joint.gb = gb
    @physics.upper_joint.csl_mode = hipCSL

    #reset manual mode noop if enabled
    @physics.abc.manual_noop = false

  set_csl_mode_lower: (kneeCSL, change_select=true) =>
    release_bias_knee = 0.025
    release_gf = 0
    release_gi = 0
    contract_gf_knee = 1.02 #1.0015 #1.006
    contract_gi = 25 #35 #26 #50
    stall_gb = 0.2
    stall_gf = 0 #0.8

    if kneeCSL is "r+"
      gf = release_gf
      gb = release_bias_knee
      gi = release_gi
      @physics.lower_joint.csl_prefill = 0.01
    else if kneeCSL is "r-"
      gf = release_gf
      gb = -release_bias_knee
      gi = release_gi
      @physics.lower_joint.csl_prefill = -0.01
    else if kneeCSL is "c"
      gf = contract_gf_knee
      gb = 0
      gi = contract_gi
      @physics.lower_joint.last_integrated = @physics.lower_joint.csl_prefill
    else if kneeCSL is "s+"
      gf = stall_gf
      gb = stall_gb
      gi = release_gi
      @physics.lower_joint.last_integrated = 0
    else if kneeCSL is "s-"
      gf = stall_gf
      gb = -stall_gb
      gi = release_gi
      @physics.lower_joint.last_integrated = 0

    if change_select
      #re-select select option in case we came from another function
      $("#csl_mode_knee option[value='" + kneeCSL + "']").attr 'selected', true

    $("#gi_param_lower").val(gi)
    @physics.lower_joint.gi = gi

    $("#gf_param_lower").val(gf)
    @physics.lower_joint.gf = gf

    $("#gb_param_lower").val(gb)
    @physics.lower_joint.gb = gb
    @physics.lower_joint.csl_mode = kneeCSL

    if physics.abc.mode_strategy is "manual"
      physics.abc.trajectory = []

    #reset manual mode noop if enabled
    @physics.abc.manual_noop = false

  getSemniTransformAsJSON: =>
    t = @physics.body.GetTransform()
    t2 = @physics.body2.GetTransform()
    t3 = @physics.body3.GetTransform()
    return JSON.stringify({"body":t,"body2":t2,"body3":t3})

  getSemniTransformAsFile: =>
    location.href = 'data:text;charset=utf-8,'+encodeURI @getSemniTransformAsJSON()

  setSemniTransformAsJSON: (tj=null) =>
    t = JSON.parse(tj)
    @physics.body.SetTransform(new b2Transform(t.body.position, t.body.R))
    @physics.body2.SetTransform(new b2Transform(t.body2.position, t.body2.R))
    @physics.body3.SetTransform(new b2Transform(t.body3.position, t.body3.R))

  setSemniTransformAsFile: (files) =>
    readFile = (file, callback) ->
      reader = new FileReader()
      reader.onload = (evt) ->
        callback file, evt  if typeof callback is "function"
      reader.readAsBinaryString file

    if files.length > 0
      readFile files[0], (file, evt) ->
        window.ui.setSemniTransformAsJSON(evt.target.result)

  getPostureGraphAsFile: =>
    svg = $("#viewport_svg").clone()
    svg.find("defs").append """
    <style>
       line {
          stroke-width: 1;
          stroke: black;
          fill: none;
      }

      text {
        font-family: Verdana; sans-serif;
        font-size: 7pt;
        text-anchor: middle;
        fill: #333333;
      }
    </style>
    """
    location.href = 'data:text;charset=utf-8,'+encodeURI ('<?xml version="1.0" encoding="UTF-8" standalone="no"?>')+svg.html()

  hsvToRgb: (h, s, v) =>
    r = undefined
    g = undefined
    b = undefined
    i = Math.floor(h * 6)
    f = h * 6 - i
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    switch i % 6
      when 0
        r = v
        g = t
        b = p
      when 1
        r = q
        g = v
        b = p
      when 2
        r = p
        g = v
        b = t
      when 3
        r = p
        g = q
        b = v
      when 4
        r = t
        g = p
        b = v
      when 5
        r = v
        g = p
        b = q
    [Math.floor(r * 255), Math.floor(g * 255), Math.floor(b * 255)]

  powerColor: (value) =>
    #for a value from 0..1 return color from green to red
    h = (0.9-value) * 0.4   #hue, 0.4 = green
    s = 0.9           #saturation
    b = 0.9           #brightness

    @hsvToRgb h,s,b

  set_realtime: (value) =>
    @realtime = value

    #we were drawing with setInterval before, cancel the interval timer
    if @realtime
      clearInterval(@realtime_timer)
      physics.update()
    else
      @realtime_timer = setInterval(@physics.update, 1)

  set_graph_animated: (value) =>
    physics.abc.graph.renderer.draw_graph_animated = value
    physics.abc.graph.renderer.redraw()

  set_color_activation: (value) =>
    physics.abc.graph.renderer.draw_color_activation = value
    physics.abc.graph.renderer.redraw()

  set_activation: (value) =>
    physics.abc.graph.renderer.draw_activation = value
    physics.abc.graph.renderer.redraw()

  set_draw_semni: (value) =>
    physics.abc.graph.renderer.draw_semni = value
    physics.abc.graph.renderer.redraw()

  set_pause_drawing: (value) =>
    physics.abc.graph.renderer.pause_drawing = value

  set_save_periodically: (value) =>
    physics.abc.save_periodically = value

  set_draw_edge_labels: (value) =>
    physics.abc.graph.renderer.draw_edge_labels = value
    physics.abc.graph.renderer.redraw()

#set up 60 fps animation loop (triggers physics)
window.requestAnimFrame = (->
  window.oRequestAnimationFrame or
  window.webkitRequestAnimationFrame or
  window.mozRequestAnimationFrame or
  window.oRequestAnimationFrame or
  window.msRequestAnimationFrame or
  (callback, element) ->
    #fall back for other/old browsers
    window.setTimeout callback, 1000 / 60
)()

$ ->
    ##after document load: read some values from the ui into variables so the ui sets the defaults
  p = new simni.Physics()
  simni.Ui = ui
  ui = new ui(p)
  p.ui = ui
  logging = new simni.Logging(p)

  #make instances global for external access
  window.physics = p
  window.ui = ui
  window.logging = logging
