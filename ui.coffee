## ui helper methods

class ui
  constructor: (physics) ->
    @draw_graphics = true
    @physics = physics
    @init()
    @halftime = true
  
  update: =>
      if @draw_graphics and @halftime
        @physics.world.DrawDebugData()
      @halftime = not @halftime

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

    #request first frame from browser to start update cycle
    requestAnimFrame @physics.update

    #fps counter
    window.stats = new Stats()
    window.stats.setMode 0
    # align top-left
    window.stats.domElement.style.position = "absolute"
    window.stats.domElement.style.left = "0px"
    window.stats.domElement.style.top = "0px"
    document.body.appendChild window.stats.domElement

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
      window.mouseX = `undefined`
      window.mouseY = `undefined`
    ), true

    handleMouseMove = (e) ->
      window.mouseX = (e.clientX - canvasPosition.x) / physics.debugDraw.GetDrawScale()
      window.mouseY = (e.clientY - canvasPosition.y) / physics.debugDraw.GetDrawScale()

    window.getBodyAtMouse = ->
      window.mousePVec = new b2Vec2(mouseX, mouseY)
      aabb = new b2AABB()
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
      @physics.lower_joint.m_maxMotorTorque = beta
    else
      @physics.lower_joint.m_maxMotorTorque = beta
    @physics.beta = beta

  set_posture: (bodyAngle, hipAngle, kneeAngle, hipCsl, kneeCsl) =>
    #TODO, hm, how to properly set joint angles? sum angles going up from body
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
    release_bias_hip = 0.7
    release_gf = 0.99
    contract_gf_hip = 1.0030 #1.0025 #1.006
    gi_hip = 30 #27 #50
    
    if hipCSL is "r+"
      gf = release_gf
      gb = release_bias_hip
    else if hipCSL is "r-"
      gf = release_gf
      gb = -release_bias_hip
    else if hipCSL is "c"
      gf = contract_gf_hip
      gb = 0
      
    if change_select
      #re-select select option in case we came from another function
      $("#csl_mode_hip option[value='" + hipCSL + "']").attr "selected", true
    
    $("#gi_param_upper").val(gi_hip)
    @physics.upper_joint.gi = gi_hip
   
    $("#gf_param_upper").val(gf)
    @physics.upper_joint.gf = gf
    
    $("#gb_param_upper").val(gb)
    @physics.upper_joint.gb = gb
    @physics.upper_joint.csl_mode = hipCSL

  set_csl_mode_lower: (kneeCSL, change_select=true) =>
    release_bias_knee = 0.7
    contract_gf_knee = 1.0020 #1.0015 #1.006
    release_gf = 0.99
    gi_knee = 35 #26 #50

    if kneeCSL is "r+"
      gf = release_gf
      gb = release_bias_knee
    else if kneeCSL is "r-"
      gf = release_gf
      gb = -release_bias_knee
    else if kneeCSL is "c"
      gf = contract_gf_knee
      gb = 0

    if change_select
      #re-select select option in case we came from another function
      $("#csl_mode_knee option[value='" + kneeCSL + "']").attr 'selected', true
    
    $("#gi_param_lower").val(gi_knee)
    @physics.lower_joint.gi = gi_knee
    
    $("#gf_param_lower").val(gf)
    @physics.lower_joint.gf = gf
    
    $("#gb_param_lower").val(gb)
    @physics.lower_joint.gb = gb
    @physics.lower_joint.csl_mode = kneeCSL

  toggleRecorder: =>
    @physics.startLog = true
    @physics.recordPhase = !@physics.recordPhase

  getLogfile: =>
    @physics.recordPhase = false
    location.href = 'data:text;charset=utf-8,'+encodeURI Functional.reduce((x, y) ->
      x + y + "\n"
    , "", @physics.logged_data)

    return

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

###
#set up 60 fps animation loop (triggers physics)
lastTime = 0
vendors = ['ms', 'moz', 'webkit', 'o']
@cancelAnimationFrame or= @cancelRequestAnimationFrame
unless @requestAnimationFrame
  for vendor in vendors
    @requestAnimationFrame or= @[vendor+'RequestAnimationFrame']
    @cancelAnimationFrame = @cancelAnimationFrame or= @[vendor+'CancelRequestAnimationFrame']
unless @requestAnimationFrame
  @requestAnimationFrame = (callback, element) ->
    currTime = new Date().getTime()
    timeToCall = Math.max 0, 16 - (currTime - lastTime)
    id = @setTimeout (-> callback currTime + timeToCall), timeToCall
    lastTime = currTime + timeToCall
    id
unless @cancelAnimationFrame
  @cancelAnimationFrame = @cancelAnimationFrame = (id) -> clearTimeout id
###

$ ->
    ##after document load: read some values from the ui into variables so the ui sets the defaults
    p = new physics()
    #make physics global for external access
    window.physics = p

    ui = new ui(p)
    p.ui = ui
    window.ui = ui
