#methods for exploring postures, ABC style

class posture
  constructor: (position, csl_mode, x_pos=0) ->
    @position = position  # [body angle, hip joint angle, knee joint angle]
    @csl_mode = csl_mode  # [upper, lower]
    @edges_in = []
    @edges_out = []
    @body_x = x_pos
    @length = 1  #quirk for searchSubarray

  #method to determine if this posture is near another one
  eps = 0.6 #0.35
  isClose: (a,i, b,j) =>
      Math.abs(a.position[0] - b[j].position[0]) < eps and Math.abs(a.position[1] - b[j].position[1]) < eps and Math.abs(a.position[2] - b[j].position[2]) < eps and a.csl_mode[0] == b[j].csl_mode[0] and a.csl_mode[1] == b[j].csl_mode[1]

class abc
  constructor: ->
    @postures = []  #posture graph
    @last_posture = null
    @explore_active = false
    
    @graph = arbor.ParticleSystem() # create the graph with sensible repulsion/stiffness/friction
    @graph.parameters  # use center-gravity to make the graph settle nicely (ymmv)
      repulsion: 5000
      stiffness: 100
      friction: .5
      gravity: true
#      timeout: 5
#      fps: 10
     
    @graph.renderer = new Renderer("#viewport", @graph)

  toggleExplore: =>
    if not physics.upper_joint.csl_active
      $("#toggle_csl").click()
    @explore_active = not @explore_active

  searchSubarray: (sub, array, cmp) =>
    #returns index(es) if subarray is found in array using cmp (list1, index1, list2, index2) as comparator
    #otherwise false
    found = []
    for i in [0..array.length-sub.length] by 1
      for j in [0..sub.length-1] by 1
        unless cmp(sub,j, array,i+j)
          break
      if j == sub.length
        found.push i
        i = _i = i+sub.length
    
    if found.length is 0
      return false
    else
      return found

  MAX_UNIX_TIME = 1924988399 #31/12/2030 23:59:59
  time = MAX_UNIX_TIME
  trajectory = []   #last n state points
  detectAttractor: (body, upper_joint, lower_joint) =>
    #detect if current csl mode found a posture
    #TODO:
    #- gb sometimes is to small to correctly prepare next contraction
    #  (sometimes very quickly same pose is detected again)
    #  should try to use smaller gb and initialize next contraction csl integrator +/-
    
    if not physics.run
      return
    
    p_body = Math.abs body.GetAngle()     #p = φ
    p_hip = Math.abs upper_joint.GetJointAngle()
    p_knee = Math.abs lower_joint.GetJointAngle()
    
    #find attractors, take a sample of trajectory and try to find it multiple times in the
    #past trajectory (with epsilon), hence (quasi)periodic behaviour
    if trajectory.length==3000   #corresponds to max periode duration that can be detected
      trajectory.shift()

    trajectory.push [p_body, p_hip, p_knee]

    if trajectory.length > 200 and (Date.now() - time) > 2000
      #take last 40 points
      last = trajectory.slice(-40)
      eps=0.025
      d = @searchSubarray last, trajectory, (a,i, b,j) ->
        Math.abs(a[i][0] - b[j][0]) < eps and Math.abs(a[i][1] - b[j][1]) < eps and Math.abs(a[i][2] - b[j][2]) < eps
      #console.log(d)

      if d.length > 4    #need to find sample more than once to be periodic
        #save posture
        position = trajectory.pop()
        @savePosture position, body, upper_joint, lower_joint

        #get rid of saved trajectory
        trajectory = []

      time = Date.now()

    if (Date.now() - @graph.renderer.click_time) > 5000
      @graph.renderer.click_time = Date.now()
      #TODO: find out why starting/stopping doesn't work, use not-minified arbor code and step through
      #@graph.stop()
      #hack for now
      if isNaN @graph.energy().max
        @graph.renderer.draw_graphics = false

  savePosture: (position, body, upper_csl, lower_csl) =>
    parent = this
    addEdge = (start_node, target_node, edge_list=start_node.edges_out) ->
      if target_node not in edge_list and parent.postures.length > 1
        console.log("adding edge from posture " + start_node.position + " to posture: " + target_node.position)
        edge_list.push target_node
        n0 = start_node.position.toString()
        n1 = target_node.position.toString()
        parent.graph.addEdge n0, n1, {"distance": (target_node.body_x - start_node.body_x).toFixed 4}
        parent.graph.current_node = parent.graph.getNode(n1)
        parent.graph.current_node.data.label = target_node.csl_mode

        #re-enable suspended graph layouting for a bit to find new layout
        parent.graph.renderer.draw_graphics = true
        parent.graph.renderer.click_time = Date.now()

    p = new posture(position, [upper_csl.csl_mode, lower_csl.csl_mode], body.GetPosition().x)
    found = @searchSubarray p, @postures, p.isClose
    if not found
      #we dont have something close to this posture yet, add it
      console.log("found new pose/attractor: " + p.position)
      @postures.push(p)
    else
      #we have this posture already
      f = found[0]
      p = @postures[f]
    
    #add node+edges
    if @last_posture and @postures.length > 1
      addEdge @last_posture, p
      
      #save posture image
      ctx = $("#simulation canvas")[0].getContext('2d')
      x = physics.body.GetWorldCenter().x * physics.debugDraw.GetDrawScale()
      y = physics.body.GetWorldCenter().y * physics.debugDraw.GetDrawScale()
      range = 120
      imageData = ctx.getImageData x-range, y-range, range*2, range*2

      #loop over each pixel and make white pixels (the background) transparent
      pix = imageData.data
      for i in [0..pix.length-4]
        if pix[i] is 255 and pix[i+1] is 255 and pix[i+2] is 255
          pix[i+4] = 0

      #scale image data
      newCanvas = $("<canvas>").attr("width", imageData.width).attr("height", imageData.height)[0]
      ctx = newCanvas.getContext("2d")
      ctx.putImageData imageData, 0, 0

      #save in node
      ctx2 = $("#tempimage")[0].getContext('2d')
      ctx2.clearRect(0,0, ctx2.canvas.width, ctx2.canvas.height)
      ctx2.scale(0.5, 0.5)
      ctx2.drawImage(newCanvas, 0, 0)
      @graph.getNode(p.position.toString()).data.imageData = ctx2.getImageData 0, 0, range * 2, range *2
      ctx2.scale(2,2)

    @last_posture = p
    @newCSLMode()


  compareModes: (a, b) =>
    a[0] == b[0] && a[1] == b[1]
    
  newCSLMode: =>
    #TODO: try different strategies
    #- randomly select next mode + different mode than the one before
    #- try new mode that is not in neighbors of last node
    #- change csl mode of the joint that was not changed from the node before
    
    #random csl modes
    which = Math.floor(Math.random()*2)
    if which
      loop
        mode = ["r+", "r-", "c"][Math.floor(Math.random()*2.99)]
        break unless @last_posture.csl_mode[0] == mode
      ui.set_csl_mode_upper mode
    else
      loop
        mode = ["r+", "r-", "c"][Math.floor(Math.random()*2.99)]
        break unless @last_posture.csl_mode[1] == mode
      ui.set_csl_mode_lower ["r+", "r-", "c"][Math.floor(Math.random()*2.99)]


  limitCSL: (upper_joint, lower_joint) =>
    # if csl goes against limit, set to release mode in same direction with
    # current csl value as bias
    if upper_joint.csl_active and upper_joint.csl_mode is "c"
      mc = upper_joint.motor_control
      limit = 20
      if Math.abs(mc) > limit
        if mc > limit
          ui.set_csl_mode_upper "r+", false
          $("#gb_param_upper").val(limit)
          physics.upper_joint.gb = limit
        else if mc < -limit
          ui.set_csl_mode_upper "r-", false
          $("#gb_param_upper").val(-limit)
          physics.upper_joint.gb = -limit
        upper_joint.csl_mode = "c"
      
    if lower_joint.csl_active and lower_joint.csl_mode is "c"
      mc = lower_joint.motor_control
      if Math.abs(mc) > limit
        if mc > limit
          ui.set_csl_mode_lower "r+", false
          $("#gb_param_lower").val(limit)
          physics.lower_joint.gb = limit
        else if mc < -limit
          ui.set_csl_mode_lower "r-", false
          $("#gb_param_lower").val(-limit)
          physics.lower_joint.gb = -limit
        lower_joint.csl_mode = "c"


  update: (body, upper_joint, lower_joint) =>
    @limitCSL upper_joint, lower_joint
    if @explore_active
      @detectAttractor body, upper_joint, lower_joint
