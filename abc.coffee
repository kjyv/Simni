#methods for exploring postures, ABC style

class posture
  constructor: (position, csl_mode) ->
    @position = position  # [body angle, hip joint angle, knee joint angle]
    @csl_mode = csl_mode  # [upper, lower]
    @edges_in = []
    @edges_out = []
    @length = 1  #quirk for searchSubarray

  #determine if this posture is near another one
  eps = 0.15
  isClose: (a,i, b,j) =>
      Math.abs(a.position[0] - b[j].position[0]) < eps and Math.abs(a.position[1] - b[j].position[1]) < eps and Math.abs(a.position[2] - b[j].position[2]) < eps

class abc
  constructor: ->
    @postures = []  #posture graph
    @last_posture = null
    @explore_active = false
    
    @graph = arbor.ParticleSystem() # create the graphtem with sensible repulsion/stiffness/friction
    @graph.parameters  # use center-gravity to make the graph settle nicely (ymmv)
      repulsion: 5000
      stiffness: 100
      friction: .5
      gravity: true
      timeout: 5
      fps: 10
    @graph.screenPadding 20,20,20,20
     
    @graph.renderer = new Renderer("#viewport")
    #TODO: find out why starting/stopping doesn't work, use not-minified arbor code and step through
    #    @graph.stop()

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
  time = time2 = MAX_UNIX_TIME
  trajectory = []   #last n state points
  detectAttractor: (body, upper_joint, lower_joint) =>
    #detect if current csl mode found a posture
    #TODO:
    #- build proper graph a found postures, collect modes, distance traveled per edge
    #- gb sometimes is to small to correctly prepare next contraction
    #  (sometimes very quickly same pose is detected again)
    #- act on detection event following different strategies:
    #  randomly select next mode or try new mode that we have not seen yet or change joint csl that was
    #  not changed before

    #get angle velocities
    #dp_body = Math.abs body.GetAngularVelocity()     #dp means ∆φ
    #dp_hip = Math.abs upper_joint.GetJointSpeed()
    #dp_knee = Math.abs lower_joint.GetJointSpeed()
    
    p_body = Math.abs body.GetAngle()     #dp means ∆φ
    p_hip = Math.abs upper_joint.GetJointAngle()
    p_knee = Math.abs lower_joint.GetJointAngle()
    
    #find attractors, take a sample of trajectory and try to find it multiple times in the
    #past trajectory (with epsilon), hence (quasi)periodic behaviour
    if trajectory.length==3000   #corresponds to max periode duration that can be detected
      trajectory.shift()

    trajectory.push [p_body, p_hip, p_knee]

    if trajectory.length > 200 and (new Date().getTime() - time) > 2000
      #take last 40 points
      last = trajectory.slice(-40)
      eps=0.025
      d = @searchSubarray last, trajectory, (a,i, b,j) ->
        Math.abs(a[i][0] - b[j][0]) < eps and Math.abs(a[i][1] - b[j][1]) < eps and Math.abs(a[i][2] - b[j][2]) < eps
      #console.log(d)

      if d.length > 3    #need to find sample more than once to be periodic
        #save posture
        position = trajectory.pop()
        @savePosture position, body, upper_joint, lower_joint

        #get rid of saved trajectory
        trajectory = []

      time = new Date().getTime()

    if (new Date().getTime() - time2) > 3000
      time2 = new Date().getTime()
      #@graph.stop()

  savePosture: (position, body, upper_csl, lower_csl) =>
    parent = this
    addEdge = (start_node, target_node, edge_list=start_node.edges_out) ->
      if target_node not in edge_list and parent.postures.length > 1
        console.log("adding edge from posture " + start_node.position + " to posture: " + target_node.position)
        edge_list.push target_node
        n0 = start_node.position.toString()
        n1 = target_node.position.toString()
        parent.graph.addEdge n0, n1
        parent.graph.getNode(n1).data.label = target_node.csl_mode

    p = new posture(position, [upper_csl.csl_mode, lower_csl.csl_mode])
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
      ctx = $("#simulation")[0].getContext('2d')
      x = physics.body.GetWorldCenter().x * physics.debugDraw.GetDrawScale()
      y = physics.body.GetWorldCenter().y * physics.debugDraw.GetDrawScale()
      imageData = ctx.getImageData x-90, y-90, 180, 180

      #loop over each pixel and make white pixels (the background) transparent
      pix = imageData.data
      for i in [0..pix.length-4]
        if pix[i] is 255 and pix[i+1] is 255 and pix[i+2] is 255
          pix[i+4] = 0

      #scale image data
      newCanvas = $("<canvas>").attr("width", imageData.width).attr("height", imageData.height)[0]
      ctx = newCanvas.getContext("2d")
      ctx.putImageData imageData, 0, 0

      #save for node
      ctx2 = $("#tempimage")[0].getContext('2d')
      ctx2.clearRect(0,0, ctx2.canvas.width, ctx2.canvas.height)
      ctx2.scale(0.25, 0.25)
      ctx2.drawImage(newCanvas, 0, 0)
      @graph.getNode(p.position.toString()).data.imageData = ctx2.getImageData 0, 0, ctx2.canvas.width / 8, ctx2.canvas.height / 8
      ctx2.scale(4,4)

    @last_posture = p



    @newCSLMode()

  newCSLMode: =>
    #random csl modes
    which = Math.floor(Math.random()*2)
    if which
      ui.set_csl_mode_upper ["r+", "r-", "c"][Math.floor(Math.random()*2.99)]
    else
      ui.set_csl_mode_lower ["r+", "r-", "c"][Math.floor(Math.random()*2.99)]

  addPostureToGraph: (posture) =>
    get_random_color = ->
      letters = "0123456789ABCDEF".split("")
      color = "#"
      i = 0

      while i < 6
        color += letters[Math.round(Math.random() * 15)]
        i++
      color

    #TODO: color nodes in clusters (best would be the same colors for the manifolds manfred used)
    #n = @graph.addNode(n1,
    #  'x': Math.random()/4
    #  'y': Math.random()/4
    #  label: n1
    #  color: get_random_color()
    #)
    #@graph.draw().startForceAtlas2()


  limitCSL: (upper_joint, lower_joint) =>
    #csl goes against limit, set to release mode with current csl value as bias
    if upper_joint.csl_active and upper_joint.csl_mode is "c"
      mc = upper_joint.motor_control
      limit = 20
      if Math.abs(mc) > limit
        if mc > limit
          ui.set_csl_mode_upper "r+"
          $("#gb_param_upper").val(limit)
          physics.upper_joint.gb = limit
        else if mc < -limit
          ui.set_csl_mode_upper "r-"
          $("#gb_param_upper").val(-limit)
          physics.upper_joint.gb = -limit
        upper_joint.csl_mode = "c"
      
    if lower_joint.csl_active and lower_joint.csl_mode is "c"
      mc = lower_joint.motor_control
      if Math.abs(mc) > limit
        if mc > limit
          ui.set_csl_mode_lower "r+"
          $("#gb_param_lower").val(limit)
          physics.lower_joint.gb = limit
        else if mc < -limit
          ui.set_csl_mode_lower "r-"
          $("#gb_param_lower").val(-limit)
          physics.lower_joint.gb = -limit
        lower_joint.csl_mode = "c"
      

  update: (body, upper_joint, lower_joint) =>
    @limitCSL upper_joint, lower_joint
    if @explore_active
      @detectAttractor body, upper_joint, lower_joint
