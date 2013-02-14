#methods for exploring postures, ABC style

if (typeof Array::clone != 'function')
  Array::clone = ->
    cloned = []
    for i in this
      cloned.push i
    cloned

if (typeof String::startsWith != 'function')
  String::startsWith = (input) ->
    this.substring(0, input.length) == input

class posture   #i.e. node
  constructor: (position, csl_mode=[], x_pos=0, timestamp=Date.now()) ->
    @name = -1
    @csl_mode = csl_mode  # [upper, lower]
    @position = position  # [body angle, hip joint angle, knee joint angle]  #TODO: rename to configuration
    @body_x = x_pos
    @timestamp = timestamp
    @edges_out = []
    @exit_directions = [0,0,0,0]   #h+,h-,k+,k- : list of how often each direction of each joint this posture has been travelled from
    @length = 1  #quirk for searchSubarray

  isEqualTo: (node) =>
    @position[0] == node.position[0] and @position[1] == node.position[1] and @position[2] == node.position[2] and @csl_mode[0] == node.csl_mode[0] and @csl_mode[1] == node.csl_mode[1]

  getEdgeTo: (target) =>
    for edge in @edges_out
      return edge if edge.target_node is target

  #method to determine if this posture is near another one
  e = 0.35  #0.6  #default possible posture distance (quite large...)
  isCloseExplore: (a,i, b,j, eps=e) =>
      Math.abs(a.position[0] - b[j].position[0]) < eps and Math.abs(a.position[1] - b[j].position[1]) < eps and Math.abs(a.position[2] - b[j].position[2]) < eps and a.csl_mode[0] == b[j].csl_mode[0] and a.csl_mode[1] == b[j].csl_mode[1]

  isClose: (a,b=this, eps=0.25) =>
      Math.abs(a.position[0] - b.position[0]) < eps and Math.abs(a.position[1] - b.position[1]) < eps and Math.abs(a.position[2] - b.position[2]) < eps and a.csl_mode[0] == b.csl_mode[0] and a.csl_mode[1] == b.csl_mode[1]

class transition  #i.e. edge
  constructor: (start_node, target_node) ->
    @start_node = start_node
    @target_node = target_node

    @distance = 0 # distance the body traveled
    @timedelta = 0 # time the transition took

  toString: =>
    @start_node + "->" + @target_node

  isInList: (list) =>
    for t in list
      return true if @start_node is t.start_node and @target_node is t.target_node
    return false

class postureGraph
  #TODO: save graph, load graph (+ arbor graph)
  constructor: () ->
    @nodes = []  #list of the posture nodes
    @walk_circle_active = false

  addNode: (node) =>
    node.name = @nodes.length
    @nodes.push node

  getNode: (index) =>
    @nodes[index]

  length: =>
    @nodes.length

  findElementaryCircles: =>
    #implement Tarjan's algorithm for finding circles in directed graphs
    #expects no negative weights, no multiedges
    # R. Tarjan, Enumeration of the elementary circuits of a directed graph, SIAM Journal on Computing,
    # 2 (1973), pp. 211-216
    # based on an implementation from https://github.com/josch/cycles_tarjan/blob/master/cycles.py
    # might be slow for larger graphs, then consider using
    # Enumerating Circuits and Loops in Graphs with Self-Arcs and Multiple-Arcs, K.A.Hawick and H.A.James,
    # Proc. 2008 International Conference on Foundations of Computer Science (FCS'08), Las Vegas, USA, 14-17 July 2008

    #prepare node lists
    A = []
    A.push [] for a in [0..@nodes.length-1]

    for num in [0..@nodes.length-1]
      node = @nodes[num]
      break unless node
      for edge in node.edges_out
        A[num].push edge.target_node.name

    point_stack = []
    marked = {}
    marked_stack = []
    circles = []

    #walk through edges from node v depth-first, marking nodes and remembering the path
    parent = this
    backtrack = (v) ->
      f = false
      point_stack.push(v)
      marked[v] = true
      marked_stack.push(v)
      for w in A[v]
        if w<s
          A[w] = 0
        else if w==s
          #we found a circle
          path = []
          d = 0
          t = 0
          for n in [1..point_stack.length]
            #we're at the last node of the path, insert edge to first node
            if n is point_stack.length then m = n-1; n = 0; else m = n-1
            edge = parent.nodes[point_stack[m]].getEdgeTo parent.nodes[point_stack[n]]
            path.push edge
            d += edge.distance
            t += edge.timedelta
          path = path.concat [d, t, (d/t)*1000]
          circles.push(path)

          f = true
        else if not marked[w]
          f = backtrack(w) or f
      if f
        while marked_stack.slice(-1)[0] != v
          u = marked_stack.pop()
          marked[u] = false
        marked_stack.pop()
        marked[v] = false
      point_stack.pop()
      return f

    #initialise markers
    for i in [0..A.length-1]
      marked[i] = false

    #start walking from every node
    for s in [0..A.length-1]
      backtrack(s)
      while marked_stack.length
        u = marked_stack.pop()
        marked[u] = false

    #sort by travel distance
    @circles = circles.sort (a,b) ->
      if a.slice(-1)[0]<=b.slice(-1)[0] then -1 else 1

  walkCircle: =>
    if @circles
      if @walk_circle_active
        @walk_circle_active = false
        @best_circle.length = 0
        @best_circle = undefined
      else
        p.abc.explore_active = false

        @best_circle = @circles.slice(-1)[0]  #last circle is the one with largest distance

        #TODO: go to first posture before we can start walking
        #find path from current posture to this one
        #use best_circle[0].start_node .csl_mode .position

        @walk_circle_active = true

        #start with first transition
        @best_circle[0].active = true
        p.abc.graph.renderer.redraw()


class abc
  constructor: ->
    @posture_graph = new postureGraph()   # posture graph, logical representation
    @last_posture = null             # the posture that was visited/created last
    @previous_posture = null         # the posture that was visited before
    @explore_active = false

    @graph = arbor.ParticleSystem()  # display graph, has its own nodes and edges and data for display
    @graph.parameters                # use center-gravity to make the graph settle nicely (ymmv)
      repulsion: 500 #1000
      stiffness: 20 #100
      friction: .5
      gravity: true
#      timeout: 5
#      fps: 10

    @graph.renderer = new RendererSVG("#viewport_svg", @graph, @)
    @mode_strategy = "unseen"

  toggleExplore: =>
    if not physics.upper_joint.csl_active
      $("#toggle_csl").click()
    @explore_active = not @explore_active
    if @explore_active
      console.log "start explore run at "+new Date

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

  wrapAngle: (angle) =>
    twoPi = 2*Math.PI
    return angle - twoPi * Math.floor( angle / twoPi )

  MAX_UNIX_TIME = 1924988399 #31/12/2030 23:59:59
  time = MAX_UNIX_TIME
  trajectory = []   #last n state points
  detectAttractor: (body, upper_joint, lower_joint, action) =>
    #detect if current csl mode found a posture
    if not physics.run
      return

    p_body = @wrapAngle body.GetAngle()     #p = φ
    p_hip = upper_joint.GetJointAngle()
    p_knee = lower_joint.GetJointAngle()

    #find attractors, take a sample of trajectory and try to find it multiple times in the
    #past trajectory (with epsilon), hence (quasi)periodic behaviour
    if trajectory.length==10000   #corresponds to max periode duration that can be detected
      trajectory.shift()

    trajectory.push [p_body, p_hip, p_knee]

    if trajectory.length > 200 and (Date.now() - time) > 2000
      #take last 50 points
      last = trajectory.slice(-50)
      eps=0.025
      d = @searchSubarray last, trajectory, (a,i, b,j) ->
        Math.abs(a[i][0] - b[j][0]) < eps and Math.abs(a[i][1] - b[j][1]) < eps and Math.abs(a[i][2] - b[j][2]) < eps
      #console.log(d)

      if d.length > 3    #need to find sample more than once to be periodic
        #found a posture, call user method
        position = trajectory.pop()
        action(position, @)

        #get rid of saved trajectory
        trajectory = []

      time = Date.now()

    if (Date.now() - @graph.renderer.click_time) > 5000
      @graph.renderer.click_time = Date.now()
      @graph.stop()

  savePosture: (position, body, upper_csl, lower_csl) =>
    parent = this
    addEdge = (start_node, target_node, edge_list=start_node.edges_out) ->
      edge = new transition start_node, target_node
      if not edge.isInList(edge_list) and parent.posture_graph.length() > 1 and not start_node.isEqualTo target_node
        console.log("adding edge from posture " + start_node.name + " to posture: " + target_node.name)

        #add new edge to logic graph
        distance = target_node.body_x - start_node.body_x
        edge.distance = distance
        timedelta = target_node.timestamp - start_node.timestamp
        edge.timedelta = timedelta
        edge_list.push edge

        #display new edge
        n0 = start_node.name
        n1 = target_node.name

        #position new node close to previous one (if there is one)
        if parent.posture_graph.length() > 2
          source_node = parent.graph.getNode n0
          parent.graph.getNode(n1) or parent.graph.addNode n1, {'x': source_node.p.x + 0.01, 'y': source_node.p.y + 0.01}

        parent.graph.addEdge n0, n1,
          distance: distance.toFixed(3)
          timedelta: timedelta

        #if we're here for the first time, n0 is not yet initialized (this time addEdge adds two nodes)
        if n0 == 0 and n1 == 1
          init_node = parent.graph.getNode(n0)
          init_node.data.label = start_node.csl_mode
          init_node.data.number = start_node.name

        parent.graph.current_node = current_node = parent.graph.getNode(n1)
        current_node.data.label = target_node.csl_mode
        current_node.data.number = target_node.name

        #re-enable suspended graph layouting for a bit to find new layout
        parent.graph.start(true)
        parent.graph.renderer.click_time = Date.now()

    p = new posture(position, [upper_csl.csl_mode, lower_csl.csl_mode], body.GetWorldCenter().x)
    found = @searchSubarray p, @posture_graph.nodes, p.isCloseExplore
    if not found
      #we dont have something close to this posture yet, add it
      console.log("found new pose/attractor: " + p.position)
      @posture_graph.addNode p
    else
      #we have this posture already, update it
      f = found[0]
      current_p = p
      p = @posture_graph.getNode f

      #update to mean of positions
      p.position[0] = (current_p.position[0] + p.position[0]) / 2
      p.position[1] = (current_p.position[1] + p.position[1]) / 2
      p.position[2] = (current_p.position[2] + p.position[2]) / 2

      #update graph render stuff
      @graph.current_node = @graph.getNode p.name
      @graph.renderer.redraw()

    #add node+edges
    if @last_posture and @posture_graph.length() > 1
      #refresh target position to current x so distance calc works
      p.body_x = body.GetWorldCenter().x
      p.timestamp = Date.now()
      addEdge @last_posture, p

      ## save posture image
      ###
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
      n = @graph.getNode(p.name)
      n.data.imageData = ctx2.getImageData 0, 0, range * 2, range *2
      ctx2.scale(2,2)
      ###

      #save posture for graph drawing
      n = @graph.getNode(p.name)
      n.data.positions = [physics.body.GetPosition(), physics.body2.GetPosition(), physics.body3.GetPosition()]
      n.data.angles = [physics.body.GetAngle(), physics.body2.GetAngle(), physics.body3.GetAngle()]

    @previous_posture = @last_posture
    @last_posture = p
    @newCSLMode()

  compareModes: (a, b) =>
    if not a or not b
      return false
    a[0] == b[0] && a[1] == b[1]

  set_strategy: (strategy) =>
    @mode_strategy = strategy

  newCSLMode: =>
    #random: randomly select next mode + different mode than the current one
    #unseen: try new mode that is not in neighbors of the current one, prefer previous mode. if all are seen, try random

    #TODO: try more strategies
    #- change csl mode of the joint that was not changed from the node before
    #- alternate between c and r modes

    set_random_mode = (curent_mode) ->
      which = Math.floor(Math.random()*2)    #just change one joint at a time
      if which
        loop    #try as long as we dont have a new mode to prevent same mode again
          mode = ["r+", "r-", "c"][Math.floor(Math.random()*2.99)]
          break if current_mode[0] isnt mode
        ui.set_csl_mode_upper mode
      else
        loop
          mode = ["r+", "r-", "c"][Math.floor(Math.random()*2.99)]
          break if curent_mode[1] isnt mode
        ui.set_csl_mode_lower mode

    next_mode_for_direction = (old_mode, direction) ->
      if direction is "+"
        switch old_mode
          when "r+" then return "c"
          when "r-" then return "r+"
          when "c" then return "r+"
          when "s-" then return "r+"
          when "s+" then return "s+"  #this one should not be taken anyway
      else if direction is "-"
        switch old_mode
          when "r+" then return "r-"
          when "r-" then return "c"
          when "c" then return "r-"
          when "s+" then return "r-"
          when "s-" then return "s-"

    current_mode = @last_posture.csl_mode
    if @previous_posture
      previous_mode = @previous_posture.csl_mode
    else
      previous_mode = undefined


    if @mode_strategy is "unseen"
      #use list of +/- possibilities for each joint: -1 stall, 0 not visited, >0 visited count
      #[h+,h-,k+,k-]

      #try to go back to previous mode (same joint)
      if @last_dir and @last_joint_index in [0, 1]
        back_dir = if @last_dir is "+" then "-" else "+"         #reverse direction
        back_dir_offset = if @last_dir is "+" then 0 else 1      #offset for index
        dir_index = @last_joint_index+back_dir_offset            #get index for reverse direction
        if @last_posture.exit_directions[dir_index] is 0         #if we have not gone this direction from here, we go back
          next_mode = next_mode_for_direction current_mode[@last_joint_index], back_dir
          direction = back_dir
          joint_index = @last_joint_index
        else
          dir_index = undefined

      unless next_mode
        #we are not going back again

        if 0 in @last_posture.exit_directions
          #we have not seen one of the directions yet
          dir_index =  @last_posture.exit_directions.indexOf 0
          found_index = 0

          #go through all directions we haven't tried yet and see if switching the joint is possible
          while found_index > -1
            joint_index = Math.ceil((found_index+1) / 2) - 1
            if joint_index != @last_joint_index
              dir_index = found_index
              break
            found_index = @last_posture.exit_directions.indexOf 0, found_index+1
        else
          #we went all directions already, take random one
          while not dir_index? or @last_posture.exit_directions[dir_index] is -1
            dir_index = Math.floor(Math.random()*3.99)  #choose a random one of four, replace four by # of joints - 0.01

        joint_index = Math.ceil((dir_index+1) / 2) - 1
        if dir_index % 2
          direction = "+"  #odds are +, evens are -
        else
          direction = "-"
        next_mode = next_mode_for_direction current_mode[joint_index], direction

      @last_posture.exit_directions[dir_index] += 1   #count number of times we went this way
      if joint_index is 0
        ui.set_csl_mode_upper next_mode
      else
        ui.set_csl_mode_lower next_mode

      @last_dir = direction
      @last_joint_index = joint_index

    else if @mode_strategy is "random"
      set_random_mode current_mode

  limitCSL: (upper_joint, lower_joint) =>
    # if csl goes against limit, set to release mode in same direction with
    # current csl value as bias (stall mode)
    if upper_joint.csl_active and upper_joint.csl_mode is "c"
      mc = upper_joint.motor_control
      limit = 15
      if Math.abs(mc) > limit #and upper_joint.GetJointSpeed() closeTo 0
        if mc > limit
          ui.set_csl_mode_upper "s+"
          $("#gb_param_upper").val(limit)
          physics.upper_joint.gb = limit
        else if mc < -limit
          ui.set_csl_mode_upper "s-"
          $("#gb_param_upper").val(-limit)
          physics.upper_joint.gb = -limit

    if lower_joint.csl_active and lower_joint.csl_mode is "c"
      mc = lower_joint.motor_control
      if Math.abs(mc) > limit
        if mc > limit
          ui.set_csl_mode_lower "s+"
          $("#gb_param_lower").val(limit)
          physics.lower_joint.gb = limit
        else if mc < -limit
          ui.set_csl_mode_lower "s-", false
          $("#gb_param_lower").val(-limit)
          physics.lower_joint.gb = -limit


  update: (body, upper_joint, lower_joint) =>
    @limitCSL upper_joint, lower_joint
    if @explore_active
      @detectAttractor body, upper_joint, lower_joint, (position, parent) ->
        #save posture
        parent.savePosture position, body, upper_joint, lower_joint

    if @posture_graph.walk_circle_active
      @detectAttractor body, upper_joint, lower_joint, (position, parent) ->
        current_posture = new posture(position, [physics.upper_joint.csl_mode, physics.lower_joint.csl_mode])
        for edge in parent.posture_graph.best_circle
          if edge.start_node.isClose(current_posture)
            edge.active = true

            #update graph display
            parent.graph.current_node = parent.graph.getNode(edge.start_node.name)
            parent.graph.renderer.redraw()

          if edge.target_node.isClose(current_posture)
            #this is the last edge we already travelled
            edge.active = false

          if edge.active
            #go to next posture
            csl_mode = edge.target_node.csl_mode
            csl_mode[0] = if csl_mode[0].startsWith("s") then "c" else csl_mode[0]
            csl_mode[1] = if csl_mode[1].startsWith("s") then "c" else csl_mode[1]

            ui.set_csl_mode_upper csl_mode[0]
            ui.set_csl_mode_lower csl_mode[1]

            #we found a matching edge so there should be no other with the current posture
            break

        #TODO: if we're here, there was no matching edge in the circle which means we are not there yet
        #find a path from the current posture (we found one) to one posture on the circle (the shortest/safest please)

