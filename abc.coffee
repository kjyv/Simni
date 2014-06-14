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

squared = (val) ->
  val*val

thresholdDistance = 0.15       #the euclidian distance that is considered to be small enough to say two postures are the same
thresholdExplore = 0.45        #distance for finding possibly equal postures, has to be verified with position control though

class posture   #i.e. node
  constructor: (configuration, csl_mode=[], x_pos=0, timestamp=Date.now()) ->
    @name = -99
    @csl_mode = csl_mode  # [upper, lower]
    @configuration = configuration  # [body angle, hip joint angle, knee joint angle]
    @mean_n = 1   #count the amount of configurations we have merged into this one
    @positions = []  #positions of the body part for svg drawing
    @body_x = x_pos
    @timestamp = timestamp
    @edges_out = []
    #@edges_in = []
    @exit_directions = [0,0,0,0]   #h+,h-,k+,k- : list of the target nodes for each direction of each joint
    @length = 1  #quirk for searchSubarray
    @activation = 1
    @subManifoldId = 0

  asJSON: =>
    #prevent circular references
    replacer = (edges)->
      new_edges = []
      for e in edges
        new_edges.push e.target_node.name
      new_edges

    JSON.stringify {"name":@name, "csl_mode":@csl_mode, "configuration":@configuration, "mean_n":@mean_n, "positions":@positions, "body_x":@body_x, "timestamp":@timestamp, "exit_directions":@exit_directions, "activation":@activation, "edges_out": replacer(@edges_out)}, null, 4

  getEdgeTo: (target) =>
    for edge in @edges_out
      return edge if edge.target_node is target

  getEdgeFrom: (source) =>
    for edge in source.edges_out
      return edge if edge.target_node is this

  isEqualTo: (node) =>
    @configuration[0] == node.configuration[0] and @configuration[1] == node.configuration[1] and @configuration[2] == node.configuration[2] and @csl_mode[0] == node.csl_mode[0] and @csl_mode[1] == node.csl_mode[1]

  #methods to determine if this posture is near another one
  euclidDistance: (to) =>
    squared(physics.abc.smallestAngleDistance(
      physics.abc.wrapAngle(@configuration[0])
      physics.abc.wrapAngle(to.configuration[0]))) +
    squared(@configuration[1] - to.configuration[1]) +
    squared(@configuration[2] - to.configuration[2])

  #detect if postures are close enough to be possibly the same
  isClose: (to, eps=thresholdDistance) =>
    @euclidDistance(to) < eps and @csl_mode[0] == to.csl_mode[0] and @.csl_mode[1] == to.csl_mode[1]

  #comparator for exploring
  isCloseExplore: (a,i, b,j) =>
    #TODO: also consider e.g. r+,s+ and r+,c equal. test that properly, might produce weird results...
    if a? and b and b[j]
      a.isClose(b[j], thresholdExplore)
    else
      false

  getSubmanifold: =>
    # 0   1   2    3    4     5     6      7     8      9    10   11    12     13   14    15 16
    # idx grp hip- hip+ knee- knee+ z_body y_hip x_knee stab ener t_hip t_knee mode xleft dx COGpos

    #test_p = new posture([0,0,0], @csl_mode, @body_x, @timestamp)
    old_dist = 2
    grp = 0
    for p in semni_manifold
      #test_p.configuration[0] = physics.abc.wrapAngle -p[6]
      #test_p.configuration[1] = physics.abc.wrapAngle -p[7]
      #test_p.configuration[2] = physics.abc.wrapAngle -p[8]
      #dist = @euclidDistance(test_p)
      dist = squared(physics.abc.wrapAngleManifold @configuration[0] - physics.abc.wrapAngleManifold -p[6]) +
      squared(@configuration[1] - -p[7]) +
      squared(@configuration[2] - -p[8])
      if dist < old_dist
        old_dist = dist
        grp = p[1]
        #console.log(p[0] + " distance: " + dist + " grp: " + grp)
    return grp


#end class posture

class transition  #i.e. edge
  constructor: (start_node, target_node) ->
    @start_node = start_node
    @target_node = target_node

    @distance = 0 # distance the body traveled
    @timedelta = 0 # time the transition took

    @csl_mode = []

  toString: =>
    @start_node.name + "->" + @target_node.name

  asJSON: =>
    JSON.stringify {"name": @toString(), "csl_mode":@csl_mode, "start_node": @start_node.name, "target_node": @target_node.name, "distance": @distance, "timedelta": @timedelta}, null, 4

  isInList: (list) =>
    for t in list
      return true if @start_node is t.start_node and @target_node is t.target_node
    return false

#end class transition

class postureGraph
  #class holding the abc learning graph structure
  #because arbor (library for drawing graphs) has it's own internal structure, there are two graphs in memory,
  #one for drawing and one that also holds the additional data and methods
  constructor: (arborGraph) ->
    @nodes = []  #list of the posture nodes
    @walk_circle_active = false
    @arborGraph = arborGraph

  addNode: (node) =>
    node.name = @nodes.length+1
    @nodes.push node
    node.subManifoldId = node.getSubmanifold()
    return node.name

  addPosture: (p) =>
    node_name = @addNode p
    console.log("found new posture: " + p.configuration + " (posture " + node_name + ")")
    window.logging.logNewPosture()
    manifoldRenderer.addFixpoint(manifoldRenderer.internalToManifold(p.configuration))

    #add an arbor node as well (not connected yet)
    data =
      label: p.csl_mode
      number: p.name
      activation: p.activation
      configuration: p.configuration
      positions: p.positions
      subManifoldId: p.subManifoldId
      visits: p.mean_n
    @arborGraph.addNode p.name, data

    return node_name

  deletePosture: (p_name) =>
    an = @arborGraph.getNode(p_name)

    #get outgoing edges
    edges = []
    for e in @getNodeByName(p_name).edges_out
      if e?
        edges.push e
    for n in @nodes   #search for incoming edges
      if n? and n.edges_out.length
        for e in [0..n.edges_out.length-1]
          en = n.edges_out[e]
          if en?
            if en.target_node.name is p_name  #found one
              edges.push en
              #delete start node references to edge
              for d in [0..3]
                if n.exit_directions[d] is p_name
                  n.exit_directions[d] = 0
              delete n.edges_out[e]

    #delete svg elements
    an.data.semni.remove()
    an.data.label_svg.remove()
    an.data.label_svg2.remove()
    an.data.label_svg3.remove()
    @arborGraph.renderer.svg_nodes[p_name].remove()
    delete @arborGraph.renderer.svg_nodes[p_name]
    for e in edges
      @arborGraph.renderer.svg_edges[e.start_node.name+"-"+e.target_node.name].remove()
      delete @arborGraph.renderer.svg_edges[e.start_node.name+"-"+e.target_node.name]

    #delete posture and arbor node
    @arborGraph.pruneNode(an)
    delete @nodes[p_name-1]

  getNodeByIndex: (index) =>
    @nodes[index]

  getNodeByName: (name) =>
    if name>0
      @nodes[name-1]
    else
      console.log("warning: tried to get node with index <= 0!")

  length: =>
    @nodes.length

  meanActivation: =>
    act = 0
    for n in @nodes
      if n?
        act += n.activation

    return act / @length()

  meanVisits: =>
    visits = 0
    for n in @nodes
      if n?
        visits += n.mean_n

    e = visits / @length()

    #standard deviation
    sigma = 0
    for n in @nodes
      if n?
        sigma += (n.mean_n - e)*(n.mean_n - e)

    console.log("sigma: " + Math.sqrt(sigma/@length()))
    return e

  maxVisits: =>
    visits = 0
    for n in @nodes
      if n?
        visits = if n.mean_n > visits then n.mean_n else visits
    return visits

  minVisits: =>
    visits = 99999
    for n in @nodes
      if n?
        visits = if n.mean_n < visits then n.mean_n else visits
    return visits

  findDuplicates: =>
    for n in @nodes
      if n?
        for nn in @nodes
          if nn?
            if n.name isnt nn.name and n.isClose(nn, thresholdExplore)
              console.log("duplicates "+ n.name+" and " + nn.name + ". distance " + n.euclidDistance(nn))
    return

  saveGaphToFile: =>
    graph_as_string = ""
    edges = []
    for n in @nodes
      graph_as_string += "\n"+n.asJSON()+","
      for e in n.edges_out
        edges.push e

    edges_as_string = ""
    for e in edges
      edges_as_string += "\n"+e.asJSON()+","

    #remove last comma
    graph_as_string = graph_as_string.substring(0, graph_as_string.length - 1)
    edges_as_string = edges_as_string.substring(0, edges_as_string.length - 1)
    location.href = 'data:text;charset=utf-8,'+encodeURI "{\n"+"\"nodes\": ["+graph_as_string+"],\n"+"\"edges\": ["+edges_as_string+"]"+"\n}"


  populateGraphFromJSON: (tj=null) =>
    #flatten file and parse it (parse chokes on newlines)
    tj = tj.replace(/(\r\n|\n|\r)/gm,"")
    t = JSON.parse(tj)

    #clear nodes we might already have
    @nodes = []

    #put the new nodes in
    for n in t.nodes
      nn = new posture(n.configuration, n.csl_mode, n.body_x, n.timestamp)
      nn.name = n.name
      nn.mean_n = n.mean_n
      nn.activation = n.activation
      nn.exit_directions = n.exit_directions
      nn.positions = n.positions
      nn.configuration = n.configuration
      nn.subManifoldId = nn.getSubmanifold()
      nn.visits = n.mean_n
      @nodes.push nn
      manifoldRenderer.addFixpoint(manifoldRenderer.internalToManifold(nn.configuration))

    #put in edges
    for e in t.edges
      n = @getNodeByName e.start_node
      nn = @getNodeByName e.target_node
      ee = new transition(n, nn)
      ee.csl_mode = e.csl_mode
      ee.distance = e.distance
      ee.timedelta = e.timedelta
      n.edges_out.push(ee)
      if n.edges_out.length > 4
        console.log("warning: more than 4 outgoing edges in " +n.name)

    #refresh display graph
    ag = @arborGraph
    @arborGraph.prune()

    @arborGraph.renderer.svg_nodes = {}
    @arborGraph.renderer.svg_edges = {}

    $("#viewport_svg svg g").slice(1).remove()
    $("#viewport_svg svg rect").remove()
    $("#viewport_svg svg text").remove()
    $("#viewport_svg svg line").remove()
    @arborGraph.renderer.initManifoldBar()
    @arborGraph.renderer.initVisitCountLegend()

    for n in @nodes
      physics.abc.drawManifoldStripe(n)
      for e in n.edges_out
        nn = e.target_node
        ag.addEdge(n.name, nn.name, {"label": e.csl_mode, "distance": e.distance, "timedelta": e.timedelta})
        source_node = ag.getNode(n.name)
        target_node = ag.getNode(nn.name)

        source_node.data =
          label: n.csl_mode
          number: n.name
          activation: n.activation
          configuration: n.configuration
          positions: n.positions
          subManifoldId: n.subManifoldId
          visits: n.mean_n

        target_node.data =
          label: nn.csl_mode
          number: nn.name
          activation: nn.activation
          configuration: nn.configuration
          positions: nn.positions
          subManifoldId: nn.subManifoldId
          visits: nn.mean_n
    return

  loadGraphFromFile: (files) =>
    readFile = (file, callback) ->
      reader = new FileReader()
      reader.onload = (evt) ->
        callback file, evt  if typeof callback is "function"
      reader.readAsBinaryString file

    if files.length > 0
      readFile files[0], (file, evt) ->
        physics.abc.posture_graph.populateGraphFromJSON evt.target.result

    @arborGraph.renderer.pause_drawing = false
    $("#graph_pause_drawing").attr('checked', false)
    @arborGraph.start(true)
    @arborGraph.renderer.click_time = Date.now()
    @arborGraph.renderer.redraw()


  populateGraphFromSemniFile: (data=null) =>
    csl_mode_to_string_mode = (mode) ->
      for m in [0..mode.length-1]
        mode[m] = ["r+","r-","c","s+","s-"][mode[m]]
      return mode

    #clear nodes we might already have
    @nodes = []

    data = data.split("\n")
    if data[data.length-1] is ''
      data.pop()

    #create the new nodes
    for l in data
      vals = []
      for v in l.split(" ")
        v = v.replace(/(\]|\[)/gm,"").split(",")
        for n in [0..v.length-1]
          v[n]=Number(v[n])
        vals.push(v)

      nn = new posture(vals[3], csl_mode_to_string_mode vals[2], 0, Date.now())
      nn.name = vals[0][0]
      nn.mean_n = 0
      nn.activation = vals[4][0]/100
      nn.exit_directions = vals[1]
      nn.positions = [0,0,0]

      #semni to angle mappings
      #body: 260 => 0
      #508 => 1.57
      #0 => -1.57

      #knee
      #5 => -3.194
      #640 => 0
      #1007 => 1.908

      #hip
      #361 => 0
      #895 => 2.716
      #176 => -0.94

      #TODO: use
      #body_angle = Math.atan2(accely, accelz)

      nn.configuration = [((((vals[3][0])-250)/1023)*2*Math.PI), ((vals[3][1]-361)/1023)*0.818*2*Math.PI, ((vals[3][2]-640)/1023)*0.818*2*Math.PI]
      nn.subManifoldId = nn.getSubmanifold()
      @nodes.push nn
      manifoldRenderer.addFixpoint(manifoldRenderer.internalToManifold(nn.configuration))

    #put in edges
    for l in data
      vals = []
      for v in l.split(" ")
        vals.push(v.replace(/(\]|\[)/gm,"").split(","))

      n = @getNodeByName vals[0][0]
      for e in vals[1]
        if e > 0 and e <= @nodes.length
          nn = @getNodeByName e
          if n and nn
            ee = new transition(n, nn)
            ee.csl_mode = csl_mode_to_string_mode vals[2]
            ee.distance = 0
            ee.timedelta = 0
            n.edges_out.push(ee)

    #refresh display graph
    ag = @arborGraph
    @arborGraph.prune()

    @arborGraph.renderer.svg_nodes = {}
    @arborGraph.renderer.svg_edges = {}

    $("#viewport_svg svg g").slice(1).remove()
    $("#viewport_svg svg rect").remove()
    $("#viewport_svg svg text").remove()
    $("#viewport_svg svg line").remove()
    @arborGraph.renderer.initManifoldBar()

    for n in @nodes
      physics.abc.drawManifoldStripe(n)
      for e in n.edges_out
        nn = e.target_node
        ag.addEdge(n.name, nn.name, {"label": e.csl_mode, "distance": e.distance, "timedelta": e.timedelta})
        source_node = ag.getNode(n.name)
        target_node = ag.getNode(nn.name)

        source_node.data =
          label: n.csl_mode
          number: n.name
          activation: n.activation
          configuration: n.configuration
          positions: n.positions
          subManifoldId: n.subManifoldId
          visits: n.mean_n

        target_node.data =
          label: nn.csl_mode
          number: nn.name
          activation: nn.activation
          configuration: nn.configuration
          positions: nn.positions
          subManifoldId: nn.subManifoldId
          visits: nn.mean_n

    #return empty so we dont collect results in for loop (coffee-script...)
    return

  loadGraphFromSemniFile: (files) =>
    readFile = (file, callback) ->
      reader = new FileReader()
      reader.onload = (evt) ->
        callback file, evt  if typeof callback is "function"
      reader.readAsBinaryString file

    if files.length > 0
      readFile files[0], (file, evt) ->
        physics.abc.posture_graph.populateGraphFromSemniFile evt.target.result

    @arborGraph.renderer.pause_drawing = false
    $("#graph_pause_drawing").attr('checked', false)
    @arborGraph.start(true)
    @arborGraph.renderer.click_time = Date.now()
    @arborGraph.renderer.redraw()

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
            if n is point_stack.length
              m = n-1
              n = 0
            else
              m = n-1
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
        physics.abc.explore_active = false

        @best_circle = @circles.slice(-1)[0]  #last circle is the one with largest distance

        #TODO: go to first posture before we can start walking
        #find path from current posture to this one
        #use best_circle[0].start_node .csl_mode .configuration

        @walk_circle_active = true

        #start with first transition
        @best_circle[0].active = true
        physics.abc.graph.renderer.redraw()

  diffuseLearnProgress: =>
    #for each node, get activation through all outgoing edges and sum them up
    #loop over nodes twice to properly deal with recurrent loops
    unless @nodes.length > 1
      return
    for node in @nodes
      if node?
        #divide self activation by proper amount of possible edges,
        #e.g. s+,r+ only has 3 possible outgoing edges, s-,s- only has two, otherwise we have 4
        if "s" in node.csl_mode[0] and "s" in node.csl_mode[1]
          divisor = 0.5
        else if "s" in node.csl_mode[0] or "s" in node.csl_mode[1]
          divisor = 1/3
        else
          divisor = 0.25

        activation_in = 0
        node.activation_self = divisor * node.exit_directions.reduce ((x, y) -> if y is 0 then x+1 else x), 0
        if node.edges_out.length
          for e in node.edges_out
            if e?
              activation_in += e.target_node.activation
          activation_in /= node.edges_out.length
        node.activation_tmp = node.activation_self * 0.7 + activation_in * 0.3
    for node in @nodes
      if node?
        node.activation = node.activation_tmp
        @arborGraph.getNode(node.name).data.activation = node.activation
    return

#end class postureGraph

class abc
  constructor: ->
    @graph = arbor.ParticleSystem()  # display graph, has its own nodes and edges and data for display
    @graph.parameters                # use center-gravity to make the graph settle nicely (ymmv)
      repulsion: 1000 #500
      stiffness: 100 #20
      friction: 0.5
      gravity: true

    @graph.renderer = new simni.RendererSVG("#viewport_svg", @graph, @)
    @posture_graph = new postureGraph(@graph)   # posture graph, logical representation
    @last_posture = null                        # the posture that was visited/created last (i.e. the "current" posture)
    @previous_posture = null                    # the posture that was visited before
    @trajectory = []                            #last n state points

    #defaults
    @heuristic = "unseen"
    @heuristic_keep_dir = false
    @heuristic_keep_joint = true

    @explore_active = false
    @save_periodically = false

  toggleExplore: =>
    if not physics.upper_joint.csl_active
      $("#toggle_csl").click()
    @explore_active = not @explore_active
    if @explore_active
      console.log "start explore run at "+new Date
    else
      console.log "stop explore at "+new Date

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
    return angle - twoPi * (Math.floor( angle / twoPi))

  wrapAngleManifold: (bodyangle) =>
    #wrap angle in asymmetric range around 0, useful with manifold data
    while bodyangle < -1.74*Math.PI
      bodyangle+= 2*Math.PI
    while bodyangle > 0.77*Math.PI
      bodyangle-= 2*Math.PI
    return bodyangle

  smallestAngleDistance: (a1, a2) =>
    #get the shorter of the positive or negative facing angle
    angle = Math.PI - Math.abs(Math.abs(a1 - a2) - Math.PI)

  MAX_UNIX_TIME = 1924988399 #31/12/2030 23:59:59
  time = MAX_UNIX_TIME

  detectAttractor: (body, upper_joint, lower_joint, action) =>
    ##detect if we are currently in a fixpoint or periodic attractor -> posture
    p_body = @wrapAngle body.GetAngle()     #p = φ
    p_hip = upper_joint.GetJointAngle()
    p_knee = lower_joint.GetJointAngle()

    #find attractors, take a sample of trajectory and try to find it multiple times in the
    #past trajectory (with threshold), hence (quasi)periodic behaviour
    if @trajectory.length==10000   #corresponds to max periode duration that can be detected
      @trajectory.shift()

    @trajectory.push [p_body, p_hip, p_knee]

    if @trajectory.length > 200 and (Date.now() - time) > 2000
      #take last 50 points
      last = @trajectory.slice(-50)
      eps=0.01
      d = @searchSubarray last, @trajectory, (a,i, b,j) ->
        Math.abs(a[i][0] - b[j][0]) < eps and Math.abs(a[i][1] - b[j][1]) < eps and Math.abs(a[i][2] - b[j][2]) < eps
      #console.log(d)

      if d.length > 2    #need to find sample a few times to be periodic
        #found a posture, call user method
        configuration = @trajectory.pop()
        action(configuration, @)

        #get rid of saved trajectory
        @trajectory = []

      time = Date.now()

  addEdge: (start_node, target_node, edge_list=start_node.edges_out) =>
    #add an edge between two nodes in logical and drawing graphs, may add one or two new nodes to drawing graph
    #(does some additional stuff over respective methods)
    edge = new transition start_node, target_node
    if not edge.isInList(edge_list) and @posture_graph.length() > 1 and not start_node.isEqualTo target_node
      console.log("adding edge from posture " + start_node.name + " to posture: " + target_node.name)

      #add new edge to logical graph
      distance = target_node.body_x - start_node.body_x
      edge.distance = distance
      timedelta = target_node.timestamp - start_node.timestamp
      edge.timedelta = timedelta
      edge.csl_mode = target_node.csl_mode
      edge_list.push edge
      if edge_list.length > 4
        console.log("warning: now more than 4 outgoing edges in " +start_node.name)
      #target_node.edges_in.push edge

      ##create new edge in display graph
      n0 = start_node.name
      n1 = target_node.name

      #position new node close to previous one (if there is one)
      if @posture_graph.length() > 2
        source_node = @graph.getNode n0
        offset = 0.2
        #randomly place left or right / above or below source node
        if Math.floor(Math.random() * 2) then offset_x = offset else offset_x = -offset
        if Math.floor(Math.random() * 2) then offset_y = offset else offset_y = -offset
        nn1 = @graph.getNode(n1)
        unless nn1?
          nn1 = @graph.addNode n1
        nn1.p.x = source_node.p.x + offset_x
        nn1.p.y = source_node.p.y + offset_y

      @graph.addEdge n0, n1,
        distance: distance.toFixed(3)
        timedelta: timedelta
        label: @transition_mode

      #if we're here for the first time, n0 is not yet initialized (this time addEdge adds two nodes)
      if n0 == 1 and n1 == 2
        init_node = @graph.getNode n0
        init_node.data.label = start_node.csl_mode
        init_node.data.number = start_node.name
        init_node.data.activation = start_node.activation
        init_node.data.positions = start_node.positions
        init_node.data.configuration = start_node.configuration
        init_node.data.subManifoldId = start_node.subManifoldId
        init_node.data.visits = start_node.mean_n

      source_node = @graph.getNode n0
      @graph.current_node = current_node = @graph.getNode(n1)
      current_node.data.label = target_node.csl_mode
      current_node.data.number = target_node.name
      current_node.data.positions = target_node.positions
      current_node.data.configuration = target_node.configuration
      current_node.data.activation = target_node.activation
      current_node.data.subManifoldId = target_node.subManifoldId
      current_node.data.visits = target_node.mean_n
      source_node.data.activation = start_node.activation

      #re-enable suspended graph layouting for a bit to find new layout
      @graph.start(true)
      @graph.renderer.click_time = Date.now()

  switch_to_random_release_after_position: (joint) =>
    @last_posture = null
    @previous_posture = null
    @graph.current_node = null

    #set random release modes
    which = Math.floor(Math.random()*2)
    ui.set_csl_mode_upper(["r+", "r-"][which])
    which = Math.floor(Math.random()*2)
    ui.set_csl_mode_lower(["r+", "r-"][which])

    #(assuming position controller is ON)
    physics.togglePositionController(joint)

    #enable csl again
    $("#toggle_csl").click()

  connectLastPosture: (p) =>
    if @last_posture
      @last_posture.exit_directions[@last_dir_index] = p.name
      if p.name == @last_posture.name
        console.log("warning: added self loop for posture "+ p.name)

    #set/update body positions for svg drawing
    p.positions = [physics.body.GetPosition(), physics.body2.GetPosition(), physics.body3.GetPosition()]
    p.configuration = [@wrapAngle(physics.body.GetAngle()), physics.upper_joint.GetJointAngle(), physics.lower_joint.GetJointAngle()]
    p.body_x = physics.body.GetWorldCenter().x
    p.timestamp = Date.now()

    #put node and edges into drawing graph
    if @last_posture and @posture_graph.length() > 1
      #refresh target position to current x for distance calc
      @addEdge @last_posture, p

      #update graph render stuff
      a_p = @graph.getNode p.name
      @graph.current_node = a_p
      a_p.data.configuration = p.configuration
      a_p.data.positions = p.positions
      @graph.renderer.draw_once()

  savePosture: (configuration, body, upper_csl, lower_csl) =>
    if @manual_noop
      return
    #create temporary posture object
    p = new posture(configuration, [upper_csl, lower_csl], body.GetWorldCenter().x)
    p.positions = [physics.body.GetPosition(), physics.body2.GetPosition(), physics.body3.GetPosition()]

    uj = physics.upper_joint
    lj = physics.lower_joint

    #if we have used the position controller to get to this posture, we now have to check if we're
    #in the previously expected posture
    if physics.upper_joint.position_controller_active and physics.lower_joint.position_controller_active and @last_expected_node
      #quirk: expected node can have been saved with or without stall mode instead of contraction
      #so we also compare with the expected mode (this gets into a lot of assuming...)
      p_expect = new posture(configuration, @last_expected_node.csl_mode, body.GetWorldCenter().x)
      if @last_expected_node.isClose(p) or @last_expected_node.isClose(p_expect)
        #we're now in the expected node and reached it via position controller (should still be same fixpoint since
        #proper body angle resulted from arm angles)

        #set posture with the mode that the expected posture has
        if not @last_expected_node.isClose(p) and @last_expected_node.isClose(p_expect)
          p = p_expect

        #disable position controller
        physics.togglePositionController(uj)
        physics.togglePositionController(lj)
        console.log("collected node "+ @last_expected_node.name+" with position controller, back to csl")

        #enable csl again
        #this will/should use the same mode that didn't reach this node
        ui.set_csl_mode_upper @last_expected_node.csl_mode[0]
        ui.set_csl_mode_lower @last_expected_node.csl_mode[1]
        $("#toggle_csl").click()
        found = [@last_expected_node.name-1]
      else
        #this could also mean there is indeed another position in the
        #same direction. could also be another situation now
        console.log("warning, could not collect node with position controller. either we fell off the manifold or the context changed. continuing somewhere else.")
        #try random csl mode from here and set last_posture etc to null
        #(so that way we simply go on and don't make a connection to the next node)
        @switch_to_random_release_after_position uj
        @switch_to_random_release_after_position lj
        @last_expected_node = null
        @last_detected = null
        @last_posture = null
        return

      @last_expected_node = null

    #if we're trying to reach the closest position that is already saved, we need to catch it here
    #also, if we are not in the possible posture, we have to create a new node at the one detected before (it is indeed new)
    else if physics.upper_joint.position_controller_active and physics.lower_joint.position_controller_active and @last_test_posture
      if p.isClose(@last_test_posture)
        console.log("arrived in posture that was too far away for thresholding but was reachable")
        found = [@last_test_posture.name-1]
        #TODO: merge the previously detected posture and the one we have to get a mean
        @last_test_posture = null

        #re-enable csl
        physics.togglePositionController(uj)
        physics.togglePositionController(lj)
        $("#toggle_csl").click()
      else
        #create previous posture as new node
        console.log("candidate was not a reachable posture, creating new posture for previously found one")
        node_name = @posture_graph.addPosture @last_detected
        console.log("connecting new posture "+node_name+ " with previous posture "+@last_posture.name)
        @connectLastPosture(@last_detected)

        @drawManifoldStripe(p)
        @graph.renderer.redraw()

        #continue with csl whereever we landed instead
        @switch_to_random_release_after_position uj
        @switch_to_random_release_after_position lj
        @last_posture = @posture_graph.getNodeByName(node_name)
        @last_test_posture = null
        @last_detected = null
        return

    #check if there is a posture that we would have expected from the last posture and the last direction we went
    #TODO: this is using exit_directions while the node we actually went could be a wrong edge with
    #a different target
    expected_node = undefined
    if @last_posture? and @last_dir_index? and @last_posture.exit_directions[@last_dir_index] > 0
      expected_node = @posture_graph.getNodeByName(@last_posture.exit_directions[@last_dir_index])

    #search for detected posture in all the nodes that we already have (using larger threshold)
    if not found
      found = @searchSubarray(p, @posture_graph.nodes, p.isCloseExplore)

    if found.length and not expected_node?
      parent = @
      found.sort (a,b) ->
        aa = parent.posture_graph.getNodeByIndex(a)
        bb = parent.posture_graph.getNodeByIndex(b)
        aa.euclidDistance(p) - bb.euclidDistance(p)

      #if the closest existing posture is further away than safe error range (but it was found
      #with the larger error range that isCloseExplore uses) we try to reach it with position controller
      pp = @posture_graph.getNodeByIndex(found[0])
      if thresholdDistance < pp.euclidDistance(p)
        #enable position control
        console.log("found an existing posture (" + pp.name + ") that is a bit far away but possibly right. trying with position controller if we can reach it from here.")
        uj.set_position = pp.configuration[1]
        lj.set_position = pp.configuration[2]
        physics.togglePositionController(uj)
        physics.togglePositionController(lj)
        @last_test_posture = pp
        @last_detected = p
        return

    #if we found no close posture (so we would create a new one) but expected one or the one we found is not the
    #one we expected from the graph, we try to reach it explicitly (i.e. we are close to the attractor already)
    if not found or (@last_posture? and expected_node? and
          @posture_graph.getNodeByIndex(found[0]).name isnt expected_node.name)
      if expected_node
        console.log("we should have arrived in node "+ expected_node.name + ", but we didn't (or arrived too far away)")
        console.log("trying to reach it with position controller")

        #try to go to this posture with pos controller and see if next detected posture is the expected one
        uj.set_position = expected_node.configuration[1]
        lj.set_position = expected_node.configuration[2]
        #deactivate csl
        physics.togglePositionController(uj)
        physics.togglePositionController(lj)

        #found = [expected_node.name-1]
        @last_expected_node = expected_node
        return
      else
        #we didn't find a posture close to this one yet and we didn't expect another one, so add a new one
        node_name = @posture_graph.addPosture p
        @graph.renderer.redraw()

    #we have this posture already, update it
    if found.length
      new_p = p
      f = found[0]
      p = @posture_graph.getNodeByIndex f
      console.log("re-visiting node " + p.name)

      #update to mean of old and current configurations
      #(counter is used for proper weighting)
      #body angle is wrapped because one turn around of one position and none of the other gives
      #weird results
      bodyAngle_p = @wrapAngle(p.configuration[0])
      bodyAngle_new_p = @wrapAngle(new_p.configuration[0])
      if (bodyAngle_p < 1 and bodyAngle_new_p > 6) or (bodyAngle_p > 6 and bodyAngle_new_p < 1)
        #wrapping means we need to make sure that angles of e.g 0.1 and 6.2 don't produce wrong
        #results (0.0something instead of 3.1), so don't calc any means for now
        p.configuration[0] = bodyAngle_new_p
      else
        p.configuration[0] = (bodyAngle_new_p + bodyAngle_p*p.mean_n) / (p.mean_n+1)
      p.configuration[1] = (new_p.configuration[1] + p.configuration[1]*p.mean_n) / (p.mean_n+1)
      p.configuration[2] = (new_p.configuration[2] + p.configuration[2]*p.mean_n) / (p.mean_n+1)
      p.mean_n += 1
      #TODO: save scattering too

      #make renderer draw updated semni posture
      #(deleting will recreate)
      n = @graph.getNode p.name

      if n.data.semni
        n.data.semni.remove()
        n.data.semni = undefined

    @connectLastPosture(p)

    @previous_posture = @last_posture
    @last_posture = p

    @newCSLMode()

    @drawManifoldStripe(p)
    @graph.renderer.redraw()

    if @save_periodically
      #save svg and json
      @posture_graph.saveGaphToFile()
      graph_func = -> ui.getPostureGraphAsFile()
      setTimeout graph_func, 1000

  compareModes: (a, b) =>
    if not a or not b
      return false
    a[0] == b[0] && a[1] == b[1]

  ## next mode heuristics
  newCSLMode: =>
    #random: randomly select next mode + different mode than the current one
    #unseen: try new mode that is not in neighbors of the current one, prefer previous mode. if all are seen,
    #        use direction of largest expected learn progress through diffuse
    #manual: detect postures and add nodes but don't set new modes automatically

    #TODO: try more strategies
    #- alternate between c and r modes
    #- always use certain direction/joint first, no random
    #-

    ### helpers ###
    set_random_mode = (current_mode) ->
      which = Math.floor(Math.random()*2)    #just change one joint at a time
      if which == 0
        loop    #try as long as we dont have a new mode to prevent same mode again
          mode = ["r+", "r-", "c"][Math.floor(Math.random()*2.99)]
          break if current_mode[0] isnt mode
        ui.set_csl_mode_upper mode
      else
        loop
          mode = ["r+", "r-", "c"][Math.floor(Math.random()*2.99)]
          break if current_mode[1] isnt mode
        ui.set_csl_mode_lower mode
      new_mode = current_mode.clone()
      new_mode[which] = mode
      return new_mode

    next_mode_for_direction = (old_mode, direction) ->
      if direction is 0
        switch old_mode
          when "r+" then return "c"
          when "r-" then return "r+"
          when "c" then return "r+"
          when "s-" then return "r+"
          when "s+" then return "s+"
      else if direction is 1
        switch old_mode
          when "r+" then return "r-"
          when "r-" then return "c"
          when "c" then return "r-"
          when "s+" then return "r-"
          when "s-" then return "s-"

    dir_index_for_modes = (start_mode, target_mode) ->
      #get dir index to get from start mode to target mode
      [s_h, s_k] = start_mode
      [t_h, t_k] = target_mode

      if (s_h is t_h) or ("s" in s_h and "c" in t_h) or ("c" in s_h and "s" in t_h)
        #it's a change in the knee joint, now get direction
        a = s_k
        b = t_k
        i = 2
      else
        #it's a change in the hip joint
        a = s_h
        b = t_h
        i = 0

      d = 0
      if a in ["s+","s-"]
        a = "c"
      if b in ["s+","s-"]
        b = "c"
      if      a == "r+" and b == "c" then d = 0
      else if a == "r+" and b == "r-" then d = 1
      else if a == "r-" and b == "r+" then d = 0
      else if a == "r-" and b == "c" then d = 1
      else if a == "c" and b == "r-" then d = 1
      else if a == "c" and b == "r+" then d = 0
      return i+d

    dir_index_for_dir_and_joint = (dir, joint_index) ->
      if dir is "-" then dir_index = 1 else dir_index = 0
      dir_index + 2*joint_index

    stall_index_for_mode = (mode, joint_index) ->
      #return 0 or 1 depending on what direction (+ or -) is stalling for the given mode
      if mode is "s+"
        0+joint_index*2
      else if mode is "s-"
        1+joint_index*2

    joint_from_dir_index = (index) ->
      #get joint number for index (0: first joint, 1: second, ...)
      Math.ceil((index+1) / 2) - 1

    dir_from_index = (index) ->
      #if index % 2 then "-" else "+"
      index%2

    other_joint = (joint) ->
      #get the number if the other joint for a given index
      (joint+1)%2

    other_dir = (dir) ->
      #get the number of the other direction for a given index
      (dir+1)%2

    ### end helpers ###

    ### get some values ###
    current_mode = @last_posture.csl_mode
    if @previous_posture
      previous_mode = @previous_posture.csl_mode
    else
      previous_mode = undefined

    #we just added a new posture; if it has a stall mode set (from limitCSL()),
    #prevent going further in the last direction if we now are in stall
    for joint in [0,1]
      if "s" in current_mode[joint]
        @last_posture.exit_directions[stall_index_for_mode current_mode[joint], joint] = -1

    #initialise some stuff
    unless @last_dir_index
      @last_dir = 0
      @last_joint_index = 0
      @last_dir_index = 0

    ### heuristics ###

    if @heuristic is "unseen"
      #for each joint: -1 stall, 0 not visited, >0 visited count
      #exit directions: [h+,h-,k+,k-]

      if 0 in @last_posture.exit_directions
        trial_level = 0
        while not next_dir_index?     #we can't simply go back again, look for possible new edges
          #depending on heuristic options, try indexes in certain order
          if          @heuristic_keep_dir and     @heuristic_keep_joint   #1+2
            #go through various alternatives if options directly don't result in edge we haven't gone before
            switch trial_level
              when 0 then try_dir_index = @last_dir_index
              when 1 then try_dir_index = @last_dir + 2*other_joint(@last_joint_index)
              when 2 then try_dir_index = other_dir(@last_dir) + 2*@last_joint_index
          else if     @heuristic_keep_dir and not @heuristic_keep_joint   #1+4
            switch trial_level
              when 0 then try_dir_index = @last_dir + 2*other_joint(@last_joint_index)
              when 1 then try_dir_index = other_dir(@last_dir) + 2*other_joint(@last_joint_index)
              when 2 then try_dir_index = @last_dir + 2*@last_joint_index
          else if not @heuristic_keep_dir and     @heuristic_keep_joint   #2+3
            switch trial_level
              when 0 then try_dir_index = other_dir(@last_dir) + 2*@last_joint_index
              when 1 then try_dir_index = @last_dir + 2*@last_joint_index
              when 2 then try_dir_index = other_dir(@last_dir) + 2*other_joint(@last_joint_index)
          else if not @heuristic_keep_dir and not @heuristic_keep_joint   #3+4
            switch trial_level
              when 0 then try_dir_index = other_dir(@last_dir) + 2*other_joint(@last_joint_index)
              when 1 then try_dir_index = @last_dir + 2*other_joint(@last_joint_index)
              when 2 then try_dir_index = other_dir(@last_dir) + 2*@last_joint_index

          if @last_posture.exit_directions[try_dir_index] is 0
            next_dir_index = try_dir_index
          else
            trial_level++

          if trial_level == 3
            #there are no alternatives matching the options, simply take the first open edge
            next_dir_index = @last_posture.exit_directions.indexOf 0

        console.log("leaving node in direction " + next_dir_index)
      else   #we went all directions already
        #take random edge
        #next_dir_index = Math.floor(Math.random()*3.99)  #choose a random one of four, replace four
        #by # of joints - 0.01

        #take edge with largest activation
        #get edge with largest activation at target
        go_this_edge = @last_posture.edges_out[0]
        for e in @last_posture.edges_out
          if e.target_node.activation > go_this_edge.target_node.activation
            go_this_edge = e

        if not go_this_edge
          #if we didn't find an edge (i.e. there are no outgoing edges),
          #get first one that isn't stalling
          next_dir_index = dir for dir in @last_posture.exit_directions when dir > -1
          console.log("warning: take first non stalling direction, this should probably not happen")
        else
          next_dir_index = dir_index_for_modes @last_posture.csl_mode, go_this_edge.target_node.csl_mode
          console.log("following the edge "+go_this_edge.start_node.name+"->"+go_this_edge.target_node.name+" because of largest activation.")

      joint_index = joint_from_dir_index next_dir_index
      direction = dir_from_index next_dir_index
      next_mode = next_mode_for_direction current_mode[joint_index], direction

      if joint_index is 0
        ui.set_csl_mode_upper next_mode
      else
        ui.set_csl_mode_lower next_mode

      @last_dir = direction
      @last_dir_index = next_dir_index
      @last_joint_index = joint_index

      #save the next mode that was set to save in edge
      @transition_mode = current_mode.clone()
      @transition_mode[joint_index] = next_mode

      @graph.renderer.redraw()

    else if @heuristic is "random"
      new_mode = set_random_mode current_mode
      @last_dir_index = dir_index_for_modes current_mode, new_mode

    else if @heuristic is "manual"
      #don't do anything
      @manual_noop = true   #will be reset to false (so next posture is saved) once a new mode is set from ui
      if previous_mode
        @last_dir_index = dir_index_for_modes previous_mode, current_mode


  ### ui helpers ###
  set_heuristic: (heuristic) =>
    @heuristic = heuristic

    if heuristic is "unseen"
      $("#unseen_options").show()
    else
      $("#unseen_options").hide()


  set_heuristic_keep_dir: (value) =>
    @heuristic_keep_dir = value

  set_heuristic_keep_joint: (value) =>
    @heuristic_keep_joint = value

  drawManifoldStripe: (p) =>
    #add manifold stripes for this posture
    m = @graph.renderer
    m.mt_count++
    w = m.mt_width/m.mt_count
    selection = m.manifold_over_time.selectAll("rect").each (d,i) ->
      if i > 0
        d3.select(this)
          .attr("width", w)
          .attr("x", (m.mt_x)+(w*(i-1)))

    m.manifold_over_time.append("rect")
        .attr("width", w)
        .attr("height", m.mt_height)
        .attr("x", (m.mt_x)+(w* (m.mt_count-1)))
        .attr("y", m.mt_y)
        .attr("fill", ui.getSubmanifoldColor(p.subManifoldId))

  ### end ui helpers ###

  limitCSL: (upper_joint, lower_joint) =>
    # if csl goes against limit, set to stall mode (hold with small bias)
    limit = 10
    if upper_joint.csl_active and upper_joint.csl_mode is "c"
      mc = upper_joint.motor_control
      if mc > limit
        ui.set_csl_mode_upper "s+"
      else if mc < -limit
        ui.set_csl_mode_upper "s-"

    if lower_joint.csl_active and lower_joint.csl_mode is "c"
      mc = lower_joint.motor_control
      if mc > limit
        ui.set_csl_mode_lower "s+"
      else if mc < -limit
        ui.set_csl_mode_lower "s-"

  update: (body, upper_joint, lower_joint) =>
    @limitCSL upper_joint, lower_joint
    if @explore_active
      @detectAttractor body, upper_joint, lower_joint, (configuration, parent) ->
        #when a new attractor is found, save posture
        parent.savePosture configuration, body, upper_joint.csl_mode, lower_joint.csl_mode

        #update once if we're not constantly updating so we can see what's going on
        if not parent.graph.renderer.draw_graph
          parent.graph.renderer.draw_graph = true
          parent.graph.renderer.redraw()
          parent.graph.renderer.draw_graph = false

      @posture_graph.diffuseLearnProgress()

    if @posture_graph.walk_circle_active
      @detectAttractor body, upper_joint, lower_joint, (configuration, parent) ->
        current_posture = new posture(configuration, [physics.upper_joint.csl_mode, physics.lower_joint.csl_mode])
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

        return
        #TODO: if we're here, there was no matching edge in the circle which means we are not there yet
        #find a path from the current posture (we found one) to one posture on the circle (the shortest/safest please)

#end class abc


window.simni.Abc = abc
window.simni.Posture = posture
