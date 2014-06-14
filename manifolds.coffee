
class manifoldRenderer
  constructor: ->
    @scale = 3
    @y_offset = 7
    @stopStateUpdate = false
    @do_render = true
    @halftime = true

  createGeometry: (geom, data) =>
    for i in data.data
      geom.vertices.push(new THREE.Vector3(i[8]*@scale, i[6]*@scale-@y_offset, i[7]*@scale))
      c = new THREE.Color( 0xff0000 )

      switch i[1]
        when 1 then c.setRGB(255/255,150/255,0)
        when 2 then c.setRGB(0, 195/255, 80/255)
        when 3 then c.setRGB(0,190/255,255/255)
        when 4 then c.setRGB(197/255,0,169/255)
      geom.colors.push( c )
    return

  #convert a simni position to a coordinate vector that is suitable for use with the manifold data
  #(scaling and offset is just for visual)
  internalToManifold: (configuration) =>
    return {
      x: -configuration[2]*@scale,
      y: -physics.abc.wrapAngleManifold(configuration[0])*@scale - @y_offset
      z: -configuration[1]*@scale,
    }

  animate: =>
    if @do_render
      requestAnimationFrame(@animate)

      if @halftime
        # render the scene
        @controls.update()

        if physics? and physics.body? and not window.stopStateUpdate
          @updateCurrentState @internalToManifold [
            physics.body.GetAngle()
            physics.upper_joint.GetJointAngle()
            physics.lower_joint.GetJointAngle()
          ]
        @renderer.render(@scene, @camera)
        @halftime = false
      else
        @halftime = true

  init: =>
    canvas = $("#webglCanvas")
    width = canvas[0].width
    height = canvas[0].height

    @scene = new THREE.Scene()
    @camera = new THREE.PerspectiveCamera( 30, width / height, 0.1, 1000 )
    #THREE.OrthographicCamera
    @camera.position.set -35, 10, 0
    @camera.lookAt new THREE.Vector3(0, 0, 0)
    @scene.add @camera

    @renderer = new THREE.WebGLRenderer({canvas:canvas[0]})
    @renderer.setClearColor( 0xffffff, 1)

    line_material = new THREE.LineBasicMaterial(
      #linewidth: 1
      #opacity: 1
      vertexColors: THREE.VertexColors
    )

    #draw mesh using particle system
    #particle_material = new THREE.ParticleSystemMaterial(
    #  size: 0.05
    #  fog: true
    #  vertexColors: THREE.VertexColors
    #)

    #if navigator.appVersion.indexOf("Win") isnt -1
    #  particle_material.size = 0.3

    #geom = new THREE.Geometry()
    #@createGeometry geom,
    #  data: semni_manifold #.slice(0,8)

    #ps = new THREE.ParticleSystem(geom, particle_material)
    #@scene.add ps

    #draw using line geometry with the indexes of the neighbors from file
    geometry = new THREE.Geometry()
    for i in [0..semni_manifold.length-1]
      hipp = semni_manifold[i][3]-1
      kneep = semni_manifold[i][5]-1
      grp = semni_manifold[i][1]

      if(hipp > 0)
        p1 = new THREE.Vector3(semni_manifold[i][8]*@scale
                    semni_manifold[i][6]*@scale-@y_offset
                    semni_manifold[i][7]*@scale)
        geometry.vertices.push( p1 )


        p2 = new THREE.Vector3(semni_manifold[hipp][8]*@scale
                    semni_manifold[hipp][6]*@scale-@y_offset
                    semni_manifold[hipp][7]*@scale)
        geometry.vertices.push( p2 )

        #set color for this line (both vertices should have same grp)
        c = new THREE.Color( 0xff0000 )
        #switch Math.round(Math.random()*4)
        switch grp
          when 1 then c.setRGB(255/255,150/255,0)
          when 2 then c.setRGB(0, 195/255, 80/255)
          when 3 then c.setRGB(0,190/255,255/255)
          when 4 then c.setRGB(197/255,0,169/255)
        geometry.colors.push( c )
        geometry.colors.push( c )

      if(kneep > 0)
        p1 = new THREE.Vector3(semni_manifold[i][8]*@scale
                    semni_manifold[i][6]*@scale-@y_offset
                    semni_manifold[i][7]*@scale)
        geometry.vertices.push( p1 )

        p2 = new THREE.Vector3(semni_manifold[kneep][8]*@scale
                    semni_manifold[kneep][6]*@scale-@y_offset
                    semni_manifold[kneep][7]*@scale)
        geometry.vertices.push( p2 )

        #set color for this line (both vertices should have same grp)
        c = new THREE.Color( 0xff0000 )
        switch grp
          when 1 then c.setRGB(255/255,150/255,0)
          when 2 then c.setRGB(0, 195/255, 80/255)
          when 3 then c.setRGB(0,190/255,255/255)
          when 4 then c.setRGB(197/255,0,169/255)
        geometry.colors.push( c )
        geometry.colors.push( c )

    line = new THREE.Line(geometry, line_material, THREE.LinePieces)
    #line.matrixAutoUpdate = false   #should speed up things a little?
    @scene.add line

    @controls = new THREE.OrbitControls(@camera, @renderer.domElement)

# draw semni configuration into manifold
  initCurrentState: =>
    # set up the sphere vars
    radius = 0.4
    segments = 12
    rings = 12

    sphereMaterial = new THREE.MeshLambertMaterial(color: 0xffffff, opacity: 0.7, transparent: true)
    @current_state = new THREE.Mesh(new THREE.SphereGeometry(radius, segments, rings), sphereMaterial)
    @scene.add @current_state

  updateCurrentState: (state) =>
    if @do_render
      @current_state.position.x = state.x
      @current_state.position.y = state.y
      @current_state.position.z = state.z
      @camera.lookAt(state)
      @controls.target.x = state.x
      @controls.target.y = state.y
      @controls.target.z = state.z

# draw trajectories into manifold
  initFixpoints: =>
    @fp_material = new THREE.ParticleSystemMaterial(
      size: 1.5
      fog: true
      vertexColors: THREE.VertexColors
    )

    #if navigator.appVersion.indexOf("Win") isnt -1
    #  fp_material.size = 0.3
    @fixpoints = []

  addFixpoint: (point) =>
    @fixpoints.push new THREE.Vector3(point.x, point.y, point.z)
    if @do_render
      @scene.remove @fpps
      delete @fpps

      geom = new THREE.Geometry()
      for p in @fixpoints
        geom.vertices.push p

      @fpps = new THREE.ParticleSystem geom, @fp_material
      @scene.add @fpps

  initTrajectoryPoints: =>
    @traj_material = new THREE.ParticleSystemMaterial(
      size: 0.5
      fog: true
      vertexColors: THREE.VertexColors
    )
    @trajectory = []

  addTrajectoryPoint: (point) =>
    @trajectory.push new THREE.Vector3(point.x, point.y, point.z)
    if @do_render
      delete @traj_geom
      @scene.remove @traj_ps
      delete @traj_ps

      @traj_geom = new THREE.Geometry()
      for p in @trajectory
        @traj_geom.vertices.push p
        c = new THREE.Color( 0x7f7f7f )
        @traj_geom.colors.push(c)

      @traj_ps = new THREE.ParticleSystem @traj_geom, @traj_material
      @scene.add @traj_ps

# end class manifoldRenderer

if not window.simni?
  window.simni = {}

window.simni.ManifoldRenderer = manifoldRenderer

$(document).ready ->
  manifoldRenderer = new manifoldRenderer
  manifoldRenderer.init()
  if physics?
    manifoldRenderer.initCurrentState()
    manifoldRenderer.initTrajectoryPoints()
    manifoldRenderer.initFixpoints()
  manifoldRenderer.animate()
  window.manifoldRenderer = manifoldRenderer
