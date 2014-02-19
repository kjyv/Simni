
class manifoldRenderer
  constructor: ->
    @scale = 3
    @y_offset = 7
    @stopStateUpdate = false
    @do_render = true

  createGeometry: (geom, data) =>
    for i in data.data
      geom.vertices.push(new THREE.Vector3(i[8]*@scale, i[6]*@scale-@y_offset, i[7]*@scale))
      c = new THREE.Color( 0xff0000 )

      switch i[1]
        when 1 then c.setRGB(255/255,150/255,0)
        when 2 then c.setRGB(0, 195/255, 80/255)
        when 3 then c.setRGB(0,173/255,244/255)
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

      # render the scene
      @controls.update()

      if physics.body? and not window.stopStateUpdate
        @updateCurrentState @internalToManifold [
          physics.body.GetAngle()
          physics.upper_joint.GetJointAngle()
          physics.lower_joint.GetJointAngle()
        ]
      @renderer.render(@scene, @camera)

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

    geom = new THREE.Geometry()
    @createGeometry geom,
      data: semni_manifold #.slice(0,8)

    #line_material = new THREE.LineBasicMaterial(
    #      color: 0x00FF00
    #)

    mesh_material = new THREE.MeshBasicMaterial(
      color: 0xFF0000
      wireframe: true
    )

    particle_material = new THREE.ParticleSystemMaterial(
      size: 0.05
      fog: true
      vertexColors: THREE.VertexColors
    )

    if navigator.appVersion.indexOf("Win") isnt -1
      particle_material.size = 0.3

    #mesh = new THREE.Mesh(geom, mesh_material)
    #scene.add mesh
    ps = new THREE.ParticleSystem(geom, particle_material)
    @scene.add ps

#  semni_manifold.sort (a,b) -> b[1] - a[1]
#
#  i = 0
#  grp = 0
#  while i < 5000 #semni_manifold.length
#    geometry = new THREE.Geometry()
#    geometry.vertices.push(new THREE.Vector3(semni_manifold[i][8]*scale, semni_manifold[i][7]*scale, semni_manifold[i][6]*scale))
#    if grp == semni_manifold[i+1][1]
#      geometry.vertices.push(new THREE.Vector3(semni_manifold[i+1][8]*scale, semni_manifold[i+1][7]*scale, semni_manifold[i+1][6]*scale))
#    line = new THREE.Line(geometry, line_material)
#    scene.add line
#    grp = semni_manifold[i][1]
#    i+=2

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
    @current_state.position.x = state.x
    @current_state.position.y = state.y
    @current_state.position.z = state.z
    @camera.lookAt(state)
    @controls.target.x = state.x
    @controls.target.y = state.y
    @controls.target.z = state.z

# end class manifoldRenderer

window.simni.ManifoldRenderer = manifoldRenderer

$(document).ready ->
  manifoldRenderer = new manifoldRenderer
  manifoldRenderer.init()
  manifoldRenderer.initCurrentState()
  manifoldRenderer.animate()
