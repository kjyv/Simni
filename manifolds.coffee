scale = 3
y_offset = 7

createGeometry = (geom, data) ->
  index = 0

  for i in data.data
  #  createSquare geom, index,
  #      x: i[8]*5-10
  #      y: i[6]*5
  #      z: i[7]*5
  #      data.squareWidth
  #  index++
    geom.vertices.push(new THREE.Vector3(i[8]*scale, i[6]*scale-y_offset, i[7]*scale))
    c = new THREE.Color( 0xff0000 )

    switch i[1]
      when 1 then c.setRGB(255/255,150/255,0)
      when 2 then c.setRGB(0, 195/255, 80/255)
      when 3 then c.setRGB(0,173/255,244/255)
      when 4 then c.setRGB(197/255,0,169/255)
    geom.colors.push( c )
  return

createSquare = (geom, index, center, width) ->
  square = [[-1,  1],[1,  1],[1, -1],[-1, -1]]
  square.push square[0]
  i = 0

  while i < 4
    v1 = new THREE.Vector3(0 + center.x, 0 + center.z, 0 + center.y)
    v2 = new THREE.Vector3(square[i][0] * width / 2 + center.x, 0 + center.z, square[i][1] * width / 2 + center.y)
    v3 = new THREE.Vector3(square[i + 1][0] * width / 2 + center.x, 0 + center.z, square[i + 1][1] * width / 2 + center.y)

    geom.vertices.push v1
    geom.vertices.push v2
    geom.vertices.push v3
    face = new THREE.Face3((i * 3) + (index * 12), (i * 3 + 1) + (index * 12), (i * 3 + 2) + (index * 12))
    face.normal = (->
      vx = (v1.y - v3.y) * (v2.z - v3.z) - (v1.z - v3.z) * (v2.y - v3.y)
      vy = (v1.z - v3.z) * (v2.x - v3.x) - (v1.x - v3.x) * (v2.z - v3.z)
      vz = (v1.x - v3.x) * (v2.y - v3.y) - (v1.y - v3.y) * (v2.x - v3.x)
      va = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2) + Math.pow(vz, 2))
      new THREE.Vector3(vx / va, vy / va, vz / va)
    )()
    geom.faces.push face
    i++
  return

scene = camera = controls = renderer = undefined

animate = ->
  requestAnimationFrame(animate)

  # render the scene.
  controls.update()
  if physics.body?
    updateCurrentState(
      x: -physics.lower_joint.GetJointAngle()*scale
      z: -physics.upper_joint.GetJointAngle()*scale
      y: -physics.body.GetAngle()*scale - y_offset
    )
  renderer.render(scene, camera)

init = ->
  canvas = $("#webglCanvas")
  width = canvas[0].width
  height = canvas[0].height

  scene = new THREE.Scene()
  camera = new THREE.PerspectiveCamera( 30, width / height, 0.1, 1000 )
  #THREE.OrthographicCamera
  camera.position.set -35, 10, 0
  camera.lookAt new THREE.Vector3(50, 0, 50)
  scene.add camera

  renderer = new THREE.WebGLRenderer({canvas:canvas[0]})
  renderer.setClearColor( 0xffffff, 1)

  geom = new THREE.Geometry()
  createGeometry geom,
    squareWidth: 0.2
    data: semni_manifold #.slice(0,8)

  line_material = new THREE.LineBasicMaterial(
        color: 0x00FF00
  )

  mesh_material = new THREE.MeshBasicMaterial(
    color: 0xFF0000
    wireframe: true
  )

  particle_material = new THREE.ParticleSystemMaterial(
    size: 0.05
    fog: true
    vertexColors: THREE.VertexColors
  )

  #mesh = new THREE.Mesh(geom, mesh_material)
  #scene.add mesh

  ps = new THREE.ParticleSystem(geom, particle_material)
  scene.add ps

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

  controls = new THREE.OrbitControls(camera, renderer.domElement)

current_state = undefined

initCurrentState = () ->
  # state is a vector {x,y,z}

  # set up the sphere vars
  radius = 0.2
  segments = 4
  rings = 4

  sphereMaterial = new THREE.MeshLambertMaterial(color: 0xffffff)
  current_state = new THREE.Mesh(new THREE.SphereGeometry(radius, segments, rings), sphereMaterial)
  scene.add current_state

updateCurrentState = (state) ->
  current_state.position.x = state.x
  current_state.position.y = state.y
  current_state.position.z = state.z

$(document).ready ->
  init()
  initCurrentState()
  animate()
