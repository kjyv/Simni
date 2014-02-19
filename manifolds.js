// Generated by CoffeeScript 1.3.3
var manifoldRenderer,
  __bind = function(fn, me){ return function(){ return fn.apply(me, arguments); }; };

manifoldRenderer = (function() {

  function manifoldRenderer() {
    this.updateCurrentState = __bind(this.updateCurrentState, this);

    this.initCurrentState = __bind(this.initCurrentState, this);

    this.init = __bind(this.init, this);

    this.animate = __bind(this.animate, this);

    this.internalToManifold = __bind(this.internalToManifold, this);

    this.createGeometry = __bind(this.createGeometry, this);
    this.scale = 3;
    this.y_offset = 7;
    this.stopStateUpdate = false;
    this.do_render = true;
  }

  manifoldRenderer.prototype.createGeometry = function(geom, data) {
    var c, i, _i, _len, _ref;
    _ref = data.data;
    for (_i = 0, _len = _ref.length; _i < _len; _i++) {
      i = _ref[_i];
      geom.vertices.push(new THREE.Vector3(i[8] * this.scale, i[6] * this.scale - this.y_offset, i[7] * this.scale));
      c = new THREE.Color(0xff0000);
      switch (i[1]) {
        case 1:
          c.setRGB(255 / 255, 150 / 255, 0);
          break;
        case 2:
          c.setRGB(0, 195 / 255, 80 / 255);
          break;
        case 3:
          c.setRGB(0, 173 / 255, 244 / 255);
          break;
        case 4:
          c.setRGB(197 / 255, 0, 169 / 255);
      }
      geom.colors.push(c);
    }
  };

  manifoldRenderer.prototype.internalToManifold = function(configuration) {
    return {
      x: -configuration[2] * this.scale,
      y: -physics.abc.wrapAngleManifold(configuration[0]) * this.scale - this.y_offset,
      z: -configuration[1] * this.scale
    };
  };

  manifoldRenderer.prototype.animate = function() {
    if (this.do_render) {
      requestAnimationFrame(this.animate);
      this.controls.update();
      if ((physics.body != null) && !window.stopStateUpdate) {
        this.updateCurrentState(this.internalToManifold([physics.body.GetAngle(), physics.upper_joint.GetJointAngle(), physics.lower_joint.GetJointAngle()]));
      }
      return this.renderer.render(this.scene, this.camera);
    }
  };

  manifoldRenderer.prototype.init = function() {
    var canvas, geom, height, mesh_material, particle_material, ps, width;
    canvas = $("#webglCanvas");
    width = canvas[0].width;
    height = canvas[0].height;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(30, width / height, 0.1, 1000);
    this.camera.position.set(-35, 10, 0);
    this.camera.lookAt(new THREE.Vector3(0, 0, 0));
    this.scene.add(this.camera);
    this.renderer = new THREE.WebGLRenderer({
      canvas: canvas[0]
    });
    this.renderer.setClearColor(0xffffff, 1);
    geom = new THREE.Geometry();
    this.createGeometry(geom, {
      data: semni_manifold
    });
    mesh_material = new THREE.MeshBasicMaterial({
      color: 0xFF0000,
      wireframe: true
    });
    particle_material = new THREE.ParticleSystemMaterial({
      size: 0.05,
      fog: true,
      vertexColors: THREE.VertexColors
    });
    ps = new THREE.ParticleSystem(geom, particle_material);
    this.scene.add(ps);
    return this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
  };

  manifoldRenderer.prototype.initCurrentState = function() {
    var radius, rings, segments, sphereMaterial;
    radius = 0.4;
    segments = 12;
    rings = 12;
    sphereMaterial = new THREE.MeshLambertMaterial({
      color: 0xffffff,
      opacity: 0.7,
      transparent: true
    });
    this.current_state = new THREE.Mesh(new THREE.SphereGeometry(radius, segments, rings), sphereMaterial);
    return this.scene.add(this.current_state);
  };

  manifoldRenderer.prototype.updateCurrentState = function(state) {
    this.current_state.position.x = state.x;
    this.current_state.position.y = state.y;
    this.current_state.position.z = state.z;
    this.camera.lookAt(state);
    this.controls.target.x = state.x;
    this.controls.target.y = state.y;
    return this.controls.target.z = state.z;
  };

  return manifoldRenderer;

})();

window.simni.ManifoldRenderer = manifoldRenderer;

$(document).ready(function() {
  manifoldRenderer = new manifoldRenderer;
  manifoldRenderer.init();
  manifoldRenderer.initCurrentState();
  return manifoldRenderer.animate();
});