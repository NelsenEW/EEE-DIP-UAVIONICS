const degToRad = 0.0174533;

let scene, camera, rendered, cube;

function parentWidth(elem) {
  return elem.parentElement.clientWidth;
}

function parentHeight(elem) {
  return elem.parentElement.clientHeight;
}

function init3D(){
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0xffffff);

  camera = new THREE.PerspectiveCamera(75, parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube")), 0.1, 1000);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));

  document.getElementById('3Dcube').appendChild(renderer.domElement);

  // Create a geometry
  const geometry = new THREE.BoxGeometry(5, 1, 4);

  // Materials of each face
  var cubeMaterials = [
    new THREE.MeshBasicMaterial({color:0x03045e}),
    new THREE.MeshBasicMaterial({color:0x023e8a}),
    new THREE.MeshBasicMaterial({color:0x0077b6}),
    new THREE.MeshBasicMaterial({color:0x03045e}),
    new THREE.MeshBasicMaterial({color:0x023e8a}),
    new THREE.MeshBasicMaterial({color:0x0077b6}),
  ];

  const material = new THREE.MeshFaceMaterial(cubeMaterials);

  cube = new THREE.Mesh(geometry, material);
  scene.add(cube);
  camera.position.z = 5;
  renderer.render(scene, camera);
}

// Resize the 3D object when the browser window changes size
function onWindowResize(){
  camera.aspect = parentWidth(document.getElementById("3Dcube")) / parentHeight(document.getElementById("3Dcube"));
  //camera.aspect = window.innerWidth /  window.innerHeight;
  camera.updateProjectionMatrix();
  //renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setSize(parentWidth(document.getElementById("3Dcube")), parentHeight(document.getElementById("3Dcube")));

}

window.addEventListener('resize', onWindowResize, false);

// Create the 3D representation
init3D();

// Create events for the sensor readings
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('all_readings', function(e) {
    var obj = JSON.parse(e.data);
    document.getElementById("accX").innerHTML = obj.accX;
    document.getElementById("accY").innerHTML = obj.accY;
    document.getElementById("accZ").innerHTML = obj.accZ;
    document.getElementById("gyroX").innerHTML = obj.gyroX;
    document.getElementById("gyroY").innerHTML = obj.gyroY;
    document.getElementById("gyroZ").innerHTML = obj.gyroZ;
    document.getElementById("magX").innerHTML = obj.magX;
    document.getElementById("magY").innerHTML = obj.magY;
    document.getElementById("magZ").innerHTML = obj.magZ;
    document.getElementById("roll").innerHTML = obj.roll;
    document.getElementById("pitch").innerHTML = obj.pitch;
    document.getElementById("yaw").innerHTML = obj.yaw;
    document.getElementById("temp").innerHTML = obj.temp;
    document.getElementById("pres").innerHTML = obj.pres;
    cube.rotation.x = obj.roll * degToRad;
    cube.rotation.z = obj.pitch * degToRad;
    cube.rotation.y = -obj.yaw * degToRad;
    renderer.render(scene, camera);
  }, false);

  source.addEventListener('accelerometer_readings', function(e) {
    // console.log("accelerometer_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("accX").innerHTML = obj.accX;
    document.getElementById("accY").innerHTML = obj.accY;
    document.getElementById("accZ").innerHTML = obj.accZ;
  }, false);

  source.addEventListener('gyro_readings', function(e) {
    //console.log("gyro_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("gyroX").innerHTML = obj.gyroX;
    document.getElementById("gyroY").innerHTML = obj.gyroY;
    document.getElementById("gyroZ").innerHTML = obj.gyroZ;
  }, false);

  source.addEventListener('gyro_readings', function(e) {
    //console.log("gyro_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("magX").innerHTML = obj.magX;
    document.getElementById("magY").innerHTML = obj.magY;
    document.getElementById("magZ").innerHTML = obj.magZ;
  }, false);

  source.addEventListener('axis_readings', function(e) {
    var obj = JSON.parse(e.data);
    // Change cube rotation after receiving the readinds
    document.getElementById("roll").innerHTML = obj.roll;
    document.getElementById("pitch").innerHTML = obj.pitch;
    document.getElementById("yaw").innerHTML = obj.yaw;
    cube.rotation.x = obj.roll;
    cube.rotation.z = obj.pitch;
    cube.rotation.y = obj.yaw;
    renderer.render(scene, camera);
  }, false);

  source.addEventListener('environment_reading', function(e) {
    // console.log("environment_reading", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("temp").innerHTML = obj.temp;
    document.getElementById("pres").innerHTML = obj.pressure;
  }, false);

}

function resetPosition(element){
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/"+element.id, true);
  console.log(element.id);
  xhr.send();
}
