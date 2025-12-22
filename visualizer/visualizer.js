import * as THREE from 'three';

const WS_URL = 'ws://localhost:3000';

const canvas = document.getElementById('canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x1a1a1a);

// Scene setup
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(5, 4, 5);
camera.lookAt(0, 0, 0);

// Lighting
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 5, 5);
scene.add(directionalLight);

// Coordinate axes at origin (X=red, Y=green, Z=blue)
const axesHelper = new THREE.AxesHelper(3);
scene.add(axesHelper);

// Grid centered at origin
const gridHelper = new THREE.GridHelper(8, 16, 0x444444, 0x222222);
scene.add(gridHelper);

// IMU board representation (rectangular box)
const geometry = new THREE.BoxGeometry(2, 0.3, 1);
const materials = [
  new THREE.MeshLambertMaterial({ color: 0x666666 }), // right
  new THREE.MeshLambertMaterial({ color: 0x666666 }), // left
  new THREE.MeshLambertMaterial({ color: 0x4a90e2 }), // top (blue)
  new THREE.MeshLambertMaterial({ color: 0x333333 }), // bottom
  new THREE.MeshLambertMaterial({ color: 0x888888 }), // front
  new THREE.MeshLambertMaterial({ color: 0x444444 }), // back
];

const imuBoard = new THREE.Mesh(geometry, materials);
scene.add(imuBoard);

// Parse incoming data
// Expected format: "Roll:26.7 Pitch:-12.4 Yaw:159.7"
function parseData(line) {
  try {
    const match = line.match(/Roll:([-\d.]+)\s+Pitch:([-\d.]+)\s+Yaw:([-\d.]+)/);
    if (!match) return null;

    return {
      roll: parseFloat(match[1]),
      pitch: parseFloat(match[2]),
      yaw: parseFloat(match[3])
    };
  } catch (err) {
    console.error('Parse error:', err, 'Line:', line);
    return null;
  }
}

// WebSocket connection
let ws;
function connectWebSocket() {
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    console.log('WebSocket connected');
  };

  ws.onmessage = (event) => {
    const data = parseData(event.data);
    if (!data) return;

    // Apply rotation (only roll and pitch, ignore yaw)
    // Convert degrees to radians
    const rollRad = data.roll * Math.PI / 180;
    const pitchRad = data.pitch * Math.PI / 180;

    // Set rotation order and apply
    imuBoard.rotation.order = 'XZY';
    imuBoard.rotation.x = pitchRad;  // Pitch around X-axis
    imuBoard.rotation.z = rollRad;   // Roll around Z-axis
    imuBoard.rotation.y = 0;         // Ignore yaw
  };

  ws.onerror = (error) => {
    console.error('WebSocket error:', error);
  };

  ws.onclose = () => {
    console.log('WebSocket disconnected - reconnecting...');
    setTimeout(connectWebSocket, 2000);
  };
}

// Render loop
function animate() {
  requestAnimationFrame(animate);

  const width = canvas.clientWidth;
  const height = canvas.clientHeight;

  if (canvas.width !== width || canvas.height !== height) {
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }

  renderer.render(scene, camera);
}

// Handle window resize
window.addEventListener('resize', () => {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  renderer.setSize(width, height, false);
});

// Start
connectWebSocket();
animate();
