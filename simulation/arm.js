// This javascript code is part of the interactive browser-based visualization
// for the simulated robot arm. Given a set of angles, it displays a 3D
// visualization of the arm. It also has controls that allow the user to send
// motion commands back to the server, setting new target joint angles.
// This code is loaded into the browser by arm.html.

// This picture included here only for quick reference. See docs/dh-notes.txt for details.
//
//                                    _           , . ,    <-- imaginary end point
//         grip finger length .......|            \   /
//                 +                 |_            \_/     <-- (Servo 1) Grip joint, open/close
// d5 =      hand-grip length .......|             /_\
//                 +                 |_            |=|     <-- (Servo 2) Hand joint, rotation
//          wrist-hand length .......|             | |
//                                   |_            (o)     <-- (Servo 3) Wrist joint, bending
//                                   |             | |
// a3 =    elbow-wrist length .......|             | |
//                                   |_            (o)     <-- (Servo 4) Elbow joint, bending
//                                   |            / /
// a2 = shoulder-elbow length .......|           / /
//                                   |          / /
//                                   |_        (o)         <-- (Servo 5) Shoulder joint, bending
// d1 =           base height .......|      ___|=|___ 
//                                   |_   /_____._____\    <-- (Servo 6) Base joint, rotation
//                                              ^---Origin of cartesian coordinate system,
//                                                  where x axis is to the right,
//                                                        y axis is into the page, and
//                                                        z axis is vertical.

const scaleFactor = 25.0;
function fromMeters(dist) { return dist * scaleFactor; } // arbitrary scale factor for the animation }

// note: This are placeholders, will get overwritten by values in json from server
let d1 = fromMeters(0.080); //  meters, height of shoulder joint above table or baseplate
let a2 = fromMeters(0.100); //  meters, distance between shoulder joint and elbow joint
let a3 = fromMeters(0.095); //  meters, distance between elbow joint and wrist joint
let d5 = fromMeters(0.160); //  meters, distance between wrist joint and an imaginary point between fingers
let axes = [ 'grip', 'hand', 'wrist', 'elbow', 'shoulder', 'base' ];
let joints = {
    grip:     { id: 1, name: "grip",      cmin: 155, cmax:  666, cini: 432,  dmin:   0,       dmax:  180,     doff: -140, orientation: -0.4  },
    hand:     { id: 2, name: "hand",      cmin: 120, cmax:  880, cini: 488,  dmin:  -90.0,    dmax:  90.0,    doff: 90,  orientation: -1  },
    wrist:    { id: 3, name: "wrist",     cmin:  70, cmax:  930, cini: 492,  dmin:  -103.476, dmax:  103.476, doff: 0,   orientation: +1  },
    elbow:    { id: 4, name: "elbow",     cmin:  10, cmax:  990, cini: 498,  dmin:  -119.118, dmax:  119.118, doff: 0,   orientation: -1  },
    shoulder: { id: 5, name: "shoulder",  cmin: 144, cmax:  880, cini: 512,  dmin:  -90.0,    dmax:  90.0,    doff: 0,   orientation: +1  },
    base:     { id: 6, name: "base",      cmin:   0, cmax: 1000, cini: 504,  dmin:  -120.321, dmax:  120.321, doff: 0,   orientation: +1  },
};

// Scene setup
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xffffff);
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
document.body.appendChild(renderer.domElement);

// Orbit controls
const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;
controls.zoomSpeed = 0.3;
controls.screenSpacePanning = false;
controls.minDistance = 5;
controls.maxDistance = 50;
controls.maxPolarAngle = Math.PI / 2;

// Materials
const baseMaterial = new THREE.MeshPhongMaterial({ 
    color: 0x444444,
    specular: 0x111111,
    shininess: 30
});
const armMaterial = new THREE.MeshPhongMaterial({ 
    color: 0x0389dd,
    specular: 0x222222,
    shininess: 30
});
const fingerMaterial = new THREE.MeshPhongMaterial({ 
    color: 0x666666,
    specular: 0x222222,
    shininess: 30
});
const targetMaterial = new THREE.MeshPhongMaterial({ 
    color: 0x990077,
    specular: 0x222222,
    shininess: 30
});
const jointMaterial = new THREE.MeshPhongMaterial({ 
    color: 0x666666,
    specular: 0x222222,
    shininess: 30
});
const circuitMaterial = new THREE.MeshPhongMaterial({ 
    color: 0x339933,
    specular: 0x222222,
    shininess: 30
});
const lineGeometry = new THREE.BufferGeometry();
const lineMaterial = new THREE.LineBasicMaterial({ color: 0xdf0000 });
const line = new THREE.Line(lineGeometry, lineMaterial);
const points = [];

// Lighting
const light = new THREE.DirectionalLight(0xffffff, 1);
light.position.set(10, 10, 10);
light.castShadow = true;
scene.add(light);
scene.add(new THREE.AmbientLight(0x404040));
    
// Ground plane
const groundGeometry = new THREE.PlaneGeometry(20, 20);
const groundMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 });
const ground = new THREE.Mesh(groundGeometry, groundMaterial);
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);

// Axis arrows
scene.add(createArrow(0x6666ff, new THREE.Vector3(1, 0, 0), 12, 0.15)); // X axis - blue
scene.add(createArrow(0x66ff66, new THREE.Vector3(0, 0, -1), 12, 0.15)); // Y axis - green
scene.add(createArrow(0xff3333, new THREE.Vector3(0, 1, 0), 12, 0.05)); // Z axis - red

// Camera position
camera.position.set(15, 10, 15);
camera.lookAt(0, 5, 0);

function buildrobot() {
    // todo: clear all except ground

    // Fixed mounting plate and circuit board
    const circuitGeometry = new THREE.BoxGeometry(1.5, 0.2, 2);
    const circuitMesh = new THREE.Mesh(circuitGeometry, circuitMaterial);
    circuitMesh.castShadow = true;
    circuitMesh.position.x = -2;
    circuitMesh.position.y = 0.2;
    circuitMesh.position.z = 0.0;
    scene.add(circuitMesh);
    const baseplateGeometry = new THREE.BoxGeometry(4, 0.2, 4);
    const baseplateMesh = new THREE.Mesh(baseplateGeometry, armMaterial);
    baseplateMesh.castShadow = true;
    baseplateMesh.position.x = 0.0;
    baseplateMesh.position.y = 0.1;
    baseplateMesh.position.z = 0.0;
    scene.add(baseplateMesh);

    // Robot arm parts
    const base = new THREE.Group();
    const shoulder = new THREE.Group();
    const elbow = new THREE.Group();
    const wrist = new THREE.Group();
    const hand = new THREE.Group();
    const grip = new THREE.Group();

    const motorGeometry = new THREE.CylinderGeometry(0.6, 0.6, 1, 32);
    const motor = new THREE.Mesh(motorGeometry, jointMaterial);
    motor.rotation.x = -Math.PI / 2;

    // Base plate and standoff
    const baseGeometry = new THREE.CylinderGeometry(1.5, 1.5, 0.1, 32);
    baseGeometry.translate(0, 1, 0);
    const baseMesh = new THREE.Mesh(baseGeometry, armMaterial);
    baseMesh.castShadow = true;
    base.add(baseMesh);
    const baseGeometry2 = new THREE.CylinderGeometry(1.5, 1.5, 0.1, 32);
    baseGeometry2.translate(0, 0.7, 0);
    const baseMesh2 = new THREE.Mesh(baseGeometry2, armMaterial);
    baseMesh2.castShadow = true;
    base.add(baseMesh2);

    const standoffGeometry = new THREE.BoxGeometry(1.8, (d1-1), 1);
    const standoffMesh = new THREE.Mesh(standoffGeometry, armMaterial);
    standoffMesh.position.x = -0.4;
    standoffMesh.position.y = d1/2;
    standoffMesh.castShadow = true;
    base.add(standoffMesh);

    // Shoulder with joint visualization
    shoulder.add(motor.clone());
    const shoulderGeometry = new THREE.BoxGeometry(1, (a2-1), 1);
    const shoulderMesh = new THREE.Mesh(shoulderGeometry, armMaterial);
    shoulderMesh.position.y = a2/2;
    shoulderMesh.castShadow = true;
    shoulder.add(shoulderMesh);

    // Elbow with joint visualization
    elbow.add(motor.clone());
    const elbowGeometry = new THREE.BoxGeometry(1, (a3-1), 1);
    const elbowMesh = new THREE.Mesh(elbowGeometry, armMaterial);
    elbowMesh.position.y = a3/2;
    elbowMesh.castShadow = true;
    elbow.add(elbowMesh);

    // Wrist with joint visualization
    wrist.add(motor.clone());
    const wristGeometry = new THREE.BoxGeometry(1, (d5-2-1), 1);
    const wristMesh = new THREE.Mesh(wristGeometry, armMaterial);
    wristMesh.position.y = (d5-2)/2;
    wristMesh.castShadow = true;
    wrist.add(wristMesh);

    // Hand with wrist rotation visualization
    const wristRotationMotor = motor.clone()
    wristRotationMotor.rotation.x = 0;
    wristRotationMotor.rotation.y = -Math.PI / 2;
    hand.add(wristRotationMotor);
    const handGeometry = new THREE.BoxGeometry(1, 0.4, 1);
    const handMesh = new THREE.Mesh(handGeometry, armMaterial);
    handMesh.position.y = 0.7;
    handMesh.castShadow = true;
    hand.add(handMesh);

    // Enhanced grip
    function createGripperFinger() {
        const finger = new THREE.Group();
        
        // Base of the finger
        const knuckle = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.8, 0.2), fingerMaterial);
        knuckle.position.y = 0.2;
        finger.add(knuckle);
        
        // Tip of the finger
        const tip = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.4, 0.15), fingerMaterial);
        tip.position.y = 0.7;
        finger.add(tip);

        finger.bend = function(angle) {
            finger.rotation.z = angle;
            tip.rotation.z = -angle;
        }
        finger.knuckle = knuckle;
        finger.tip = tip;
        
        return finger;
    }

    const leftFinger = createGripperFinger();
    const rightFinger = createGripperFinger();
    leftFinger.position.set(-0.3, 0.3, 0);
    rightFinger.position.set(0.3, 0.3, 0);
    grip.add(leftFinger);
    grip.add(rightFinger);

    targetball = new THREE.Mesh(new THREE.SphereGeometry(0.2), targetMaterial);

    // Hierarchy
    scene.add(base);
    base.add(shoulder);
    shoulder.position.y = d1;
    shoulder.add(elbow);
    elbow.position.y = a2;
    elbow.add(wrist);
    wrist.position.y = a3;
    wrist.add(hand);
    hand.position.y = d5-2; // most of length is for the hand, 1 for the grip position, 1 for ball position
    hand.add(grip);
    grip.position.y = 1;
    grip.add(targetball);
    targetball.position.y = 1;

    scene.add(line);

    // Movement
    joints['base'].move =     (value) => { base.rotation.y = setJointClicks('base', value); };
    joints['shoulder'].move = (value) => { shoulder.rotation.z = setJointClicks('shoulder', value); };
    joints['elbow'].move =    (value) => { elbow.rotation.z = setJointClicks('elbow', value); };
    joints['wrist'].move =    (value) => { wrist.rotation.z = setJointClicks('wrist', value); };
    joints['hand'].move =     (value) => { hand.rotation.y = setJointClicks('hand', value); };
    joints['grip'].move =     (value) => {
        const angle = setJointClicks('grip', value);
        leftFinger.bend(angle);
        rightFinger.bend(-angle);
    }

    // Configure sliders and initial joint values
    const coords = document.getElementById('coords');
    for (let j in joints) {
        const joint = joints[j]
        const val = joint.cini;
        const slider = document.getElementById(j);
        const display = document.getElementById(j + 'Value');
        joint.cval = val;
        slider.min = joint.cmin;
        slider.max = joint.cmax;
        slider.value = val;
        display.classList.add("display");
        // display.textContent = `${val}, ${toDegrees(joint).toFixed(2)}${degree}`;
        joint.slider = slider;
        joint.display = display;
        slider.addEventListener('input', () => panel_updateJoint(joint));
    }

    for (let j in joints) {
        const joint = joints[j];
        panel_updateJoint(joint);
    }

} // end of buildrobot()

let targetball = null;

function toDegrees(joint) {
    const val = joint.cval;
    return joint.dmin + (joint.cval - joint.cmin) / (joint.cmax - joint.cmin) * (joint.dmax - joint.dmin);
}

let ws;

const degree = '\u00B0';

function setJointClicks(j, value) { // returns angle of joint in radians
    const joint = joints[j];
    if (value <= joint.cmin || value >= joint.cmax) {
        joint.display.classList.add("warning");
    } else {
        joint.display.classList.remove("warning");
    }
    joint.cval = Math.max(joint.cmin, Math.min(joint.cmax, value));
    const deg = toDegrees(joint);
    const odeg = deg + joint.doff;
    joint.display.textContent = `${joint.cval} (${deg.toFixed(2)}${degree})`;
    return THREE.MathUtils.degToRad(odeg) * joint.orientation;
}

// User interaction
function panel_updateJoint(joint) {
    // get slider value
    const val = parseFloat(joint.slider.value);
    // update animation
    joint.move(val);
    // inform remote server
    if (ws != null && ws.readyState === WebSocket.OPEN) {
        const data = [joint.id, joint.cval & 0xff, (joint.cval >> 8) & 0xff];
        ws.send(new Uint8Array(data));
    }
}

// Remote arm control
const statusDot = document.createElement('div');
statusDot.classList.add('status-dot');
document.body.appendChild(statusDot);

function process_usb_report(msg) {
    const debug = false;
    if (debug)
        console.log('Processing USB report: [' + Array.from(msg)
            .map(b => '0x' + b.toString(16).padStart(2, '0'))
            .join(', ') +
        ']');

    if (msg.length < 3) {
        console.log('Report too short.');
        return;
    }
    if (msg[0] != 0x55 || msg[1] != 0x55) {
        console.log('Header invalid, should be [ 0x55, 0x55 ].');
        return;
    }
    const mlen = msg[2];
    if (mlen != msg.length - 2) {
        console.log('WARNING: length byte is ' + mlen + ' but report has ' + (msg.length-2) + ' bytes of data excluding the 2-byte header');
    }
    const cmd = msg[3] ?? 0;
    if (cmd == 3) { // move servo(s)
        if (debug)
            console.log("command: move servo(s)");
        const nservos = msg[4] ?? 0;
        if (debug)
            console.log("  nservos = " + nservos);
        if (mlen != 5 + nservos*3) { // 5 because len + cmd + nservo + t_lo + t_hi
            console.log('  WARNING: nservos was ' + nservos + ' but that requires length to be ' + (5 + nservos*3));
        }
        const t_lo = msg[5] ?? 0;
        const t_hi = msg[6] ?? 0;
        const t = (t_hi << 8) | t_lo;
        if (debug)
            console.log("  time = " + t);
        for (let i = 0; i < nservos; i++) {
            const axis = msg[2+5+3*i+0] ?? 0;
            const p_lo = msg[2+5+3*i+1] ?? 0;
            const p_hi = msg[2+5+3*i+2] ?? 0;
            const p = (p_hi << 8) | p_lo;
            if (debug)
                console.log("  move axis " + axis + " to position " + p);
            if (axis <= 0 || axis > axes.length) {
                console.log('  ERROR: no such axis');
                continue;
            }
            const j = axes[axis-1];
            const joint = joints[j];
            // update animation
            joint.move(p);
            // update slider
            joint.slider.value = joint.cval;
        }
        console.log('Positions: [ ' + Array.from(axes)
            .map(j => joints[j].cval.toString(10).padStart(4, ' '))
            .join(', ') +
        ' ]');
    } else if (cmd == 21) { // query position(s)
        // console.log("  ignoring query command")
    } else {
        console.log("  unrecognized command")
    }
}

function connect() {
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProtocol}//${window.location.hostname}:8001`;
    ws = new WebSocket(wsUrl);
    // const ws = new WebSocket('ws://localhost:8001');

    ws.onopen = () => {
        console.log('Connected to server');
        statusDot.style.backgroundColor = 'green';
    };

    ws.onclose = () => {
        console.log('Disconnected from server, reconnecting in 10 seconds');
        statusDot.style.backgroundColor = 'red';
        setTimeout(connect, 10000);  // Try reconnect in 10 seconds
    };

    ws.onmessage = (event) => {
        // console.log('Received:', event.data);
        if (event.data instanceof Blob) {
            // If binary data
            event.data.arrayBuffer().then(buffer => {
                const bytes = new Uint8Array(buffer);
                process_usb_report(bytes);
            });
        } else {
            // If text data
            // const bytes = new TextEncoder().encode(event.data);
            // armdata.push(...bytes);
            // pump_arm()
        }
    };

    ws.onerror = () => {
        ws.close();
    };
}

// Click red dot to try reconnecting immediately
statusDot.onclick = () => {
    if (ws.readyState === WebSocket.CLOSED) {
        connect();
    }
};

function configure() {
    fetch('parameters.json')
        .then(response => {
            if (!response.ok)
                throw new Error('Failed to get robot parameters: ' + response.status);
            return response.json();
        })
        .then(data => {
            console.log('Parameters received:', data);
            // Example: access specific parameters
            d1 = fromMeters(data.d1);
            a2 = fromMeters(data.a2);
            a3 = fromMeters(data.a3);
            d5 = fromMeters(data.d5);
            let cmin = data.cmin;
            let cmax = data.cmax;
            let rmin = data.rmin;
            let rmax = data.rmax;
            for (let j in joints) {
                const joint = joints[j]
                joint.cmin = cmin[joint.id];
                joint.cmax = cmax[joint.id];
                joint.cini = Math.round((joint.cmin + joint.cmax)/2);
                console.log(`Servo ${joint.id} range was ${joint.dmin} to ${joint.dmax} radians`);
                joint.dmin = rmin[joint.id] * 180/Math.PI;
                joint.dmax = rmax[joint.id] * 180/Math.PI;
                if (joint.slider) {
                    joint.slider.min = joint.cmin;
                    joint.slider.max = joint.cmax;
                }
                console.log(`          new range is ${joint.dmin} to ${joint.dmax} radians`);
            }
            buildrobot();
            connect();
        })
        .catch(error => {
            console.error('Fetch error:', error);
        });
}

// Create axes helper
function createArrow(color, direction, length, radius) {
    const linkGroup = new THREE.Group();
    const cylinderGeometry = new THREE.CylinderGeometry(radius, radius, length, 16);
    const material = new THREE.MeshBasicMaterial({color: color});
    const cylinder = new THREE.Mesh(cylinderGeometry, material);
    const coneHeight = 5*radius;
    const coneRadius = 3*radius;
    const coneGeometry = new THREE.ConeGeometry(coneRadius, coneHeight, 16);
    const cone = new THREE.Mesh(coneGeometry, material);
    if (direction.z === -1) {
        cylinder.rotation.x = Math.PI/2;
        cylinder.position.z = -length/2;
        cone.rotation.x = -Math.PI/2;
        cone.position.z = -length;
    } else if (direction.y === 1) {
        cylinder.position.y = length/2;
        cone.position.y = length;
    } else if (direction.x === 1) {
        cylinder.rotation.z = Math.PI/2;
        cylinder.position.x = length/2;
        cone.rotation.z = -Math.PI/2;
        cone.position.x = length;
    }
    linkGroup.add(cylinder);
    linkGroup.add(cone);
    return linkGroup;
}

// Add labels (using HTML divs positioned in 3D space)
const axisLabels = ['X', 'Z', 'Y'];
const positions = [
    new THREE.Vector3(12.2, 0, 0),
    new THREE.Vector3(0, 12.2, 0),
    new THREE.Vector3(0, 0, -12.2)
];
const colors = ['blue', 'red', 'green' ];

const axisUpdateFunctions = [];
positions.forEach((pos, i) => {
    const div = document.createElement('div');
    div.className = 'axis-label';
    div.textContent = axisLabels[i];
    div.style.color = colors[i];
    document.body.appendChild(div);

    // Update label position in animation loop
    axisUpdateFunctions.push(() => {
        const vector = pos.clone();
        vector.project(camera);
        div.style.left = (vector.x + 1) * window.innerWidth / 2 + 'px';
        div.style.top = (-vector.y + 1) * window.innerHeight / 2 + 'px';
    });
});


// Animation loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();

    // const positionL = new THREE.Vector3();
    // leftFinger.tip.getWorldPosition(positionL);
    // const positionR = new THREE.Vector3();
    // rightFinger.tip.getWorldPosition(positionR);
    // const position = positionL.clone().lerp(positionR, 0.5);
    const position = new THREE.Vector3();
    if (targetball) {
        targetball.getWorldPosition(position);
        points.push(position.clone());
        // the shoulder is a bit above the table, subtract that off
        // position.y -= shoulder.position.y
        position.multiplyScalar(1/scaleFactor);
        // THREE.js uses y as vertical (normal to the tabletop), xArm uses z as vertical
        coords.textContent = `x: ${(position.x*1000).toFixed(0)} mm  y: ${(-position.z*1000).toFixed(0)} mm  z: ${(position.y*1000).toFixed(0)} mm`
        lineGeometry.setFromPoints(points);
    }

    axisUpdateFunctions.forEach(fn => fn());

    renderer.render(scene, camera);
}

animate();

// Toggle button
const controlDiv = document.getElementById("controls");
const toggleButton = document.getElementById("toggle-button");
toggleButton.addEventListener("click", function () {
    if (controlDiv.classList.contains("collapsed")) {
        toggleButton.innerHTML = "[-]";
        controlDiv.classList.remove("collapsed");
    } else {
        toggleButton.innerHTML = "[+]";
        controlDiv.classList.add("collapsed");
    }
});

// Clear button
document.getElementById('clearButton').onclick = () => {
    points.length = 0;  // Clear array
    lineGeometry.setFromPoints(points);
};

// Home button
document.getElementById('homeButton').onclick = () => {
    for (let j in joints) {
        const joint = joints[j];
        joint.slider.value = joint.cini;
        panel_updateJoint(joint);
    }
};

// Handle window resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Initial connection
// connect();

// Initial configuration
configure();
