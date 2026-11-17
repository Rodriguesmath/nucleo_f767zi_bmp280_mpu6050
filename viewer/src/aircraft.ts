import * as THREE from 'three';

export interface AircraftRig {
  root: THREE.Group;
  propeller: THREE.Group;
  aileronLeft: THREE.Group;
  aileronRight: THREE.Group;
  elevator: THREE.Group;
  rudder: THREE.Group;
  gArrow: THREE.ArrowHelper;
  strobeMat: THREE.MeshBasicMaterial;
  updateControlSurfaces: (
    dtSec: number,
    gyroXDps: number,
    gyroYDps: number,
    gyroZDps: number,
    accelXG: number,
    accelYG: number,
    accelZG: number,
    vsiMs: number,
    elapsedSec: number
  ) => void;
}

export function createAircraft(): AircraftRig {
  const root = new THREE.Group();

  const bodyMat = new THREE.MeshStandardMaterial({
    color: 0xe2e8f0,
    metalness: 0.45,
    roughness: 0.3,
  });
  const accentMat = new THREE.MeshStandardMaterial({
    color: 0x0284c7,
    metalness: 0.55,
    roughness: 0.25,
  });
  const darkMat = new THREE.MeshStandardMaterial({
    color: 0x1e293b,
    metalness: 0.65,
    roughness: 0.35,
  });
  const canopyMat = new THREE.MeshStandardMaterial({
    color: 0x38bdf8,
    metalness: 0.85,
    roughness: 0.1,
    transparent: true,
    opacity: 0.78,
  });
  const surfaceMat = new THREE.MeshStandardMaterial({
    color: 0xf59e0b,
    metalness: 0.35,
    roughness: 0.35,
  });

  // Fuselage main body aligned along Z (nose = -Z, tail = +Z)
  const fuselageGeo = new THREE.CylinderGeometry(0.28, 0.34, 2.5, 24);
  fuselageGeo.rotateX(Math.PI / 2);
  const fuselage = new THREE.Mesh(fuselageGeo, bodyMat);
  fuselage.castShadow = true;
  root.add(fuselage);

  // Nose cone (-Z)
  const noseGeo = new THREE.ConeGeometry(0.28, 0.9, 24);
  noseGeo.rotateX(-Math.PI / 2);
  const nose = new THREE.Mesh(noseGeo, accentMat);
  nose.position.z = -1.7;
  nose.castShadow = true;
  root.add(nose);

  // Tail cone (+Z)
  const tailGeo = new THREE.ConeGeometry(0.34, 1.15, 24);
  tailGeo.rotateX(Math.PI / 2);
  const tail = new THREE.Mesh(tailGeo, bodyMat);
  tail.position.z = 1.82;
  tail.castShadow = true;
  root.add(tail);

  // Dorsal racing stripe
  const stripe = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.04, 2.3), accentMat);
  stripe.position.set(0, 0.32, 0.1);
  root.add(stripe);

  // Cockpit canopy
  const canopyGeo = new THREE.SphereGeometry(0.27, 20, 14);
  canopyGeo.scale(0.85, 0.72, 2.2);
  const canopy = new THREE.Mesh(canopyGeo, canopyMat);
  canopy.position.set(0, 0.25, -0.45);
  root.add(canopy);

  // Swept main wings
  const wingGeo = new THREE.BoxGeometry(4.4, 0.07, 0.78);
  const wing = new THREE.Mesh(wingGeo, bodyMat);
  wing.position.set(0, -0.02, -0.1);
  wing.castShadow = true;
  root.add(wing);

  // Wing leading edge accent
  const wingStripe = new THREE.Mesh(new THREE.BoxGeometry(4.42, 0.08, 0.14), accentMat);
  wingStripe.position.set(0, -0.02, -0.45);
  root.add(wingStripe);

  // Winglets
  const wingletGeo = new THREE.BoxGeometry(0.05, 0.38, 0.42);
  const wingletL = new THREE.Mesh(wingletGeo, accentMat);
  wingletL.position.set(-2.22, 0.16, 0.02);
  wingletL.rotation.z = 0.22;
  root.add(wingletL);

  const wingletR = new THREE.Mesh(wingletGeo, accentMat);
  wingletR.position.set(2.22, 0.16, 0.02);
  wingletR.rotation.z = -0.22;
  root.add(wingletR);

  // Articulated Ailerons
  const aileronGeo = new THREE.BoxGeometry(0.95, 0.045, 0.22);
  const aileronLeft = new THREE.Group();
  aileronLeft.position.set(-1.55, -0.02, 0.29);
  const aileronLMesh = new THREE.Mesh(aileronGeo, surfaceMat);
  aileronLMesh.position.z = 0.11;
  aileronLeft.add(aileronLMesh);
  root.add(aileronLeft);

  const aileronRight = new THREE.Group();
  aileronRight.position.set(1.55, -0.02, 0.29);
  const aileronRMesh = new THREE.Mesh(aileronGeo, surfaceMat);
  aileronRMesh.position.z = 0.11;
  aileronRight.add(aileronRMesh);
  root.add(aileronRight);

  // Horizontal Stabilizer + Articulated Elevator
  const hStab = new THREE.Mesh(new THREE.BoxGeometry(1.65, 0.05, 0.42), bodyMat);
  hStab.position.set(0, 0.08, 1.95);
  root.add(hStab);

  const elevator = new THREE.Group();
  elevator.position.set(0, 0.08, 2.16);
  const elevatorMesh = new THREE.Mesh(new THREE.BoxGeometry(1.55, 0.04, 0.18), surfaceMat);
  elevatorMesh.position.z = 0.09;
  elevator.add(elevatorMesh);
  root.add(elevator);

  // Vertical Stabilizer + Articulated Rudder
  const vStab = new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.72, 0.56), accentMat);
  vStab.position.set(0, 0.45, 1.88);
  root.add(vStab);

  const rudder = new THREE.Group();
  rudder.position.set(0, 0.45, 2.16);
  const rudderMesh = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.68, 0.22), surfaceMat);
  rudderMesh.position.z = 0.11;
  rudder.add(rudderMesh);
  root.add(rudder);

  // Spinning nose propeller
  const propeller = new THREE.Group();
  propeller.position.set(0, 0, -2.18);
  const spinner = new THREE.Mesh(new THREE.ConeGeometry(0.11, 0.22, 16), darkMat);
  spinner.rotation.x = -Math.PI / 2;
  propeller.add(spinner);
  const bladeGeo = new THREE.BoxGeometry(0.08, 0.85, 0.02);
  for (let i = 0; i < 3; i++) {
    const bladeWrap = new THREE.Group();
    bladeWrap.rotation.z = (i * Math.PI * 2) / 3;
    const blade = new THREE.Mesh(bladeGeo, darkMat);
    blade.position.y = 0.42;
    blade.rotation.y = 0.3;
    bladeWrap.add(blade);
    propeller.add(bladeWrap);
  }
  root.add(propeller);

  // Wingtip navigation lights (Red left, Green right, White tail strobe)
  const navGeo = new THREE.SphereGeometry(0.055, 10, 10);
  const redLight = new THREE.Mesh(navGeo, new THREE.MeshBasicMaterial({ color: 0xef4444 }));
  redLight.position.set(-2.23, 0.36, 0.02);
  root.add(redLight);

  const greenLight = new THREE.Mesh(navGeo, new THREE.MeshBasicMaterial({ color: 0x22c55e }));
  greenLight.position.set(2.23, 0.36, 0.02);
  root.add(greenLight);

  const strobeMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
  const tailStrobe = new THREE.Mesh(navGeo, strobeMat);
  tailStrobe.position.set(0, 0.83, 1.95);
  root.add(tailStrobe);

  // Live 3D Acceleration G-Vector arrow
  const gArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, 0, 0),
    1.5,
    0xf59e0b,
    0.26,
    0.14
  );
  root.add(gArrow);

  const gVec = new THREE.Vector3();

  const updateControlSurfaces = (
    dtSec: number,
    gyroXDps: number,
    gyroYDps: number,
    gyroZDps: number,
    accelXG: number,
    accelYG: number,
    accelZG: number,
    vsiMs: number,
    elapsedSec: number
  ): void => {
    const propRpmRad = 22 + Math.max(-8, Math.min(18, vsiMs * 2.5));
    propeller.rotation.z += propRpmRad * dtSec;

    const aileronDeflect = THREE.MathUtils.clamp(gyroXDps * 0.012, -0.55, 0.55);
    aileronLeft.rotation.x = aileronDeflect;
    aileronRight.rotation.x = -aileronDeflect;

    elevator.rotation.x = THREE.MathUtils.clamp(-gyroYDps * 0.012, -0.5, 0.5);
    rudder.rotation.y = THREE.MathUtils.clamp(-gyroZDps * 0.012, -0.55, 0.55);

    // Map body accelerometer (X forward, Y right, Z up/down) into Three.js body frame
    gVec.set(accelYG, accelZG, -accelXG);
    const mag = gVec.length();
    if (mag > 0.05) {
      gVec.normalize();
      gArrow.setDirection(gVec);
      gArrow.setLength(THREE.MathUtils.clamp(mag * 1.4, 0.4, 3.2), 0.28, 0.15);
      gArrow.visible = true;
    } else {
      gArrow.visible = false;
    }

    const pulse = (elapsedSec % 1.1) < 0.12;
    strobeMat.color.setHex(pulse ? 0xffffff : 0x334155);
  };

  return {
    root,
    propeller,
    aileronLeft,
    aileronRight,
    elevator,
    rudder,
    gArrow,
    strobeMat,
    updateControlSurfaces,
  };
}
