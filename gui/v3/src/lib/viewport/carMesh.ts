import * as THREE from 'three'
import type { VehicleGeom } from './vehicleGeom'
import { attachWheelForces, type WheelForceArrows } from './wheelForces'

const FLEET_COLORS = [0x01a0e9, 0xdc291e, 0x34c759, 0xf5a623]
const FLEET_CABIN = [0x9fd8f5, 0xf5a8a4, 0xb8f0c8, 0xffe0b0]

export interface CarMesh {
  group: THREE.Group
  wheels: THREE.Object3D[]
  wheelRoll: number[]
  steerIdx: number[]
  level: string
  forces: WheelForceArrows
}

function makeWheel(R: number): THREE.Group {
  const spin = new THREE.Group()
  spin.rotation.order = 'YXZ'
  const w = new THREE.Mesh(
    new THREE.CylinderGeometry(R, R, 0.22, 14),
    new THREE.MeshStandardMaterial({ color: 0x14181f }),
  )
  w.rotation.x = Math.PI / 2
  spin.add(w)
  return spin
}

export function buildCarMesh(geom: VehicleGeom, level: string, vid: number): CarMesh {
  const grp = new THREE.Group()
  grp.rotation.order = 'YXZ'
  grp.userData.vid = vid

  const a = geom.cg_to_front
  const b = geom.cg_to_rear
  const tf = geom.track_front
  const tr = geom.track_rear
  const R = geom.wheel_radius_nominal
  const bike = level === 'K' || level === 'L1'
  const L = a + b
  const W = Math.max(tf, tr, 0.6)
  const bodyH = 0.5
  const bodyY = R + 0.45
  const bodyW = bike ? 0.45 : Math.max(0.4, W - 0.5)

  const bm = new THREE.MeshStandardMaterial({
    color: FLEET_COLORS[vid % FLEET_COLORS.length],
    metalness: 0.3,
    roughness: 0.5,
  })
  const cm = new THREE.MeshStandardMaterial({
    color: FLEET_CABIN[vid % FLEET_CABIN.length],
    metalness: 0.25,
    roughness: 0.55,
  })

  const body = new THREE.Mesh(new THREE.BoxGeometry(L * 0.78, bodyH, bodyW), bm)
  body.position.set((a - b) / 2, bodyY, 0)
  grp.add(body)

  const cabin = new THREE.Mesh(
    new THREE.BoxGeometry(L * 0.34, 0.38, bike ? 0.34 : Math.max(0.3, W * 0.55)),
    cm,
  )
  cabin.position.set((a - b) / 2 - 0.15, bodyY + bodyH / 2 + 0.19, 0)
  grp.add(cabin)

  const pos: [number, number][] = bike
    ? [[a, 0], [-b, 0]]
    : [
        [a, -tf / 2],
        [a, tf / 2],
        [-b, -tr / 2],
        [-b, tr / 2],
      ]

  const wheels: THREE.Object3D[] = []
  for (const [x, z] of pos) {
    const w = makeWheel(R)
    w.position.set(x, R, z)
    grp.add(w)
    wheels.push(w)
  }

  const mesh: CarMesh = {
    group: grp,
    wheels,
    wheelRoll: wheels.map(() => 0),
    steerIdx: bike ? [0] : [0, 1],
    level,
    forces: { axX: [], axY: [], axZ: [] },
  }
  mesh.forces = attachWheelForces(mesh)
  return mesh
}

export function applySteer(wheels: THREE.Object3D[], steerIdx: number[], steer: number) {
  for (const i of steerIdx) {
    if (wheels[i]) wheels[i].rotation.y = steer
  }
}

export function spinWheels(
  wheels: THREE.Object3D[],
  wheelRoll: number[],
  spin: number[],
  dt: number,
) {
  while (wheelRoll.length < wheels.length) wheelRoll.push(0)
  for (let i = 0; i < wheels.length; i++) {
    const rate =
      wheels.length === 2
        ? i === 0
          ? ((spin[0] ?? 0) + (spin[1] ?? 0)) * 0.5
          : ((spin[2] ?? 0) + (spin[3] ?? 0)) * 0.5
        : (spin[i] ?? 0)
    wheelRoll[i] -= rate * dt
    wheels[i].rotation.z = wheelRoll[i]
  }
}
