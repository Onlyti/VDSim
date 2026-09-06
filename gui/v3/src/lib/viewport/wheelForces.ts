import * as THREE from 'three'
import type { CarMesh } from './carMesh'

const FSCALE = 0.0012
const FZSCALE = 0.0006
const HEAD = 0.3

export interface WheelForceArrows {
  axX: THREE.Group[]
  axY: THREE.Group[]
  axZ: THREE.Group[]
}

function makeForceArrow(parent: THREE.Object3D, x: number, z: number, color: number): THREE.Group {
  const m = new THREE.MeshBasicMaterial({ color })
  const g = new THREE.Group()
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 1, 12), m)
  shaft.position.y = 0.5
  const head = new THREE.Mesh(new THREE.ConeGeometry(0.14, 0.3, 16), m)
  head.position.y = 1.0
  g.add(shaft, head)
  g.position.set(x, 0.02, z)
  g.visible = false
  g.userData = { shaft, head }
  parent.add(g)
  return g
}

export function attachWheelForces(car: CarMesh): WheelForceArrows {
  const axX: THREE.Group[] = []
  const axY: THREE.Group[] = []
  const axZ: THREE.Group[] = []
  for (const w of car.wheels) {
    const { x, z } = w.position
    axX.push(makeForceArrow(car.group, x, z, 0xff3b30))
    axY.push(makeForceArrow(car.group, x, z, 0x34c759))
    axZ.push(makeForceArrow(car.group, x, z, 0x0a84ff))
  }
  return { axX, axY, axZ }
}

function setArrow(ar: THREE.Group, dir: THREE.Vector3, mag: number, scale: number) {
  if (mag <= 5) {
    ar.visible = false
    return
  }
  ar.visible = true
  const L = Math.max(0.18, mag * scale)
  const shaft = ar.userData.shaft as THREE.Mesh
  const head = ar.userData.head as THREE.Mesh
  ar.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize())
  const sLen = Math.max(0.02, L - HEAD)
  shaft.scale.y = sLen
  shaft.position.y = sLen / 2
  head.position.y = sLen + HEAD / 2
}

export function updateWheelForces(
  car: CarMesh,
  forces: WheelForceArrows,
  Ft: number[][] | undefined,
  Fz: number[] | undefined,
  visible: boolean,
) {
  const { wheels, level } = car
  if (!visible || !wheels.length || level === 'K') {
    hideWheelForces(forces)
    return
  }
  const fz = Fz ?? [0, 0, 0, 0]
  const ft = (i: number) => (Ft && Ft[i] ? Ft[i] : [0, 0])
  const bike = wheels.length === 2
  const wf: [number, number, number][] = bike
    ? [
        [ft(0)[0] + ft(1)[0], ft(0)[1] + ft(1)[1], (fz[0] || 0) + (fz[1] || 0)],
        [ft(2)[0] + ft(3)[0], ft(2)[1] + ft(3)[1], (fz[2] || 0) + (fz[3] || 0)],
      ]
    : [0, 1, 2, 3].map((i) => [ft(i)[0], ft(i)[1], fz[i] || 0])

  const Y = new THREE.Vector3(0, 1, 0)
  for (let i = 0; i < wheels.length; i++) {
    const [Fx, Fy, Fzz] = wf[i]
    const w = wheels[i].position
    ;[forces.axX[i], forces.axY[i], forces.axZ[i]].forEach((a) => a.position.set(w.x, 0.02, w.z))
    setArrow(forces.axZ[i], Y, Fzz, FZSCALE)
    if (bike) {
      forces.axX[i].visible = false
      forces.axY[i].visible = false
    } else {
      setArrow(forces.axX[i], new THREE.Vector3(Math.sign(Fx) || 1, 0, 0), Math.abs(Fx), FSCALE)
      setArrow(forces.axY[i], new THREE.Vector3(0, 0, -(Math.sign(Fy) || 1)), Math.abs(Fy), FSCALE)
    }
  }
}

export function hideWheelForces(forces: WheelForceArrows) {
  for (const ar of [...forces.axX, ...forces.axY, ...forces.axZ]) ar.visible = false
}

export function disposeWheelForces(forces: WheelForceArrows) {
  for (const ar of [...forces.axX, ...forces.axY, ...forces.axZ]) {
    ar.parent?.remove(ar)
    ar.traverse((o) => {
      const m = o as THREE.Mesh
      if (m.geometry) m.geometry.dispose()
      if (m.material) (m.material as THREE.Material).dispose()
    })
  }
}
