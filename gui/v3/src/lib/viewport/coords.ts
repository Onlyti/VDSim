import * as THREE from 'three'

const _q = new THREE.Quaternion()

export function toThree(x: number, y: number, z = 0): THREE.Vector3 {
  return new THREE.Vector3(x, z, -y)
}

export function vdsimIsoToThreeQuat(roll: number, pitch: number, yaw: number, out = _q): THREE.Quaternion {
  const e = new THREE.Euler(roll, yaw, -pitch, 'ZYX')
  return out.setFromEuler(e)
}

export function lerpAngle(a: number, b: number, t: number): number {
  let d = ((b - a + Math.PI) % (2 * Math.PI)) - Math.PI
  if (d < -Math.PI) d += 2 * Math.PI
  return a + d * t
}
