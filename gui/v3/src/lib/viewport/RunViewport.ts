import * as THREE from 'three'
import { OrbitControls } from '@vdsim/vendor-orbit-controls'
import { applySteer, buildCarMesh, spinWheels, type CarMesh } from './carMesh'
import { lerpAngle, toThree, vdsimIsoToThreeQuat } from './coords'
import type { SimStreamState } from './types'
import { pickVehicleState } from './types'
import { fetchVehicleGeom } from './vehicleGeom'
import { disposeWheelForces, hideWheelForces, updateWheelForces } from './wheelForces'

export type CamMode = 'chase' | 'orbit' | 'top'

export class RunViewport {
  private scene = new THREE.Scene()
  private camera = new THREE.PerspectiveCamera(55, 1, 0.1, 2000)
  private ortho = new THREE.OrthographicCamera(-35, 35, 35, -35, 0.1, 2000)
  private renderer: THREE.WebGLRenderer
  private controls: InstanceType<typeof OrbitControls>
  private clock = new THREE.Clock()
  private raf = 0
  private car: CarMesh | null = null
  private carLevel = ''
  private liveVid = 0
  private camMode: CamMode = 'chase'
  private running = false
  private showForces = true
  private tgt = {
    has: false,
    pos: new THREE.Vector3(),
    yaw: 0,
    roll: 0,
    pitch: 0,
    vx: 0,
    vy: 0,
    r: 0,
    wheelSpin: [0, 0, 0, 0] as number[],
    Ft: [] as number[][],
    Fz: [] as number[],
  }
  private pathLine: THREE.Line | null = null
  private pathCount = 0
  private onState: ((s: SimStreamState) => void) | null = null

  constructor(private mount: HTMLElement) {
    this.scene.background = new THREE.Color(0xe9eef3)
    this.camera.position.set(15, 14, 22)
    this.ortho.position.set(0, 120, 0)
    this.ortho.up.set(0, 0, -1)
    this.ortho.lookAt(0, 0, 0)

    this.renderer = new THREE.WebGLRenderer({ antialias: true })
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
    this.mount.appendChild(this.renderer.domElement)

    this.controls = new OrbitControls(this.camera, this.renderer.domElement)
    this.controls.minDistance = 4
    this.controls.maxDistance = 480

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x9aa6b2, 1.15))
    const dir = new THREE.DirectionalLight(0xffffff, 0.8)
    dir.position.set(20, 40, 15)
    this.scene.add(dir)
    this.scene.add(new THREE.GridHelper(120, 60, 0xb6c2cf, 0xd2dae3))

    this.resize()
    const ro = new ResizeObserver(() => this.resize())
    ro.observe(mount)
    this.mount.dataset.ro = '1'

    this.loop()
    void this.loadPath()
  }

  setCamMode(m: CamMode) {
    this.camMode = m
    this.controls.enabled = m === 'orbit'
  }

  setOnState(cb: (s: SimStreamState) => void) {
    this.onState = cb
  }

  setShowForces(on: boolean) {
    this.showForces = on
    if (!on && this.car) hideWheelForces(this.car.forces)
  }

  async applyState(s: SimStreamState) {
    this.running = !!(s.running && !s.setup_mode)
    const vid = s.live_vid ?? 0
    this.liveVid = vid
    const preview = !!(s.setup_mode || !s.running)
    const st = pickVehicleState(s, vid)

    if (preview) {
      if (this.car) hideWheelForces(this.car.forces)
      const level = st.level ?? s.level ?? 'L2'
      if (!this.car || this.carLevel !== level) {
        await this.rebuildCar(level, vid)
      }
      if (this.car && st.x != null) {
        this.tgt.pos.copy(toThree(st.x, st.y ?? 0, st.z ?? 0))
        this.tgt.yaw = st.yaw ?? 0
        this.tgt.roll = 0
        this.tgt.pitch = 0
        this.tgt.has = true
      }
      const npath = s.npath
      if (npath != null && npath !== this.pathCount) {
        this.pathCount = npath
        void this.loadPath()
      }
      this.onState?.(s)
      return
    }

    const level = st.level ?? s.level ?? 'L2'
    if (!this.car || this.carLevel !== level) {
      await this.rebuildCar(level, vid)
    }

    if (st.x == null || !this.car) {
      this.onState?.(s)
      return
    }

    this.tgt.pos.copy(toThree(st.x, st.y ?? 0, st.z ?? 0))
    this.tgt.yaw = st.yaw ?? 0
    const dyn3 = level === 'L3' || level === 'L4' || level === 'L5'
    this.tgt.roll = dyn3 ? (st.roll ?? 0) : 0
    this.tgt.pitch = dyn3 ? (st.pitch ?? 0) : 0
    this.tgt.vx = st.vx ?? 0
    this.tgt.vy = st.vy ?? 0
    this.tgt.r = st.r ?? 0
    this.tgt.wheelSpin = st.wheel_spin ?? [0, 0, 0, 0]
    this.tgt.Ft = st.Ft ?? []
    this.tgt.Fz = st.Fz ?? []
    this.tgt.has = true

    if (st.steer != null) applySteer(this.car.wheels, this.car.steerIdx, st.steer)
    this.onState?.(s)
  }

  private async rebuildCar(level: string, vid: number) {
    if (this.car) {
      disposeWheelForces(this.car.forces)
      this.scene.remove(this.car.group)
      this.car.group.traverse((o: THREE.Object3D) => {
        const m = o as THREE.Mesh
        if (m.geometry) m.geometry.dispose()
      })
    }
    const geom = await fetchVehicleGeom(vid)
    this.car = buildCarMesh(geom, level, vid)
    this.carLevel = level
    this.scene.add(this.car.group)
  }

  private async loadPath() {
    try {
      const r = await fetch('/api/path')
      const j = await r.json()
      const pts = (j.pts ?? []) as [number, number][]
      if (pts.length < 2) return
      const v = pts.map((p) => toThree(p[0], p[1], 0.15))
      if (this.pathLine) {
        this.scene.remove(this.pathLine)
        this.pathLine.geometry.dispose()
      }
      const mat = new THREE.LineBasicMaterial({ color: 0x2f6d8f })
      this.pathLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(v), mat)
      this.scene.add(this.pathLine)
      this.pathCount = pts.length
    } catch {
      /* optional */
    }
  }

  private resize() {
    const w = this.mount.clientWidth
    const h = this.mount.clientHeight
    if (w < 1 || h < 1) return
    this.camera.aspect = w / h
    this.camera.updateProjectionMatrix()
    const aspect = w / h
    const s = 70
    this.ortho.left = (-s / 2) * aspect
    this.ortho.right = (s / 2) * aspect
    this.ortho.top = s / 2
    this.ortho.bottom = -s / 2
    this.ortho.updateProjectionMatrix()
    this.renderer.setSize(w, h, false)
  }

  private loop = () => {
    this.raf = requestAnimationFrame(this.loop)
    const dt = Math.min(this.clock.getDelta(), 0.1)
    const car = this.car

    if (car && this.tgt.has) {
      if (this.running) {
        const yaw = this.tgt.yaw
        const vE = this.tgt.vx * Math.cos(yaw) - this.tgt.vy * Math.sin(yaw)
        const vN = this.tgt.vx * Math.sin(yaw) + this.tgt.vy * Math.cos(yaw)
        car.group.position.x += vE * dt
        car.group.position.z += -vN * dt
        car.group.position.lerp(this.tgt.pos, 0.1)
        vdsimIsoToThreeQuat(
          this.tgt.roll,
          this.tgt.pitch,
          lerpAngle(yaw + this.tgt.r * dt, this.tgt.yaw, 0.15),
          car.group.quaternion,
        )
        spinWheels(car.wheels, car.wheelRoll, this.tgt.wheelSpin, dt)
        updateWheelForces(car, car.forces, this.tgt.Ft, this.tgt.Fz, this.showForces)
      } else {
        car.group.position.copy(this.tgt.pos)
        vdsimIsoToThreeQuat(0, 0, this.tgt.yaw, car.group.quaternion)
        hideWheelForces(car.forces)
      }

      const cp = car.group.position
      if (this.camMode === 'chase') {
        const fx = Math.cos(this.tgt.yaw)
        const fz = -Math.sin(this.tgt.yaw)
        const chase = new THREE.Vector3(cp.x - fx * 11, cp.y + 5, cp.z - fz * 11)
        if (this.running) this.camera.position.lerp(chase, 0.12)
        else this.camera.position.copy(chase)
        this.camera.lookAt(cp.x, cp.y + 1.2, cp.z)
      } else if (this.camMode === 'top') {
        this.ortho.position.set(cp.x, 120, cp.z)
        this.ortho.lookAt(cp.x, 0, cp.z)
      } else if (this.running) {
        this.controls.target.lerp(cp, 0.15)
        this.controls.update()
      } else {
        this.controls.target.copy(cp)
        this.controls.update()
      }
    }

    const active = this.running && this.camMode === 'top' ? this.ortho : this.camera
    this.renderer.render(this.scene, active)
  }

  dispose() {
    cancelAnimationFrame(this.raf)
    this.controls.dispose()
    this.renderer.dispose()
    if (this.renderer.domElement.parentElement === this.mount) {
      this.mount.removeChild(this.renderer.domElement)
    }
    if (this.pathLine) {
      this.pathLine.geometry.dispose()
      ;(this.pathLine.material as THREE.Material).dispose()
    }
    if (this.car) {
      disposeWheelForces(this.car.forces)
      this.car.group.traverse((o: THREE.Object3D) => {
        const m = o as THREE.Mesh
        if (m.geometry) m.geometry.dispose()
      })
    }
  }
}
