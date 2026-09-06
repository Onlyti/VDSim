import type { FleetAgent } from '../api/types'

export interface Viewport {
  xmin: number
  xmax: number
  ymin: number
  ymax: number
  scale: number
  width: number
  height: number
  pad: number
}

export function fitViewport(
  points: [number, number][],
  width: number,
  height: number,
  pad = 16,
): Viewport {
  let xmin = 0
  let xmax = 40
  let ymin = -20
  let ymax = 20
  if (points.length) {
    xmin = xmax = points[0][0]
    ymin = ymax = points[0][1]
    for (const [x, y] of points) {
      if (x < xmin) xmin = x
      if (x > xmax) xmax = x
      if (y < ymin) ymin = y
      if (y > ymax) ymax = y
    }
  }
  const span = Math.max(xmax - xmin, ymax - ymin, 24)
  const cx = 0.5 * (xmin + xmax)
  const cy = 0.5 * (ymin + ymax)
  const content = span * 1.1 + 16
  const drawW = Math.max(width - 2 * pad, 1)
  const drawH = Math.max(height - 2 * pad, 1)
  const scale = Math.min(drawW / content, drawH / content)
  const visibleW = drawW / scale
  const visibleH = drawH / scale
  return {
    xmin: cx - visibleW * 0.5,
    xmax: cx + visibleW * 0.5,
    ymin: cy - visibleH * 0.5,
    ymax: cy + visibleH * 0.5,
    scale,
    width,
    height,
    pad,
  }
}

export function worldToScreen(vp: Viewport, x: number, y: number): [number, number] {
  const { pad, height, xmin, ymin, scale } = vp
  const sx = pad + (x - xmin) * scale
  const sy = height - pad - (y - ymin) * scale
  return [sx, sy]
}

export function screenToWorld(vp: Viewport, sx: number, sy: number): [number, number] {
  const { pad, height, xmin, ymin, scale } = vp
  const x = xmin + (sx - pad) / scale
  const y = ymin + (height - pad - sy) / scale
  return [x, y]
}

export function drawMapScene(
  ctx: CanvasRenderingContext2D,
  vp: Viewport,
  path: [number, number][],
  fleet: FleetAgent[],
  selectedVid: number,
  pathEdit?: { selectedWp?: number },
) {
  const { width, height } = vp
  ctx.clearRect(0, 0, width, height)
  ctx.fillStyle = '#eef1f5'
  ctx.fillRect(0, 0, width, height)

  ctx.strokeStyle = '#d8dee8'
  ctx.lineWidth = 1
  const gridStep = niceGridStep(vp.xmax - vp.xmin)
  for (let gx = Math.floor(vp.xmin / gridStep) * gridStep; gx <= vp.xmax; gx += gridStep) {
    const [sx] = worldToScreen(vp, gx, 0)
    ctx.beginPath()
    ctx.moveTo(sx, 0)
    ctx.lineTo(sx, height)
    ctx.stroke()
  }
  for (let gy = Math.floor(vp.ymin / gridStep) * gridStep; gy <= vp.ymax; gy += gridStep) {
    const [, sy] = worldToScreen(vp, 0, gy)
    ctx.beginPath()
    ctx.moveTo(0, sy)
    ctx.lineTo(width, sy)
    ctx.stroke()
  }

  for (let gy = Math.floor(vp.ymin / gridStep) * gridStep; gy <= vp.ymax; gy += gridStep) {
    const [, sy] = worldToScreen(vp, 0, gy)
    ctx.beginPath()
    ctx.moveTo(0, sy)
    ctx.lineTo(width, sy)
    ctx.stroke()
  }

  ctx.fillStyle = '#8b95a5'
  ctx.font = '9px system-ui'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'top'
  for (let gx = Math.floor(vp.xmin / gridStep) * gridStep; gx <= vp.xmax; gx += gridStep) {
    const [sx] = worldToScreen(vp, gx, 0)
    if (sx >= 4 && sx <= width - 4) ctx.fillText(`${Math.round(gx)}`, sx, 2)
  }
  ctx.textAlign = 'right'
  ctx.textBaseline = 'middle'
  for (let gy = Math.floor(vp.ymin / gridStep) * gridStep; gy <= vp.ymax; gy += gridStep) {
    const [, sy] = worldToScreen(vp, 0, gy)
    if (sy >= 10 && sy <= height - 4) ctx.fillText(`${Math.round(gy)}`, width - 3, sy)
  }

  if (path.length > 1) {
    ctx.strokeStyle = '#2f6d8f'
    ctx.lineWidth = 2
    ctx.beginPath()
    path.forEach(([x, y], i) => {
      const [sx, sy] = worldToScreen(vp, x, y)
      if (i === 0) ctx.moveTo(sx, sy)
      else ctx.lineTo(sx, sy)
    })
    if (path.length > 2) {
      const [x0, y0] = path[0]
      const [xn, yn] = path[path.length - 1]
      if (Math.hypot(x0 - xn, y0 - yn) < 2) {
        const [sx, sy] = worldToScreen(vp, x0, y0)
        ctx.lineTo(sx, sy)
      }
    }
    ctx.stroke()

    ctx.fillStyle = '#01a0e9'
    path.forEach(([x, y], i) => {
      const [sx, sy] = worldToScreen(vp, x, y)
      const sel = pathEdit?.selectedWp === i
      ctx.beginPath()
      ctx.arc(sx, sy, pathEdit ? (sel ? 7 : 5) : 4, 0, Math.PI * 2)
      ctx.fillStyle = i === 0 ? '#34c759' : sel ? '#ff7a18' : '#01a0e9'
      ctx.fill()
      if (pathEdit) {
        ctx.strokeStyle = sel ? '#1a1d24' : '#ffffff'
        ctx.lineWidth = sel ? 2 : 1
        ctx.stroke()
        ctx.fillStyle = '#1a1d24'
        ctx.font = '9px system-ui'
        ctx.textAlign = 'center'
        ctx.textBaseline = 'middle'
        ctx.fillText(String(i + 1), sx, sy)
      }
    })
    ctx.fillStyle = '#01a0e9'
  }

  const colors = ['#01a0e9', '#ff9500', '#34c759', '#af52de']
  fleet.forEach((agent, i) => {
    const x = agent.x0 ?? 0
    const y = agent.y0 ?? 0
    const [sx, sy] = worldToScreen(vp, x, y)
    const sel = agent.id === selectedVid
    ctx.save()
    ctx.translate(sx, sy)
    ctx.rotate(-(agent.yaw0 ?? 0))
    ctx.fillStyle = colors[i % colors.length]
    ctx.strokeStyle = sel ? '#1a1d24' : '#ffffff'
    ctx.lineWidth = sel ? 2.5 : 1.5
    ctx.beginPath()
    ctx.moveTo(10, 0)
    ctx.lineTo(-7, 5)
    ctx.lineTo(-4, 0)
    ctx.lineTo(-7, -5)
    ctx.closePath()
    ctx.fill()
    ctx.stroke()
    ctx.restore()

    ctx.fillStyle = '#1a1d24'
    ctx.font = '10px system-ui'
    ctx.textAlign = 'center'
    ctx.fillText(`V${agent.id}`, sx, sy - 12)
  })
}

function niceGridStep(span: number): number {
  const raw = span / 8
  const mag = 10 ** Math.floor(Math.log10(raw))
  const n = raw / mag
  if (n < 1.5) return mag
  if (n < 3.5) return 2 * mag
  if (n < 7.5) return 5 * mag
  return 10 * mag
}

export function hitFleetAgent(
  vp: Viewport,
  fleet: FleetAgent[],
  sx: number,
  sy: number,
  radius = 14,
): FleetAgent | null {
  for (let i = fleet.length - 1; i >= 0; i--) {
    const a = fleet[i]
    const [ax, ay] = worldToScreen(vp, a.x0 ?? 0, a.y0 ?? 0)
    if (Math.hypot(ax - sx, ay - sy) <= radius) return a
  }
  return null
}

export function hitPathWaypoint(
  vp: Viewport,
  path: [number, number][],
  sx: number,
  sy: number,
  radius = 10,
): number {
  for (let i = path.length - 1; i >= 0; i--) {
    const [px, py] = worldToScreen(vp, path[i][0], path[i][1])
    if (Math.hypot(px - sx, py - sy) <= radius) return i
  }
  return -1
}
