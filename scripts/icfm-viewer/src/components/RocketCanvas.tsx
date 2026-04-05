import { useEffect, useRef, type MutableRefObject } from "react"
import * as THREE from "three"

type RocketCanvasProps = {
  finAnglesRef: MutableRefObject<number[]>
  quaternionRef: MutableRefObject<{ w: number; x: number; y: number; z: number }>
}

export function RocketCanvas({ finAnglesRef, quaternionRef }: RocketCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const host = canvas.parentElement
    if (!host) return

    const renderer = new THREE.WebGLRenderer({ canvas, antialias: true })
    renderer.setClearColor(0xf8fafc)
    renderer.setPixelRatio(window.devicePixelRatio)

    const scene = new THREE.Scene()
    const camera = new THREE.PerspectiveCamera(40, 2, 0.1, 100)
    scene.add(new THREE.AmbientLight(0xffffff, 1.35))

    const axisExtent = 25
    const axisGeometry = new THREE.BufferGeometry()
    axisGeometry.setAttribute(
      "position",
      new THREE.Float32BufferAttribute(
        [
          -axisExtent, 0, 0, axisExtent, 0, 0,
          0, -axisExtent, 0, 0, axisExtent, 0,
          0, 0, -axisExtent, 0, 0, axisExtent,
        ],
        3
      )
    )
    axisGeometry.setAttribute(
      "color",
      new THREE.Float32BufferAttribute(
        [
          0.88, 0.40, 0.40, 0.88, 0.40, 0.40,
          0.40, 0.78, 0.50, 0.40, 0.78, 0.50,
          0.40, 0.56, 0.88, 0.40, 0.56, 0.88,
        ],
        3
      )
    )
    scene.add(new THREE.LineSegments(axisGeometry, new THREE.LineBasicMaterial({ vertexColors: true })))

    const worldFrameGroup = new THREE.Group()
    worldFrameGroup.rotation.z = Math.PI / 2
    scene.add(worldFrameGroup)

    const rocketGroup = new THREE.Group()
    worldFrameGroup.add(rocketGroup)
    const rocketModel = new THREE.Group()
    rocketModel.rotation.z = -Math.PI / 2
    rocketGroup.add(rocketModel)

    rocketModel.add(
      new THREE.Mesh(
        new THREE.CylinderGeometry(0.2, 0.2, 2.5, 24),
        new THREE.MeshStandardMaterial({ color: 0x3a3a3a, metalness: 0.35, roughness: 0.55 })
      )
    )

    const nose = new THREE.Mesh(
      new THREE.ConeGeometry(0.2, 0.7, 24),
      new THREE.MeshStandardMaterial({ color: 0x555555 })
    )
    nose.position.y = 1.6
    rocketModel.add(nose)

    const finColors = [0x6095eb, 0x60eb95, 0xebc060, 0xeb6060]
    const finMeshes: THREE.Mesh[] = []
    const FIN_SCALE = 3

    for (let i = 0; i < 4; i++) {
      const orientGroup = new THREE.Group()
      orientGroup.rotation.y = i * Math.PI / 2
      rocketModel.add(orientGroup)

      const hingeGroup = new THREE.Group()
      hingeGroup.position.set(0.2, -0.8, 0)
      orientGroup.add(hingeGroup)

      const shape = new THREE.Shape()
      shape.moveTo(0, 0)
      shape.lineTo(0.38, -0.15)
      shape.lineTo(0.38, -0.6)
      shape.lineTo(0, -0.45)
      shape.closePath()

      const finMesh = new THREE.Mesh(
        new THREE.ShapeGeometry(shape),
        new THREE.MeshStandardMaterial({ color: finColors[i], side: THREE.DoubleSide })
      )
      finMeshes.push(finMesh)
      hingeGroup.add(finMesh)
    }

    let CAM_R = 7
    let thetaY = 0.4
    let thetaX = 0.2
    let drag = false
    let lx = 0
    let ly = 0

    const onMouseDown = (e: MouseEvent) => {
      drag = true
      lx = e.clientX
      ly = e.clientY
    }
    const onMouseUp = () => {
      drag = false
    }
    const onMouseMove = (e: MouseEvent) => {
      if (!drag) return
      thetaY -= (e.clientX - lx) * 0.007
      thetaX += (e.clientY - ly) * 0.004
      thetaX = Math.max(-0.5, Math.min(0.8, thetaX))
      lx = e.clientX
      ly = e.clientY
    }
    const onWheel = (e: WheelEvent) => {
      e.preventDefault()
      CAM_R -= e.deltaY * 0.01
      CAM_R = Math.max(2, Math.min(12, CAM_R))
    }

    canvas.addEventListener("mousedown", onMouseDown)
    window.addEventListener("mouseup", onMouseUp)
    window.addEventListener("mousemove", onMouseMove)
    canvas.addEventListener("wheel", onWheel, { passive: false })

    let raf = 0
    const resize = () => {
      const rect = host.getBoundingClientRect()
      const w = Math.max(1, Math.floor(rect.width))
      const h = Math.max(1, Math.floor(rect.height))
      renderer.setSize(w, h, false)
      camera.aspect = w / h
      camera.updateProjectionMatrix()
    }

    resize()
    const ro = new ResizeObserver(resize)
    ro.observe(host)

    const animate = () => {
      raf = requestAnimationFrame(animate)

      const q = quaternionRef.current
      const n = Math.hypot(q.w, q.x, q.y, q.z) || 1
      rocketGroup.quaternion.set(q.x / n, q.y / n, q.z / n, q.w / n)

      for (let i = 0; i < 4; i++) {
        finMeshes[i].rotation.x = -finAnglesRef.current[i] * FIN_SCALE * Math.PI / 180
      }

      camera.position.x = CAM_R * Math.sin(thetaY) * Math.cos(thetaX)
      camera.position.y = CAM_R * Math.sin(thetaX) + 0.5
      camera.position.z = CAM_R * Math.cos(thetaY) * Math.cos(thetaX)
      camera.lookAt(0, 0, 0)
      renderer.render(scene, camera)
    }
    animate()

    return () => {
      cancelAnimationFrame(raf)
      canvas.removeEventListener("mousedown", onMouseDown)
      window.removeEventListener("mouseup", onMouseUp)
      window.removeEventListener("mousemove", onMouseMove)
      canvas.removeEventListener("wheel", onWheel)
      ro.disconnect()
      renderer.dispose()
      scene.traverse((obj: THREE.Object3D) => {
        if (obj instanceof THREE.Mesh) {
          obj.geometry.dispose()
          const material = obj.material
          if (Array.isArray(material)) material.forEach((m) => m.dispose())
          else material.dispose()
        }
      })
    }
  }, [finAnglesRef, quaternionRef])

  return (
    <div className="relative h-full w-full overflow-hidden bg-white">
      <canvas ref={canvasRef} className="block h-full w-full cursor-grab" />
    </div>
  )
}
