import { useEffect, useRef, useState } from "react"
import { ChevronRight, RefreshCcw } from "lucide-react"
import logoMark from "../../assets/icfm-logo-2.svg"

import { RocketCanvas } from "@/components/RocketCanvas"
import { TelemetryCharts } from "@/components/TelemetryCharts"
import { Button } from "@/components/ui/button"

type Metrics = Record<string, string>

const metricKeys = [
  "p_x", "p_y", "p_z",
  "v_x", "v_y", "v_z",
  "a_x", "a_y", "a_z",
  "wx", "wy", "wz",
  "d_theta", "d_alpha", "d_beta",
  "q_w", "q_x", "q_y", "q_z",
  "fin0", "fin1", "fin2", "fin3",
]

function quatToEuler(w: number, x: number, y: number, z: number) {
  const n = Math.hypot(w, x, y, z) || 1
  w /= n
  x /= n
  y /= n
  z /= n
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))))
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
  return [roll, pitch, yaw]
}

export default function App() {
  const [tab, setTab] = useState("space")
  const [wsState, setWsState] = useState("connecting")
  const [serialText, setSerialText] = useState("")
  const [metrics, setMetrics] = useState<Metrics>(() =>
    Object.fromEntries(metricKeys.map((k) => [k, "-"]))
  )

  const wsRef = useRef<WebSocket | null>(null)
  const serialViewportRef = useRef<HTMLDivElement | null>(null)
  const commonTimeRef = useRef(0)
  const quaternionRef = useRef({ w: 1, x: 0, y: 0, z: 0 })
  const finAnglesRef = useRef([0, 0, 0, 0])

  const altRef = useRef<number[]>([])
  const speedRef = useRef<number[]>([])
  const attRef = useRef({ roll: [] as number[], pitch: [] as number[], yaw: [] as number[] })
  const accRef = useRef({ x: [] as number[], y: [] as number[], z: [] as number[] })
  const finRef = useRef({ fin0: [] as number[], fin1: [] as number[], fin2: [] as number[], fin3: [] as number[] })

  useEffect(() => {
    const ws = new WebSocket("ws://localhost:8765")
    wsRef.current = ws

    ws.onopen = () => {
      setWsState("connected")
    }
    ws.onclose = () => {
      setWsState("disconnected")
    }
    ws.onerror = () => {
      setWsState("error")
    }

    ws.onmessage = (e) => {
      const msg = JSON.parse(e.data)

      if (msg.type === "serial_raw") {
        const line = String(msg.line ?? "")
        setSerialText((prev) => {
          const next = prev ? `${prev}\n${line}` : line
          const maxChars = 200_000
          return next.length > maxChars ? next.slice(next.length - maxChars) : next
        })
        return
      }

      if (msg.type === "command_ack") {
        return
      }

      if (msg.type === "kalman") {
        const d = msg.data
        commonTimeRef.current = d.t_s

        setMetrics((prev) => {
          const next = { ...prev }
          for (const k of ["p_x","p_y","p_z","v_x","v_y","v_z","a_x","a_y","a_z","wx","wy","wz","d_theta","d_alpha","d_beta","q_w","q_x","q_y","q_z"]) {
            if (d[k] !== undefined) next[k] = d[k].toFixed(2)
          }
          return next
        })

        const speed = Math.hypot(d.v_x, d.v_y, d.v_z)
        const [roll, pitch, yaw] = quatToEuler(d.q_w, d.q_x, d.q_y, d.q_z)
        altRef.current.push(d.p_z)
        speedRef.current.push(speed)
        attRef.current.roll.push(roll)
        attRef.current.pitch.push(pitch)
        attRef.current.yaw.push(yaw)
        accRef.current.x.push(d.a_x)
        accRef.current.y.push(d.a_y)
        accRef.current.z.push(d.a_z)

        if (d.q_w !== undefined) {
          quaternionRef.current = { w: d.q_w, x: d.q_x, y: d.q_y, z: d.q_z }
        }
      }

      if (msg.type === "ctrl") {
        const d = msg.data
        setMetrics((prev) => ({
          ...prev,
          fin0: d.fin0.toFixed(1),
          fin1: d.fin1.toFixed(1),
          fin2: d.fin2.toFixed(1),
          fin3: d.fin3.toFixed(1),
        }))
        finAnglesRef.current = [d.fin0, d.fin1, d.fin2, d.fin3]
        finRef.current.fin0.push(d.fin0)
        finRef.current.fin1.push(d.fin1)
        finRef.current.fin2.push(d.fin2)
        finRef.current.fin3.push(d.fin3)
      }
    }

    return () => {
      ws.close()
    }
  }, [])

  useEffect(() => {
    if (tab !== "serial") return
    const viewport = serialViewportRef.current
    if (!viewport) return
    requestAnimationFrame(() => {
      viewport.scrollTop = viewport.scrollHeight
    })
  }, [serialText, tab])

  const sendCommand = (cmd: "CALIBRATE" | "RESET") => {
    const ws = wsRef.current
    if (!ws || ws.readyState !== WebSocket.OPEN) {
      return
    }
    ws.send(JSON.stringify({ type: "command", cmd }))
  }

  return (
    <div className="h-screen bg-zinc-100 text-foreground">
      <div className="grid h-full grid-cols-1 md:grid-cols-[320px_1fr]">
        <aside className="flex min-h-0 flex-col gap-4 border-r bg-sidebar p-4 text-sidebar-foreground">
          <div className="flex items-center justify-between px-1">
            <div className="flex items-center gap-2">
              <img src={logoMark} alt="ICFM logo" className="h-8 w-8 rounded-sm" />
              <div className="flex flex-col leading-none">
                <span className="text-sm font-semibold">ICFM</span>
                <span className="text-xs font-medium text-muted-foreground">Viewer</span>
              </div>
            </div>
            <span
              className={`inline-block size-2 rounded-full ${
                wsState === "connected" ? "bg-green-500" : "bg-red-500"
              }`}
              aria-label={`WebSocket ${wsState}`}
              title={`WebSocket ${wsState}`}
            />
          </div>

          <div className="flex items-center gap-2">
            <Button size="sm" className="flex-1" onClick={() => sendCommand("CALIBRATE")}>Calibrate</Button>
            <Button size="sm" variant="secondary" className="flex-1" onClick={() => sendCommand("RESET")}>
              <RefreshCcw className="size-4" />Reset
            </Button>
          </div>

          <nav className="mt-5 space-y-1">
            <button
              type="button"
              onClick={() => setTab("space")}
              className={`flex w-full items-center justify-between rounded-md px-3 py-2 text-sm ${
                tab === "space" ? "bg-zinc-100 text-foreground" : "text-foreground hover:bg-zinc-100"
              }`}
            >
              <span>3D Space View</span>
              <ChevronRight className="size-4" />
            </button>
            <button
              type="button"
              onClick={() => setTab("graphs")}
              className={`flex w-full items-center justify-between rounded-md px-3 py-2 text-sm ${
                tab === "graphs" ? "bg-zinc-100 text-foreground" : "text-foreground hover:bg-zinc-100"
              }`}
            >
              <span>Graphs</span>
              <ChevronRight className="size-4" />
            </button>
            <button
              type="button"
              onClick={() => setTab("serial")}
              className={`flex w-full items-center justify-between rounded-md px-3 py-2 text-sm ${
                tab === "serial" ? "bg-zinc-100 text-foreground" : "text-foreground hover:bg-zinc-100"
              }`}
            >
              <span>Serial Monitor</span>
              <ChevronRight className="size-4" />
            </button>
          </nav>
        </aside>

        <main className="min-h-0 min-w-0">
          <div className="h-full w-full">
            <section className={tab === "space" ? "relative h-full w-full" : "hidden"}>
              <RocketCanvas finAnglesRef={finAnglesRef} quaternionRef={quaternionRef} />
              <div className="pointer-events-none absolute bottom-3 left-3 right-3 overflow-x-auto rounded-md border bg-sidebar px-3 py-2 text-sidebar-foreground">
                <div className="flex min-w-max items-center gap-4 text-xs">
                  {metricKeys.map((k) => (
                    <div key={k} className="flex items-center gap-1.5">
                      <span className="text-muted-foreground">{k}</span>
                      <span className="font-mono">{metrics[k]}</span>
                    </div>
                  ))}
                </div>
              </div>
            </section>
            <section className={tab === "graphs" ? "h-full w-full overflow-auto" : "hidden"}>
              <TelemetryCharts
                isLive={wsState === "connected"}
                timeRef={commonTimeRef}
                altRef={altRef}
                speedRef={speedRef}
                attRef={attRef}
                accRef={accRef}
                finRef={finRef}
              />
            </section>
            <section className={tab === "serial" ? "h-full w-full p-6" : "hidden"}>
              <div
                ref={serialViewportRef}
                className="h-full overflow-auto rounded-md border bg-black p-3 font-mono text-xs leading-relaxed text-zinc-100"
              >
                {serialText.length === 0 ? (
                  <span className="text-zinc-500">Waiting for serial data...</span>
                ) : (
                  <pre className="whitespace-pre">{serialText}</pre>
                )}
              </div>
            </section>
          </div>
        </main>
      </div>
    </div>
  )
}
