import { useEffect, useRef, type MutableRefObject } from "react"
import { Chart, registerables } from "chart.js"

Chart.register(...registerables)

type TelemetryChartsProps = {
  isLive: boolean
  timeRef: MutableRefObject<number>
  altRef: MutableRefObject<number[]>
  speedRef: MutableRefObject<number[]>
  attRef: MutableRefObject<{ roll: number[]; pitch: number[]; yaw: number[] }>
  accRef: MutableRefObject<{ x: number[]; y: number[]; z: number[] }>
  finRef: MutableRefObject<{ fin0: number[]; fin1: number[]; fin2: number[]; fin3: number[] }>
}

const MAX_POINTS = 200

export function TelemetryCharts({
  isLive,
  timeRef,
  altRef,
  speedRef,
  attRef,
  accRef,
  finRef,
}: TelemetryChartsProps) {
  const altEl = useRef<HTMLCanvasElement | null>(null)
  const attEl = useRef<HTMLCanvasElement | null>(null)
  const accEl = useRef<HTMLCanvasElement | null>(null)
  const finEl = useRef<HTMLCanvasElement | null>(null)
  const liveRef = useRef(isLive)

  useEffect(() => {
    liveRef.current = isLive
  }, [isLive])

  useEffect(() => {
    Chart.defaults.font.size = 11

    const mk = (el: HTMLCanvasElement, datasets: { label: string; color: string }[]) =>
      new Chart(el, {
        type: "line",
        data: {
          labels: [],
          datasets: datasets.map((d) => ({
            label: d.label,
            data: [],
            borderColor: d.color,
            backgroundColor: `${d.color}33`,
            borderWidth: 1.5,
            pointRadius: 0,
            tension: 0.1,
          })),
        },
        options: {
          animation: false,
          responsive: true,
          maintainAspectRatio: false,
          plugins: { legend: { position: "top", align: "end", labels: { boxWidth: 12, padding: 4 } } },
          scales: {
            x: { grid: { color: "rgba(128,128,128,0.2)" }, ticks: { maxTicksLimit: 8 } },
            y: { grid: { color: "rgba(128,128,128,0.2)" }, ticks: { maxTicksLimit: 6 } },
          },
        },
      })

    if (!altEl.current || !attEl.current || !accEl.current || !finEl.current) return

    const altChart = mk(altEl.current, [
      { label: "alt", color: "#1f77b4" },
      { label: "speed", color: "#2ca02c" },
    ])
    const attChart = mk(attEl.current, [
      { label: "roll", color: "#d62728" },
      { label: "pitch", color: "#ff7f0e" },
      { label: "yaw", color: "#9467bd" },
    ])
    const accChart = mk(accEl.current, [
      { label: "a_x", color: "#1f77b4" },
      { label: "a_y", color: "#2ca02c" },
      { label: "a_z", color: "#d62728" },
    ])
    const finChart = mk(finEl.current, [
      { label: "fin0", color: "#1f77b4" },
      { label: "fin1", color: "#2ca02c" },
      { label: "fin2", color: "#ff7f0e" },
      { label: "fin3", color: "#d62728" },
    ])

    let lastTime = -1
    const interval = window.setInterval(() => {
      if (!liveRef.current) return
      const t = timeRef.current
      if (!Number.isFinite(t) || t === lastTime) return
      lastTime = t

      const apply = (chart: Chart, values: number[]) => {
        chart.data.labels?.push(t.toFixed(1))
        values.forEach((v, i) => chart.data.datasets[i].data.push(v))
        if ((chart.data.labels?.length ?? 0) > MAX_POINTS) {
          chart.data.labels?.shift()
          chart.data.datasets.forEach((ds) => ds.data.shift())
        }
        chart.update("none")
      }

      const altLast = altRef.current.at(-1)
      const speedLast = speedRef.current.at(-1)
      if (altLast !== undefined && speedLast !== undefined) apply(altChart, [altLast, speedLast])

      const r = attRef.current.roll.at(-1)
      const p = attRef.current.pitch.at(-1)
      const y = attRef.current.yaw.at(-1)
      if (r !== undefined && p !== undefined && y !== undefined) apply(attChart, [r, p, y])

      const ax = accRef.current.x.at(-1)
      const ay = accRef.current.y.at(-1)
      const az = accRef.current.z.at(-1)
      if (ax !== undefined && ay !== undefined && az !== undefined) apply(accChart, [ax, ay, az])

      const f0 = finRef.current.fin0.at(-1)
      const f1 = finRef.current.fin1.at(-1)
      const f2 = finRef.current.fin2.at(-1)
      const f3 = finRef.current.fin3.at(-1)
      if (f0 !== undefined && f1 !== undefined && f2 !== undefined && f3 !== undefined) {
        apply(finChart, [f0, f1, f2, f3])
      }
    }, 120)

    return () => {
      window.clearInterval(interval)
      altChart.destroy()
      attChart.destroy()
      accChart.destroy()
      finChart.destroy()
    }
  }, [accRef, altRef, attRef, finRef, speedRef, timeRef])

  return (
    <div className="grid gap-4 p-4 md:grid-cols-2">
      <div className="h-60 rounded-lg border p-3"><canvas ref={altEl} className="h-full w-full" /></div>
      <div className="h-60 rounded-lg border p-3"><canvas ref={attEl} className="h-full w-full" /></div>
      <div className="h-60 rounded-lg border p-3"><canvas ref={accEl} className="h-full w-full" /></div>
      <div className="h-60 rounded-lg border p-3"><canvas ref={finEl} className="h-full w-full" /></div>
    </div>
  )
}
