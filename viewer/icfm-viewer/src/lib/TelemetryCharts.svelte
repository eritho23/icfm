<script lang="ts">
  import { onMount } from 'svelte'
  import { Chart, registerables } from 'chart.js'

  Chart.register(...registerables)

  let { liveState, timeState, altState, speedState, attState, accState, finState } = $props<{
    liveState: { value: boolean }
    timeState: { value: number }
    altState: { values: number[] }
    speedState: { values: number[] }
    attState: { roll: number[]; pitch: number[]; yaw: number[] }
    accState: { x: number[]; y: number[]; z: number[] }
    finState: { fin0: number[]; fin1: number[]; fin2: number[]; fin3: number[] }
  }>()

  const maxPoints = 200
  let altCanvas: HTMLCanvasElement | null = null
  let attCanvas: HTMLCanvasElement | null = null
  let accCanvas: HTMLCanvasElement | null = null
  let finCanvas: HTMLCanvasElement | null = null

  const createChart = (canvas: HTMLCanvasElement, datasets: { label: string; color: string }[]) =>
    new Chart(canvas, {
      type: 'line',
      data: {
        labels: [],
        datasets: datasets.map((dataset) => ({
          label: dataset.label,
          data: [],
          borderColor: dataset.color,
          backgroundColor: `${dataset.color}33`,
          borderWidth: 1.5,
          pointRadius: 0,
          tension: 0.1,
        })),
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { position: 'top', align: 'end', labels: { boxWidth: 12, padding: 4 } } },
        scales: {
          x: { grid: { color: 'rgba(128,128,128,0.2)' }, ticks: { maxTicksLimit: 8 } },
          y: { grid: { color: 'rgba(128,128,128,0.2)' }, ticks: { maxTicksLimit: 6 } },
        },
      },
    })

  onMount(() => {
    if (!altCanvas || !attCanvas || !accCanvas || !finCanvas) return

    Chart.defaults.font.family = 'Berlin Grotesk, ui-sans-serif, sans-serif'
    Chart.defaults.font.size = 11

    const altChart = createChart(altCanvas, [
      { label: 'alt', color: '#1f77b4' },
      { label: 'speed', color: '#2ca02c' },
    ])
    const attChart = createChart(attCanvas, [
      { label: 'roll', color: '#d62728' },
      { label: 'pitch', color: '#ff7f0e' },
      { label: 'yaw', color: '#9467bd' },
    ])
    const accChart = createChart(accCanvas, [
      { label: 'a_x', color: '#1f77b4' },
      { label: 'a_y', color: '#2ca02c' },
      { label: 'a_z', color: '#d62728' },
    ])
    const finChart = createChart(finCanvas, [
      { label: 'fin0', color: '#1f77b4' },
      { label: 'fin1', color: '#2ca02c' },
      { label: 'fin2', color: '#ff7f0e' },
      { label: 'fin3', color: '#d62728' },
    ])

    let lastTime = -1
    const interval = window.setInterval(() => {
      if (!liveState.value) return
      const t = timeState.value
      if (!Number.isFinite(t) || t === lastTime) return
      lastTime = t

      const apply = (chart: Chart, values: number[]) => {
        chart.data.labels?.push(t.toFixed(1))
        values.forEach((value, index) => chart.data.datasets[index].data.push(value))
        if ((chart.data.labels?.length ?? 0) > maxPoints) {
          chart.data.labels?.shift()
          const dataSets = chart.data.datasets as Array<{ data: unknown[] }>
          dataSets.forEach((dataset) => dataset.data.shift())
        }
        chart.update('none')
      }

      const altLast = altState.values.at(-1)
      const speedLast = speedState.values.at(-1)
      if (altLast !== undefined && speedLast !== undefined) apply(altChart, [altLast, speedLast])

      const roll = attState.roll.at(-1)
      const pitch = attState.pitch.at(-1)
      const yaw = attState.yaw.at(-1)
      if (roll !== undefined && pitch !== undefined && yaw !== undefined) apply(attChart, [roll, pitch, yaw])

      const ax = accState.x.at(-1)
      const ay = accState.y.at(-1)
      const az = accState.z.at(-1)
      if (ax !== undefined && ay !== undefined && az !== undefined) apply(accChart, [ax, ay, az])

      const f0 = finState.fin0.at(-1)
      const f1 = finState.fin1.at(-1)
      const f2 = finState.fin2.at(-1)
      const f3 = finState.fin3.at(-1)
      if (f0 !== undefined && f1 !== undefined && f2 !== undefined && f3 !== undefined) apply(finChart, [f0, f1, f2, f3])
    }, 120)

    return () => {
      window.clearInterval(interval)
      altChart.destroy()
      attChart.destroy()
      accChart.destroy()
      finChart.destroy()
    }
  })
</script>

<div class="grid gap-3 p-3 md:grid-cols-2">
  <div class="h-56 rounded-xl border border-slate-300 bg-white p-3"><canvas bind:this={altCanvas} class="h-full w-full"></canvas></div>
  <div class="h-56 rounded-xl border border-slate-300 bg-white p-3"><canvas bind:this={attCanvas} class="h-full w-full"></canvas></div>
  <div class="h-56 rounded-xl border border-slate-300 bg-white p-3"><canvas bind:this={accCanvas} class="h-full w-full"></canvas></div>
  <div class="h-56 rounded-xl border border-slate-300 bg-white p-3"><canvas bind:this={finCanvas} class="h-full w-full"></canvas></div>
</div>
