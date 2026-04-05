<script lang="ts">
  import { onDestroy, onMount, tick } from 'svelte'
  import RocketCanvas from '$lib/RocketCanvas.svelte'
  import TelemetryCharts from '$lib/TelemetryCharts.svelte'

  type Metrics = Record<string, string>

  type KalmanData = {
    t_s: number
    p_x: number
    p_y: number
    p_z: number
    v_x: number
    v_y: number
    v_z: number
    a_x: number
    a_y: number
    a_z: number
    wx: number
    wy: number
    wz: number
    d_theta: number
    d_alpha: number
    d_beta: number
    q_w: number
    q_x: number
    q_y: number
    q_z: number
  }

  type CtrlData = { fin0: number; fin1: number; fin2: number; fin3: number }

  type TelemetryMessage =
    | { type: 'serial_raw'; line?: string }
    | { type: 'command_ack'; ok?: boolean; cmd?: string; reason?: string }
    | { type: 'kalman'; data: KalmanData }
    | { type: 'ctrl'; data: CtrlData }

  const metricKeys = [
    'p_x', 'p_y', 'p_z',
    'v_x', 'v_y', 'v_z',
    'a_x', 'a_y', 'a_z',
    'wx', 'wy', 'wz',
    'd_theta', 'd_alpha', 'd_beta',
    'q_w', 'q_x', 'q_y', 'q_z',
    'fin0', 'fin1', 'fin2', 'fin3',
  ] as const

  const maxSamples = 1200

  let tab = $state<'space' | 'graphs' | 'serial'>('space')
  let wsState = $state<'connecting' | 'connected' | 'disconnected' | 'error'>('connecting')
  let commandStatus = $state('Awaiting command')
  let serialText = $state('')
  let metrics = $state<Metrics>(Object.fromEntries(metricKeys.map((key) => [key, '-'])))
  let serialViewport = $state<HTMLDivElement | null>(null)

  const liveState = { value: false }
  const timeState = { value: 0 }
  const quaternionState = { w: 1, x: 0, y: 0, z: 0 }
  const finAnglesState = { values: [0, 0, 0, 0] }
  const altState = { values: [] as number[] }
  const speedState = { values: [] as number[] }
  const attState = { roll: [] as number[], pitch: [] as number[], yaw: [] as number[] }
  const accState = { x: [] as number[], y: [] as number[], z: [] as number[] }
  const finState = { fin0: [] as number[], fin1: [] as number[], fin2: [] as number[], fin3: [] as number[] }

  let ws: WebSocket | null = null

  function pushLimited(values: number[], value: number) {
    values.push(value)
    if (values.length > maxSamples) values.splice(0, values.length - maxSamples)
  }

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

  function sendCommand(cmd: 'CALIBRATE' | 'RESET') {
    if (!ws || ws.readyState !== WebSocket.OPEN) return
    ws.send(JSON.stringify({ type: 'command', cmd }))
  }

  onMount(() => {
    ws = new WebSocket('ws://localhost:8765')

    ws.onopen = () => {
      wsState = 'connected'
      liveState.value = true
    }

    ws.onclose = () => {
      wsState = 'disconnected'
      liveState.value = false
    }

    ws.onerror = () => {
      wsState = 'error'
      liveState.value = false
    }

    ws.onmessage = (event) => {
      const msg = JSON.parse(event.data) as TelemetryMessage

      if (msg.type === 'serial_raw') {
        const line = String(msg.line ?? '')
        const next = serialText ? `${serialText}\n${line}` : line
        const maxChars = 200_000
        serialText = next.length > maxChars ? next.slice(next.length - maxChars) : next
        return
      }

      if (msg.type === 'command_ack') {
        commandStatus = msg.ok
          ? `Command ${msg.cmd ?? 'unknown'} accepted`
          : `Command failed: ${msg.reason ?? 'unknown_error'}`
        return
      }

      if (msg.type === 'kalman') {
        const d = msg.data
        timeState.value = d.t_s

        const nextMetrics = { ...metrics }
        for (const key of [
          'p_x', 'p_y', 'p_z',
          'v_x', 'v_y', 'v_z',
          'a_x', 'a_y', 'a_z',
          'wx', 'wy', 'wz',
          'd_theta', 'd_alpha', 'd_beta',
          'q_w', 'q_x', 'q_y', 'q_z',
        ] as const) {
          nextMetrics[key] = d[key].toFixed(2)
        }
        metrics = nextMetrics

        const speed = Math.hypot(d.v_x, d.v_y, d.v_z)
        const [roll, pitch, yaw] = quatToEuler(d.q_w, d.q_x, d.q_y, d.q_z)
        pushLimited(altState.values, d.p_z)
        pushLimited(speedState.values, speed)
        pushLimited(attState.roll, roll)
        pushLimited(attState.pitch, pitch)
        pushLimited(attState.yaw, yaw)
        pushLimited(accState.x, d.a_x)
        pushLimited(accState.y, d.a_y)
        pushLimited(accState.z, d.a_z)

        quaternionState.w = d.q_w
        quaternionState.x = d.q_x
        quaternionState.y = d.q_y
        quaternionState.z = d.q_z
        return
      }

      const d = msg.data
      metrics = {
        ...metrics,
        fin0: d.fin0.toFixed(1),
        fin1: d.fin1.toFixed(1),
        fin2: d.fin2.toFixed(1),
        fin3: d.fin3.toFixed(1),
      }
      finAnglesState.values = [d.fin0, d.fin1, d.fin2, d.fin3]
      pushLimited(finState.fin0, d.fin0)
      pushLimited(finState.fin1, d.fin1)
      pushLimited(finState.fin2, d.fin2)
      pushLimited(finState.fin3, d.fin3)
    }
  })

  onDestroy(() => {
    ws?.close()
  })

  $effect(() => {
    serialText
    tab
    if (tab !== 'serial' || !serialViewport) return
    tick().then(() => {
      if (!serialViewport) return
      serialViewport.scrollTop = serialViewport.scrollHeight
    })
  })
</script>

<div class="h-dvh p-3 md:p-4">
  <div class="grid h-full min-h-0 grid-cols-1 overflow-hidden rounded-2xl border border-slate-300/80 bg-white/90 shadow-lg backdrop-blur md:grid-cols-[300px_1fr]">
    <aside class="flex min-h-0 flex-col gap-4 border-b border-slate-300/80 bg-slate-100/95 p-4 md:border-b-0 md:border-r">
      <div class="flex items-center justify-between">
        <div class="flex items-center gap-2.5">
          <img src="/assets/icfm-logo-2.svg" alt="ICFM logo" class="h-8 w-8 rounded-md" />
          <div class="leading-none">
            <p class="text-sm font-bold tracking-wide">ICFM</p>
            <p class="text-xs text-slate-600">Svelte Viewer</p>
          </div>
        </div>
        <span class={`h-2.5 w-2.5 rounded-full ${wsState === 'connected' ? 'bg-emerald-500' : 'bg-rose-500'}`}></span>
      </div>

      <div class="grid grid-cols-2 gap-2">
        <button type="button" class="rounded-lg bg-[#2f7cb2] px-3 py-2 text-sm font-semibold text-white transition hover:brightness-105" onclick={() => sendCommand('CALIBRATE')}>Calibrate</button>
        <button type="button" class="rounded-lg bg-slate-200 px-3 py-2 text-sm font-semibold text-slate-800 transition hover:bg-slate-300" onclick={() => sendCommand('RESET')}>Reset</button>
      </div>
      <p class="text-xs text-slate-600">{commandStatus}</p>

      <nav class="mt-1 flex flex-col gap-1.5">
        <button type="button" class={`flex items-center justify-between rounded-lg px-3 py-2 text-left text-sm transition ${tab === 'space' ? 'bg-white shadow-sm ring-1 ring-slate-300' : 'hover:bg-white/60'}`} onclick={() => (tab = 'space')}>
          <span>3D Space View</span>
          <span class="text-slate-500">&gt;</span>
        </button>
        <button type="button" class={`flex items-center justify-between rounded-lg px-3 py-2 text-left text-sm transition ${tab === 'graphs' ? 'bg-white shadow-sm ring-1 ring-slate-300' : 'hover:bg-white/60'}`} onclick={() => (tab = 'graphs')}>
          <span>Graphs</span>
          <span class="text-slate-500">&gt;</span>
        </button>
        <button type="button" class={`flex items-center justify-between rounded-lg px-3 py-2 text-left text-sm transition ${tab === 'serial' ? 'bg-white shadow-sm ring-1 ring-slate-300' : 'hover:bg-white/60'}`} onclick={() => (tab = 'serial')}>
          <span>Serial Monitor</span>
          <span class="text-slate-500">&gt;</span>
        </button>
      </nav>
    </aside>

    <main class="min-h-0 min-w-0">
      {#if tab === 'space'}
        <section class="relative h-full w-full">
          <RocketCanvas {finAnglesState} {quaternionState} />
          <div class="absolute inset-x-2 bottom-2 overflow-x-auto rounded-xl border border-slate-300 bg-white/90 px-3 py-2 backdrop-blur">
            <div class="flex min-w-max items-center gap-4 text-xs">
              {#each metricKeys as key}
                <div class="flex items-center gap-1.5">
                  <span class="text-slate-500">{key}</span>
                  <span class="font-mono text-slate-900">{metrics[key]}</span>
                </div>
              {/each}
            </div>
          </div>
        </section>
      {:else if tab === 'graphs'}
        <section class="h-full w-full overflow-auto">
          <TelemetryCharts {liveState} {timeState} {altState} {speedState} {attState} {accState} {finState} />
        </section>
      {:else}
        <section class="h-full p-4">
          <div bind:this={serialViewport} class="h-full overflow-auto rounded-xl border border-slate-900/80 bg-slate-950 p-3 font-mono text-xs leading-relaxed text-slate-100">
            {#if serialText.length === 0}
              <span class="text-slate-500">Waiting for serial data...</span>
            {:else}
              <pre class="whitespace-pre">{serialText}</pre>
            {/if}
          </div>
        </section>
      {/if}
    </main>
  </div>
</div>
