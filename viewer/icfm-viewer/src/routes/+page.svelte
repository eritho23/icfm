<script lang="ts">
	import { onDestroy, onMount, tick } from "svelte";
	import EyeClosed from "lucide-svelte/icons/eye-closed";
	import PlugZap from "lucide-svelte/icons/plug-zap";
	import RefreshCcw from "lucide-svelte/icons/refresh-ccw";
	import Snowflake from "lucide-svelte/icons/snowflake";
	import RocketCanvas from "$lib/RocketCanvas.svelte";
	import TelemetryCharts from "$lib/TelemetryCharts.svelte";

	type Metrics = Record<string, string>;

	type KalmanData = {
		t_s: number;
		p_x: number;
		p_y: number;
		p_z: number;
		v_x: number;
		v_y: number;
		v_z: number;
		a_x: number;
		a_y: number;
		a_z: number;
		wx: number;
		wy: number;
		wz: number;
		d_theta: number;
		d_alpha: number;
		d_beta: number;
		q_w: number;
		q_x: number;
		q_y: number;
		q_z: number;
	};

	type CtrlData = { fin0: number; fin1: number; fin2: number; fin3: number };

	type TelemetryMessage =
		| { type: "serial_raw"; line?: string }
		| { type: "command_ack"; ok?: boolean; cmd?: string; reason?: string }
		| { type: "kalman"; data: KalmanData }
		| { type: "ctrl"; data: CtrlData };

	const metricKeys = [
		"p_x",
		"p_y",
		"p_z",
		"v_x",
		"v_y",
		"v_z",
		"a_x",
		"a_y",
		"a_z",
		"wx",
		"wy",
		"wz",
		"d_theta",
		"d_alpha",
		"d_beta",
		"q_w",
		"q_x",
		"q_y",
		"q_z",
		"fin0",
		"fin1",
		"fin2",
		"fin3",
	] as const;

	const maxSamples = 1200;

	let tab = $state<"space" | "graphs" | "serial">("space");
	let wsState = $state<"connecting" | "connected" | "disconnected" | "error">(
		"connecting",
	);
	let commandStatus = $state("Awaiting command");
	let serialText = $state("");
	let metrics = $state<Metrics>(
		Object.fromEntries(metricKeys.map((key) => [key, "-"])),
	);
	let serialViewport = $state<HTMLDivElement | null>(null);
	let hideInterpreted = $state(false);

	const liveState = { value: false };
	const timeState = { value: 0 };
	const quaternionState = { w: 1, x: 0, y: 0, z: 0 };
	const finAnglesState = { values: [0, 0, 0, 0] };
	const altState = { values: [] as number[] };
	const speedState = { values: [] as number[] };
	const attState = {
		roll: [] as number[],
		pitch: [] as number[],
		yaw: [] as number[],
	};
	const accState = { x: [] as number[], y: [] as number[], z: [] as number[] };
	const finState = {
		fin0: [] as number[],
		fin1: [] as number[],
		fin2: [] as number[],
		fin3: [] as number[],
	};
	const frozenState = { value: false };

	let ws: WebSocket | null = null;

	function isInterpretedLine(line: string) {
		const t = line.trim();
		if (!t) return false;
		if (t.startsWith("{") && t.endsWith("}")) return true;
		return false;
	}

	const displaySerial = $derived.by(() => {
		const raw = serialText;
		if (!hideInterpreted) return raw;
		return raw
			.split("\n")
			.filter((line) => !isInterpretedLine(line))
			.join("\n");
	});

	function formatMissionTime(t: number) {
		if (!Number.isFinite(t) || t < 0) return "00:00:00";
		const total = Math.floor(t);
		const h = Math.floor(total / 3600);
		const m = Math.floor((total % 3600) / 60);
		const s = total % 60;
		return `${String(h).padStart(2, "0")}:${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`;
	}

	function pushLimited(values: number[], value: number) {
		values.push(value);
		if (values.length > maxSamples)
			values.splice(0, values.length - maxSamples);
	}

	function quatToEuler(w: number, x: number, y: number, z: number) {
		const n = Math.hypot(w, x, y, z) || 1;
		w /= n;
		x /= n;
		y /= n;
		z /= n;
		const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
		const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
		const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
		return [roll, pitch, yaw];
	}

	function sendCommand(cmd: "CALIBRATE" | "RESET") {
		if (!ws || ws.readyState !== WebSocket.OPEN) return;
		ws.send(JSON.stringify({ type: "command", cmd }));
	}

	function toggleFreeze() {
		frozenState.value = !frozenState.value;
	}

	onMount(() => {
		ws = new WebSocket("ws://localhost:8765");

		ws.onopen = () => {
			wsState = "connected";
			liveState.value = true;
		};

		ws.onclose = () => {
			wsState = "disconnected";
			liveState.value = false;
		};

		ws.onerror = () => {
			wsState = "error";
			liveState.value = false;
		};

		ws.onmessage = (event) => {
			const msg = JSON.parse(event.data) as TelemetryMessage;

			if (msg.type === "serial_raw") {
				if (frozenState.value) return;
				const line = String(msg.line ?? "");
				const next = serialText ? `${serialText}\n${line}` : line;
				const maxChars = 200_000;
				serialText =
					next.length > maxChars ? next.slice(next.length - maxChars) : next;
				return;
			}

			if (msg.type === "command_ack") {
				commandStatus = msg.ok
					? `Command ${msg.cmd ?? "unknown"} accepted`
					: `Command failed: ${msg.reason ?? "unknown_error"}`;
				return;
			}

			if (frozenState.value) return;

			if (msg.type === "kalman") {
				const d = msg.data;
				timeState.value = d.t_s;

				const nextMetrics = { ...metrics };
				for (const key of [
					"p_x",
					"p_y",
					"p_z",
					"v_x",
					"v_y",
					"v_z",
					"a_x",
					"a_y",
					"a_z",
					"wx",
					"wy",
					"wz",
					"d_theta",
					"d_alpha",
					"d_beta",
					"q_w",
					"q_x",
					"q_y",
					"q_z",
				] as const) {
					nextMetrics[key] = d[key].toFixed(2);
				}
				metrics = nextMetrics;

				const speed = Math.hypot(d.v_x, d.v_y, d.v_z);
				const [roll, pitch, yaw] = quatToEuler(d.q_w, d.q_x, d.q_y, d.q_z);
				pushLimited(altState.values, d.p_z);
				pushLimited(speedState.values, speed);
				pushLimited(attState.roll, roll);
				pushLimited(attState.pitch, pitch);
				pushLimited(attState.yaw, yaw);
				pushLimited(accState.x, d.a_x);
				pushLimited(accState.y, d.a_y);
				pushLimited(accState.z, d.a_z);

				quaternionState.w = d.q_w;
				quaternionState.x = d.q_x;
				quaternionState.y = d.q_y;
				quaternionState.z = d.q_z;
				return;
			}

			const d = msg.data;
			metrics = {
				...metrics,
				fin0: d.fin0.toFixed(1),
				fin1: d.fin1.toFixed(1),
				fin2: d.fin2.toFixed(1),
				fin3: d.fin3.toFixed(1),
			};
			finAnglesState.values = [d.fin0, d.fin1, d.fin2, d.fin3];
			pushLimited(finState.fin0, d.fin0);
			pushLimited(finState.fin1, d.fin1);
			pushLimited(finState.fin2, d.fin2);
			pushLimited(finState.fin3, d.fin3);
		};
	});

	onDestroy(() => {
		ws?.close();
	});

	$effect(() => {
		displaySerial;
		tab;
		if (tab !== "serial" || !serialViewport) return;
		tick().then(() => {
			if (!serialViewport) return;
			serialViewport.scrollTop = serialViewport.scrollHeight;
		});
	});
</script>

<div class="page">
	<header class="page-header">
		<img
			src="/assets/icfm-text-logo.svg"
			alt="ICFM"
			class="brand-logo"
			width="55"
			height="16"
		/>
		<nav class="top-nav" aria-label="Primary">
			<button
				type="button"
				class="nav-link"
				class:nav-link--active={tab === "space"}
				onclick={() => (tab = "space")}>Space</button
			>
			<button
				type="button"
				class="nav-link"
				class:nav-link--active={tab === "graphs"}
				onclick={() => (tab = "graphs")}>Graphs</button
			>
			<button
				type="button"
				class="nav-link"
				class:nav-link--active={tab === "serial"}
				onclick={() => (tab = "serial")}>Serial</button
			>
		</nav>
	</header>

	<main class="page-main">
		{#if tab === "space"}
			<section class="panel-3d">
				<RocketCanvas {finAnglesState} {quaternionState} />
				<div class="metrics-strip">
					<div class="metrics-scroll">
						{#each metricKeys as key}
							<div class="metric-pair">
								<span class="metric-key">{key}</span>
								<span class="metric-val">{metrics[key]}</span>
							</div>
						{/each}
					</div>
				</div>
			</section>
		{:else if tab === "graphs"}
			<section class="panel-graphs">
				<TelemetryCharts
					{liveState}
					{timeState}
					{altState}
					{speedState}
					{attState}
					{accState}
					{frozenState}
				/>
			</section>
		{:else}
			<section class="panel-serial" aria-label="Serial monitor">
				<div bind:this={serialViewport} class="serial-scroll font-roboto-mono">
					{#if displaySerial.length === 0}
						<span class="serial-placeholder">Waiting for serial data...</span>
					{:else}
						<pre class="serial-pre">{displaySerial}</pre>
					{/if}
				</div>
				<div class="serial-footer">
					<button
						type="button"
						class="serial-footer-btn"
						class:serial-footer-btn--active={hideInterpreted}
						onclick={() => (hideInterpreted = !hideInterpreted)}
					>
						<EyeClosed size={16} strokeWidth={1.5} class="icon-stroke" />
						<span>Hide interpreted</span>
					</button>
					<span class="serial-time"
						>Time: {formatMissionTime(timeState.value)}</span
					>
				</div>
			</section>
		{/if}
	</main>

	<footer class="page-footnote">
		<span>Testing app 2026.04.05</span>
		<span>Data served through icfm/scripts/logs_wired_websocket.py</span>
	</footer>

	<div class="bottom-dock">
		<div class="dock-cluster">
			<button
				type="button"
				class="dock-btn"
				onclick={() => sendCommand("CALIBRATE")}>Calibrate</button
			>
			<button
				type="button"
				class="dock-btn dock-btn--icon"
				onclick={() => sendCommand("RESET")}
			>
				<RefreshCcw size={16} strokeWidth={1.5} class="icon-stroke" />
				<span>Reset</span>
			</button>
			<button
				type="button"
				class="dock-btn dock-btn--icon"
				data-on={frozenState.value}
				onclick={toggleFreeze}
			>
				<Snowflake size={16} strokeWidth={1.5} class="icon-stroke" />
				<span>Freeze</span>
			</button>
		</div>
		<div class="dock-divider" aria-hidden="true"></div>
		<div class="dock-cluster dock-cluster--status">
			<span class="dock-status">
				<span class="dock-status-label">Online</span>
				<span class="status-dot" data-live={wsState === "connected"}></span>
			</span>
			<PlugZap size={18} strokeWidth={1.5} class="icon-stroke" />
		</div>
	</div>
</div>

<style>
	.page {
		min-height: 100dvh;
		display: flex;
		flex-direction: column;
		background: #f0efee;
		color: #0d0d0d;
		font-size: 14px;
		position: relative;
		padding-bottom: 120px;
	}

	.page-header {
		display: flex;
		flex-shrink: 0;
		align-items: flex-start;
		justify-content: space-between;
		padding: 32px 40px 24px;
	}

	.brand-logo {
		display: block;
		height: 16px;
		width: auto;
	}

	.top-nav {
		display: flex;
		align-items: center;
		gap: 32px;
	}

	.nav-link {
		background: none;
		border: none;
		padding: 0;
		cursor: pointer;
		font-family: inherit;
		font-size: 14px;
		font-weight: 400;
		color: #0d0d0d;
	}

	.nav-link--active {
		color: #7b7b7b;
	}

	.page-main {
		flex: 1;
		min-height: 0;
		display: flex;
		flex-direction: column;
		padding: 0 40px;
	}

	.panel-3d {
		position: relative;
		flex: 1;
		min-height: 480px;
		overflow: hidden;
		border-radius: 8px;
		background: #f0efee;
	}

	.metrics-strip {
		position: absolute;
		left: 12px;
		right: 12px;
		bottom: 12px;
		max-height: 40%;
		overflow: hidden;
		border-radius: 8px;
		border: 1px solid #474747;
		background: rgba(17, 17, 19, 0.92);
		padding: 8px 10px;
	}

	.metrics-scroll {
		display: flex;
		min-width: max-content;
		gap: 16px;
		overflow-x: auto;
		font-size: 12px;
	}

	.metric-pair {
		display: flex;
		align-items: baseline;
		gap: 6px;
		white-space: nowrap;
	}

	.metric-key {
		color: #7b7b7b;
		font-weight: 400;
	}

	.metric-val {
		font-family: "Roboto Mono", ui-monospace, monospace;
		font-size: 12px;
		color: #bbbac1;
	}

	.panel-graphs {
		flex: 1;
		min-height: 0;
		display: flex;
		flex-direction: column;
		overflow: auto;
		padding-bottom: 8px;
	}

	.panel-serial {
		flex: 1;
		min-height: 0;
		display: flex;
		flex-direction: column;
		border-radius: 8px;
		background: #111113;
		overflow: hidden;
	}

	.serial-scroll {
		flex: 1;
		min-height: 0;
		overflow: auto;
		padding: 16px 20px;
		font-size: 14px;
		line-height: 1.5;
		color: #bbbac1;
	}

	.serial-placeholder {
		color: #bbbac1;
		opacity: 0.65;
	}

	.serial-pre {
		margin: 0;
		white-space: pre-wrap;
		word-break: break-word;
	}

	.serial-footer {
		display: flex;
		align-items: center;
		justify-content: space-between;
		gap: 16px;
		padding: 12px 20px;
		border-top: 1px solid #272727;
		font-size: 14px;
		font-weight: 400;
		color: #7b7b7b;
	}

	.serial-footer-btn {
		display: inline-flex;
		align-items: center;
		gap: 4px;
		border: none;
		background: none;
		padding: 0;
		cursor: pointer;
		font: inherit;
		color: inherit;
	}

	.serial-footer-btn :global(.icon-stroke) {
		color: #7b7b7b;
		flex-shrink: 0;
	}

	.serial-footer-btn--active {
		color: #f0efee;
	}

	.serial-footer-btn--active :global(.icon-stroke) {
		color: #f0efee;
	}

	.serial-time {
		font-weight: 400;
	}

	.page-footnote {
		position: fixed;
		left: 40px;
		bottom: 20px;
		margin: 0;
		display: flex;
		flex-wrap: nowrap;
		align-items: center;
		gap: 16px;
		font-size: 12px;
		font-weight: 400;
		line-height: 1.35;
		color: #b1b1b1;
	}

	.bottom-dock {
		position: fixed;
		left: 50%;
		bottom: 56px;
		transform: translateX(-50%);
		z-index: 20;
		display: flex;
		align-items: stretch;
		gap: 0;
		padding: 10px 20px;
		border-radius: 8px;
		border: 1px solid #474747;
		background: #111113;
		box-shadow: 0 4px 16px rgba(0, 0, 0, 0.2);
		color: #bbbac1;
		font-weight: 500;
	}

	.dock-cluster {
		display: flex;
		align-items: center;
		gap: 20px;
	}

	.dock-cluster--status {
		gap: 16px;
		padding-left: 4px;
	}

	.dock-divider {
		width: 1px;
		align-self: stretch;
		margin: 0 20px;
		background: #474747;
		flex-shrink: 0;
	}

	.dock-btn {
		display: inline-flex;
		align-items: center;
		gap: 4px;
		border: none;
		background: none;
		padding: 0;
		cursor: pointer;
		font: inherit;
		font-weight: 500;
		font-size: 14px;
		color: #bbbac1;
	}

	.dock-btn--icon {
		gap: 4px;
	}

	.dock-btn[data-on="true"] {
		color: #e8e8ea;
	}

	.dock-btn :global(.icon-stroke) {
		color: currentColor;
		flex-shrink: 0;
	}

	.dock-status {
		display: inline-flex;
		align-items: center;
		gap: 8px;
	}

	.dock-status-label {
		font-weight: 500;
	}

	.status-dot {
		width: 8px;
		height: 8px;
		border-radius: 999px;
		background: #c45c5c;
		flex-shrink: 0;
	}

	.status-dot[data-live="true"] {
		background: #3ecf8e;
	}
</style>
