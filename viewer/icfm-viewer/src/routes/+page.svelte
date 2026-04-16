<script lang="ts">
import { onDestroy, onMount, tick } from "svelte";
import { linear } from "svelte/easing";
import { fly } from "svelte/transition";
import type { TransitionConfig } from "svelte/transition";
import EyeClosed from "lucide-svelte/icons/eye-closed";
import ChevronsLeftRight from "lucide-svelte/icons/chevrons-left-right";
import ChevronsRightLeft from "lucide-svelte/icons/chevrons-right-left";
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
type Tab = "space" | "graphs" | "serial";
type WsState = "connecting" | "connected" | "disconnected" | "error";

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
const serialMaxChars = 200_000;
const kalmanColsLen = 20;
const ctrlColsLen = 9;
const websocketUrl = "ws://localhost:8765";
const navTabs: Array<{ id: Tab; label: string }> = [
	{ id: "space", label: "Space" },
	{ id: "graphs", label: "Graphs" },
	{ id: "serial", label: "Serial" },
];
const kalmanMetricKeys = [
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
] as const;

const transitionInDuration = 120;
const transitionOutDuration = 20;
const swipeDistance = 30;

// Out-transition: immediately lifts the departing element out of normal
// flow so it doesn't push or resize the incoming tab, then flies it away.
function flyOut(node: HTMLElement, params: { x: number }): TransitionConfig {
	node.style.position = "absolute";
	node.style.top = "0";
	node.style.left = "0";
	node.style.right = "0";
	node.style.pointerEvents = "none";
	return fly(node, {
		x: params.x,
		opacity: 0,
		duration: transitionOutDuration,
		easing: linear,
	});
}

let tab = $state<Tab>("space");
let tabDirection = $state<1 | -1>(1);
let wsState = $state<WsState>("connecting");
let commandStatus = $state("Awaiting command");
let serialText = $state("");
let missionTime = $state(0);
let metrics = $state<Metrics>(
	Object.fromEntries(metricKeys.map((key) => [key, "-"])),
);
let serialViewport = $state<HTMLDivElement | null>(null);
let hideInterpreted = $state(false);
let autoScrollEnabled = $state(true);
let metricsExpanded = $state(false);

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
const timeSeriesState = { values: [] as number[] };
const accState = {
	x: [] as number[],
	y: [] as number[],
	z: [] as number[],
};
const finState = {
	fin0: [] as number[],
	fin1: [] as number[],
	fin2: [] as number[],
	fin3: [] as number[],
};
let frozenState = $state({ value: false });

let ws: WebSocket | null = null;

function setConnectionState(state: WsState) {
	wsState = state;
	liveState.value = state === "connected";
}

function appendSerialLine(line: string) {
	const next = serialText ? `${serialText}\n${line}` : line;
	serialText =
		next.length > serialMaxChars
			? next.slice(next.length - serialMaxChars)
			: next;
}

function updateKalmanMetrics(data: KalmanData) {
	const nextMetrics = { ...metrics };
	for (const key of kalmanMetricKeys) {
		nextMetrics[key] = data[key].toFixed(2);
	}
	metrics = nextMetrics;
}

function handleKalmanData(data: KalmanData) {
	missionTime = data.t_s;
	timeState.value = data.t_s;
	updateKalmanMetrics(data);

	const speed = Math.hypot(data.v_x, data.v_y, data.v_z);
	const [roll, pitch, yaw] = quatToEuler(
		data.q_w,
		data.q_x,
		data.q_y,
		data.q_z,
	);

	pushLimited(timeSeriesState.values, data.t_s);
	pushLimited(altState.values, data.p_z);
	pushLimited(speedState.values, speed);
	pushLimited(attState.roll, roll);
	pushLimited(attState.pitch, pitch);
	pushLimited(attState.yaw, yaw);
	pushLimited(accState.x, data.a_x);
	pushLimited(accState.y, data.a_y);
	pushLimited(accState.z, data.a_z);

	quaternionState.w = data.q_w;
	quaternionState.x = data.q_x;
	quaternionState.y = data.q_y;
	quaternionState.z = data.q_z;
}

function handleControlData(data: CtrlData) {
	metrics = {
		...metrics,
		fin0: data.fin0.toFixed(1),
		fin1: data.fin1.toFixed(1),
		fin2: data.fin2.toFixed(1),
		fin3: data.fin3.toFixed(1),
	};
	finAnglesState.values = [data.fin0, data.fin1, data.fin2, data.fin3];
	pushLimited(finState.fin0, data.fin0);
	pushLimited(finState.fin1, data.fin1);
	pushLimited(finState.fin2, data.fin2);
	pushLimited(finState.fin3, data.fin3);
}

function handleTelemetryMessage(msg: TelemetryMessage) {
	if (frozenState.value) return;

	if (msg.type === "serial_raw") {
		appendSerialLine(String(msg.line ?? ""));
		return;
	}

	if (msg.type === "command_ack") {
		commandStatus = msg.ok
			? `Command ${msg.cmd ?? "unknown"} accepted`
			: `Command failed: ${msg.reason ?? "unknown_error"}`;
		return;
	}

	if (msg.type === "kalman") {
		handleKalmanData(msg.data);
		return;
	}

	handleControlData(msg.data);
}

function connectWebSocket() {
	ws?.close();
	setConnectionState("connecting");

	const socket = new WebSocket(websocketUrl);
	ws = socket;

	socket.onopen = () => {
		if (ws !== socket) return;
		setConnectionState("connected");
	};

	socket.onclose = () => {
		if (ws !== socket) return;
		setConnectionState("disconnected");
	};

	socket.onerror = () => {
		if (ws !== socket) return;
		setConnectionState("error");
	};

	socket.onmessage = (event) => {
		if (ws !== socket) return;
		const msg = JSON.parse(event.data) as TelemetryMessage;
		handleTelemetryMessage(msg);
	};
}

function isTelemetryCsvLine(line: string) {
	const t = line.trim();
	if (!t) return false;
	const parts = t.split(",");
	if (parts.length !== kalmanColsLen && parts.length !== ctrlColsLen)
		return false;
	return parts.every((part) => Number.isFinite(Number(part)));
}

const displaySerial = $derived.by(() => {
	const raw = serialText;
	if (!hideInterpreted) return raw;
	return raw
		.split("\n")
		.filter((line) => !isTelemetryCsvLine(line))
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
	if (values.length > maxSamples) values.splice(0, values.length - maxSamples);
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

function selectTab(nextTab: Tab) {
	if (nextTab === tab) return;
	const currentIndex = navTabs.findIndex((t) => t.id === tab);
	const nextIndex = navTabs.findIndex((t) => t.id === nextTab);
	tabDirection = nextIndex > currentIndex ? 1 : -1;
	tab = nextTab;
}

function scrollSerialToBottom() {
	if (tab !== "serial" || !serialViewport || !autoScrollEnabled) return;
	tick().then(() => {
		if (!serialViewport) return;
		serialViewport.scrollTop = serialViewport.scrollHeight;
	});
}

function handleSerialScroll() {
	if (!serialViewport) return;
	const distanceFromBottom =
		serialViewport.scrollHeight -
		serialViewport.scrollTop -
		serialViewport.clientHeight;
	autoScrollEnabled = distanceFromBottom <= 8;
}

function resumeAutoScroll() {
	autoScrollEnabled = true;
	scrollSerialToBottom();
}

function toggleFreeze() {
	frozenState.value = !frozenState.value;
	if (!frozenState.value) {
		scrollSerialToBottom();
	}
}

const dockButtonClass =
	"inline-flex h-8 cursor-pointer items-center gap-1 rounded-[4px] border-0 bg-transparent px-3 text-sm font-medium transition-colors hover:bg-[#272727]";

onMount(() => {
	connectWebSocket();
});

onDestroy(() => {
	ws?.close();
});

$effect(() => {
	displaySerial;
	tab;
	frozenState.value;
	autoScrollEnabled;
	if (frozenState.value) return;
	scrollSerialToBottom();
});
</script>

{#snippet tabContent(t: Tab)}
	{#if t === "space"}
		<section
			class="relative min-h-[calc(100dvh-12rem)] flex-1 overflow-hidden rounded-lg bg-[#f0efee]"
		>
			<RocketCanvas {finAnglesState} {quaternionState} />
			<div
				class="absolute right-4 bottom-14 left-4 flex flex-col items-center gap-2 text-xs"
			>
				{#if metricsExpanded}
					<ul
						class="font-roboto-mono flex max-w-full items-baseline gap-4 overflow-x-auto whitespace-nowrap py-2 text-xs"
					>
						{#each metricKeys as key}
							<li
								class="flex items-baseline gap-1.5 whitespace-nowrap"
							>
								<span class="font-normal text-[#7b7b7b]"
									>{key}</span
								>
								<span
									class="font-roboto-mono text-xs text-[#0d0d0d]"
									>{metrics[key]}</span
								>
							</li>
						{/each}
					</ul>
				{/if}
				<button
					type="button"
					class="inline-flex cursor-pointer items-center gap-1 border-0 bg-transparent p-0 text-xs text-[#7b7b7b]"
					onclick={() => (metricsExpanded = !metricsExpanded)}
				>
					<span>Metrics</span>
					{#if metricsExpanded}
						<ChevronsRightLeft
							size={16}
							strokeWidth={2}
							class="shrink-0"
						/>
					{:else}
						<ChevronsLeftRight
							size={16}
							strokeWidth={2}
							class="shrink-0"
						/>
					{/if}
				</button>
			</div>
		</section>
	{:else if t === "graphs"}
		<section class="relative flex flex-col overflow-visible pt-4 pb-2">
			<TelemetryCharts
				{liveState}
				{timeState}
				{timeSeriesState}
				{altState}
				{speedState}
				{attState}
				{accState}
				{frozenState}
			/>
		</section>
	{:else}
		<div class="pt-2">
			<section
				class="flex h-[820px] w-full flex-col overflow-hidden rounded-lg bg-[#111113]"
				aria-label="Serial monitor"
			>
				<div
					bind:this={serialViewport}
					onscroll={handleSerialScroll}
					onwheel={() => (autoScrollEnabled = false)}
					class="font-roboto-mono min-h-0 flex-1 overflow-auto px-5 py-4 text-sm leading-6 text-[#bbbac1]"
				>
					{#if displaySerial.length === 0}
						<span class="text-[#bbbac1] opacity-65"
							>Waiting for serial data...</span
						>
					{:else}
						<pre
							class="m-0 min-w-max whitespace-pre">{displaySerial}</pre>
					{/if}
				</div>
				<div
					class="flex items-center justify-between gap-4 border-t border-[#272727] px-5 py-3 text-sm text-[#7b7b7b]"
				>
					<div class="flex items-center gap-4">
						<button
							type="button"
							class={`inline-flex cursor-pointer items-center gap-1 border-0 bg-transparent p-0 ${hideInterpreted ? "text-[#f0efee]" : "text-[#7b7b7b]"}`}
							onclick={() => (hideInterpreted = !hideInterpreted)}
						>
							<EyeClosed
								size={16}
								strokeWidth={1.5}
								class="shrink-0"
							/>
							<span>Hide interpreted</span>
						</button>
						{#if !autoScrollEnabled}
							<button
								type="button"
								class="cursor-pointer border-0 bg-transparent p-0 text-[#f0efee]"
								onclick={resumeAutoScroll}
							>
								Resume auto-scroll
							</button>
						{/if}
					</div>
					<span>Time: {formatMissionTime(missionTime)}</span>
				</div>
			</section>
		</div>
	{/if}
{/snippet}

<div
	class="relative flex min-h-dvh flex-col bg-[#f0efee] px-4 pb-44 text-sm text-[#0d0d0d] sm:px-10 sm:pb-36"
>
	<header class="fixed top-0 right-0 left-0 z-30 bg-[#f0efee] px-4 sm:px-10">
		<div
			class="mx-auto flex w-full max-w-[600px] shrink-0 justify-between pt-8 pb-4"
		>
			<img
				src="/assets/icfm-text-logo.svg"
				alt="ICFM"
				class="block h-4 w-auto"
				width="55"
				height="16"
			/>
			<nav class="flex items-center gap-8" aria-label="Primary">
				{#each navTabs as navTab}
					<button
						type="button"
						class={`link cursor-pointer border-0 bg-transparent p-0 text-sm font-normal ${tab === navTab.id ? "text-[#7b7b7b]" : "text-[#0d0d0d]"}`}
						onclick={() => selectTab(navTab.id)}
						>{navTab.label}</button
					>
				{/each}
			</nav>
		</div>
	</header>

	<main class="flex min-h-0 flex-1 flex-col pt-20">
		<div
			class="relative min-h-0 flex-1 overflow-x-hidden overflow-y-visible"
		>
			{#key tab}
				<div
					in:fly|global={{
						x: tabDirection * swipeDistance,
						duration: transitionInDuration,
						opacity: 0,
						easing: linear,
					}}
					out:flyOut|global={{ x: tabDirection * -swipeDistance }}
					class="relative flex flex-col"
				>
					{@render tabContent(tab)}
				</div>
			{/key}
		</div>
	</main>

	{#if tab === "graphs"}
		<div
			aria-hidden="true"
			class="pointer-events-none fixed right-0 bottom-0 left-0 h-10 bg-gradient-to-t from-[#f0efee] via-[#f0efee]/85 to-transparent"
		></div>
	{/if}

	<footer
		class="fixed bottom-4 left-4 m-0 flex flex-col items-start gap-1 text-[10px] leading-[1.35] text-[#b1b1b1] sm:left-10 sm:flex-row sm:items-center sm:gap-4"
	>
		<span>Testing app 2026.04.05</span>
		<span>Data served through icfm/scripts/logs_wired_websocket.py</span>
	</footer>

	<div
		class="fixed bottom-14 left-1/2 z-20 flex w-[calc(100%-2rem)] max-w-[560px] -translate-x-1/2 items-center justify-between rounded-lg border border-[#474747] bg-[#111113] px-2 py-2 text-[#bbbac1] shadow-[0_4px_16px_rgba(0,0,0,0.2)] sm:w-auto sm:min-w-[460px]"
	>
		<div class="flex items-center gap-2">
			<button
				type="button"
				class={`${dockButtonClass} text-[#bbbac1]`}
				onclick={() => sendCommand("CALIBRATE")}>Calibrate</button
			>
			<button
				type="button"
				class={`${dockButtonClass} text-[#bbbac1]`}
				onclick={() => sendCommand("RESET")}
			>
				<RefreshCcw size={16} strokeWidth={1.5} class="shrink-0" />
				<span>Reset</span>
			</button>
			<button
				type="button"
				class={`${dockButtonClass} ${frozenState.value ? "text-[#e8e8ea]" : "text-[#bbbac1]"}`}
				onclick={toggleFreeze}
			>
				<Snowflake size={16} strokeWidth={1.5} class="shrink-0" />
				<span>{frozenState.value ? "Unfreeze" : "Freeze"}</span>
			</button>
		</div>
		<div
			class="mx-8 h-6 w-px shrink-0 bg-[#474747]"
			aria-hidden="true"
		></div>
		<div class="flex items-center gap-5 pl-1">
			<span class="inline-flex items-center gap-2">
				<span class="text-sm font-medium"
					>{wsState === "connected" ? "Online" : "Offline"}</span
				>
				<span
					class={`h-2 w-2 shrink-0 rounded-full ${wsState === "connected" ? "bg-[#3ecf8e]" : "bg-[#c45c5c]"}`}
				></span>
			</span>
			<button
				type="button"
				class="inline-flex h-8 w-8 cursor-pointer items-center justify-center rounded-[4px] border-0 bg-[#272727] text-[#bbbac1] transition-colors hover:bg-[#343434]"
				aria-label="Reconnect websocket"
				onclick={connectWebSocket}
			>
				<PlugZap size={18} strokeWidth={1.5} class="shrink-0" />
			</button>
		</div>
	</div>
</div>
