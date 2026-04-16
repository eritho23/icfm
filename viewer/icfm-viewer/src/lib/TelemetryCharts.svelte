<script lang="ts">
import { onMount, tick } from "svelte";
import { Chart, registerables } from "chart.js";

Chart.register(...registerables);

let {
	liveState,
	timeState,
	timeSeriesState,
	altState,
	speedState,
	attState,
	accState,
	frozenState,
} = $props<{
	liveState: { value: boolean };
	timeState: { value: number };
	timeSeriesState: { values: number[] };
	altState: { values: number[] };
	speedState: { values: number[] };
	attState: { roll: number[]; pitch: number[]; yaw: number[] };
	accState: { x: number[]; y: number[]; z: number[] };
	frozenState: { value: boolean };
}>();

const maxPoints = 200;

const panels = [
	{
		key: "alt",
		label: "Altitude",
		color: "#1f77b4",
		series: () => altState.values,
	},
	{
		key: "speed",
		label: "Speed",
		color: "#2ca02c",
		series: () => speedState.values,
	},
	{
		key: "roll",
		label: "Roll",
		color: "#d62728",
		series: () => attState.roll,
	},
	{
		key: "pitch",
		label: "Pitch",
		color: "#ff7f0e",
		series: () => attState.pitch,
	},
	{
		key: "yaw",
		label: "Yaw",
		color: "#9467bd",
		series: () => attState.yaw,
	},
	{
		key: "ax",
		label: "a_x",
		color: "#1f77b4",
		series: () => accState.x,
	},
	{
		key: "ay",
		label: "a_y",
		color: "#2ca02c",
		series: () => accState.y,
	},
	{
		key: "az",
		label: "a_z",
		color: "#d62728",
		series: () => accState.z,
	},
] as const;

let canvasEls = $state<(HTMLCanvasElement | null)[]>(
	Array.from({ length: 8 }, () => null),
);

function createChart(canvas: HTMLCanvasElement, label: string, color: string) {
	return new Chart(canvas, {
		type: "line",
		data: {
			labels: [] as string[],
			datasets: [
				{
					label,
					data: [] as number[],
					borderColor: color,
					backgroundColor: `${color}33`,
					borderWidth: 1.5,
					pointRadius: 0,
					tension: 0.1,
				},
			],
		},
		options: {
			animation: false,
			responsive: true,
			maintainAspectRatio: false,
			plugins: {
				legend: { display: false },
				title: {
					display: true,
					text: label,
					color: "#0d0d0d",
					font: {
						family: "Berlin Grotesk, sans-serif",
						size: 12,
						weight: 400,
					},
					padding: { bottom: 16 },
				},
			},
			scales: {
				x: {
					grid: { color: "rgba(13,13,13,0.08)" },
					ticks: {
						color: "#7b7b7b",
						maxTicksLimit: 6,
						font: { size: 10 },
					},
				},
				y: {
					grid: { color: "rgba(13,13,13,0.08)" },
					ticks: {
						color: "#7b7b7b",
						maxTicksLimit: 5,
						font: { size: 10 },
					},
				},
			},
		},
	});
}

onMount(() => {
	let charts: Chart[] = [];
	let intervalId = 0;
	let cancelled = false;
	let lastLen = 0;

	void tick().then(() => {
		if (cancelled) return;
		const els = canvasEls;
		if (els.some((c) => !c)) return;

		Chart.defaults.font.family = "Berlin Grotesk, ui-sans-serif, sans-serif";
		Chart.defaults.font.size = 11;

		charts = panels.map((p, i) => createChart(els[i]!, p.label, p.color));

		const syncAllCharts = () => {
			const allTimes = timeSeriesState.values;
			const start = Math.max(0, allTimes.length - maxPoints);
			const labels = allTimes.slice(start).map((t: number) => t.toFixed(1));
			panels.forEach((p, i) => {
				const values = p.series();
				const chart = charts[i];
				chart.data.labels = labels.slice();
				chart.data.datasets[0].data = values.slice(start) as number[];
				chart.update("none");
			});
			lastLen = allTimes.length;
		};

		syncAllCharts();

		intervalId = window.setInterval(() => {
			if (!liveState.value || frozenState.value) return;
			const len = timeSeriesState.values.length;
			if (len <= lastLen) return;
			syncAllCharts();
		}, 120);
	});

	return () => {
		cancelled = true;
		window.clearInterval(intervalId);
		charts.forEach((c) => c.destroy());
	};
});
</script>

<div class="grid grid-cols-1 gap-6 sm:grid-cols-2">
	{#each panels as p, i}
		<div
			class="flex min-h-[200px] min-w-0 flex-col rounded-lg bg-[#e7e6e4] px-14 py-10"
		>
			<canvas bind:this={canvasEls[i]} class="min-h-0 w-full flex-1"
			></canvas>
		</div>
	{/each}
</div>
