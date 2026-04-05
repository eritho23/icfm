<script lang="ts">
	import { onMount, tick } from 'svelte';
	import { Chart, registerables } from 'chart.js';

	Chart.register(...registerables);

	let {
		liveState,
		timeState,
		altState,
		speedState,
		attState,
		accState,
		frozenState,
	} = $props<{
		liveState: { value: boolean };
		timeState: { value: number };
		altState: { values: number[] };
		speedState: { values: number[] };
		attState: { roll: number[]; pitch: number[]; yaw: number[] };
		accState: { x: number[]; y: number[]; z: number[] };
		frozenState: { value: boolean };
	}>();

	const maxPoints = 200;

	const panels = [
		{ key: 'alt', label: 'Altitude', color: '#1f77b4', get: () => altState.values.at(-1) },
		{ key: 'speed', label: 'Speed', color: '#2ca02c', get: () => speedState.values.at(-1) },
		{ key: 'roll', label: 'Roll', color: '#d62728', get: () => attState.roll.at(-1) },
		{ key: 'pitch', label: 'Pitch', color: '#ff7f0e', get: () => attState.pitch.at(-1) },
		{ key: 'yaw', label: 'Yaw', color: '#9467bd', get: () => attState.yaw.at(-1) },
		{ key: 'ax', label: 'a_x', color: '#1f77b4', get: () => accState.x.at(-1) },
		{ key: 'ay', label: 'a_y', color: '#2ca02c', get: () => accState.y.at(-1) },
		{ key: 'az', label: 'a_z', color: '#d62728', get: () => accState.z.at(-1) },
	] as const;

	let canvasEls = $state<(HTMLCanvasElement | null)[]>(Array.from({ length: 8 }, () => null));

	function createChart(canvas: HTMLCanvasElement, label: string, color: string) {
		return new Chart(canvas, {
			type: 'line',
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
						color: '#0d0d0d',
						font: { family: 'Berlin Grotesk, sans-serif', size: 12, weight: 500 },
						padding: { bottom: 8 },
					},
				},
				scales: {
					x: {
						grid: { color: 'rgba(13,13,13,0.08)' },
						ticks: { color: '#7b7b7b', maxTicksLimit: 6, font: { size: 10 } },
					},
					y: {
						grid: { color: 'rgba(13,13,13,0.08)' },
						ticks: { color: '#7b7b7b', maxTicksLimit: 5, font: { size: 10 } },
					},
				},
			},
		});
	}

	onMount(() => {
		let charts: Chart[] = [];
		let intervalId = 0;
		let cancelled = false;

		void tick().then(() => {
			if (cancelled) return;
			const els = canvasEls;
			if (els.some((c) => !c)) return;

			Chart.defaults.font.family = 'Berlin Grotesk, ui-sans-serif, sans-serif';
			Chart.defaults.font.size = 11;

			charts = panels.map((p, i) => createChart(els[i]!, p.label, p.color));

			let lastTime = -1;
			intervalId = window.setInterval(() => {
				if (!liveState.value || frozenState.value) return;
				const t = timeState.value;
				if (!Number.isFinite(t) || t === lastTime) return;
				lastTime = t;

				panels.forEach((p, i) => {
					const v = p.get();
					if (v === undefined) return;
					const chart = charts[i];
					chart.data.labels?.push(t.toFixed(1));
					chart.data.datasets[0].data.push(v);
					if ((chart.data.labels?.length ?? 0) > maxPoints) {
						chart.data.labels?.shift();
						const ds = chart.data.datasets[0].data as number[];
						ds.shift();
					}
					chart.update('none');
				});
			}, 120);
		});

		return () => {
			cancelled = true;
			window.clearInterval(intervalId);
			charts.forEach((c) => c.destroy());
		};
	});
</script>

<div class="telemetry-grid">
	{#each panels as p, i}
		<div class="chart-card">
			<canvas bind:this={canvasEls[i]} class="chart-canvas"></canvas>
		</div>
	{/each}
</div>

<style>
	.telemetry-grid {
		display: grid;
		grid-template-columns: repeat(2, minmax(0, 1fr));
		grid-template-rows: repeat(4, minmax(200px, 1fr));
		gap: 16px;
		min-height: 0;
		flex: 1;
	}

	.chart-card {
		min-height: 200px;
		border-radius: 8px;
		background: #e5e3e0;
		padding: 12px;
		display: flex;
		flex-direction: column;
		min-width: 0;
	}

	.chart-canvas {
		flex: 1;
		min-height: 0;
		width: 100%;
	}
</style>
