<script lang="ts">
import { onMount } from "svelte";
import * as THREE from "three";

let { finAnglesState, quaternionState } = $props<{
	finAnglesState: { values: number[] };
	quaternionState: { w: number; x: number; y: number; z: number };
}>();

let canvasEl: HTMLCanvasElement | null = null;

onMount(() => {
	const canvas = canvasEl;
	if (!canvas) return;
	const host = canvas.parentElement;
	if (!host) return;

	const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
	renderer.setClearColor(0xf0efee);
	renderer.setPixelRatio(window.devicePixelRatio);

	const scene = new THREE.Scene();
	const camera = new THREE.PerspectiveCamera(40, 2, 0.1, 100);
	scene.add(new THREE.AmbientLight(0xffffff, 1.35));

	const axisExtent = 25;
	const axisGeometry = new THREE.BufferGeometry();
	axisGeometry.setAttribute(
		"position",
		new THREE.Float32BufferAttribute(
			[
				-axisExtent,
				0,
				0,
				axisExtent,
				0,
				0,
				0,
				-axisExtent,
				0,
				0,
				axisExtent,
				0,
				0,
				0,
				-axisExtent,
				0,
				0,
				axisExtent,
			],
			3,
		),
	);
	axisGeometry.setAttribute(
		"color",
		new THREE.Float32BufferAttribute(
			[
				0.88, 0.4, 0.4, 0.88, 0.4, 0.4, 0.4, 0.78, 0.5, 0.4, 0.78, 0.5, 0.4,
				0.56, 0.88, 0.4, 0.56, 0.88,
			],
			3,
		),
	);
	scene.add(
		new THREE.LineSegments(
			axisGeometry,
			new THREE.LineBasicMaterial({ vertexColors: true }),
		),
	);

	const worldFrameGroup = new THREE.Group();
	worldFrameGroup.rotation.z = Math.PI / 2;
	scene.add(worldFrameGroup);

	const rocketGroup = new THREE.Group();
	worldFrameGroup.add(rocketGroup);
	const rocketModel = new THREE.Group();
	rocketModel.rotation.z = -Math.PI / 2;
	rocketGroup.add(rocketModel);

	rocketModel.add(
		new THREE.Mesh(
			new THREE.CylinderGeometry(0.17, 0.17, 2.5, 28),
			new THREE.MeshStandardMaterial({
				color: 0x3a3a3a,
				metalness: 0.35,
				roughness: 0.55,
			}),
		),
	);

	const noseProfile: THREE.Vector2[] = [];
	const noseHeight = 0.72;
	const noseRadius = 0.17;
	for (let i = 0; i <= 16; i += 1) {
		const t = i / 16;
		const y = noseHeight * t;
		const x = noseRadius * (1 - Math.pow(t, 1.55));
		noseProfile.push(new THREE.Vector2(Math.max(0.0001, x), y));
	}
	noseProfile.push(new THREE.Vector2(0.0001, noseHeight + 0.01));

	const nose = new THREE.Mesh(
		new THREE.LatheGeometry(noseProfile, 36),
		new THREE.MeshStandardMaterial({
			color: 0x555555,
			metalness: 0.25,
			roughness: 0.6,
		}),
	);
	nose.position.y = 1.25;
	rocketModel.add(nose);

	const finLabels = ["A", "B", "C", "D"];
	const finColor = 0x7b7b7b;
	const finMeshes: THREE.Mesh[] = [];
	const finScale = 3;

	const makeFinLabel = (label: string) => {
		const canvas = document.createElement("canvas");
		canvas.width = 64;
		canvas.height = 64;
		const ctx = canvas.getContext("2d");
		if (!ctx) return null;
		ctx.clearRect(0, 0, canvas.width, canvas.height);
		ctx.font = "600 34px Berlin Grotesk, sans-serif";
		ctx.fillStyle = "#2c2c2c";
		ctx.textAlign = "center";
		ctx.textBaseline = "middle";
		ctx.fillText(label, 32, 34);

		const texture = new THREE.CanvasTexture(canvas);
		texture.needsUpdate = true;
		texture.generateMipmaps = false;
		texture.minFilter = THREE.LinearFilter;
		texture.magFilter = THREE.LinearFilter;

		const frontMaterial = new THREE.MeshBasicMaterial({
			map: texture,
			transparent: true,
			side: THREE.FrontSide,
			alphaTest: 0.3,
			depthWrite: false,
			polygonOffset: true,
			polygonOffsetFactor: -1,
			polygonOffsetUnits: -1,
		});
		const backMaterial = frontMaterial.clone();

		const labelGroup = new THREE.Group();
		const frontLabel = new THREE.Mesh(
			new THREE.PlaneGeometry(0.16, 0.16),
			frontMaterial,
		);
		const backLabel = new THREE.Mesh(
			new THREE.PlaneGeometry(0.16, 0.16),
			backMaterial,
		);
		backLabel.rotation.y = Math.PI;
		frontLabel.position.z = 0.003;
		backLabel.position.z = -0.003;

		labelGroup.add(frontLabel);
		labelGroup.add(backLabel);
		return labelGroup;
	};

	for (let i = 0; i < 4; i += 1) {
		const orientGroup = new THREE.Group();
		orientGroup.rotation.y = (i * Math.PI) / 2;
		rocketModel.add(orientGroup);

		const hingeGroup = new THREE.Group();
		hingeGroup.position.set(0.17, -0.8, 0);
		orientGroup.add(hingeGroup);

		const shape = new THREE.Shape();
		shape.moveTo(0, 0);
		shape.lineTo(0.3, -0.12);
		shape.lineTo(0.3, -0.48);
		shape.lineTo(0, -0.36);
		shape.closePath();

		const finMesh = new THREE.Mesh(
			new THREE.ShapeGeometry(shape),
			new THREE.MeshStandardMaterial({
				color: finColor,
				side: THREE.DoubleSide,
				metalness: 0.2,
				roughness: 0.7,
			}),
		);
		const finLabel = makeFinLabel(finLabels[i]);
		if (finLabel) {
			finLabel.position.set(0.16, -0.24, 0);
			finMesh.add(finLabel);
		}
		finMeshes.push(finMesh);
		hingeGroup.add(finMesh);
	}

	let camRadius = 7;
	let thetaY = 0.4;
	let thetaX = 0.2;
	let drag = false;
	let lastX = 0;
	let lastY = 0;

	const onMouseDown = (event: MouseEvent) => {
		drag = true;
		lastX = event.clientX;
		lastY = event.clientY;
	};

	const onMouseUp = () => {
		drag = false;
	};

	const onMouseMove = (event: MouseEvent) => {
		if (!drag) return;
		thetaY -= (event.clientX - lastX) * 0.007;
		thetaX += (event.clientY - lastY) * 0.004;
		thetaX = Math.max(-0.5, Math.min(0.8, thetaX));
		lastX = event.clientX;
		lastY = event.clientY;
	};

	const onWheel = (event: WheelEvent) => {
		event.preventDefault();
		camRadius += event.deltaY * 0.01;
		camRadius = Math.max(2, Math.min(12, camRadius));
	};

	canvas.addEventListener("mousedown", onMouseDown);
	window.addEventListener("mouseup", onMouseUp);
	window.addEventListener("mousemove", onMouseMove);
	canvas.addEventListener("wheel", onWheel, { passive: false });

	const resize = () => {
		const rect = host.getBoundingClientRect();
		const width = Math.max(1, Math.floor(rect.width));
		const height = Math.max(1, Math.floor(rect.height));
		renderer.setSize(width, height, false);
		camera.aspect = width / height;
		camera.updateProjectionMatrix();
	};

	resize();
	const resizeObserver = new ResizeObserver(resize);
	resizeObserver.observe(host);

	let frameHandle = 0;
	const animate = () => {
		frameHandle = requestAnimationFrame(animate);

		const q = quaternionState;
		const norm = Math.hypot(q.w, q.x, q.y, q.z) || 1;
		rocketGroup.quaternion.set(q.x / norm, q.y / norm, q.z / norm, q.w / norm);

		for (let i = 0; i < 4; i += 1) {
			finMeshes[i].rotation.x =
				(-finAnglesState.values[i] * finScale * Math.PI) / 180;
		}

		camera.position.x = camRadius * Math.sin(thetaY) * Math.cos(thetaX);
		camera.position.y = camRadius * Math.sin(thetaX) + 0.5;
		camera.position.z = camRadius * Math.cos(thetaY) * Math.cos(thetaX);
		camera.lookAt(0, 0, 0);
		renderer.render(scene, camera);
	};

	animate();

	return () => {
		cancelAnimationFrame(frameHandle);
		canvas.removeEventListener("mousedown", onMouseDown);
		window.removeEventListener("mouseup", onMouseUp);
		window.removeEventListener("mousemove", onMouseMove);
		canvas.removeEventListener("wheel", onWheel);
		resizeObserver.disconnect();
		renderer.dispose();

		scene.traverse((obj: THREE.Object3D) => {
			if (obj instanceof THREE.Mesh) {
				obj.geometry.dispose();
				const material = obj.material;
				if (Array.isArray(material)) {
					material.forEach((item) => {
						item.map?.dispose();
						item.dispose();
					});
				} else {
					material.map?.dispose();
					material.dispose();
				}
			}
		});
	};
});
</script>

<div class="h-full w-full overflow-hidden bg-[#F0EFEE]">
  <canvas bind:this={canvasEl} class="block h-full w-full cursor-grab"></canvas>
</div>
