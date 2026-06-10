// rapierDebugRenderer.js
// Draws Rapier collider wireframes into a Three.js scene.
// Usage:
//   import { RapierDebugRenderer } from './rapierDebugRenderer.js';
//   const dbg = new RapierDebugRenderer(threeScene, rapierWorld);
//   // each frame:
//   dbg.update();
//   // to hide:
//   dbg.setEnabled(false);

import * as THREE from 'three';

export class RapierDebugRenderer {
  constructor(scene, world) {
    this.scene   = scene;
    this.world   = world;
    this.enabled = true;

    this.mesh = new THREE.LineSegments(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: 0x00ffff, vertexColors: true })
    );
    this.mesh.frustumCulled = false;
    this.mesh.renderOrder   = 999;
    scene.add(this.mesh);
  }

  update() {
    if (!this.enabled || !this.world) {
      this.mesh.visible = false;
      return;
    }
    this.mesh.visible = true;

    const { vertices, colors } = this.world.debugRender();
    this.mesh.geometry.setAttribute('position',
      new THREE.BufferAttribute(vertices, 3)
    );
    this.mesh.geometry.setAttribute('color',
      new THREE.BufferAttribute(colors, 4)
    );
  }

  setEnabled(val) {
    this.enabled      = val;
    this.mesh.visible = val;
  }

  dispose() {
    this.scene.remove(this.mesh);
    this.mesh.geometry.dispose();
    this.mesh.material.dispose();
  }
}
