import * as THREE from 'three';
window.THREE = THREE;
import { GUI } from '../node_modules/three/examples/jsm/libs/lil-gui.module.min.js';
import { PLYLoader } from '../node_modules/three/examples/jsm/loaders/PLYLoader.js';
import { GLTFLoader } from '../node_modules/three/examples/jsm/loaders/GLTFLoader.js';
import { OrbitControls } from '../node_modules/three/examples/jsm/controls/OrbitControls.js';
import { generateSimpleSteerTargets } from './scutlGait.js';
import { ScutlGUI } from './scutlGUI.js';          // ← new
// import { DragStateManager } from './utils/DragStateManager.js';
import * as GaussianSplats3D from '@mkkellogg/gaussian-splats-3d';
import {
  setupGUI,
  downloadExampleScenesFolder,
  loadSceneFromURL,
  drawTendonsAndFlex,
  getPosition,
  getQuaternion,
  toMujocoPos,
  standardNormal
} from './mujocoUtils.js';
import load_mujoco from '../node_modules/mujoco-js/dist/mujoco_wasm.js';

function eulerDegToQuat(xDeg, yDeg, zDeg) {
  const euler = new THREE.Euler(
    THREE.MathUtils.degToRad(xDeg),
    THREE.MathUtils.degToRad(yDeg),
    THREE.MathUtils.degToRad(zDeg),
    "XYZ"
  );

  const q = new THREE.Quaternion();
  q.setFromEuler(euler);

  return [q.x, q.y, q.z, q.w];
}


const mujoco = await load_mujoco();

var initialScene = "scutl.xml";
var truckScene = "scutl_coacd.xml";
var gardenScene = "scutl_garden_coacd.xml";
var constructionScene = "scutl_construction_coacd.xml";
var gcrRoomScene = "scutl_gcr_room.xml";
var stairsScene = "scutl_stairs_wide.xml";
var scutl3seg = "scutl_3seg.xml";
var scutl4seg = "scutl_4seg.xml";
var scutl5seg = "scutl_5seg.xml";
var scutl6seg = "scutl.xml";
var stairs3seg = "scutl_stairs_wide_3seg.xml";
var stairs4seg = "scutl_stairs_wide_4seg.xml";
var stairs5seg = "scutl_stairs_wide_5seg.xml";
// stairs 6seg = existing stairsScene

var truck3seg = "scutl_coacd_3seg.xml";
var truck4seg = "scutl_coacd_4seg.xml";
var truck5seg = "scutl_coacd_5seg.xml";

var garden3seg = "scutl_garden_coacd_3seg.xml";
var garden4seg = "scutl_garden_coacd_4seg.xml";
var garden5seg = "scutl_garden_coacd_5seg.xml";

mujoco.FS.mkdir('/working');
mujoco.FS.mount(mujoco.MEMFS, { root: '.' }, '/working');


// mujoco.FS.writeFile(
//   "/working/" + initialScene,
//   await (await fetch("./assets/scenes/" + initialScene)).text()
// );

const initialXmlText = await (await fetch("./assets/scenes/" + initialScene)).text();
mujoco.FS.writeFile("/working/" + initialScene, initialXmlText);

// Pre-load STL assets for initial scene
const fileMatches = [...initialXmlText.matchAll(/file="([^"]+)"/g)];
const assetFiles = [...new Set(fileMatches.map(m => m[1]))];
for (const file of assetFiles) {
  const buffer = await (await fetch("./assets/scenes/" + file)).arrayBuffer();
  mujoco.FS.writeFile("/working/" + file, new Uint8Array(buffer));
}

export class MuJoCoDemo {
  constructor() {
    this.mujoco = mujoco;

    this.model = mujoco.MjModel.loadFromXML("/working/" + initialScene);
    this.data  = new mujoco.MjData(this.model);

    this.params = {
      scene:          initialScene,
      paused:         false,
      help:           false,
      ctrlnoiserate:  0.0,
      ctrlnoisestd:   0.0,
      keyframeNumber: 0
    };

    // Replaced this.scutl + lil-gui folder with ScutlGUI
    this.scutlGUI             = new ScutlGUI();
    this.scutlLastLegRaw      = null;
    this.scutlUnwrappedLegRaw = null;
    this.scutlHoldCtrl = null;
    this.scutlLastPublishedLeg = null;

    this.mujoco_time = 0.0;
    this.bodies      = {};
    this.lights      = {};
    this.tmpVec      = new THREE.Vector3();
    this.tmpQuat     = new THREE.Quaternion();
    this.updateGUICallbacks = [];

    this.container = document.createElement('div');
    document.body.appendChild(this.container);

    this.scene = new THREE.Scene();
    this.scene.name = 'scene';

    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.001, 100);
    this.camera.name = 'PerspectiveCamera';
    // this.camera.position.set(2.0, 1.7, 1.7);
    this.camera.position.set(
    //   2.84175187593427,
    //   -0.55127776086459,
    //   0.18539126483749335
    // );
      0.23546240317712652,
      -0.31107733023432327,
      -2.9085427420402246 
      // -1.0595671463750809,
      // -7.651798074177322,
      // 9.014369073954851
    );

    this.scene.add(this.camera);

    this.scene.background = new THREE.Color(0.15, 0.25, 0.35);
    this.scene.fog         = new THREE.Fog(this.scene.background, 15, 25.5);

    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.1 * 3.14);
    this.ambientLight.name = 'AmbientLight';
    this.scene.add(this.ambientLight);

    this.spotlight = new THREE.SpotLight();
    this.spotlight.angle                  = 1.11;
    this.spotlight.distance               = 10000;
    this.spotlight.penumbra               = 0.5;
    this.spotlight.castShadow             = true;
    this.spotlight.intensity              = this.spotlight.intensity * 3.14 * 10.0;
    this.spotlight.shadow.mapSize.width   = 1024;
    this.spotlight.shadow.mapSize.height  = 1024;
    this.spotlight.shadow.camera.near     = 0.1;
    this.spotlight.shadow.camera.far      = 100;
    this.spotlight.position.set(0, 3, 3);

    const targetObject = new THREE.Object3D();
    this.scene.add(targetObject);
    this.spotlight.target = targetObject;
    targetObject.position.set(0, 1, 0);
    this.scene.add(this.spotlight);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(1.0);
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled      = true;
    this.renderer.shadowMap.type         = THREE.PCFSoftShadowMap;
    THREE.ColorManagement.enabled        = false;
    this.renderer.outputColorSpace       = THREE.LinearSRGBColorSpace;
    this.renderer.useLegacyLights        = true;

    this.renderer.setAnimationLoop(this.render.bind(this));
    this.container.appendChild(this.renderer.domElement);
    this.splatViewer = null;
    this.currentEnvironment = "none";
    this.currentSegments = 6;
    this.environmentSelect = null;

    this.loadingOverlay = document.createElement("div");

    Object.assign(this.loadingOverlay.style, {
      position: "fixed",
      top: "0",
      left: "0",
      width: "100vw",
      height: "100vh",
      display: "none",
      alignItems: "center",
      justifyContent: "center",
      background: "rgba(0,0,0,0.5)",
      color: "white",
      fontSize: "32px",
      fontWeight: "bold",
      zIndex: "99999"
    });

    this.loadingOverlay.innerHTML = `
      <div>
        <div id="loading-text">Loading scene...</div>
        <div id="loading-percent" style="font-size:20px; margin-top:10px;">0%</div>
      </div>
    `;
    document.body.appendChild(this.loadingOverlay);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    // this.controls.target.set(0, 0.7, 0);
    this.controls.target.set(
    //   0.5890702571687148,
    //   -1.3809601489295469,
    //   0.5151907276595901
    // );

      0.3113039252833211,
      -1.7759607844025347,
      0.1813860823267129
      // -0.8311778625249248,
      // -3.775220663868011,
      // -1.5888947833850986
    );

    this.controls.panSpeed        = 2;
    this.controls.zoomSpeed       = 1;
    this.controls.enableDamping   = true;
    this.controls.dampingFactor   = 0.10;
    this.controls.screenSpacePanning = true;
    this.controls.update();

    window.addEventListener('resize', this.onWindowResize.bind(this));

    // this.dragStateManager = new DragStateManager(
    //   this.scene,
    //   this.renderer,
    //   this.camera,
    //   this.container.parentElement,
    //   this.controls
    // );
  }

  async init() {
    await downloadExampleScenesFolder(mujoco);
  


    [this.model, this.data, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, initialScene, this);

    this.resetRobotControlState();
    mujoco.mj_forward(this.model, this.data);

    this.groundVisuals = [];

    this.scene.traverse(obj => {
      if (obj.type === "Reflector") {
        this.groundVisuals.push(obj);
        console.log("Ground visual found:", obj);
      }
    });

    // this.hideGroundVisuals();
    this.groundVisuals?.forEach(obj => {
      obj.visible = true;
    });

    // this.groundVisual = null;

    // this.scene.traverse(obj => {
    //   if (obj.type === "Reflector") {
    //     this.groundVisual = obj;
    //     console.log("Found ground visual:", obj);
    //   }
    // });

    // // ── lil-gui for camera / noise / keyframe controls ─────────────────────
    // this.gui = new GUI();
    // setupGUI(this);

    // ── SCUTL web control panel ────────────────────────────────────────────
    // Creates a floating overlay in the top-right corner.
    // Swap the containerEl for any element you prefer.
    const panelEl = document.createElement('div');
    Object.assign(panelEl.style, {
      position:   'fixed',
      top:        '10px',
      right:      '10px',
      zIndex:     '1000',
      maxHeight:  '96vh',
      overflowY:  'auto',
      width:      '360px',
      pointerEvents: 'auto',
    });
    document.body.appendChild(panelEl);
    this.scutlGUI.mount(panelEl);
    requestAnimationFrame(() => {
      const selects = panelEl.querySelectorAll("select");

      for (const select of selects) {
        const editOption = [...select.options].find(opt =>
          opt.textContent.toLowerCase().includes("edit") ||
          opt.value.toLowerCase().includes("edit")
        );

        if (editOption) {
          select.value = editOption.value;
          select.dispatchEvent(new Event("change", { bubbles: true }));
          break;
        }
      }
    });
    // Environment dropdown
    const envBox = document.createElement("div");
    Object.assign(envBox.style, {
      position: "fixed",
      top: "0.25vw",
      right: "7vw",
      zIndex: "1001",
      background: "#161a24",
      color: "#e2e8f0",
      border: "1px solid #252a36",
      borderRadius: "5px",
      padding: "6px",
      fontFamily: "Courier New, monospace",
      fontSize: "12px",
    });

    envBox.innerHTML = `
      <label style="margin-right:6px;">Environment</label>
      <select id="env-select"
              style="
                background:#0f1117;
                color:#e2e8f0;
                border:1px solid #252a36;
                border-radius:4px;
                padding:4px;">
        <option value="none">None</option>
        <option value="playground">Playground</option>
        <option value="truck">Truck</option>
        <option value="garden">Garden</option>
      </select>
    `;

    document.body.appendChild(envBox);

    this.environmentSelect = envBox.querySelector("#env-select");

    // this.environmentSelect.addEventListener("change", async e => {
    //   this.showLoadingScene();
    //   await this.setEnvironment(e.target.value);
    // });


    this.environmentSelect.addEventListener("change", async e => {
      const n = this.currentSegments;
      const stairMap  = { 3: stairs3seg, 4: stairs4seg, 5: stairs5seg, 6: "scutl_stairs_wide.xml" };
      const truckMap  = { 3: truck3seg,  4: truck4seg,  5: truck5seg,  6: "scutl_coacd.xml" };
      const gardenMap = { 3: garden3seg, 4: garden4seg, 5: garden5seg, 6: "scutl_garden_coacd.xml" };
      stairsScene = stairMap[n];
      truckScene  = truckMap[n];
      gardenScene = gardenMap[n];
      this.showLoadingScene();
      await this.setEnvironment(e.target.value);
    });

    // Camera dropdown
    const camBox = document.createElement("div");
    Object.assign(camBox.style, {
      position: "fixed",
      top: "0.25vw",
      right: "36vw",
      zIndex: "1001",
      background: "#161a24",
      color: "#e2e8f0",
      border: "1px solid #252a36",
      borderRadius: "5px",
      padding: "6px",
      fontFamily: "Courier New, monospace",
      fontSize: "12px",
    });

    camBox.innerHTML = `
      <label style="margin-right:6px;">Camera</label>
      <select id="cam-select"
              style="
                background:#0f1117;
                color:#e2e8f0;
                border:1px solid #252a36;
                border-radius:4px;
                padding:4px;">
        <option value="third">Third Person</option>
        <option value="first">First Person</option>
      </select>
    `;
    document.body.appendChild(camBox);

    this.cameraMode = "third";
    this.fpYaw = 0;
    this.fpPitch = 0;
    this.fpDragging = false;
    this.fpLastX = 0;
    this.fpLastY = 0;
    document.getElementById("cam-select").addEventListener("change", e => {
      this.cameraMode = e.target.value;
      if (this.cameraMode === "third") {
        this.controls.enabled = true;
        // this.resetCameraToRobotStart();
        if (this.currentEnvironment === "playground") {
          this.resetCameraToPlaygroundStart();
        } else {
          this.resetCameraToRobotStart();
        }
      } else {
        this.controls.enabled = false;
      }
    });

    // Segment count dropdown
    const segBox = document.createElement("div");
    Object.assign(segBox.style, {
      position: "fixed",
      top: "0.25vw",
      right: "55vw",
      zIndex: "1001",
      background: "#161a24",
      color: "#e2e8f0",
      border: "1px solid #252a36",
      borderRadius: "5px",
      padding: "6px",
      fontFamily: "Courier New, monospace",
      fontSize: "12px",
    });
    segBox.innerHTML = `
      <label style="margin-right:6px;">Body Segments</label>
      <select id="seg-select"
              style="background:#0f1117;color:#e2e8f0;border:1px solid #252a36;border-radius:4px;padding:4px;">
        <option value="6">6</option>
        <option value="5">5</option>
        <option value="4">4</option>
        <option value="3">3</option>
      </select>
    `;
    document.body.appendChild(segBox);

    // document.getElementById("seg-select").addEventListener("change", async e => {
    //   const n = e.target.value;
    //   const segScenes = { "3": scutl3seg, "4": scutl4seg, "5": scutl5seg, "6": scutl6seg };
    //   this.showLoadingScene();
    //   try {
    //     // Reload current environment's scene but swap robot XML
    //     // For "none" just reload the seg XML directly
    //     if (this.currentEnvironment === "none") {
    //       initialScene = segScenes[n];
    //       await this.loadMujocoScene(initialScene);
    //       this.groundVisuals?.forEach(obj => { obj.visible = true; });
    //       this.resetCameraToRobotStart();
    //     } else {
    //       // Store the choice and re-trigger current environment
    //       initialScene = segScenes[n];
    //       await this.setEnvironment(this.currentEnvironment);
    //     }
    //   } finally {
    //     this.hideLoadingScene();
    //   }
    // });


    document.getElementById("seg-select").addEventListener("change", async e => {
      const n = parseInt(e.target.value);
      this.currentSegments = n;

      const noneMap   = { 3: scutl3seg,  4: scutl4seg,  5: scutl5seg,  6: scutl6seg };
      const stairMap  = { 3: stairs3seg, 4: stairs4seg, 5: stairs5seg, 6: "scutl_stairs_wide.xml" };
      const truckMap  = { 3: truck3seg,  4: truck4seg,  5: truck5seg,  6: "scutl_coacd.xml" };
      const gardenMap = { 3: garden3seg, 4: garden4seg, 5: garden5seg, 6: "scutl_garden_coacd.xml" };

      initialScene = noneMap[n];
      stairsScene  = stairMap[n];
      truckScene   = truckMap[n];
      gardenScene  = gardenMap[n];

      this.showLoadingScene();
      try {
        await this.setEnvironment(this.currentEnvironment);
      } finally {
        this.hideLoadingScene();
      }
    });


    this.renderer.domElement.addEventListener("mousedown", e => {
      if (this.cameraMode === "first" && e.button === 0) {
        this.fpDragging = true;
        this.fpLastX = e.clientX;
        this.fpLastY = e.clientY;
      }
    });

    this.renderer.domElement.addEventListener("mousemove", e => {
      if (this.cameraMode === "first" && this.fpDragging) {
        this.fpYaw   -= (e.clientX - this.fpLastX) * 0.003;
        this.fpPitch -= (e.clientY - this.fpLastY) * 0.003;
        this.fpPitch  = Math.max(-Math.PI/2, Math.min(Math.PI/2, this.fpPitch));
        this.fpLastX = e.clientX;
        this.fpLastY = e.clientY;
      }
    });

    this.renderer.domElement.addEventListener("mouseup", () => {
      this.fpDragging = false;
    });

    this.renderer.domElement.addEventListener("touchstart", e => {
      if (this.cameraMode === "first" && e.touches.length === 1) {
        this.fpDragging = true;
        this.fpLastX = e.touches[0].clientX;
        this.fpLastY = e.touches[0].clientY;
        e.preventDefault();
      }
    }, { passive: false });

    this.renderer.domElement.addEventListener("touchmove", e => {
      if (this.cameraMode === "first" && this.fpDragging && e.touches.length === 1) {
        this.fpYaw   -= (e.touches[0].clientX - this.fpLastX) * 0.003;
        this.fpPitch -= (e.touches[0].clientY - this.fpLastY) * 0.003;
        this.fpPitch  = Math.max(-Math.PI/2, Math.min(Math.PI/2, this.fpPitch));
        this.fpLastX = e.touches[0].clientX;
        this.fpLastY = e.touches[0].clientY;
        e.preventDefault();
      }
    }, { passive: false });

    this.renderer.domElement.addEventListener("touchend", () => {
      this.fpDragging = false;
    });

  }

  async loadMujocoScene(xmlName) {
    console.log("Loading MuJoCo XML:", xmlName);

    const xmlText = await (await fetch("./assets/scenes/" + xmlName)).text();

    mujoco.FS.writeFile("/working/" + xmlName, xmlText);

    const fileMatches = [...xmlText.matchAll(/file="([^"]+)"/g)];
    const assetFiles = [...new Set(fileMatches.map(m => m[1]))];

    for (let i = 0; i < assetFiles.length; i++) {
      const file = assetFiles[i];

      const url = "./assets/scenes/" + file;
      const path = "/working/" + file;

      const dir = path.substring(0, path.lastIndexOf("/"));
      if (dir && dir !== "/working") {
        try {
          mujoco.FS.mkdirTree(dir);
        } catch (e) {}
      }

      const response = await fetch(url);
      if (!response.ok) {
        throw new Error("Could not load asset: " + url);
      }

      const buffer = await response.arrayBuffer();
      mujoco.FS.writeFile(path, new Uint8Array(buffer));
      this.setLoadingProgress(
        10 + (i + 1) / assetFiles.length * 45,
        `Loading MuJoCo assets... ${i + 1}/${assetFiles.length}`
      );
          }

    // Remove old MuJoCo visual tree before loading new one
    if (this.mujocoRoot) {
      this.scene.remove(this.mujocoRoot);
      this.mujocoRoot.traverse(obj => {
        if (obj.geometry) obj.geometry.dispose();
        if (obj.material) {
          if (Array.isArray(obj.material)) {
            obj.material.forEach(m => m.dispose());
          } else {
            obj.material.dispose();
          }
        }
      });
      this.mujocoRoot = null;
    }

    [this.model, this.data, this.bodies, this.lights] =
      await loadSceneFromURL(mujoco, xmlName, this);

    this.resetRobotControlState();
    mujoco.mj_forward(this.model, this.data);

    this.groundVisuals = [];

    this.scene.traverse(obj => {
      if (obj.type === "Reflector") {
        this.groundVisuals.push(obj);
      }
    });

    this.params.scene = xmlName;
    this.mujoco_time = 0.0;
  }



  
  hideGroundVisuals() {
    this.groundVisuals?.forEach(obj => {
      obj.visible = false;
    });

    this.scene.traverse(obj => {
      const n = (obj.name || "").toLowerCase();
      if (
        obj.type === "Reflector" ||
        n.includes("ground") ||
        n.includes("plane")
      ) {
        obj.visible = false;
      }
    });
  }

  hideTruckMujocoVisuals() {
    this.scene.traverse(obj => {
      const n = (obj.name || "").toLowerCase();

      if (
        n.includes("truck_visual") ||
        n.includes("truck_collision") ||
        n.includes("truck_col")
      ) {
        obj.visible = false;
      }
    });
  }

  hideGardenMujocoVisuals() {
    this.scene.traverse(obj => {
      const n = (obj.name || "").toLowerCase();

      if (
        n.includes("garden_visual") ||
        n.includes("garden_collision") ||
        n.includes("garden_col")
      ) {
        obj.visible = false;
      }
    });
  }

  hideConstructionMujocoVisuals() {
    this.scene.traverse(obj => {
      const n = (obj.name || "").toLowerCase();

      if (
        n.includes("construction_visual") ||
        n.includes("construction_collision") ||
        n.includes("construction_col")
      ) {
        obj.visible = false;
      }
    });
  }

  resetRobotControlState() {
    this.scutlLastLegRaw = null;
    this.scutlUnwrappedLegRaw = null;
    this.scutlHoldCtrl = null;
    this.scutlLastPublishedLeg = null;
    this.scutlSmoothParams = null;

    if (this.data?.ctrl) {
      for (let i = 0; i < this.data.ctrl.length; i++) {
        this.data.ctrl[i] = 0.0;
      }
    }
  }

  showLoadingScene() {
    this.loadingOverlay.style.display = "flex";
    this.setLoadingProgress(0);
  }

  setLoadingProgress(percent, text = "Loading scene...") {
    const p = Math.max(0, Math.min(100, Math.round(percent)));

    const textEl = document.getElementById("loading-text");
    const percentEl = document.getElementById("loading-percent");

    if (textEl) textEl.textContent = text;
    if (percentEl) percentEl.textContent = `${p}%`;
  }


  hideLoadingScene() {
    this.loadingOverlay.style.display = "none";
  }

  // async setEnvironment(name) {
  //   this.currentEnvironment = name;

  async loadGCRRoomVisual() {
    if (this.gcrRoomVisual) {
      this.scene.remove(this.gcrRoomVisual);
      this.gcrRoomVisual = null;
    }

    const loader = new PLYLoader();

    const geometry = await loader.loadAsync("/assets/scenes/gcr_room_visual.ply");
    geometry.computeVertexNormals();

    const material = new THREE.MeshStandardMaterial({
      color: 0xffffff,
      side: THREE.DoubleSide,
    });

    const mesh = new THREE.Mesh(geometry, material);
    mesh.name = "gcr_room_visual";

    mesh.position.set(0, 0, 0);
    mesh.rotation.set(0, 0, 0);
    mesh.scale.set(0.2, 0.2, 0.2);

    this.scene.add(mesh);
    this.gcrRoomVisual = mesh;
  }

  async loadGCRRoomVisual() {
    if (this.gcrRoomVisual) {
      this.scene.remove(this.gcrRoomVisual);
      this.gcrRoomVisual.traverse(obj => {
        if (obj.geometry) obj.geometry.dispose();
        if (obj.material) {
          const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
          mats.forEach(m => m.dispose());
        }
      });
      this.gcrRoomVisual = null;
    }

    const loader = new GLTFLoader();
    const gltf = await loader.loadAsync("/assets/scenes/gcr_room.glb");

    const room = gltf.scene;
    room.name = "gcr_room_visual";

    room.position.set(-5.2, -2.5, 0);
    room.rotation.set(0, 0, 0);
    room.scale.set(3,3,3);

    this.scene.add(room);
    this.gcrRoomVisual = room;
  }

  resetCameraToRobotStart() {
    this.camera.position.set(
      0.23546240317712652,
      -0.31107733023432327,
      -2.9085427420402246
    );
    this.controls.target.set(
      0.3113039252833211,
      -1.7759607844025347,
      0.1813860823267129
    );
    this.controls.update();
  }


  // resetCameraToGCRRoomStart() {
  //   this.camera.position.set(-5.265242698884813, 1.948546018115253, 9.437849386290221);
  //   this.controls.target.set(-5.731220068999387, 0.3122420747918781, 6.428204714677335);
  //   this.controls.update();
  // }

  resetCameraToPlaygroundStart() {
    this.camera.position.set(
      -7.740499620005834,
      -0.7495773868024169,
      -0.3987669970866019
    );
    this.controls.target.set(
      -4.01529496122985,
      -2.0291980716383042,
      -0.32287181896955625
    );
    this.controls.update();
  }

  async setEnvironment(name) {
    this.showLoadingScene();

    try {

      this.currentEnvironment = name;

  

    if (this.splatViewer) {
      this.scene.remove(this.splatViewer);

      try {
        await this.splatViewer.dispose();
      } catch (e) {
        console.warn("Could not dispose splat viewer:", e);
      }

      this.splatViewer = null;
    }
    if (this.gcrRoomVisual) {
      this.scene.remove(this.gcrRoomVisual);

      this.gcrRoomVisual.traverse(obj => {
        if (obj.geometry) obj.geometry.dispose();

        if (obj.material) {
          const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
          mats.forEach(m => m.dispose());
        }
      });

      this.gcrRoomVisual = null;
    }


    if (name === "none") {
      await this.loadMujocoScene(initialScene);

      this.groundVisuals?.forEach(obj => {
        obj.visible = true;
      });

      console.log("Environment: none");
      this.resetCameraToRobotStart();
      return;
    }

    // if (name === "garden") {
    //   await this.loadMujocoScene(initialScene);
    //   this.splatViewer = new GaussianSplats3D.DropInViewer({
    //     gpuAcceleratedSort: false,
    //     sharedMemoryForWorkers: false,
    //   });
    //   this.setLoadingProgress(60, "Loading garden splat...");

    //   this.scene.add(this.splatViewer);

    //   await this.splatViewer.addSplatScene("/assets/splats/test.ksplat", {
    //     splatAlphaRemovalThreshold: 1,
    //     showLoadingUI: false,
    //     position: [1, 1.05, 0],
    //     rotation: eulerDegToQuat(150, 0, 0),
    //     scale: [1, 1, 1],
    //   });
    //   this.setLoadingProgress(95, "Finalizing garden scene...");
    //   console.log("Environment: garden loaded");

    //   this.hideGroundVisuals();
    //   this.hideGardenMujocoVisuals();
    //   if (this.groundVisual) {
    //   }
    // }

    if (name === "garden") {
      await this.loadMujocoScene(gardenScene);

      this.splatViewer = new GaussianSplats3D.DropInViewer({
        gpuAcceleratedSort: false,
        sharedMemoryForWorkers: false,
      });

      this.scene.add(this.splatViewer);

      this.setLoadingProgress(60, "Loading test splat...");

      await this.splatViewer.addSplatScene(
        "/assets/splats/garden_c2_a150.ksplat",
        {
          splatAlphaRemovalThreshold: 1,
          showLoadingUI: false,

          position: [1, 1.05, 0],
          rotation: eulerDegToQuat(150, 0, 0),
          scale: [1, 1, 1],
        }
      );

      this.setLoadingProgress(95, "Finalizing garden scene...");

      console.log("Environment: garden loaded using truck splat test");

      this.hideGroundVisuals();
      this.hideGardenMujocoVisuals();
      this.resetCameraToRobotStart();
    }



    // if (name === "construction_site") {
    //   await this.loadMujocoScene(constructionScene);
    //   this.scene.traverse(obj => {
    //     const n = (obj.name || "").toLowerCase();

    //     if (n.includes("construction_col")) {
    //       obj.visible = true;

    //       if (obj.material) {
    //         const mats = Array.isArray(obj.material) ? obj.material : [obj.material];

    //         for (const mat of mats) {
    //           mat.color?.set?.(0xff0000);
    //           mat.transparent = true;
    //           mat.opacity = 0.8;
    //           mat.depthWrite = false;
    //           mat.needsUpdate = true;
    //         }
    //       }
    //     }
    //   });

    //   this.splatViewer = new GaussianSplats3D.DropInViewer({
    //     gpuAcceleratedSort: false,
    //     sharedMemoryForWorkers: false,
    //   });

    //   this.scene.add(this.splatViewer);

    //   this.setLoadingProgress(60, "Loading construction splat...");

    //   await this.splatViewer.addSplatScene(
    //     "/assets/splats/construction_c2_a150.ksplat",
    //     {
    //       splatAlphaRemovalThreshold: 1,
    //       showLoadingUI: false,
    //       position: [3, -2.15, 0],
    //       rotation: eulerDegToQuat(180, 0, 0),
    //       scale: [1, 1, 1],
    //     }
    //   );

    //   this.setLoadingProgress(95, "Finalizing construction scene...");

    //   console.log("Environment: construction loaded");

    //   this.hideGroundVisuals();
    //   // this.hideConstructionMujocoVisuals();
    // }


    // if (name === "gcr_room") {
    //   await this.loadMujocoScene(gcrRoomScene);
    //   await this.loadGCRRoomVisual();

    //   console.log("Environment: GCR room loaded");

    //   // this.hideGroundVisuals();
    //   this.resetCameraToGCRRoomStart();
    // }


    if (name === "playground") {
      await this.loadMujocoScene(stairsScene);
      // this.hideGroundVisuals();
      this.resetCameraToPlaygroundStart();
      console.log("Environment: playground loaded");
    }

    if (name === "truck") {
      await this.loadMujocoScene(truckScene);
      this.splatViewer = new GaussianSplats3D.DropInViewer({
        gpuAcceleratedSort: false,
        sharedMemoryForWorkers: false,
      });

      this.scene.add(this.splatViewer);
      this.setLoadingProgress(60, "Loading truck splat...");
      await this.splatViewer.addSplatScene(
        "/assets/splats/truck.ksplat",
        {
          splatAlphaRemovalThreshold: 1,
          showLoadingUI: false,

          position: [0, -0.8, 2],
          rotation: eulerDegToQuat(170, 0, 0),
          scale: [1, 1, 1],
        }
      );
      this.setLoadingProgress(95, "Finalizing truck scene...");
      if (this.groundMesh) {
        this.groundMesh.visible = false;
      }

      console.log("Environment: truck loaded");
      // this.groundVisuals?.forEach(obj => {
      //   obj.visible = false;
      // });
      this.hideGroundVisuals();
      this.hideTruckMujocoVisuals();
      this.resetCameraToRobotStart();
      }
    } finally {
        this.hideLoadingScene();
    }
  }
  onWindowResize() {
    this.camera.aspect = window.innerWidth / window.innerHeight;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(window.innerWidth, window.innerHeight);
  }

  applySCUTLControl(timeMS) {
    // Zero all actuators first
    // for (let i = 0; i < this.data.ctrl.length; i++) {
    //   this.data.ctrl[i] = 0.0;
    // }

    // Ask the GUI for the current params
    const guiParams = this.scutlGUI.getParams();
    if (!window.lastParamPrint || timeMS - window.lastParamPrint > 500) {
      // console.log("SCUTL params:", guiParams);
      window.lastParamPrint = timeMS;
    }

    // Smooth joystick-derived command values so robot motion transitions gradually
    if (!this.scutlSmoothParams) {
      this.scutlSmoothParams = { ...guiParams };
    }

    const alpha = 0.015; // lower = smoother, higher = faster response

    for (const key of ["ampHorz", "steerRatio", "turnFlag", "direction"]) {
      if (typeof guiParams[key] === "number") {
        this.scutlSmoothParams[key] += alpha * (guiParams[key] - this.scutlSmoothParams[key]);
      }
    }

    const smoothGuiParams = {
      ...guiParams,
      ampHorz: this.scutlSmoothParams.ampHorz,
      steerRatio: this.scutlSmoothParams.steerRatio,
      turnFlag: this.scutlSmoothParams.turnFlag,
      direction: guiParams.direction
    };

    // if (guiParams.stop) {
    //   this.scutlLastLegRaw      = null;
    //   this.scutlUnwrappedLegRaw = null;
    //   return;
    // }

    // if (guiParams.stop) {
    //   this.scutlLastLegRaw      = null;
    //   this.scutlUnwrappedLegRaw = null;

    //   // Hold last commanded actuator pose instead of returning to zero/home.
    //   if (this.scutlHoldCtrl !== null) {
    //     for (let i = 0; i < Math.min(this.data.ctrl.length, this.scutlHoldCtrl.length); i++) {
    //       this.data.ctrl[i] = this.scutlHoldCtrl[i];
    //     }
    //   }

    //   return;
    // }

    if (guiParams.stop) {
      // Do NOT reset leg unwrap state here.
      // Holding should preserve the current phase so restart does not snap backward.

      if (this.scutlHoldCtrl !== null) {
        for (let i = 0; i < Math.min(this.data.ctrl.length, this.scutlHoldCtrl.length); i++) {
          this.data.ctrl[i] = this.scutlHoldCtrl[i];
        }
      }

      return;
    }

    const t       = timeMS * 0.001;
    const targets = generateSimpleSteerTargets(t, smoothGuiParams);

    // ── Continuous phase unwrapping ───────────────────────────────────────
    // if (this.scutlLastLegRaw === null) {
    //   this.scutlLastLegRaw      = targets.legRaw.slice();
    //   this.scutlUnwrappedLegRaw = targets.legRaw.slice();
    // } else {
    //   for (let i = 0; i < targets.legRaw.length; i++) {
    //     let diff = targets.legRaw[i] - this.scutlLastLegRaw[i];
    //     if (diff >  2048.0) diff -= 4096.0;
    //     if (diff < -2048.0) diff += 4096.0;
    //     this.scutlUnwrappedLegRaw[i] += diff;
    //     this.scutlLastLegRaw[i]       = targets.legRaw[i];
    //   }
    //   targets.legRad = this.scutlUnwrappedLegRaw.map(
    //     x => (x * (360.0 / 4096.0)) * Math.PI / 180.0 - Math.PI
    //   );
    // }

    // ── Continuous leg unwrap, matching the ROS controller idea ─────────────
    if (this.scutlLastLegRaw === null) {
      this.scutlLastLegRaw = targets.legRaw.slice();

      if (this.scutlLastPublishedLeg !== null) {
        // Start from the last actually commanded leg angle.
        this.scutlUnwrappedLegRaw = this.scutlLastPublishedLeg.map(rad => {
          return ((rad + Math.PI) * 180.0 / Math.PI) / (360.0 / 4096.0);
        });

        // Shift incoming wrapped raw values to be closest to held pose.
        for (let i = 0; i < targets.legRaw.length; i++) {
          let diff = targets.legRaw[i] - this.scutlUnwrappedLegRaw[i];

          diff = ((diff + 2048.0) % 4096.0 + 4096.0) % 4096.0 - 2048.0;

          this.scutlUnwrappedLegRaw[i] += diff;
          this.scutlLastLegRaw[i] = targets.legRaw[i];
        }
      } else {
        this.scutlUnwrappedLegRaw = targets.legRaw.slice();
      }
    } else {
      for (let i = 0; i < targets.legRaw.length; i++) {
        let diff = targets.legRaw[i] - this.scutlLastLegRaw[i];

        diff = ((diff + 2048.0) % 4096.0 + 4096.0) % 4096.0 - 2048.0;

        this.scutlUnwrappedLegRaw[i] += diff;
        this.scutlLastLegRaw[i] = targets.legRaw[i];
      }
    }

    targets.legRad = this.scutlUnwrappedLegRaw.map(x =>
      (x * (360.0 / 4096.0)) * Math.PI / 180.0 - Math.PI
    );


    // ── Actuator map ─────────────────────────────────────────────────────
    // XML actuator order:
    //   0-1  knees L6/R6    2-3  vert/horz 6
    //   4-5  knees L5/R5    6-7  vert/horz 5
    //   8-9  knees L4/R4   10-11 vert/horz 4
    //  12-13 knees L3/R3   14-15 vert/horz 3
    //  16-17 knees L2/R2   18-19 vert/horz 2
    //  20-21 knees L1/R1
    const actuatorMap = [
      { type: "leg",  idx: 10 }, // knee_L6
      { type: "leg",  idx: 11 }, // knee_R6
      { type: "vert", idx: 4  },
      { type: "horz", idx: 4  },

      { type: "leg",  idx: 8  }, // knee_L5
      { type: "leg",  idx: 9  }, // knee_R5
      { type: "vert", idx: 3  },
      { type: "horz", idx: 3  },

      { type: "leg",  idx: 6  }, // knee_L4
      { type: "leg",  idx: 7  }, // knee_R4
      { type: "vert", idx: 2  },
      { type: "horz", idx: 2  },

      { type: "leg",  idx: 4  }, // knee_L3
      { type: "leg",  idx: 5  }, // knee_R3
      { type: "vert", idx: 1  },
      { type: "horz", idx: 1  },

      { type: "leg",  idx: 2  }, // knee_L2
      { type: "leg",  idx: 3  }, // knee_R2
      { type: "vert", idx: 0  },
      { type: "horz", idx: 0  },

      { type: "leg",  idx: 0  }, // knee_L1
      { type: "leg",  idx: 1  }, // knee_R1
    ];

  //   for (let a = 0; a < Math.min(this.data.ctrl.length, actuatorMap.length); a++) {
  //     const m = actuatorMap[a];
  //     if      (m.type === "leg")  this.data.ctrl[a] = targets.legRad[m.idx];
  //     else if (m.type === "horz") this.data.ctrl[a] = targets.horzRad[m.idx];
  //     else if (m.type === "vert") this.data.ctrl[a] = targets.vertRad[m.idx];
  //   }
    
  //   this.scutlHoldCtrl = Array.from(this.data.ctrl);
  //   this.scutlLastPublishedLeg = targets.legRad.slice();
    
  // }

    for (let a = 0; a < Math.min(this.data.ctrl.length, actuatorMap.length); a++) {
      const m = actuatorMap[a];

      if (m.type === "leg") {
        this.data.ctrl[a] = targets.legRad[m.idx];
      } else if (m.type === "horz") {
        this.data.ctrl[a] = targets.horzRad[m.idx];
      } else if (m.type === "vert") {
        this.data.ctrl[a] = targets.vertRad[m.idx];
      }
    }

    this.scutlHoldCtrl = Array.from(this.data.ctrl);
    this.scutlLastPublishedLeg = targets.legRad.slice();
  }
  

  render(timeMS) {
    this.controls.update();

    if (!this.params["paused"]) {
      const timestep = this.model.opt.timestep;

      if (timeMS - this.mujoco_time > 35.0) {
        this.mujoco_time = timeMS;
      }

      while (this.mujoco_time < timeMS) {
        if (this.params["ctrlnoisestd"] > 0.0) {
          const rate  = Math.exp(-timestep / Math.max(1e-10, this.params["ctrlnoiserate"]));
          const scale = this.params["ctrlnoisestd"] * Math.sqrt(1 - rate * rate);
          const ctrl  = this.data.ctrl;
          for (let i = 0; i < ctrl.length; i++) {
            ctrl[i] = rate * ctrl[i] + scale * standardNormal();
            this.params["Actuator " + i] = ctrl[i];
          }
        }

        for (let i = 0; i < this.data.qfrc_applied.length; i++) {
          this.data.qfrc_applied[i] = 0.0;
        }

        // const dragged = this.dragStateManager.physicsObject;
        // if (dragged && dragged.bodyID) {
        //   for (let b = 0; b < this.model.nbody; b++) {
        //     if (this.bodies[b]) {
        //       getPosition(this.data.xpos,   b, this.bodies[b].position);
        //       getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
        //       this.bodies[b].updateWorldMatrix();
        //     }
        //   }
        //   const bodyID = dragged.bodyID;
        //   this.dragStateManager.update();
        //   const force = toMujocoPos(
        //     this.dragStateManager.currentWorld.clone()
        //       .sub(this.dragStateManager.worldHit)
        //       .multiplyScalar(this.model.body_mass[bodyID] * 250)
        //   );
        //   const point = toMujocoPos(this.dragStateManager.worldHit.clone());
        //   mujoco.mj_applyFT(
        //     this.model, this.data,
        //     [force.x, force.y, force.z],
        //     [0, 0, 0],
        //     [point.x, point.y, point.z],
        //     bodyID,
        //     this.data.qfrc_applied
        //   );
        // }

        this.applySCUTLControl(timeMS);
        mujoco.mj_step(this.model, this.data);
        this.mujoco_time += timestep * 1000.0;
      }

      } else if (this.params["paused"]) {
        mujoco.mj_forward(this.model, this.data);
      }

    // } else if (this.params["paused"]) {
    //   this.dragStateManager.update();
    //   // const dragged = this.dragStateManager.physicsObject;
    //   if (dragged && dragged.bodyID) {
    //     const b = dragged.bodyID;
    //     getPosition(this.data.xpos,   b, this.tmpVec,  false);
    //     getQuaternion(this.data.xquat, b, this.tmpQuat, false);
    //     const offset = toMujocoPos(
    //       this.dragStateManager.currentWorld.clone()
    //         .sub(this.dragStateManager.worldHit)
    //         .multiplyScalar(0.3)
    //     );
    //     if (this.model.body_mocapid[b] >= 0) {
    //       const addr = this.model.body_mocapid[b] * 3;
    //       const pos  = this.data.mocap_pos;
    //       pos[addr + 0] += offset.x;
    //       pos[addr + 1] += offset.y;
    //       pos[addr + 2] += offset.z;
    //     } else {
    //       const root = this.model.body_rootid[b];
    //       const addr = this.model.jnt_qposadr[this.model.body_jntadr[root]];
    //       const pos  = this.data.qpos;
    //       pos[addr + 0] += offset.x;
    //       pos[addr + 1] += offset.y;
    //       pos[addr + 2] += offset.z;
    //     }
    //   }
    //   mujoco.mj_forward(this.model, this.data);
    // }

    for (let b = 0; b < this.model.nbody; b++) {
      if (this.bodies[b]) {
        getPosition(this.data.xpos,   b, this.bodies[b].position);
        getQuaternion(this.data.xquat, b, this.bodies[b].quaternion);
        this.bodies[b].updateWorldMatrix();
      }
    }

    for (let l = 0; l < this.model.nlight; l++) {
      if (this.lights[l]) {
        getPosition(this.data.light_xpos, l, this.lights[l].position);
        getPosition(this.data.light_xdir, l, this.tmpVec);
        this.lights[l].lookAt(this.tmpVec.add(this.lights[l].position));
      }
    }

    drawTendonsAndFlex(this.mujocoRoot, this.model, this.data);
    // this.renderer.render(this.scene, this.camera);
    // if (this.splatViewer) {
    //   this.splatViewer.update();
    // }


    // if (this.cameraMode === "first" && this.model && this.data) {
    //   const camId = mujoco.mj_name2id(this.model, mujoco.mjtObj.mjOBJ_CAMERA.value, "fp_camera");
    //   if (camId >= 0) {
    //     const pos = this.data.cam_xpos;
    //     const mat = this.data.cam_xmat;
    //     const base = camId * 3;
    //     const mbase = camId * 9;
    //     this.camera.position.set(pos[base], pos[base+1], pos[base+2]);
    //     const m = mat;
    //     const rm = new THREE.Matrix4().set(
    //       m[mbase+0], m[mbase+3], m[mbase+6], 0,
    //       m[mbase+1], m[mbase+4], m[mbase+7], 0,
    //       m[mbase+2], m[mbase+5], m[mbase+8], 0,
    //       0, 0, 0, 1
    //     );
    //     this.camera.quaternion.setFromRotationMatrix(rm);
    //   }
    // }


    // if (this.cameraMode === "first" && this.model && this.data) {
    //   const camId = mujoco.mj_name2id(this.model, mujoco.mjtObj.mjOBJ_CAMERA.value, "fp_camera");
    //   if (camId >= 0) {
    //     // Use getPosition to handle MuJoCo->Three.js coordinate conversion
    //     getPosition(this.data.cam_xpos, camId, this.camera.position);

    //     // Build quaternion from cam_xmat with axis swap
    //     const m = this.data.cam_xmat;
    //     const b = camId * 9;
    //     const rm = new THREE.Matrix4().set(
    //       m[b+0], -m[b+3],  m[b+6], 0,
    //       m[b+2], -m[b+5],  m[b+8], 0,
    //     -m[b+1],  m[b+4], -m[b+7], 0,
    //       0, 0, 0, 1
    //     );
    //     this.camera.quaternion.setFromRotationMatrix(rm);
    //   }
    // }

    // if (this.cameraMode === "first" && this.model && this.data) {
    //   const camId = mujoco.mj_name2id(this.model, mujoco.mjtObj.mjOBJ_CAMERA.value, "fp_camera");
    //   if (camId >= 0) {
    //     getPosition(this.data.cam_xpos, camId, this.camera.position);
    //     getQuaternion(this.data.cam_xquat, camId, this.camera.quaternion);
    //   }
    // }

    if (this.cameraMode === "first" && this.model && this.data) {
      // Find body_middle3 (center segment) index
      // const bodyId = mujoco.mj_name2id(this.model, mujoco.mjtObj.mjOBJ_BODY.value, "body_middle3");

            // Pick camera anchor: middle segment of however many are loaded
      const camAnchor = { 3: "body_middle5", 4: "body_middle5", 5: "body_middle4", 6: "body_middle3" };
      const bodyId = mujoco.mj_name2id(
        this.model,
        mujoco.mjtObj.mjOBJ_BODY.value,
        camAnchor[this.currentSegments || 6]
      );

      if (bodyId >= 0) {
        const bodyPos = new THREE.Vector3();
        getPosition(this.data.xpos, bodyId, bodyPos);

        // Fixed offset from robot body to camera (tuned from third person view)
        const offset = new THREE.Vector3(0.07, 0.211, -0.5);

        // Get robot body rotation and apply it to the offset so camera rotates with robot
        const bodyQuat = new THREE.Quaternion();
        getQuaternion(this.data.xquat, bodyId, bodyQuat);

        const guiP = this.scutlGUI.getParams();
        const goingBackward = guiP.direction < 0 || (guiP.stop === false && guiP.ampHorz < 0);
        if (goingBackward) {
          const flipQ = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0), Math.PI);
          bodyQuat.multiply(flipQ);
        }


        offset.applyQuaternion(bodyQuat);

        this.camera.position.copy(bodyPos).add(offset);

        // Look target: slightly ahead in robot's forward direction
        // const lookOffset = new THREE.Vector3(0.037, 0.211, 0.5);
        // lookOffset.applyQuaternion(bodyQuat);
        // const lookTarget = bodyPos.clone().add(lookOffset);
        // this.camera.lookAt(lookTarget);



        const yawQ   = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0), this.fpYaw   || 0);
        const pitchQ = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0), this.fpPitch || 0);
        const lookOffset = new THREE.Vector3(0.037, -0.011, 0.5);
        lookOffset.applyQuaternion(pitchQ).applyQuaternion(yawQ).applyQuaternion(bodyQuat);
        const lookTarget = this.camera.position.clone().add(lookOffset);
        this.camera.lookAt(lookTarget);
      }
    }



    this.renderer.render(this.scene, this.camera);

  }
}

window.demo = new MuJoCoDemo();
await window.demo.init();