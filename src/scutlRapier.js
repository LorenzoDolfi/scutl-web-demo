// scutlRapier.js
// Builds SCUTL in a Rapier world, mirroring the MuJoCo XML structure.
// Returns handles needed to drive joints each frame.
//
// Usage:
//   import { buildSCUTL } from './scutlRapier.js';
//   const robot = buildSCUTL(world, spawnPos);
//   // each frame:
//   robot.setJointTargets(targets); // targets from generateSimpleSteerTargets()
//   robot.update();                 // read back positions for Three.js

import RAPIER from '@dimforge/rapier3d-compat';

// ─── constants from XML ──────────────────────────────────────────────────────

// Body box half-sizes [x, y, z]
const BODY_BOX  = [0.0666575, 0.041973, 0.0374445];
const BODY_POS  = [0.074417,  0.12845,  0.025525 ];
const BODY_MASS = 0.08684;

// Segment-to-segment offset (horzjoint_body -> next body_middle)
const SEG_OFFSET = [0.00357, -0.18092, 0.003];

// Vert joint anchor in parent segment frame
const VERT_ANCHOR = [0.094905, 0.086975, 0.02858];
// Horz joint anchor in vert body frame
const HORZ_ANCHOR = [0.07753, -0.009025, 0.045952];

// Leg geometry
const LEG_HALF_LEN = 0.045;  // cylinder half-length
const LEG_RADIUS   = 0.006;  // cylinder radius (collision)
const KNEE_ANCHOR  = [-0.009764, 0.1294, 0.01076]; // knee hinge pos in parent

// Left/right leg positions in segment frame
const LEG_L_POS = [0.16407,  0.12939, -0.027134];
const LEG_R_POS = [-0.016106, 0.1294,  -0.027164];
const MOTOR_L_POS = [0.13525, 0.12938, 0.022634];
const MOTOR_R_POS = [0.012712, 0.12941, 0.022604];

// Actuator gains (matching MuJoCo kp)
const KP_KNEE = 10;
const KP_VERT = 10;
const KP_HORZ = 10;
const FORCE_KNEE = 4;
const FORCE_VERT = 5;
const FORCE_HORZ = 4;

// ─── helpers ─────────────────────────────────────────────────────────────────

function addVec(a, b) {
  return [a[0]+b[0], a[1]+b[1], a[2]+b[2]];
}

function toRapierVec(arr) {
  return { x: arr[0], y: arr[1], z: arr[2] };
}

// Create a dynamic rigid body at world position
function createBody(world, pos, mass, desc = null) {
  const rbDesc = RAPIER.RigidBodyDesc.dynamic()
    .setTranslation(pos[0], pos[1], pos[2]);
  const rb = world.createRigidBody(rbDesc);

  const colDesc = desc ?? RAPIER.ColliderDesc.cuboid(
    BODY_BOX[0], BODY_BOX[1], BODY_BOX[2]
  );
  colDesc.setMass(mass);
  world.createCollider(colDesc, rb);
  return rb;
}

// Create a hinge (revolute) joint between two bodies
// anchor1/anchor2 are local-space anchors on body1/body2
// axis is the hinge axis in local space
function createHinge(world, body1, body2, anchor1, anchor2, axis) {
  const params = RAPIER.JointData.revolute(
    toRapierVec(anchor1),
    toRapierVec(anchor2),
    toRapierVec(axis)
  );
  return world.createImpulseJoint(params, body1, body2, true);
}

// ─── main builder ────────────────────────────────────────────────────────────

export function buildSCUTL(world, spawnPos = [0, 0, 0.5]) {

  const segments  = [];  // 6 segment rigid bodies (index 0 = seg1/tail, 5 = seg6/root)
  const legBodies = [];  // 12 leg rigid bodies [L1,R1, L2,R2, ... L6,R6]
  const kneeJoints = []; // 12 knee revolute joints
  const vertJoints = []; // 5 vert joints (between segs)
  const horzJoints = []; // 5 horz joints (between segs)

  // Root segment spawns at spawnPos
  // Segments are chained: root(6) -> vert/horz -> seg5 -> ... -> seg1
  // We build from root outward

  let prevSegBody = null;
  let prevSegWorldPos = null;

  // XML chain: body_middle6 (root) -> horzjoint6 -> body_middle5 -> ... -> body_middle1
  // Segment indices in XML: 6,5,4,3,2,1
  // We iterate root-first

  for (let i = 6; i >= 1; i--) {
    // World position of this segment
    let worldPos;
    if (i === 6) {
      worldPos = [...spawnPos];
    } else {
      worldPos = addVec(prevSegWorldPos, SEG_OFFSET);
    }

    // Create segment body (box)
    const segColDesc = RAPIER.ColliderDesc.cuboid(
      BODY_BOX[0], BODY_BOX[1], BODY_BOX[2]
    ).setTranslation(BODY_POS[0], BODY_POS[1], BODY_POS[2])
     .setMass(BODY_MASS);

    const segRbDesc = RAPIER.RigidBodyDesc.dynamic()
      .setTranslation(worldPos[0], worldPos[1], worldPos[2]);
    const segBody = world.createRigidBody(segRbDesc);
    world.createCollider(segColDesc, segBody);

    segments.push({ body: segBody, worldPos: [...worldPos], segIdx: i });

    // Connect to previous segment with vert+horz joints
    if (prevSegBody !== null) {
      // Vert joint: hinge around X axis between prevSeg and this seg
      // anchor on prevSeg = VERT_ANCHOR, anchor on this seg = VERT_ANCHOR (mirrored)
      const vj = createHinge(
        world,
        prevSegBody, segBody,
        VERT_ANCHOR,
        VERT_ANCHOR,
        [1, 0, 0]  // X axis
      );
      vertJoints.push(vj);

      // Horz joint: hinge around Z axis
      const hj = createHinge(
        world,
        prevSegBody, segBody,
        HORZ_ANCHOR,
        HORZ_ANCHOR,
        [0, 0, 1]  // Z axis
      );
      horzJoints.push(hj);
    }

    // Create left and right legs for this segment
    for (const side of ['L', 'R']) {
      const isLeft = side === 'L';
      const legLocalPos = isLeft ? LEG_L_POS : LEG_R_POS;
      const legWorldPos = addVec(worldPos, legLocalPos);

      // Leg cylinder collider — oriented along X (matching quat 0.707 0 0.707 0 in XML)
      const legColDesc = RAPIER.ColliderDesc.cylinder(LEG_HALF_LEN, LEG_RADIUS)
        .setMass(0.10587964);

      const legRbDesc = RAPIER.RigidBodyDesc.dynamic()
        .setTranslation(legWorldPos[0], legWorldPos[1], legWorldPos[2]);
      const legBody = world.createRigidBody(legRbDesc);
      world.createCollider(legColDesc, legBody);
      legBodies.push({ body: legBody, segIdx: i, side });

      // Knee hinge: X axis, anchored at KNEE_ANCHOR in seg frame
      const kj = createHinge(
        world,
        segBody, legBody,
        KNEE_ANCHOR,
        [0, 0, 0],  // center of leg body
        [1, 0, 0]   // X axis
      );
      kneeJoints.push(kj);
    }

    prevSegBody = segBody;
    prevSegWorldPos = worldPos;
  }

  // ─── joint target API ──────────────────────────────────────────────────────
  // targets = { legRad: Float32Array(12), vertRad: Float32Array(5), horzRad: Float32Array(5) }
  // matching the order from generateSimpleSteerTargets()

  function setJointTargets(targets) {
    // Knee joints: 12 total, order matches actuatorMap in main.js
    // actuatorMap order: L6,R6, L5,R5, L4,R4, L3,R3, L2,R2, L1,R1
    for (let k = 0; k < kneeJoints.length; k++) {
      const jt = kneeJoints[k];
      if (!jt) continue;
      const target = targets.legRad[k] ?? 0;
      const current = 0; // Rapier doesn't expose joint angle directly yet; use motor
      const torque = Math.max(-FORCE_KNEE, Math.min(FORCE_KNEE,
        KP_KNEE * (target - current)
      ));
      jt.configureMotorVelocity(0, torque);
    }

    // Vert joints: 5, order 6->5->4->3->2
    for (let v = 0; v < vertJoints.length; v++) {
      const jt = vertJoints[v];
      if (!jt) continue;
      const target = targets.vertRad[v] ?? 0;
      const torque = Math.max(-FORCE_VERT, Math.min(FORCE_VERT,
        KP_VERT * target
      ));
      jt.configureMotorVelocity(0, torque);
    }

    // Horz joints: 5
    for (let h = 0; h < horzJoints.length; h++) {
      const jt = horzJoints[h];
      if (!jt) continue;
      const target = targets.horzRad[h] ?? 0;
      const torque = Math.max(-FORCE_HORZ, Math.min(FORCE_HORZ,
        KP_HORZ * target
      ));
      jt.configureMotorVelocity(0, torque);
    }
  }

  // Returns world transforms for Three.js to consume
  function getTransforms() {
    const out = {
      segments: segments.map(s => {
        const t = s.body.translation();
        const r = s.body.rotation();
        return { position: [t.x, t.y, t.z], quaternion: [r.x, r.y, r.z, r.w] };
      }),
      legs: legBodies.map(l => {
        const t = l.body.translation();
        const r = l.body.rotation();
        return { position: [t.x, t.y, t.z], quaternion: [r.x, r.y, r.z, r.w], segIdx: l.segIdx, side: l.side };
      })
    };
    return out;
  }

  // Returns root segment body (for camera follow etc)
  function getRootBody() {
    return segments[segments.length - 1].body; // seg6 = root
  }

  return { setJointTargets, getTransforms, getRootBody, segments, legBodies, kneeJoints, vertJoints, horzJoints };
}
