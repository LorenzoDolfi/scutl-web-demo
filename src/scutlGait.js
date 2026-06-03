const PI = Math.PI;
const DYNAMIXEL_ZERO = 2048.0;
const DEG_PER_TICK = 360.0 / 4096.0;

function degToRad(deg) {
  return deg * PI / 180.0;
}

function convXL430(angleDeg) {
  return angleDeg / DEG_PER_TICK + DYNAMIXEL_ZERO;
}

function calcPosFromTime(ts, stancePhase, stanceDuration, minStance) {
  let tn = (1 - stanceDuration) * (minStance / (1 - stancePhase));
  let tx = stanceDuration * (((minStance / stanceDuration) - minStance + 1 - stancePhase) / (1 - stancePhase));

  let C1 = (1 - stancePhase) / (1 - stanceDuration);
  let C2 = stancePhase / stanceDuration;
  let C3 = ((stanceDuration - stancePhase) / (stanceDuration * (1 - stancePhase))) * minStance;
  let C4 = (stancePhase - stanceDuration) / (1 - stanceDuration);

  let countNeg = 0;
  let countPos = 0;

  while (ts < 0) {
    ts += 1;
    countNeg += 1;
  }

  while (ts > 1) {
    ts -= 1;
    countPos += 1;
  }

  let pos;

  if (minStance + stancePhase > 1) {
    let initSlow = minStance + stancePhase - 1;
    let fastPhase = 1 - stancePhase;

    if (ts < initSlow / C2) {
      pos = ts * C2;
    } else if (ts < fastPhase / C1 + initSlow / C2) {
      pos = (ts - initSlow / C2) * C1 + initSlow;
    } else {
      pos = ts * C2 + initSlow + fastPhase - C2 * (fastPhase / C1 + initSlow / C2);
    }
  } else {
    if (ts < tn) {
      pos = ts * C1;
    } else if (ts <= tx) {
      pos = ts * C2 + C3;
    } else {
      pos = ts * C1 + C4;
    }
  }

  pos = pos * 4096;
  pos = pos + 4096 * countPos - 4096 * countNeg;

  return pos;
}

function calcLegPosAll(numLegs, phi, xis, stancePhase, stanceDuration, minStance, contralegPhase = 1.0) {
  let act = new Array(numLegs).fill(0);

  for (let i = 0; i < numLegs; i += 2) {
    let c = (phi + (i / 2) * xis * 2 * PI) % (2 * PI);
    if (c < 0) c += 2 * PI;
    c = c / (2 * PI);
    act[i] = calcPosFromTime(c, stancePhase, stanceDuration, minStance);

    let rightLegOffset = PI * contralegPhase;
    c = (phi + rightLegOffset + (i / 2) * xis * 2 * PI) % (2 * PI);
    if (c < 0) c += 2 * PI;
    c = c / (2 * PI);
    act[i + 1] = calcPosFromTime(c, stancePhase, stanceDuration, minStance);
  }

  return act;
}
function calcLegPosAllStep(numLegs, phi, xis, stancePhase, stanceDuration, minStance, contralegPhase = 1.0) {
  let act = new Array(numLegs).fill(0);

  for (let i = 0; i < numLegs; i += 2) {
    let c = (phi + (i / 2) * xis * 2 * PI) % (2 * PI);
    if (c < 0) c += 2 * PI;

    if (c < (1 - stanceDuration) * PI) {
      act[i] = minStance - stancePhase;
    } else if (c < (1 + stanceDuration) * PI) {
      act[i] = minStance;
    } else {
      act[i] = minStance - stancePhase;
    }

    let rightLegOffset = PI * contralegPhase;
    c = (phi + rightLegOffset + (i / 2) * xis * 2 * PI) % (2 * PI);
    if (c < 0) c += 2 * PI;

    if (c < (1 - stanceDuration) * PI) {
      act[i + 1] = minStance - stancePhase;
    } else if (c < (1 + stanceDuration) * PI) {
      act[i + 1] = minStance;
    } else {
      act[i + 1] = minStance - stancePhase;
    }
  }

  return act.map(x => x * 4096.0);
}

export function generateSimpleSteerTargets(t, params = {}) {
  const numJoints = 5;
  const numSegments = 6;

  const timePerCycle = params.timePerCycle ?? 2.8;
  const tFreq = (1.0 / timePerCycle) * 2.0 * PI;

    // ── Self-right modes ─────────────────────────────────────
//   if (params.cmd === "SelfRightPositive" || params.cmd === "SelfRightNegative") {
//     const sign = params.cmd === "SelfRightPositive" ? 1.0 : -1.0;
//     const phase = sign * tFreq * t;

//     const legRaw = new Array(12).fill(2048.0);
//     const legRad = new Array(12).fill(0.0);

//     // Strong body oscillation for self-right testing.
//     const horzRad = [0, 1, 2, 3, 4].map(i =>
//       sign * degToRad(55.0 * Math.sin(phase + i * 0.8))
//     );

//     const vertRad = [0, 1, 2, 3, 4].map(i =>
//       sign * degToRad(45.0 * Math.sin(phase + i * 0.8 + PI / 2.0))
//     );

//     return {
//       legRaw,
//       legRad,
//       horzRad,
//       vertRad
//     };
//   }

  // if (params.cmd === "SelfRightPositive" || params.cmd === "SelfRightNegative") {
  //   const sign = params.cmd === "SelfRightPositive" ? 1.0 : -1.0;
  //   const phase = sign * tFreq * t;

  //   const legRaw = new Array(12).fill(2048.0);
  //   const legRad = new Array(12).fill(0.0);

  //   const horzRad = [0, 1, 2, 3, 4].map(i =>
  //       sign * degToRad(55.0 * Math.sin(phase + i * 0.8))
  //   );

  //   const vertRad = [0, 1, 2, 3, 4].map(i =>
  //       sign * degToRad(45.0 * Math.sin(phase + i * 0.8 + PI / 2.0))
  //   );

  // //   return { legRaw, legRad, horzRad, vertRad };
  //   }

    // if (params.cmd === "TurnInPlace") {
    // const phase = tFreq * t;
    // const legRaw = new Array(12).fill(2048.0);
    // const legRad = new Array(12).fill(0.0);

    // const turnSign = params.turnFlag || 1.0;

    // const horzRad = [0, 1, 2, 3, 4].map(i =>
    //     turnSign * degToRad(45.0 * Math.sin(phase + i * 0.6))
    // );

    // const vertRad = [0, 1, 2, 3, 4].map(i =>
    //     degToRad(20.0 * Math.sin(phase + i * 0.6 + PI / 2.0))
    // );

    // return { legRaw, legRad, horzRad, vertRad };
    // }

  const ampBodyHorzTotal = params.ampHorz ?? 30.0;
  const steerRatio = params.steerRatio ?? 0.0;
  const turnFlag = params.turnFlag ?? 0;
  const direction = params.direction ?? 1;

  // const etaEllipse = 1.0;
  const etaEllipse = params.etaEllipse ?? 1.0;
  const horzDirection = params.horzDirection ?? 1.0;

//   const xisHorz = 1.0 / numJoints;
// //   const xisLeg = -1.0 / numJoints;
//   const spatialFreq = params.spatialFreq ?? 1.0;
//   const xisHorz = spatialFreq / numJoints;
//   const xisLeg = -spatialFreq / numJoints;


  const spatialFreq = params.spatialFreq ?? 1.0;
  const spatialFreqLeg = params.spatialFreqLeg ?? spatialFreq;

  const xisHorz = -spatialFreq / numJoints;
  const xisLeg = -spatialFreqLeg / numJoints;
  const offset = params.offset ?? 0.0;

//   const minStance = 0.5;
//   const stancePhase = 0.5;
//   const stanceDuration = 0.5;
//   const legPhase = 0.5;
//   const contralegPhase = 1.0;
  const minStance = params.minStance ?? 0.5;
  const stancePhase = params.stancePhase ?? 0.5;
  const stanceDuration = params.stanceDuration ?? 0.5;
  const legPhase = params.legPhase ?? 0.5;
  const contralegPhase = params.contraLegPhase ?? 1.0;

  let tReal = horzDirection * direction * t;

  let ampForward = -ampBodyHorzTotal / (1 + steerRatio);
  let ampTurn = turnFlag * (steerRatio / (1 + steerRatio)) * ampBodyHorzTotal;

  let indVec = [0, 1, 2, 3, 4];

  let w1 = ampForward * Math.cos(tFreq * tReal);
  let w2 = etaEllipse * ampForward * Math.sin(tFreq * tReal);
  if (Math.abs(ampForward) < 1e-9) {
    w1 = Math.cos(tFreq * tReal);
    w2 = Math.sin(tFreq * tReal);
    }

  let bodyHorzDeg = indVec.map(i => {
    let s1 = Math.sin(i * xisHorz * 2 * PI + offset);
    let s2 = Math.cos(i * xisHorz * 2 * PI + offset);
    let turn = ampTurn;
    return -(s1 * w1 + s2 * w2 + turn);
  });

  let maxPhase = PI * xisHorz - offset + PI * legPhase;
  let phi = Math.atan2(w2, w1) - maxPhase;

//   let legRaw = calcLegPosAll(
//     numSegments * 2,
//     phi,
//     xisLeg,
//     stancePhase,
//     stanceDuration,
//     minStance,
//     contralegPhase
//   );

  let legRaw;

  if ((params.steppingFlag ?? 0.0) > 0.5) {
    legRaw = calcLegPosAllStep(
        numSegments * 2,
        phi,
        xisLeg,
        stancePhase,
        stanceDuration,
        minStance,
        contralegPhase
    );
}   else {
        legRaw = calcLegPosAll(
            numSegments * 2,
            phi,
            xisLeg,
            stancePhase,
            stanceDuration,
            minStance,
            contralegPhase
    );
}



  let legRad = legRaw.map(x => degToRad(x * DEG_PER_TICK) - PI);
  let horzRad = bodyHorzDeg.map(deg => degToRad(convXL430(deg) - DYNAMIXEL_ZERO) * DEG_PER_TICK);


  // const vertAmp = params.vertAmp ?? 0.0;
  // const vertFreq = params.vertFreq ?? 1.0;
  // const vertPhase = params.vertPhase ?? 0.0;

  // let vertRad = indVec.map(i => {
  //   let angleDeg =
  //     vertAmp *
  //     Math.sin(i * vertFreq * 2 * PI / numJoints + tFreq * tReal + vertPhase);
  // return degToRad(angleDeg);
  // });

  const vertAmp = params.vertAmp ?? 0.0;
  const vertFreq = params.vertFreq ?? 1.0;
  const vertPhaseOffset = params.vertPhase ?? 0.0;

  const xisVert = vertFreq / numJoints;

  // This matches Python:
  // vertPhase = atan2(w2, w1)
  const dynamicVertPhase = Math.atan2(w2, w1);

  let vertRad = indVec.map(i => {
    const s1 = Math.sin(i * xisVert * 2 * PI + offset + vertPhaseOffset);
    const s2 = Math.cos(i * xisVert * 2 * PI + offset + vertPhaseOffset);

    const angleDeg = vertAmp * (
      s1 * Math.cos(dynamicVertPhase) +
      s2 * Math.sin(dynamicVertPhase)
    );

    return degToRad(angleDeg);
  });

//   return {
//     legRad,
//     horzRad,
//     vertRad
//   };

  return {
    legRaw,
    legRad,
    horzRad,
    vertRad
    };
}