/**
 * scutlGUI.js — Web Control Panel for SCUTL Robot
 * ================================================
 * Drop-in replacement for scutl_gui.py (ROS 2) → pure browser JS.
 *
 * Usage in main.js:
 *
 *   import { ScutlGUI } from './scutlGUI.js';
 *   const gui = new ScutlGUI();
 *   gui.mount(document.body);          // or any container element
 *
 *   // In your render / applySCUTLControl loop:
 *   const p = gui.getParams();
 *   // p = { stop, timePerCycle, ampHorz, steerRatio, turnFlag, direction }
 *   // Feed directly into generateSimpleSteerTargets(t, p)
 *
 * The panel also sets window.scutlGetParams() so you can call it from
 * anywhere without holding a reference to the ScutlGUI instance.
 */

const _PI = Math.PI;
const STEER_DEADZONE    = 20;
const STEER_TO_SIDEWIND = 10;
const MIN_RY            = 10;
const STEER_MAX_DEFAULT = 3.0;
const MODE_NORMAL       = "normal";
const MODE_PITTER       = "pitter";
const MODE_CRAWL        = "crawl";

function _clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

// ─────────────────────────────────────────────
//  Gait logic  (ported 1-to-1 from scutl_gui.py)
// ─────────────────────────────────────────────
function buildRadioRequest(s) {
  if (s.stop) return { cmd: null, params: null, tpc: null, dir: 1, stop: true };

  const dir = s.ry < 0 ? 1 : -1;

  // if (s.self_right_positive)
  //   return { cmd: "SelfRightPositive", params: null, tpc: 3.0, dir: 1, stop: false };

  if (s.self_right_positive)
    return {
      cmd: "SelfRightPositive",
      tpc: 3.0,
      dir: 1,
      stop: false,
      params: {
        HorizontalBodyUndulation: {
          eta_ellipse: 1,
          spatial_frequency: 0,
          direction: 1,
          offset: 0,
          total_amplitude: 60,
          steer_ratio: 0
        },
        VerticalBodyUndulation: {
          vertical_body_amplitude: -60,
          spatial_frequency_vertical: 0,
          vertical_phase: 4.71238898038469
        },
        LegRotation: {
          min_stance: 0.4,
          stance_phase: 0.2,
          stance_duration: 0.5,
          leg_phase: 0.5,
          contra_legPhase: 1.0
        },
        Flags: {
          turnFlag: 0,
          stepping_flag: 0
        }
      }
    };

  if (s.self_right_negative)
    return {
      cmd: "SelfRightNegative",
      tpc: 3.0,
      dir: 1,
      stop: false,
      params: {
        HorizontalBodyUndulation: {
          eta_ellipse: 1,
          spatial_frequency: 0,
          direction: 1,
          offset: 0,
          total_amplitude: -60,
          steer_ratio: 0
        },
        VerticalBodyUndulation: {
          vertical_body_amplitude: 60,
          spatial_frequency_vertical: 0,
          vertical_phase: 4.71238898038469
        },
        LegRotation: {
          min_stance: 0.4,
          stance_phase: 0.2,
          stance_duration: 0.5,
          leg_phase: 0.5,
          contra_legPhase: 1.0
        },
        Flags: {
          turnFlag: 0,
          stepping_flag: 0
        }
      }
    };



  // if (s.self_right_negative)
  //   return { cmd: "SelfRightNegative", params: null, tpc: 3.0, dir: 1, stop: false };
  if (s.turn_left)
    return {
      cmd: "TurnInPlace",
      tpc: 2.0,
      dir: 1,
      stop: false,
      params: {
        HorizontalBodyUndulation: {
          eta_ellipse: 1,
          spatial_frequency: 0.43,
          direction: 1,
          offset: 0,
          total_amplitude: 58.8,
          steer_ratio: 0
        },
        VerticalBodyUndulation: {
          vertical_body_amplitude: 20,
          spatial_frequency_vertical: 1.27,
          vertical_phase: 4.74
        },
        LegRotation: {
          min_stance: 0.0,
          stance_phase: 0.0,
          stance_duration: 0.9999,
          leg_phase: 0.5,
          contra_legPhase: 0
        },
        Flags: {
          turnFlag: 0,
          stepping_flag: 0
        }
      }
    };

  if (s.turn_right)
    return {
      cmd: "TurnInPlace",
      tpc: 2.0,
      dir: 1,
      stop: false,
      params: {
        HorizontalBodyUndulation: {
          eta_ellipse: 1,
          spatial_frequency: 0.43,
          direction: -1,
          offset: 0,
          total_amplitude: 58.8,
          steer_ratio: 0
        },
        VerticalBodyUndulation: {
          vertical_body_amplitude: 20,
          spatial_frequency_vertical: 1.27,
          vertical_phase: 4.74
        },
        LegRotation: {
          min_stance: 0.0,
          stance_phase: 0.0,
          stance_duration: 0.9999,
          leg_phase: 0.5,
          contra_legPhase: 0
        },
        Flags: {
          turnFlag: 0,
          stepping_flag: 0
        }
      }
    };

  if (Math.abs(s.ry) < MIN_RY && Math.abs(s.rx) < 30)
    return { cmd: null, params: null, tpc: null, dir: 1, stop: false };

  const tmax = 4.2, tmin = 2.2;
  const val = Math.sqrt(s.ry ** 2 + s.rx ** 2);
  const tpc = -(tmax - tmin) * val / 70 + tmax + (tmax - tmin) / 7;

  const steer_ratio     = Math.floor(10 * s.STEER_MAX * Math.abs(s.rx / 100)) / 10;
  const amp_horz_steer  = Math.max(60, s.amp_horz + 25 * (steer_ratio / s.STEER_MAX));
  const amp_vert        = s.vertical_enabled ? s.vert_amp : 0.0;
  const freq_vert       = -s.vertical_sign * s.vert_freq;
  const turn_flag       = s.rx < -STEER_DEADZONE ? -1 : (s.rx > STEER_DEADZONE ? 1 : 0);
  const mode            = s.mode;

  if (mode === MODE_NORMAL) {
    if (s.sd_normal_mode) {
      if (Math.abs(s.ry) < STEER_TO_SIDEWIND && Math.abs(s.rx) > 30) {
        // return { cmd: "Sidewind", tpc, dir, stop: false, params: {
        //   HorizontalBodyUndulation: { spatial_frequency: 0.7, total_amplitude: s.amp_horz },
        //   VerticalBodyUndulation: { vertical_phase: s.rx > 0 ? 0 : _PI, spatial_frequency_vertical: 0.7, vertical_body_amplitude: Math.max(s.vert_amp, 30) },
        return { cmd: "Sidewind", tpc, dir, stop: false, params: {
          HorizontalBodyUndulation: { 
            spatial_frequency: 0.7,
            total_amplitude: s.amp_horz 
          },
          VerticalBodyUndulation: {
            vertical_phase: s.rx > 0 ? _PI : 0,
            spatial_frequency_vertical: 0.7,
            vertical_body_amplitude: Math.max(s.vert_amp, 30)
          },
        }};
      }
      //   }};
      // }
      
      if (Math.abs(s.ry) >= STEER_TO_SIDEWIND) {
        const is_turn = Math.abs(s.rx) > STEER_DEADZONE;
        const horz = {}, vert = {}, leg = {}, flags = {};
        if (is_turn) {
          horz.steer_ratio = s.STEER_MAX;  horz.total_amplitude = 47.5;  flags.turnFlag = turn_flag;
        } else {
          if (Math.abs(s.amp_horz - 30) > 1e-6) horz.total_amplitude = s.amp_horz;
          if (Math.abs(s.freq_horz - 1)  > 1e-6) horz.spatial_frequency = s.freq_horz;
        }
        if (!is_turn) {
          if (Math.abs(amp_vert)              > 1e-6) vert.vertical_body_amplitude = amp_vert;
          if (Math.abs(freq_vert - 1)         > 1e-6) vert.spatial_frequency_vertical = freq_vert;
          if (Math.abs(s.vert_phase)          > 1e-6) vert.vertical_phase = s.vert_phase;
          if (Math.abs(s.leg_phase_offset)    > 1e-6) leg.leg_phase = 0.5 + s.leg_phase_offset;
          if (Math.abs(s.duty_offset)         > 1e-6) leg.stance_duration = 0.5 + s.duty_offset;
        }
        const p = {};
        if (Object.keys(horz).length)  p.HorizontalBodyUndulation = horz;
        if (Object.keys(vert).length)  p.VerticalBodyUndulation   = vert;
        if (Object.keys(leg).length)   p.LegRotation              = leg;
        if (Object.keys(flags).length) p.Flags                    = flags;
        return { cmd: "Steer", params: Object.keys(p).length ? p : null, tpc, dir, stop: false };
      }
    } else {
      if (Math.abs(s.ry) < STEER_TO_SIDEWIND && Math.abs(s.rx) > 30) {
        return { cmd: "Sidewind", tpc, dir, stop: false, params: {
          HorizontalBodyUndulation: { spatial_frequency: 0.7, total_amplitude: s.amp_horz },
          VerticalBodyUndulation: { vertical_phase: s.rx > 0 ? _PI : 0, spatial_frequency_vertical: 0.7, vertical_body_amplitude: Math.max(s.vert_amp, 30) },
        }};
      }
      if (Math.abs(s.ry) >= STEER_TO_SIDEWIND) {
        const horz = {}, vert = {}, leg = {}, flags = {};
        if (Math.abs(s.amp_horz - 45)   > 1e-6) horz.total_amplitude = s.amp_horz;
        if (Math.abs(s.freq_horz - 0.92) > 1e-6) horz.spatial_frequency = s.freq_horz;
        if (steer_ratio > 0) { horz.steer_ratio = steer_ratio;  horz.total_amplitude = amp_horz_steer; }
        if (Math.abs(amp_vert)           > 1e-6) vert.vertical_body_amplitude = amp_vert;
        if (Math.abs(freq_vert - 1)      > 1e-6) vert.spatial_frequency_vertical = freq_vert;
        if (Math.abs(s.vert_phase)       > 1e-6) vert.vertical_phase = s.vert_phase;
        const lp = 0.15 + s.leg_phase_offset;
        if (Math.abs(lp - 0.15)          > 1e-6) leg.leg_phase = lp;
        const sd = _clamp(0.39 + s.duty_offset, 0.05, 0.95);
        if (Math.abs(sd - 0.39)          > 1e-6) leg.stance_duration = sd;
        if (steer_ratio > 0)                     leg.spatial_frequency_leg = s.freq_horz;
        if (turn_flag !== 0)                     flags.turnFlag = turn_flag;
        const p = {};
        if (Object.keys(horz).length)  p.HorizontalBodyUndulation = horz;
        if (Object.keys(vert).length)  p.VerticalBodyUndulation   = vert;
        if (Object.keys(leg).length)   p.LegRotation              = leg;
        if (Object.keys(flags).length) p.Flags                    = flags;
        return { cmd: "Optimized", params: Object.keys(p).length ? p : null, tpc, dir, stop: false };
      }
    }
  } else if (mode === MODE_PITTER) {
    if (Math.abs(s.ry) >= STEER_TO_SIDEWIND) {
      return { cmd: "Stepping", tpc, dir, stop: false, params: {
        Flags:                    { stepping_flag: 1, turnFlag: turn_flag },
        VerticalBodyUndulation:   { vertical_body_amplitude: amp_vert, spatial_frequency_vertical: freq_vert, vertical_phase: s.vert_phase },
        HorizontalBodyUndulation: { total_amplitude: s.amp_horz, steer_ratio, spatial_frequency: s.freq_horz },
        LegRotation:              { min_stance: 0.5, stance_phase: s.ry < 0 ? 0.25 : -0.25, spatial_frequency_leg: s.freq_horz, leg_phase: 0.5 + s.leg_phase_offset, stance_duration: 0.5 + s.duty_offset },
      }};
    }
    if (Math.abs(s.rx) > 30 && Math.abs(s.ry) < STEER_TO_SIDEWIND) {
      return { cmd: "Sidewind", tpc, dir, stop: false, params: {
        HorizontalBodyUndulation: { spatial_frequency: 0.7, total_amplitude: s.amp_horz },
        VerticalBodyUndulation:   { vertical_phase: s.rx > 0 ? _PI : 0, spatial_frequency_vertical: 0.7, vertical_body_amplitude: Math.max(s.vert_amp, 30) },
      }};
    }
  } else if (mode === MODE_CRAWL) {
    if (Math.abs(s.ry) >= STEER_TO_SIDEWIND) {
      return { cmd: "Crawl", tpc, dir, stop: false, params: {
        VerticalBodyUndulation:   { spatial_frequency_vertical: freq_vert, vertical_body_amplitude: amp_vert, vertical_phase: s.vert_phase },
        LegRotation:              { min_stance: 0.4, stance_phase: 0.2, contra_legPhase: 0, spatial_frequency_leg: s.freq_horz, leg_phase: 0.5 + s.leg_phase_offset, stance_duration: 0.5 + s.duty_offset },
        HorizontalBodyUndulation: { total_amplitude: s.amp_horz, steer_ratio, spatial_frequency: -s.freq_horz },
        Flags:                    { turnFlag: s.rx < -10 ? -1 : s.rx > 10 ? 1 : 0 },
      }};
    }
    if (Math.abs(s.rx) > 30 && Math.abs(s.ry) < STEER_TO_SIDEWIND) {
      return { cmd: "Sidewind", tpc, dir, stop: false, params: {
        HorizontalBodyUndulation: { spatial_frequency: 0.7, total_amplitude: s.amp_horz },
        VerticalBodyUndulation:   { vertical_phase: s.rx > 0 ? _PI : 0, spatial_frequency_vertical: 0.7, vertical_body_amplitude: Math.max(s.vert_amp, 30) },
      }};
    }
  }

  return { cmd: null, params: null, tpc: null, dir: 1, stop: false };
}

/** Converts buildRadioRequest output → the flat param shape generateSimpleSteerTargets() expects. */
// function toSimpleSteerParams(s) {
//   const r = buildRadioRequest(s);
//   if (r.stop || !r.cmd) {
//     return { stop: true, timePerCycle: 2.8, ampHorz: 30, steerRatio: 0, turnFlag: 0, direction: 1 };
//   }
//   return {
//     stop:         false,
//     timePerCycle: r.tpc ?? 2.8,
//     ampHorz:      r.params?.HorizontalBodyUndulation?.total_amplitude ?? s.amp_horz,
//     steerRatio:   r.params?.HorizontalBodyUndulation?.steer_ratio     ?? 0,
//     turnFlag:     r.params?.HorizontalBodyUndulation?.direction
//                ?? r.params?.Flags?.turnFlag
//                ?? 0,
//     direction:    r.dir,
//     cmd: r.cmd,

//     vertAmp: r.params?.VerticalBodyUndulation?.vertical_body_amplitude ?? 0.0,
//     vertFreq: r.params?.VerticalBodyUndulation?.spatial_frequency_vertical ?? 1.0,
//     vertPhase: r.params?.VerticalBodyUndulation?.vertical_phase ?? 0.0,

//     spatialFreq: r.params?.HorizontalBodyUndulation?.spatial_frequency ?? s.freq_horz,
//     legPhase: r.params?.LegRotation?.leg_phase ?? 0.5,
//     stancePhase: r.params?.LegRotation?.stance_phase ?? 0.5,
//     stanceDuration: r.params?.LegRotation?.stance_duration ?? 0.5,
//     minStance: r.params?.LegRotation?.min_stance ?? 0.5,
//     contraLegPhase: r.params?.LegRotation?.contra_legPhase ?? 1.0,
//     steppingFlag: r.params?.Flags?.stepping_flag ?? 0.0,
//   };
// }
function toSimpleSteerParams(s) {
  const r = buildRadioRequest(s);

  if (r.stop || !r.cmd) {
    return {
      stop: true,
      cmd: null,
      timePerCycle: 2.8,
      ampHorz: 30,
      steerRatio: 0,
      turnFlag: 0,
      direction: 1
    };
  }

  const isOptimized = r.cmd === "Optimized";
  const isStepping  = r.cmd === "Stepping";
  const isCrawl     = r.cmd === "Crawl";
  const isSelfRightPositive = r.cmd === "SelfRightPositive";
  const isSelfRightNegative = r.cmd === "SelfRightNegative";
  const isSidewind = r.cmd === "Sidewind";
  const isTurnInPlace = r.cmd === "TurnInPlace";

  // return {
  //   stop: false,
  //   cmd: r.cmd,

  //   timePerCycle: r.tpc ?? 2.8,
  //   direction: r.dir,

  //   ampHorz:
  //     r.params?.HorizontalBodyUndulation?.total_amplitude ??
  //     (isOptimized ? 45.0 : 30.0),

  //   steerRatio:
  //     r.params?.HorizontalBodyUndulation?.steer_ratio ?? 0.0,

  //   turnFlag:
  //     r.params?.Flags?.turnFlag ??
  //     r.params?.HorizontalBodyUndulation?.direction ??
  //     0,

  //   spatialFreq:
  //     r.params?.HorizontalBodyUndulation?.spatial_frequency ??
  //     (isOptimized ? 0.92 : 1.0),

  //   spatialFreqLeg:
  //     r.params?.LegRotation?.spatial_frequency_leg ??
  //     r.params?.HorizontalBodyUndulation?.spatial_frequency ??
  //     (isOptimized ? 0.92 : 1.0),

  //   legPhase:
  //     r.params?.LegRotation?.leg_phase ??
  //     (isOptimized ? 0.15 : 0.5),

  //   stanceDuration:
  //     r.params?.LegRotation?.stance_duration ??
  //     (isOptimized ? 0.39 : 0.5),

  //   stancePhase:
  //     r.params?.LegRotation?.stance_phase ??
  //     (isStepping ? 0.25 : isCrawl ? 0.2 : 0.5),

  //   minStance:
  //     r.params?.LegRotation?.min_stance ??
  //     (isCrawl ? 0.4 : 0.5),

  //   contraLegPhase:
  //     r.params?.LegRotation?.contra_legPhase ??
  //     (isCrawl ? 0.0 : 1.0),

  //   steppingFlag:
  //     r.params?.Flags?.stepping_flag ??
  //     (isStepping ? 1.0 : 0.0),

  //   vertAmp:
  //     r.params?.VerticalBodyUndulation?.vertical_body_amplitude ?? 0.0,

  //   vertFreq:
  //     r.params?.VerticalBodyUndulation?.spatial_frequency_vertical ?? 1.0,

  //   vertPhase:
  //     r.params?.VerticalBodyUndulation?.vertical_phase ?? 0.0
  // };

  return {
    stop: false,
    cmd: r.cmd,

    timePerCycle: r.tpc ?? 2.8,
    direction: r.dir,

    ampHorz:
      r.params?.HorizontalBodyUndulation?.total_amplitude ??
      (isSelfRightPositive ? 60.0 :
      isSelfRightNegative ? -60.0 :
      isOptimized ? 45.0 : 30.0),

    steerRatio:
      r.params?.HorizontalBodyUndulation?.steer_ratio ?? 0.0,

    // turnFlag:
    //   r.params?.Flags?.turnFlag ??
    //   r.params?.HorizontalBodyUndulation?.direction ??
    //   0,

    turnFlag:
      r.params?.Flags?.turnFlag ??
      (r.cmd === "Steer" ? -1 : 0),

    spatialFreq:
      r.params?.HorizontalBodyUndulation?.spatial_frequency ??
      ((isSelfRightPositive || isSelfRightNegative) ? 0.0 :
      isOptimized ? 0.92 : 1.0),

    spatialFreqLeg:
      r.params?.LegRotation?.spatial_frequency_leg ??
      r.params?.HorizontalBodyUndulation?.spatial_frequency ??
      ((isSelfRightPositive || isSelfRightNegative) ? 0.0 :
      isOptimized ? 0.92 : 1.0),

    legPhase:
      r.params?.LegRotation?.leg_phase ??
      ((isSelfRightPositive || isSelfRightNegative) ? 0.5 :
      isOptimized ? 0.15 : 0.5),

    // stanceDuration:
    //   r.params?.LegRotation?.stance_duration ??
    //   ((isSelfRightPositive || isSelfRightNegative) ? 0.5 :
    //   isOptimized ? 0.39 : 0.5),

    // // stancePhase:
    // //   r.params?.LegRotation?.stance_phase ??
    // //   ((isSelfRightPositive || isSelfRightNegative) ? 0.2 :
    // //   isStepping ? 0.25 :
    // //   isCrawl ? 0.2 : 0.5),

    // stancePhase:
    //   r.params?.LegRotation?.stance_phase ??
    //   ((isSelfRightPositive || isSelfRightNegative) ? 0.2 :
    //   isOptimized ? 0.15 :
    //   isStepping ? 0.25 :
    //   isCrawl ? 0.2 : 0.5),

    // // minStance:
    // //   r.params?.LegRotation?.min_stance ??
    // //   ((isSelfRightPositive || isSelfRightNegative) ? 0.4 :
    // //   isCrawl ? 0.4 : 0.5),

    // minStance:
    //   r.params?.LegRotation?.min_stance ??
    //   ((isSelfRightPositive || isSelfRightNegative) ? 0.4 :
    //   isOptimized ? 0.0 :
    //   isCrawl ? 0.4 : 0.5),

    // // contraLegPhase:
    // //   r.params?.LegRotation?.contra_legPhase ??
    // //   ((isSelfRightPositive || isSelfRightNegative) ? 1.0 :
    // //   isCrawl ? 0.0 : 1.0),

    // contraLegPhase:
    //   r.params?.LegRotation?.contra_legPhase ??
    //   ((isSelfRightPositive || isSelfRightNegative) ? 1.0 :
    //   isOptimized ? 0.94 :
    //   isCrawl ? 0.0 : 1.0),


    stanceDuration:
      r.params?.LegRotation?.stance_duration ??
      ((isSidewind || isTurnInPlace) ? 0.9999 :
      (isSelfRightPositive || isSelfRightNegative) ? 0.5 :
      isOptimized ? 0.39 : 0.5),

    stancePhase:
      r.params?.LegRotation?.stance_phase ??
      ((isSidewind || isTurnInPlace) ? 0.0 :
      (isSelfRightPositive || isSelfRightNegative) ? 0.2 :
      isOptimized ? 0.15 :
      isStepping ? 0.25 :
      isCrawl ? 0.2 : 0.2),

    minStance:
      r.params?.LegRotation?.min_stance ??
      ((isSidewind || isTurnInPlace) ? 0.0 :
      (isSelfRightPositive || isSelfRightNegative) ? 0.4 :
      isOptimized ? 0.0 :
      isCrawl ? 0.4 : 0.4),

    contraLegPhase:
      r.params?.LegRotation?.contra_legPhase ??
      ((isSidewind || isTurnInPlace) ? 0.0 :
      (isSelfRightPositive || isSelfRightNegative) ? 1.0 :
      isOptimized ? 0.94 :
      isCrawl ? 0.0 : 1.0),

    steppingFlag:
      r.params?.Flags?.stepping_flag ??
      (isStepping ? 1.0 : 0.0),

    vertAmp:
      r.params?.VerticalBodyUndulation?.vertical_body_amplitude ??
      (isSelfRightPositive ? -60.0 :
      isSelfRightNegative ? 60.0 : 0.0),

    // vertFreq:
    //   r.params?.VerticalBodyUndulation?.spatial_frequency_vertical ??
    //   ((isSelfRightPositive || isSelfRightNegative) ? 0.0 : 1.0),

    vertFreq:
      r.params?.VerticalBodyUndulation?.spatial_frequency_vertical ??
      ((isSelfRightPositive || isSelfRightNegative) ? 0.0 :
      isSidewind ? 0.7 :
      isTurnInPlace ? 1.27 :
      isCrawl ? -1.0 :
      r.cmd === "Steer" ? 2.0 :
      1.0),
    

    vertPhase:
      r.params?.VerticalBodyUndulation?.vertical_phase ??
      ((isSelfRightPositive || isSelfRightNegative) ? 4.71238898038469 : 0.0),
    
    etaEllipse:
      r.params?.HorizontalBodyUndulation?.eta_ellipse ?? 1.0,

    horzDirection:
      r.params?.HorizontalBodyUndulation?.direction ?? 1,

    offset:
      r.params?.HorizontalBodyUndulation?.offset ?? 0.0,
    
  };




}

// ─────────────────────────────────────────────
//  HTML / CSS template
// ─────────────────────────────────────────────

const CSS = `
.sg-wrap *{
  box-sizing:border-box;
  margin:0;
  padding:0;
}

.sg-wrap{
  --bg:#0f1117;
  --panel:#161a24;
  --bd:#252a36;
  --acc:#00c896;
  --acc2:#009e76;
  --fg:#e2e8f0;
  --dim:#64748b;
  --lbl:#94a3b8;
  --tr:#1e2535;

  font-family:'Courier New',monospace;
  font-size:clamp(10px,0.8vw,12px);
  background:var(--bg);
  color:var(--fg);

  padding:clamp(5px,0.6vw,8px);
  width:min(22vw,300px);
  min-width:190px;
  max-width:calc(100vw - 20px);
  max-height:95vh;

  overflow-y:auto;
  overflow-x:hidden;
  -webkit-overflow-scrolling:touch;
}

.sg-title{
  display:flex;
  align-items:center;
  gap:8px;
  padding:6px 2px 10px;
  min-width:0;
}

.sg-title h2{
  color:var(--acc);
  font-size:clamp(12px,1vw,15px);
  letter-spacing:.5px;
  white-space:nowrap;
}

.sg-title select{
  min-width:0;
  max-width:135px;
  font-size:10px;
}

.sg-title .sub{
  color:var(--dim);
  font-size:10px;
}

.sg-status{
  display:block;
  color:var(--dim);
  font-size:10px;
  margin-bottom:6px;
  white-space:nowrap;
  overflow:hidden;
  text-overflow:ellipsis;
}

.sg-grid{
  display:block;
}

.sg-col{
  display:flex;
  flex-direction:column;
  gap:6px;
  min-width:0;
}

.sg-card{
  background:var(--panel);
  border:1px solid var(--bd);
  border-radius:5px;
  overflow:hidden;
  min-width:0;
}

.sg-head{
  padding:5px 8px;
  border-bottom:1px solid var(--bd);
  color:var(--acc);
  font-size:9px;
  letter-spacing:.8px;
  text-transform:uppercase;
  white-space:nowrap;
  overflow:hidden;
  text-overflow:ellipsis;
}

.sg-body{
  padding:6px;
  min-width:0;
}

canvas.sg-joy{
  background:#0d1520;
  border-radius:3px;
  display:block;
  cursor:crosshair;
  touch-action:none;
  width:clamp(110px,16vw,220px);
  height:clamp(110px,16vw,220px);
  max-width:100%;
}

.sg-axis{
  font-size:10px;
  color:var(--acc);
  text-align:center;
  padding:3px 0;
  white-space:nowrap;
}

.sg-btn{
  width:100%;
  padding:5px;
  background:transparent;
  border:1px solid var(--bd);
  color:var(--lbl);
  border-radius:3px;
  cursor:pointer;
  font-size:11px;
  font-family:inherit;
}

.sg-btn:hover{
  background:var(--bd);
  color:var(--fg);
}

.sg-hold-grid{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:5px;
  min-width:0;
}

.sg-hbtn{
  padding:7px 4px;
  background:transparent;
  border:1px solid var(--bd);
  color:var(--lbl);
  border-radius:3px;
  cursor:pointer;
  font-size:10px;
  font-family:inherit;
  user-select:none;
  -webkit-user-select:none;
  min-width:0;
}

.sg-hbtn:active,
.sg-hbtn.on{
  background:var(--acc2);
  color:#fff;
  border-color:var(--acc);
}

.sg-modes{
  display:grid;
  grid-template-columns:1fr;
  gap:6px;
  min-width:0;
}

.sg-vc{
  border:1px solid var(--bd);
  border-radius:3px;
  padding:6px;
  display:flex;
  flex-direction:column;
  gap:2px;
  min-width:0;
}

.sg-vc-title{
  color:var(--acc);
  font-size:10px;
  font-weight:700;
  margin-bottom:2px;
}

label.sg-r,
label.sg-c{
  display:flex;
  align-items:center;
  gap:5px;
  cursor:pointer;
  padding:2px 0;
  font-size:11px;
  color:var(--lbl);
  min-width:0;
}

input[type=radio],
input[type=checkbox]{
  accent-color:var(--acc);
}

.sg-psect{
  display:flex;
  flex-direction:column;
  gap:8px;
  min-width:0;
}

.sg-pboxes{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:3px;
  width:100%;
  min-width:0;
}

.sg-pb{
  padding:4px 5px;
  background:#00c896;
  color:#fff;
  border:none;
  border-radius:3px;
  cursor:pointer;
  font-size:10px;
  font-family:inherit;
  text-align:center;
  width:100%;
  min-width:0;
  overflow:hidden;
  text-overflow:ellipsis;
  white-space:nowrap;
}

.sg-pb.sel{
  background:#009e76;
}

.sg-pb:hover{
  background:#10d8a0;
}

.sg-param-inner{
  width:100%;
  min-width:0;
  display:flex;
  flex-direction:column;
  gap:6px;
}

.sg-prow{
  display:grid;
  grid-template-columns:auto minmax(0,1fr) auto auto;
  align-items:center;
  gap:5px;
  min-width:0;
  width:100%;
}

.sg-prow input[type=range]{
  width:100%;
  min-width:0;
  accent-color:var(--acc);
}

.sg-pval{
  min-width:36px;
  max-width:42px;
  text-align:right;
  color:var(--lbl);
  font-size:10px;
  overflow:hidden;
}

.sg-sb{
  padding:3px 7px;
  background:transparent;
  border:1px solid var(--bd);
  color:var(--fg);
  border-radius:3px;
  cursor:pointer;
  font-size:13px;
  font-family:inherit;
}

.sg-sb:hover{
  background:var(--bd);
}

.sg-util{
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:5px;
  min-width:0;
}

.sg-ub{
  min-width:0;
  padding:6px 4px;
  background:transparent;
  border:1px solid var(--bd);
  color:var(--lbl);
  border-radius:3px;
  cursor:pointer;
  font-size:10px;
  font-family:inherit;
  overflow:hidden;
  text-overflow:ellipsis;
  white-space:nowrap;
}

.sg-ub:hover{
  background:var(--bd);
  color:var(--fg);
}

.sg-ub.acc{
  background:var(--acc);
  color:#0a0f14;
  border-color:var(--acc);
  font-weight:700;
}

.sg-ub.acc:hover{
  background:var(--acc2);
}

.sg-full{
  margin-top:6px;
}
`;

const HTML = `
<div class="sg-title">
  <h2>GCR SCUTL</h2>

  <select id="sg-menu"
          style="
            margin-left:auto;
            background:#161a24;
            color:#e2e8f0;
            border:1px solid #252a36;
            border-radius:4px;
            padding:4px;">
    <option value="joystick">Joystick Only</option>
    <option value="edit">Edit Parameters</option>
  </select>
</div>

<span class="sg-status" id="sg-status">● Idle</span>

<!-- JOYSTICK ALWAYS VISIBLE -->
<div class="sg-card">
  <div class="sg-head">Joystick · WASD</div>
  <div class="sg-body" style="display:flex;flex-direction:column;align-items:center;gap:5px">
    <canvas class="sg-joy" id="sg-joy" width="240" height="240"></canvas>
    <div class="sg-axis" id="sg-axis">rx=+0.0  ry=+0.0</div>
    <button class="sg-btn" id="sg-center">⏹ Center / Stop</button>
  </div>
</div>

<!-- EVERYTHING BELOW IS HIDDEN UNTIL "EDIT PARAMETERS" -->
<div id="sg-full-controls" style="display:none; margin-top:6px;">

  <div class="sg-card">
    <div class="sg-head">Hold Buttons</div>
    <div class="sg-body">
      <div class="sg-hold-grid">
        <button class="sg-hbtn" id="sg-srp">↑ Self-right +</button>
        <button class="sg-hbtn" id="sg-srn">↓ Self-right −</button>
        <button class="sg-hbtn" id="sg-tl">↺ Turn left</button>
        <button class="sg-hbtn" id="sg-tr">↻ Turn right</button>
      </div>
    </div>
  </div>

  <div class="sg-card" style="margin-top:6px;">
    <div class="sg-head">Modes & Switches</div>
    <div class="sg-body">
      <div class="sg-modes">
        <div style="display:flex;flex-direction:column;gap:2px">
          <label class="sg-r"><input type="radio" name="sg-mode" value="normal" checked> Normal</label>
          <label class="sg-r"><input type="radio" name="sg-mode" value="pitter"> Pitter/Stepping</label>
          <label class="sg-r"><input type="radio" name="sg-mode" value="crawl"> Crawl</label>
          <label class="sg-c" style="margin-top:4px">
            <input type="checkbox" id="sg-sdn" checked> SD normal
          </label>
        </div>

        <div class="sg-vc">
          <div class="sg-vc-title">Vertical</div>

          <label class="sg-c">
            <input type="checkbox" id="sg-ven"> Enabled
          </label>

          <div style="font-size:10px;color:var(--lbl);margin-top:3px">
            Direction:
          </div>

          <label class="sg-r">
            <input type="radio" name="sg-vs" value="-1" checked> +
          </label>

          <label class="sg-r">
            <input type="radio" name="sg-vs" value="1"> −
          </label>
        </div>
      </div>
    </div>
  </div>

  <div class="sg-full">
    <div class="sg-card">
      <div class="sg-head">Parameters</div>

      <div class="sg-body">
        <div class="sg-psect">

          <div class="sg-pboxes" id="sg-pboxes"></div>

          <div class="sg-param-inner">

            <div class="sg-prow">

              <button class="sg-sb" id="sg-pminus">−</button>

              <div style="position:relative;flex:1;">

                <div id="sg-tooltip"
                     style="
                        position:absolute;
                        top:-22px;
                        left:50%;
                        transform:translateX(-50%);
                        background:#00c896;
                        color:white;
                        padding:2px 6px;
                        border-radius:4px;
                        font-size:11px;
                        pointer-events:none;">
                  30.0
                </div>

                <input type="range" id="sg-pslider">

              </div>

              <button class="sg-sb" id="sg-pplus">+</button>

              <span class="sg-pval" id="sg-pval">30.0</span>

            </div>

          </div>
        </div>
      </div>
    </div>
  </div>

  <div class="sg-full">
    <div class="sg-card">
      <div class="sg-head">Utilities</div>

      <div class="sg-body">
        <div class="sg-util">

          <button class="sg-ub" id="sg-reset">
            ↺ Reset params
          </button>

          <button class="sg-ub" id="sg-stop">
            ⏸ Stop / Hold
          </button>

          <button class="sg-ub acc" id="sg-resume">
            ▶ Resume
          </button>

          <button class="sg-ub" id="sg-copy">
            ⎘ Copy JSON
          </button>

        </div>
      </div>
    </div>
  </div>

</div>
`;

// ─────────────────────────────────────────────
//  ScutlGUI class
// ─────────────────────────────────────────────
export class ScutlGUI {
  constructor() {
    this._state = {
      rx: 0, ry: 0,
      self_right_positive: false, self_right_negative: false,
      turn_left: false, turn_right: false,
      mode: MODE_NORMAL, sd_normal_mode: true,
      vertical_enabled: false, vertical_sign: -1.0,
      amp_horz: 30, freq_horz: 1, vert_amp: 30, vert_freq: 1,
      vert_phase: 0, leg_phase_offset: 0, duty_offset: 0,
      STEER_MAX: STEER_MAX_DEFAULT,
      stop: false,
    };

    this._paramSpecs = [
      { key: "amp_horz",         label: "Horiz amp",    lo: 0,    hi: 60,      step: 5,       dec: 1 },
      { key: "freq_horz",        label: "Horiz freq",   lo: 0.0,  hi: 2,       step: 0.1,     dec: 2 },
      { key: "vert_amp",         label: "Vert amp",     lo: 0,    hi: 60,      step: 5,       dec: 1 },
      { key: "vert_freq",        label: "Vert freq",    lo: 0.1,  hi: 2,       step: 0.1,     dec: 2 },
      { key: "vert_phase",       label: "Vert phase",   lo: 0,    hi: 2 * _PI, step: _PI / 8, dec: 3 },
      { key: "leg_phase_offset", label: "Leg phase",    lo: -0.5, hi: 0.5,     step: 0.05,    dec: 3 },
      { key: "duty_offset",      label: "Duty offset",  lo: -0.35,hi: 0.35,    step: 0.05,    dec: 3 },
      { key: "STEER_MAX",        label: "Steer radius", lo: 0.5,  hi: 3,       step: 0.5,     dec: 1 },
    ];

    this._selIdx = 0;
    this._joyDragging = false;
    this._pressed = new Set();
    this._releaseJobs = {};
    this._wrapper = null;

    // Expose globally
    window.scutlGetParams = () => this.getParams();
    window.scutlState     = this._state;
  }

  /** Returns the flat params object for generateSimpleSteerTargets(t, params). */
  getParams() {
    return toSimpleSteerParams(this._state);
  }

  /** Mounts the control panel into containerEl. */
  mount(containerEl) {
    // Inject styles once
    if (!document.getElementById("sg-style")) {
      const tag = document.createElement("style");
      tag.id = "sg-style";
      tag.textContent = CSS;
      document.head.appendChild(tag);
    }

    // const wrap = document.createElement("div");
    // wrap.className = "sg-wrap";
    // wrap.innerHTML = HTML;
    // containerEl.appendChild(wrap);

    const wrap = document.createElement("div");
    wrap.className = "sg-wrap";

    wrap.style.position = "fixed";
    wrap.style.left = "10px";
    wrap.style.top = "10px";
    wrap.style.zIndex = "9999";
    wrap.style.maxHeight = "95vh";
    wrap.style.overflowY = "auto";
    wrap.style.overflowX = "hidden";

    wrap.innerHTML = HTML;
    containerEl.appendChild(wrap);


    this._wrapper = wrap;

    this._initJoystick();
    this._initHoldButtons();
    this._initModes();
    this._initParams();
    this._initUtils();
    this._initKeyboard();
    this._initMenu();
    this._updateStatus();
  }

  // ── Joystick ──────────────────────────────
  _initJoystick() {
    const s = this._state;
    const canvas = this._wrapper.querySelector("#sg-joy");
    const JR = 95, JC = 120;
    const axisEl = this._wrapper.querySelector("#sg-axis");

    const draw = (dx, dy, drag) => {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, 240, 240);
      ctx.strokeStyle = "#252a36"; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(JC, JC, JR, 0, 2 * _PI); ctx.stroke();
      ctx.setLineDash([4, 4]); ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(JC, JC - JR); ctx.lineTo(JC, JC + JR); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(JC - JR, JC); ctx.lineTo(JC + JR, JC); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = drag ? "#ffffff" : "#00c896";
      ctx.beginPath(); ctx.arc(JC + dx, JC + dy, 13, 0, 2 * _PI); ctx.fill();
    };
    draw(0, 0, false);
    this._drawJoy = draw;
    this._JR = JR; this._JC = JC;

    const move = (ex, ey) => {
      let dx = ex - JC, dy = ey - JC;
      const mag = Math.hypot(dx, dy);
      if (mag > JR) { dx *= JR / mag; dy *= JR / mag; }
      // let rx = 100 * dx / JR, ry = 100 * dy / JR;
      const sensitivity = 1;
      let rx = sensitivity * 100 * dx / JR;
      let ry = sensitivity * 100 * dy / JR;
      if (Math.abs(rx) < 12) rx = 0;
      if (Math.abs(ry) < 12) ry = 0;
      // s.rx = rx; s.ry = ry; s.stop = false;

      // Snap joystick to 8 directions: W, W+A, A, S+A, S, S+D, D, D+W
      // const magRaw = Math.hypot(rx, ry);

      // if (magRaw < 12) {
      //   rx = 0;
      //   ry = 0;
      // } else {
      //   const angle = Math.atan2(ry, rx);
      //   const snap = Math.round(angle / (Math.PI / 4)) * (Math.PI / 4);

      //   const mag = 100;   // always command full strength at snapped direction

      //   rx = mag * Math.cos(snap);
      //   ry = mag * Math.sin(snap);

      //   // Clean tiny floating errors
      //   if (Math.abs(rx) < 1e-6) rx = 0;
      //   if (Math.abs(ry) < 1e-6) ry = 0;
      // }

      // s.rx = rx;
      // s.ry = ry;
      // s.stop = false;

      // Snap joystick to 8 directions and 3 speed levels
      const magRaw = Math.hypot(rx, ry);

      if (magRaw < 12) {
        rx = 0;
        ry = 0;
      } else {
        const angle = Math.atan2(ry, rx);
        const snap = Math.round(angle / (Math.PI / 4)) * (Math.PI / 4);

        let mag;
        if (magRaw < 40) {
          mag = 33;      // low
        } else if (magRaw < 75) {
          mag = 66;      // mid
        } else {
          mag = 100;     // max
        }

        rx = mag * Math.cos(snap);
        ry = mag * Math.sin(snap);

        if (Math.abs(rx) < 1e-6) rx = 0;
        if (Math.abs(ry) < 1e-6) ry = 0;
      }

      s.rx = rx;
      s.ry = ry;
      s.stop = false;


      // draw(dx, dy, true);
      draw(JR * rx / 100, JR * ry / 100, true);
      axisEl.textContent = `rx=${rx >= 0 ? "+" : ""}${rx.toFixed(1)}  ry=${ry >= 0 ? "+" : ""}${ry.toFixed(1)}`;
      this._updateStatus();
    };

    this._centerJoy = () => {
      s.rx = 0; s.ry = 0;
      draw(0, 0, false);
      axisEl.textContent = "rx=+0.0  ry=+0.0";
      this._updateStatus();
    };

    canvas.addEventListener("mousedown",  e => { this._joyDragging = true;  move(e.offsetX, e.offsetY); });
    canvas.addEventListener("mousemove",  e => { if (this._joyDragging) move(e.offsetX, e.offsetY); });
    canvas.addEventListener("mouseup",    ()  => { this._joyDragging = false; this._centerJoy(); });
    canvas.addEventListener("mouseleave", ()  => { if (this._joyDragging) { this._joyDragging = false; this._centerJoy(); } });

    canvas.addEventListener("touchstart", e => {
      e.preventDefault();
      const r = canvas.getBoundingClientRect();
      move(e.touches[0].clientX - r.left, e.touches[0].clientY - r.top);
    }, { passive: false });
    canvas.addEventListener("touchmove", e => {
      e.preventDefault();
      const r = canvas.getBoundingClientRect();
      move(e.touches[0].clientX - r.left, e.touches[0].clientY - r.top);
    }, { passive: false });
    canvas.addEventListener("touchend", () => this._centerJoy());

    this._wrapper.querySelector("#sg-center").addEventListener("click", () => {
      this._centerJoy(); this._state.stop = true; this._updateStatus();
    });
  }

  // ── Hold buttons ─────────────────────────
  _initHoldButtons() {
    const s = this._state;
    const defs = [
      ["sg-srp", "self_right_positive"],
      ["sg-srn", "self_right_negative"],
      ["sg-tl",  "turn_left"],
      ["sg-tr",  "turn_right"],
    ];
    defs.forEach(([id, key]) => {
      const btn = this._wrapper.querySelector(`#${id}`);
      const press   = () => { s.rx = 0; s.ry = 0; s.stop = false; s[key] = true;  btn.classList.add("on");  this._updateStatus(); };
      const release = () => { s[key] = false; btn.classList.remove("on"); this._updateStatus(); };



      // btn.addEventListener("mousedown",  press);
      // btn.addEventListener("mouseup",    release);
      // // btn.addEventListener("mouseleave", release);
      // btn.addEventListener("touchstart", e => { e.preventDefault(); press(); }, { passive: false });
      // btn.addEventListener("touchend",   release);

      btn.addEventListener("pointerdown", e => {
        e.preventDefault();
        btn.setPointerCapture(e.pointerId);
        press();
      });

      btn.addEventListener("pointerup", e => {
        e.preventDefault();
        release();
      });

      btn.addEventListener("pointercancel", e => {
        e.preventDefault();
        release();


      });
    });
  }

  // ── Modes & vertical ─────────────────────
  _initModes() {
    const s = this._state;
    this._wrapper.querySelectorAll('input[name="sg-mode"]')
      .forEach(r => r.addEventListener("change", () => { s.mode = r.value; this._updateStatus(); }));
    this._wrapper.querySelector("#sg-sdn")
      .addEventListener("change", e => { s.sd_normal_mode = e.target.checked; this._updateStatus(); });
    this._wrapper.querySelector("#sg-ven")
      .addEventListener("change", e => { s.vertical_enabled = e.target.checked; this._updateStatus(); });
    this._wrapper.querySelectorAll('input[name="sg-vs"]')
      .forEach(r => r.addEventListener("change", () => { s.vertical_sign = parseFloat(r.value); this._updateStatus(); }));
  }

  // ── Parameter sliders ────────────────────
  _initParams() {
    const s = this._state;
    const boxContainer = this._wrapper.querySelector("#sg-pboxes");
    const slider       = this._wrapper.querySelector("#sg-pslider");
    const tooltip = this._wrapper.querySelector("#sg-tooltip");
    const valEl        = this._wrapper.querySelector("#sg-pval");

    this._paramSpecs.forEach((sp, i) => {
      const btn = document.createElement("button");
      btn.className = "sg-pb" + (i === 0 ? " sel" : "");
      btn.textContent = sp.label;
      btn.addEventListener("click", () => {
        this._selIdx = i;
        boxContainer.querySelectorAll(".sg-pb").forEach((b, j) => b.classList.toggle("sel", j === i));
        this._refreshSlider();
      });
      boxContainer.appendChild(btn);
    });

    // this._refreshSlider = () => {
    //   const sp = this._paramSpecs[this._selIdx];
    //   slider.min   = sp.lo;
    //   slider.max   = sp.hi;
    //   slider.step  = sp.step;
    //   slider.value = s[sp.key];
    //   valEl.textContent = Number(s[sp.key]).toFixed(sp.dec);
    // };

    this._refreshSlider = () => {
      const sp = this._paramSpecs[this._selIdx];

      slider.min   = sp.lo;
      slider.max   = sp.hi;
      slider.step  = sp.step;
      slider.value = s[sp.key];

      // valEl.textContent = Number(s[sp.key]).toFixed(sp.dec);

      // tooltip.textContent = Number(s[sp.key]).toFixed(sp.dec);

      let displayValue = Number(s[sp.key]).toFixed(sp.dec);

      if (sp.key === "STEER_MAX") {
        const v = Number(s[sp.key]);

        if (Math.abs(v - 0.5) < 1e-6) {
          displayValue = "BIG";
        } else if (Math.abs(v - 3.0) < 1e-6) {
          displayValue = "SMALL";
        } else {
          displayValue = "";
        }
      }

      valEl.textContent = displayValue;
      tooltip.textContent = displayValue;
      

      const pct =
        (Number(s[sp.key]) - sp.lo) /
        (sp.hi - sp.lo);

      tooltip.style.left = `${pct * 100}%`;
    };


    this._refreshSlider();

    // slider.addEventListener("input", () => {
    //   const sp  = this._paramSpecs[this._selIdx];
    //   const val = parseFloat(slider.value);
    //   s[sp.key] = val;
    //   valEl.textContent = val.toFixed(sp.dec);
    //   this._updateStatus();
    // });

    slider.addEventListener("input", () => {
      const sp  = this._paramSpecs[this._selIdx];
      const val = parseFloat(slider.value);

      s[sp.key] = val;

      // valEl.textContent = val.toFixed(sp.dec);
      // tooltip.textContent = val.toFixed(sp.dec);

      let displayValue = val.toFixed(sp.dec);

      if (sp.key === "STEER_MAX") {
        if (Math.abs(val - 0.5) < 1e-6) {
          displayValue = "BIG";
        } else if (Math.abs(val - 3.0) < 1e-6) {
          displayValue = "SMALL";
        } else {
          displayValue = "";
        }
      }

      valEl.textContent = displayValue;
      tooltip.textContent = displayValue;

      const pct =
          (val - parseFloat(slider.min)) /
          (parseFloat(slider.max) - parseFloat(slider.min));

      tooltip.style.left = `${pct * 100}%`;

      this._updateStatus();
    });


    const step = dir => {
      const sp  = this._paramSpecs[this._selIdx];
      s[sp.key] = _clamp(s[sp.key] + dir * sp.step, sp.lo, sp.hi);
      this._refreshSlider();
      this._updateStatus();
    };
    this._wrapper.querySelector("#sg-pminus").addEventListener("click", () => step(-1));
    this._wrapper.querySelector("#sg-pplus").addEventListener("click",  () => step(+1));
  }

  // ── Utility buttons ──────────────────────
  _initUtils() {
    const s = this._state;

    this._wrapper.querySelector("#sg-reset").addEventListener("click", () => {
      Object.assign(s, {
        amp_horz: 30, freq_horz: 1, vert_amp: 30, vert_freq: 1,
        vert_phase: 0, leg_phase_offset: 0, duty_offset: 0,
        STEER_MAX: STEER_MAX_DEFAULT,
        mode: MODE_NORMAL, sd_normal_mode: true,
        vertical_enabled: false, vertical_sign: -1.0,
      });
      this._wrapper.querySelectorAll('input[name="sg-mode"]').forEach(r => { r.checked = r.value === MODE_NORMAL; });
      this._wrapper.querySelector("#sg-sdn").checked = true;
      this._wrapper.querySelector("#sg-ven").checked = false;
      this._wrapper.querySelectorAll('input[name="sg-vs"]').forEach(r => { r.checked = r.value === "-1"; });
      this._refreshSlider();
      this._updateStatus();
    });

    this._wrapper.querySelector("#sg-stop").addEventListener("click", () => {
      s.stop = true; this._updateStatus();
    });
    this._wrapper.querySelector("#sg-resume").addEventListener("click", () => {
      s.stop = false; this._updateStatus();
    });
    this._wrapper.querySelector("#sg-copy").addEventListener("click", () => {
      navigator.clipboard.writeText(JSON.stringify(this.getParams(), null, 2)).catch(() => {});
    });
  }

  // ── WASD keyboard ────────────────────────
  _initKeyboard() {
    const s = this._state;
    const WASD = new Set(["w", "a", "s", "d"]);
    const DEBOUNCE_MS = 45;

    const applyKeys = () => {
      let rx = 0, ry = 0;
      if (this._pressed.has("w")) ry -= 100;
      if (this._pressed.has("s")) ry += 100;
      if (this._pressed.has("a")) rx -= 100;
      if (this._pressed.has("d")) rx += 100;
      const mag = Math.hypot(rx, ry);
      if (mag > 100) { rx *= 100 / mag; ry *= 100 / mag; }
      s.rx = rx; s.ry = ry;
      s.stop = !this._pressed.size && !Object.keys(this._releaseJobs).length;

      const dxp = this._JR * rx / 100, dyp = this._JR * ry / 100;
      this._drawJoy(dxp, dyp, this._pressed.size > 0);
      const axisEl = this._wrapper.querySelector("#sg-axis");
      axisEl.textContent = `rx=${rx >= 0 ? "+" : ""}${rx.toFixed(1)}  ry=${ry >= 0 ? "+" : ""}${ry.toFixed(1)}`;
      this._updateStatus();
    };

    document.addEventListener("keydown", e => {
      const k = e.key.toLowerCase();
      if (!WASD.has(k)) return;
      if (this._releaseJobs[k]) { clearTimeout(this._releaseJobs[k]); delete this._releaseJobs[k]; }
      if (!this._pressed.has(k)) { this._pressed.add(k); applyKeys(); }
    });

    // document.addEventListener("keyup", e => {
    //   const k = e.key.toLowerCase();
    //   if (!WASD.has(k)) return;
    //   if (this._releaseJobs[k]) clearTimeout(this._releaseJobs[k]);
    //   this._releaseJobs[k] = setTimeout(() => {
    //     delete this._releaseJobs[k];
    //     this._pressed.delete(k);
    //     applyKeys();
    //   }, DEBOUNCE_MS);
    // });


    document.addEventListener("keyup", e => {
      const k = e.key.toLowerCase();
      if (!WASD.has(k)) return;

      if (this._releaseJobs[k]) clearTimeout(this._releaseJobs[k]);

      this._releaseJobs[k] = setTimeout(() => {
        delete this._releaseJobs[k];
        this._pressed.delete(k);

        // Important: do not apply an intermediate W/A/S/D state
        // while other keys are still in their release debounce window.
        if (Object.keys(this._releaseJobs).length === 0) {
          applyKeys();
        }
      }, DEBOUNCE_MS);
    });

    document.addEventListener("blur", () => {
      Object.values(this._releaseJobs).forEach(t => clearTimeout(t));
      this._releaseJobs = {};
      this._pressed.clear();
      applyKeys();
    });
  }

  _initMenu() {
    const menu = this._wrapper.querySelector("#sg-menu");
    const full = this._wrapper.querySelector("#sg-full-controls");

    menu.addEventListener("change", () => {
      full.style.display =
        menu.value === "edit" ? "block" : "none";
    });
  }



  // ── Status bar ───────────────────────────
  _updateStatus() {
    const r = buildRadioRequest(this._state);
    let txt;
    if (r.stop)       txt = "⏸ Holding pose";
    else if (r.cmd)   txt = `● ${r.cmd}  ${r.dir === -1 ? "fwd" : "rev"}  ${r.tpc.toFixed(1)}s/cycle`;
    else              txt = "● Idle";
    this._wrapper.querySelector("#sg-status").textContent = txt;
  }
}
