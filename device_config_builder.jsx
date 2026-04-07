import { useState, useCallback, useRef } from "react";

// ── Tiny YAML serializer (no external deps) ──────────────────────────
function toYAML(obj, indent = 0) {
  const pad = "  ".repeat(indent);
  if (obj === null || obj === undefined) return "null";
  if (typeof obj === "boolean") return obj ? "true" : "false";
  if (typeof obj === "number") return String(obj);
  if (typeof obj === "string") {
    if (obj.includes("\n") || obj.includes(":") || obj.includes("#") || obj.includes("'") || obj.includes('"') || obj === "") {
      return `"${obj.replace(/"/g, '\\"')}"`;
    }
    return obj;
  }
  if (Array.isArray(obj)) {
    if (obj.length === 0) return "[]";
    // Simple arrays (numbers, strings) → inline
    if (obj.every(i => typeof i !== "object" || i === null)) {
      return `[${obj.map(i => toYAML(i, 0)).join(", ")}]`;
    }
    return "\n" + obj.map(item => `${pad}- ${toYAML(item, indent + 1).trimStart()}`).join("\n");
  }
  if (typeof obj === "object") {
    const entries = Object.entries(obj).filter(([, v]) => v !== undefined && v !== "");
    if (entries.length === 0) return "{}";
    return "\n" + entries.map(([k, v]) => {
      const val = toYAML(v, indent + 1);
      if (val.startsWith("\n")) return `${pad}${k}:${val}`;
      return `${pad}${k}: ${val}`;
    }).join("\n");
  }
  return String(obj);
}

// ── Constants ─────────────────────────────────────────────────────────
const ACTION_TYPES = ["hold", "unscrew", "pick_up", "pry"];
const HOLD_STRATEGIES = ["lateral_clamp", "top_down_clamp", "edge_clamp", "fixture_press"];
const APPROACH_AXES = ["+x", "-x", "+y", "-y", "+z", "-z"];
const SCREW_TYPES = ["phillips", "torx", "hex", "flat", "tri_wing", "pentalobe"];
const GRASP_TYPES = ["parallel", "pinch", "wide", "precision"];
const PRY_METHODS = ["edge_insertion", "corner_lift", "slot_lever"];
const ARMS = ["uf850", "xarm5"];

const COLORS = {
  bg: "#0a0f1a",
  surface: "#111827",
  surfaceHover: "#1a2235",
  border: "#1e2a3a",
  borderFocus: "#3b82f6",
  accent: "#3b82f6",
  accentDim: "#1e3a5f",
  accentGlow: "rgba(59,130,246,0.15)",
  danger: "#ef4444",
  dangerDim: "#3b1111",
  success: "#22c55e",
  successDim: "#0a2d14",
  warning: "#f59e0b",
  warningDim: "#3d2800",
  text: "#e2e8f0",
  textDim: "#64748b",
  textMuted: "#475569",
  white: "#ffffff",
};

// ── Styles ────────────────────────────────────────────────────────────
const S = {
  app: {
    minHeight: "100vh",
    background: COLORS.bg,
    color: COLORS.text,
    fontFamily: "'JetBrains Mono', 'SF Mono', 'Fira Code', monospace",
    fontSize: 13,
    lineHeight: 1.6,
  },
  header: {
    padding: "20px 24px",
    borderBottom: `1px solid ${COLORS.border}`,
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
    background: COLORS.surface,
  },
  headerTitle: {
    fontSize: 16,
    fontWeight: 700,
    color: COLORS.white,
    letterSpacing: "-0.02em",
  },
  headerSub: {
    fontSize: 11,
    color: COLORS.textDim,
    marginTop: 2,
    letterSpacing: "0.05em",
    textTransform: "uppercase",
  },
  main: {
    display: "flex",
    height: "calc(100vh - 65px)",
  },
  sidebar: {
    width: 260,
    minWidth: 260,
    borderRight: `1px solid ${COLORS.border}`,
    background: COLORS.surface,
    display: "flex",
    flexDirection: "column",
    overflow: "hidden",
  },
  sidebarSection: {
    padding: "12px 16px",
    borderBottom: `1px solid ${COLORS.border}`,
  },
  sidebarLabel: {
    fontSize: 10,
    fontWeight: 600,
    color: COLORS.textDim,
    letterSpacing: "0.1em",
    textTransform: "uppercase",
    marginBottom: 8,
  },
  content: {
    flex: 1,
    overflow: "auto",
    padding: 24,
  },
  card: {
    background: COLORS.surface,
    border: `1px solid ${COLORS.border}`,
    borderRadius: 8,
    marginBottom: 16,
    overflow: "hidden",
  },
  cardHeader: {
    padding: "12px 16px",
    borderBottom: `1px solid ${COLORS.border}`,
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
    cursor: "pointer",
    userSelect: "none",
  },
  cardTitle: {
    fontSize: 13,
    fontWeight: 600,
    color: COLORS.white,
  },
  cardBody: {
    padding: 16,
  },
  fieldRow: {
    display: "flex",
    gap: 12,
    marginBottom: 12,
    flexWrap: "wrap",
  },
  field: {
    flex: 1,
    minWidth: 140,
  },
  label: {
    display: "block",
    fontSize: 10,
    fontWeight: 600,
    color: COLORS.textDim,
    letterSpacing: "0.08em",
    textTransform: "uppercase",
    marginBottom: 4,
  },
  input: {
    width: "100%",
    padding: "7px 10px",
    background: COLORS.bg,
    border: `1px solid ${COLORS.border}`,
    borderRadius: 4,
    color: COLORS.text,
    fontSize: 12,
    fontFamily: "inherit",
    outline: "none",
    boxSizing: "border-box",
    transition: "border-color 0.15s",
  },
  select: {
    width: "100%",
    padding: "7px 10px",
    background: COLORS.bg,
    border: `1px solid ${COLORS.border}`,
    borderRadius: 4,
    color: COLORS.text,
    fontSize: 12,
    fontFamily: "inherit",
    outline: "none",
    boxSizing: "border-box",
    appearance: "none",
    cursor: "pointer",
  },
  textarea: {
    width: "100%",
    padding: "8px 10px",
    background: COLORS.bg,
    border: `1px solid ${COLORS.border}`,
    borderRadius: 4,
    color: COLORS.text,
    fontSize: 11,
    fontFamily: "inherit",
    outline: "none",
    boxSizing: "border-box",
    resize: "vertical",
    minHeight: 100,
    lineHeight: 1.5,
  },
  btn: {
    padding: "6px 14px",
    borderRadius: 4,
    border: "none",
    fontSize: 11,
    fontWeight: 600,
    fontFamily: "inherit",
    cursor: "pointer",
    transition: "all 0.15s",
    letterSpacing: "0.03em",
  },
  btnPrimary: {
    background: COLORS.accent,
    color: COLORS.white,
  },
  btnDanger: {
    background: COLORS.dangerDim,
    color: COLORS.danger,
    border: `1px solid ${COLORS.danger}33`,
  },
  btnGhost: {
    background: "transparent",
    color: COLORS.textDim,
    border: `1px solid ${COLORS.border}`,
  },
  btnSuccess: {
    background: COLORS.successDim,
    color: COLORS.success,
    border: `1px solid ${COLORS.success}33`,
  },
  tag: {
    display: "inline-flex",
    alignItems: "center",
    gap: 4,
    padding: "2px 8px",
    borderRadius: 3,
    fontSize: 10,
    fontWeight: 600,
    letterSpacing: "0.05em",
    textTransform: "uppercase",
  },
  tagAction: (type) => {
    const colors = {
      hold: { bg: "#1e3a5f", color: "#60a5fa" },
      unscrew: { bg: "#3d2800", color: "#fbbf24" },
      pick_up: { bg: "#0a2d14", color: "#4ade80" },
      pry: { bg: "#3b1130", color: "#f472b6" },
    };
    const c = colors[type] || { bg: COLORS.border, color: COLORS.textDim };
    return { ...{ padding: "2px 8px", borderRadius: 3, fontSize: 10, fontWeight: 600, letterSpacing: "0.05em", textTransform: "uppercase" }, background: c.bg, color: c.color };
  },
  emptyState: {
    display: "flex",
    flexDirection: "column",
    alignItems: "center",
    justifyContent: "center",
    padding: "60px 24px",
    color: COLORS.textDim,
    textAlign: "center",
  },
  stepItem: {
    display: "flex",
    alignItems: "flex-start",
    gap: 10,
    padding: "8px 12px",
    borderRadius: 4,
    cursor: "pointer",
    transition: "background 0.1s",
    marginBottom: 2,
  },
  stepNum: {
    width: 20,
    height: 20,
    borderRadius: "50%",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    fontSize: 10,
    fontWeight: 700,
    flexShrink: 0,
    marginTop: 1,
  },
  preview: {
    position: "fixed",
    top: 0,
    right: 0,
    bottom: 0,
    width: 480,
    background: COLORS.surface,
    borderLeft: `1px solid ${COLORS.border}`,
    zIndex: 100,
    display: "flex",
    flexDirection: "column",
    boxShadow: "-8px 0 32px rgba(0,0,0,0.4)",
  },
  previewHeader: {
    padding: "14px 20px",
    borderBottom: `1px solid ${COLORS.border}`,
    display: "flex",
    alignItems: "center",
    justifyContent: "space-between",
  },
  previewCode: {
    flex: 1,
    overflow: "auto",
    padding: 20,
    fontSize: 11,
    lineHeight: 1.7,
    color: "#93c5fd",
    whiteSpace: "pre-wrap",
    wordBreak: "break-all",
  },
  modal: {
    position: "fixed",
    inset: 0,
    background: "rgba(0,0,0,0.7)",
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    zIndex: 200,
  },
  modalContent: {
    background: COLORS.surface,
    border: `1px solid ${COLORS.border}`,
    borderRadius: 8,
    padding: 24,
    width: 500,
    maxHeight: "80vh",
    overflow: "auto",
  },
  componentTag: {
    display: "inline-flex",
    alignItems: "center",
    gap: 6,
    padding: "4px 10px",
    borderRadius: 4,
    fontSize: 11,
    background: COLORS.bg,
    border: `1px solid ${COLORS.border}`,
    marginRight: 6,
    marginBottom: 6,
    cursor: "default",
  },
};

// ── Helper components ─────────────────────────────────────────────────
function Field({ label, children, style }) {
  return (
    <div style={{ ...S.field, ...style }}>
      <label style={S.label}>{label}</label>
      {children}
    </div>
  );
}

function Input({ value, onChange, type = "text", placeholder, step, min, max, style }) {
  return (
    <input
      type={type}
      value={value ?? ""}
      onChange={e => onChange(type === "number" ? (e.target.value === "" ? "" : Number(e.target.value)) : e.target.value)}
      placeholder={placeholder}
      step={step}
      min={min}
      max={max}
      style={{ ...S.input, ...style }}
      onFocus={e => (e.target.style.borderColor = COLORS.borderFocus)}
      onBlur={e => (e.target.style.borderColor = COLORS.border)}
    />
  );
}

function Select({ value, onChange, options, placeholder }) {
  return (
    <select value={value || ""} onChange={e => onChange(e.target.value)} style={S.select}>
      {placeholder && <option value="">{placeholder}</option>}
      {options.map(o => (
        <option key={o} value={o}>{o}</option>
      ))}
    </select>
  );
}

function Btn({ children, variant = "ghost", onClick, style, disabled }) {
  const variants = { primary: S.btnPrimary, danger: S.btnDanger, ghost: S.btnGhost, success: S.btnSuccess };
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      style={{ ...S.btn, ...variants[variant], opacity: disabled ? 0.4 : 1, ...style }}
    >
      {children}
    </button>
  );
}

// ── Default action params ─────────────────────────────────────────────
function defaultHold() {
  return {
    strategy: "lateral_clamp",
    grip_width_mm: 85,
    approach_axis: "+z",
    stabilization_force_n: 15,
    arm: "uf850",
    position_offset: { x: 0, y: 0, z: 0 },
    orientation_rpy: { roll: 0, pitch: 0, yaw: 0 },
    notes: "",
  };
}

function defaultUnscrew() {
  return {
    target_component: "",
    screw_type: "phillips",
    screw_count: 1,
    screw_positions: [],
    engagement_depth_mm: 3,
    initial_torque_nm: 0.5,
    max_torque_nm: 1.5,
    rotation_speed_rpm: 60,
    arm: "xarm5",
    use_ft300: true,
    use_endoscopy_cam: true,
    notes: "",
  };
}

function defaultPickUp() {
  return {
    target_component: "",
    grasp_type: "parallel",
    grip_width_mm: 50,
    approach_axis: "+z",
    lift_height_mm: 50,
    lift_speed_mm_s: 20,
    arm: "uf850",
    grasp_points: [],
    notes: "",
  };
}

function defaultPry() {
  return {
    target_component: "",
    method: "edge_insertion",
    insertion_depth_mm: 2,
    pry_force_limit_n: 10,
    approach_axis: "+x",
    arm: "xarm5",
    use_ft300: true,
    notes: "",
  };
}

const DEFAULT_CREATORS = { hold: defaultHold, unscrew: defaultUnscrew, pick_up: defaultPickUp, pry: defaultPry };

function defaultDevice() {
  return {
    device_class: "",
    device_model: "",
    dimensions: { length_mm: 0, width_mm: 0, height_mm: 0 },
    material: "",
    fixturing: "gripper_only",
    notes: "",
  };
}

// ── Vision Import Modal ───────────────────────────────────────────────
function VisionImportModal({ onClose, onImport }) {
  const [mode, setMode] = useState("paste");
  const [jsonText, setJsonText] = useState("");
  const [manualComponents, setManualComponents] = useState([{ id: "", class: "", status: "present", loc: { x: 0, y: 0, z: 0 } }]);
  const [error, setError] = useState("");

  const handlePasteImport = () => {
    try {
      const data = JSON.parse(jsonText);
      const components = data.components || data.detections || (Array.isArray(data) ? data : []);
      if (components.length === 0) { setError("No components found in JSON"); return; }
      onImport(components);
      onClose();
    } catch (e) {
      setError("Invalid JSON: " + e.message);
    }
  };

  const addManualComp = () => setManualComponents(p => [...p, { id: "", class: "", status: "present", loc: { x: 0, y: 0, z: 0 } }]);
  const updateManualComp = (i, field, val) => {
    setManualComponents(p => p.map((c, j) => j === i ? (field.includes(".") ? { ...c, loc: { ...c.loc, [field.split(".")[1]]: val } } : { ...c, [field]: val }) : c));
  };
  const removeManualComp = (i) => setManualComponents(p => p.filter((_, j) => j !== i));

  const handleManualImport = () => {
    const valid = manualComponents.filter(c => c.id);
    if (valid.length === 0) { setError("Add at least one component with an ID"); return; }
    onImport(valid);
    onClose();
  };

  return (
    <div style={S.modal} onClick={onClose}>
      <div style={S.modalContent} onClick={e => e.stopPropagation()}>
        <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 16 }}>
          <span style={{ fontSize: 14, fontWeight: 700, color: COLORS.white }}>Import Vision Data</span>
          <Btn onClick={onClose}>✕</Btn>
        </div>

        <div style={{ display: "flex", gap: 8, marginBottom: 16 }}>
          <Btn variant={mode === "paste" ? "primary" : "ghost"} onClick={() => setMode("paste")}>Paste JSON</Btn>
          <Btn variant={mode === "manual" ? "primary" : "ghost"} onClick={() => setMode("manual")}>Manual Entry</Btn>
        </div>

        {mode === "paste" ? (
          <>
            <label style={{ ...S.label, marginBottom: 6 }}>RF-DETR / Vision JSON Output</label>
            <textarea
              style={S.textarea}
              rows={8}
              value={jsonText}
              onChange={e => { setJsonText(e.target.value); setError(""); }}
              placeholder={'{\n  "components": [\n    {"id": "screw_1", "class": "screw", "status": "present", "loc": [0.12, 0.05, 0.03]},\n    {"id": "lid_main", "class": "lid", "status": "present", "loc": [0.15, 0.10, 0.02]}\n  ]\n}'}
            />
            {error && <div style={{ color: COLORS.danger, fontSize: 11, marginTop: 6 }}>{error}</div>}
            <Btn variant="primary" onClick={handlePasteImport} style={{ marginTop: 12 }}>Import Components</Btn>
          </>
        ) : (
          <>
            {manualComponents.map((comp, i) => (
              <div key={i} style={{ ...S.card, marginBottom: 8 }}>
                <div style={{ padding: 12 }}>
                  <div style={S.fieldRow}>
                    <Field label="Component ID" style={{ minWidth: 120 }}>
                      <Input value={comp.id} onChange={v => updateManualComp(i, "id", v)} placeholder="e.g. screw_1" />
                    </Field>
                    <Field label="Class" style={{ minWidth: 100 }}>
                      <Input value={comp.class} onChange={v => updateManualComp(i, "class", v)} placeholder="e.g. screw" />
                    </Field>
                    <Field label="Status" style={{ minWidth: 80 }}>
                      <Select value={comp.status} onChange={v => updateManualComp(i, "status", v)} options={["present", "removed", "damaged", "stripped", "occluded"]} />
                    </Field>
                  </div>
                  <div style={S.fieldRow}>
                    <Field label="X (m)"><Input type="number" step={0.001} value={comp.loc.x} onChange={v => updateManualComp(i, "loc.x", v)} /></Field>
                    <Field label="Y (m)"><Input type="number" step={0.001} value={comp.loc.y} onChange={v => updateManualComp(i, "loc.y", v)} /></Field>
                    <Field label="Z (m)"><Input type="number" step={0.001} value={comp.loc.z} onChange={v => updateManualComp(i, "loc.z", v)} /></Field>
                    <div style={{ display: "flex", alignItems: "flex-end", paddingBottom: 2 }}>
                      <Btn variant="danger" onClick={() => removeManualComp(i)}>✕</Btn>
                    </div>
                  </div>
                </div>
              </div>
            ))}
            <div style={{ display: "flex", gap: 8, marginTop: 8 }}>
              <Btn variant="ghost" onClick={addManualComp}>+ Add Component</Btn>
              <Btn variant="primary" onClick={handleManualImport}>Import All</Btn>
            </div>
            {error && <div style={{ color: COLORS.danger, fontSize: 11, marginTop: 6 }}>{error}</div>}
          </>
        )}
      </div>
    </div>
  );
}

// ── Screw Position Editor ─────────────────────────────────────────────
function ScrewPositions({ positions, onChange }) {
  const add = () => onChange([...positions, { id: `screw_${positions.length + 1}`, x: 0, y: 0, z: 0 }]);
  const update = (i, field, val) => onChange(positions.map((p, j) => j === i ? { ...p, [field]: val } : p));
  const remove = (i) => onChange(positions.filter((_, j) => j !== i));

  return (
    <div>
      {positions.map((pos, i) => (
        <div key={i} style={{ ...S.fieldRow, alignItems: "flex-end" }}>
          <Field label={i === 0 ? "Screw ID" : ""} style={{ minWidth: 100 }}>
            <Input value={pos.id} onChange={v => update(i, "id", v)} />
          </Field>
          <Field label={i === 0 ? "X (m)" : ""} style={{ minWidth: 70 }}>
            <Input type="number" step={0.001} value={pos.x} onChange={v => update(i, "x", v)} />
          </Field>
          <Field label={i === 0 ? "Y (m)" : ""} style={{ minWidth: 70 }}>
            <Input type="number" step={0.001} value={pos.y} onChange={v => update(i, "y", v)} />
          </Field>
          <Field label={i === 0 ? "Z (m)" : ""} style={{ minWidth: 70 }}>
            <Input type="number" step={0.001} value={pos.z} onChange={v => update(i, "z", v)} />
          </Field>
          <Btn variant="danger" onClick={() => remove(i)} style={{ marginBottom: 12 }}>✕</Btn>
        </div>
      ))}
      <Btn variant="ghost" onClick={add} style={{ marginTop: 4 }}>+ Add Screw Position</Btn>
    </div>
  );
}

// ── Grasp Points Editor ───────────────────────────────────────────────
function GraspPoints({ points, onChange }) {
  const add = () => onChange([...points, { x: 0, y: 0, z: 0 }]);
  const update = (i, field, val) => onChange(points.map((p, j) => j === i ? { ...p, [field]: val } : p));
  const remove = (i) => onChange(points.filter((_, j) => j !== i));

  return (
    <div>
      {points.map((pt, i) => (
        <div key={i} style={{ ...S.fieldRow, alignItems: "flex-end" }}>
          <Field label={i === 0 ? "X (m)" : ""} style={{ minWidth: 70 }}>
            <Input type="number" step={0.001} value={pt.x} onChange={v => update(i, "x", v)} />
          </Field>
          <Field label={i === 0 ? "Y (m)" : ""} style={{ minWidth: 70 }}>
            <Input type="number" step={0.001} value={pt.y} onChange={v => update(i, "y", v)} />
          </Field>
          <Field label={i === 0 ? "Z (m)" : ""} style={{ minWidth: 70 }}>
            <Input type="number" step={0.001} value={pt.z} onChange={v => update(i, "z", v)} />
          </Field>
          <Btn variant="danger" onClick={() => remove(i)} style={{ marginBottom: 12 }}>✕</Btn>
        </div>
      ))}
      <Btn variant="ghost" onClick={add} style={{ marginTop: 4 }}>+ Add Grasp Point</Btn>
    </div>
  );
}

// ── Action Editor ─────────────────────────────────────────────────────
function HoldEditor({ params, onChange }) {
  const u = (field, val) => onChange({ ...params, [field]: val });
  const uPos = (axis, val) => onChange({ ...params, position_offset: { ...params.position_offset, [axis]: val } });
  const uOri = (axis, val) => onChange({ ...params, orientation_rpy: { ...params.orientation_rpy, [axis]: val } });

  return (
    <>
      <div style={S.fieldRow}>
        <Field label="Hold Strategy"><Select value={params.strategy} onChange={v => u("strategy", v)} options={HOLD_STRATEGIES} /></Field>
        <Field label="Grip Width (mm)"><Input type="number" value={params.grip_width_mm} onChange={v => u("grip_width_mm", v)} min={0} max={160} /></Field>
        <Field label="Approach Axis"><Select value={params.approach_axis} onChange={v => u("approach_axis", v)} options={APPROACH_AXES} /></Field>
      </div>
      <div style={S.fieldRow}>
        <Field label="Stabilization Force (N)"><Input type="number" value={params.stabilization_force_n} onChange={v => u("stabilization_force_n", v)} step={0.5} /></Field>
        <Field label="Assigned Arm"><Select value={params.arm} onChange={v => u("arm", v)} options={ARMS} /></Field>
      </div>
      <div style={{ ...S.label, marginBottom: 6, marginTop: 4 }}>Position Offset (m) — relative to device centroid</div>
      <div style={S.fieldRow}>
        <Field label="X"><Input type="number" step={0.001} value={params.position_offset.x} onChange={v => uPos("x", v)} /></Field>
        <Field label="Y"><Input type="number" step={0.001} value={params.position_offset.y} onChange={v => uPos("y", v)} /></Field>
        <Field label="Z"><Input type="number" step={0.001} value={params.position_offset.z} onChange={v => uPos("z", v)} /></Field>
      </div>
      <div style={{ ...S.label, marginBottom: 6, marginTop: 4 }}>Orientation (degrees) — Roll, Pitch, Yaw</div>
      <div style={S.fieldRow}>
        <Field label="Roll"><Input type="number" step={1} value={params.orientation_rpy.roll} onChange={v => uOri("roll", v)} /></Field>
        <Field label="Pitch"><Input type="number" step={1} value={params.orientation_rpy.pitch} onChange={v => uOri("pitch", v)} /></Field>
        <Field label="Yaw"><Input type="number" step={1} value={params.orientation_rpy.yaw} onChange={v => uOri("yaw", v)} /></Field>
      </div>
      {params.arm === "uf850" && (
        <div style={{ fontSize: 10, color: COLORS.warning, marginTop: 4, padding: "4px 8px", background: COLORS.warningDim, borderRadius: 3 }}>
          ⚠ UF850 has no FT300 — force control uses current-based estimation only
        </div>
      )}
      <Field label="Notes" style={{ marginTop: 8 }}>
        <textarea style={{ ...S.textarea, minHeight: 40 }} value={params.notes} onChange={e => u("notes", e.target.value)} placeholder="e.g. Hold HDD sideways so top lid is accessible" />
      </Field>
    </>
  );
}

function UnscrewEditor({ params, onChange, components }) {
  const u = (field, val) => onChange({ ...params, [field]: val });

  return (
    <>
      <div style={S.fieldRow}>
        <Field label="Target Component">
          {components.length > 0 ? (
            <Select value={params.target_component} onChange={v => u("target_component", v)} options={components.map(c => c.id)} placeholder="Select component..." />
          ) : (
            <Input value={params.target_component} onChange={v => u("target_component", v)} placeholder="e.g. lid_screws" />
          )}
        </Field>
        <Field label="Screw Type"><Select value={params.screw_type} onChange={v => u("screw_type", v)} options={SCREW_TYPES} /></Field>
        <Field label="Screw Count"><Input type="number" value={params.screw_count} onChange={v => u("screw_count", v)} min={1} /></Field>
      </div>
      <div style={S.fieldRow}>
        <Field label="Engagement Depth (mm)"><Input type="number" value={params.engagement_depth_mm} onChange={v => u("engagement_depth_mm", v)} step={0.5} /></Field>
        <Field label="Initial Torque (Nm)"><Input type="number" value={params.initial_torque_nm} onChange={v => u("initial_torque_nm", v)} step={0.1} /></Field>
        <Field label="Max Torque (Nm)"><Input type="number" value={params.max_torque_nm} onChange={v => u("max_torque_nm", v)} step={0.1} /></Field>
      </div>
      <div style={S.fieldRow}>
        <Field label="Rotation Speed (RPM)"><Input type="number" value={params.rotation_speed_rpm} onChange={v => u("rotation_speed_rpm", v)} /></Field>
        <Field label="Assigned Arm"><Select value={params.arm} onChange={v => u("arm", v)} options={ARMS} /></Field>
      </div>
      <div style={{ display: "flex", gap: 12, marginBottom: 8 }}>
        <label style={{ display: "flex", alignItems: "center", gap: 6, fontSize: 11, color: COLORS.textDim, cursor: "pointer" }}>
          <input type="checkbox" checked={params.use_ft300} onChange={e => u("use_ft300", e.target.checked)} /> Use FT300
        </label>
        <label style={{ display: "flex", alignItems: "center", gap: 6, fontSize: 11, color: COLORS.textDim, cursor: "pointer" }}>
          <input type="checkbox" checked={params.use_endoscopy_cam} onChange={e => u("use_endoscopy_cam", e.target.checked)} /> Use Endoscopy Cam
        </label>
      </div>
      {params.arm === "uf850" && params.use_ft300 && (
        <div style={{ fontSize: 10, color: COLORS.danger, marginBottom: 8, padding: "4px 8px", background: COLORS.dangerDim, borderRadius: 3 }}>
          ✕ FT300 is only on xArm5 — disable or switch arm
        </div>
      )}
      <div style={{ ...S.label, marginBottom: 6 }}>Screw Positions</div>
      <ScrewPositions positions={params.screw_positions} onChange={v => u("screw_positions", v)} />
      <Field label="Notes" style={{ marginTop: 8 }}>
        <textarea style={{ ...S.textarea, minHeight: 40 }} value={params.notes} onChange={e => u("notes", e.target.value)} placeholder="e.g. Torx T8 screws, may be tight on older units" />
      </Field>
    </>
  );
}

function PickUpEditor({ params, onChange, components }) {
  const u = (field, val) => onChange({ ...params, [field]: val });

  return (
    <>
      <div style={S.fieldRow}>
        <Field label="Target Component">
          {components.length > 0 ? (
            <Select value={params.target_component} onChange={v => u("target_component", v)} options={components.map(c => c.id)} placeholder="Select component..." />
          ) : (
            <Input value={params.target_component} onChange={v => u("target_component", v)} placeholder="e.g. pcb_main" />
          )}
        </Field>
        <Field label="Grasp Type"><Select value={params.grasp_type} onChange={v => u("grasp_type", v)} options={GRASP_TYPES} /></Field>
        <Field label="Grip Width (mm)"><Input type="number" value={params.grip_width_mm} onChange={v => u("grip_width_mm", v)} min={0} max={160} /></Field>
      </div>
      <div style={S.fieldRow}>
        <Field label="Approach Axis"><Select value={params.approach_axis} onChange={v => u("approach_axis", v)} options={APPROACH_AXES} /></Field>
        <Field label="Lift Height (mm)"><Input type="number" value={params.lift_height_mm} onChange={v => u("lift_height_mm", v)} /></Field>
        <Field label="Lift Speed (mm/s)"><Input type="number" value={params.lift_speed_mm_s} onChange={v => u("lift_speed_mm_s", v)} /></Field>
      </div>
      <div style={S.fieldRow}>
        <Field label="Assigned Arm"><Select value={params.arm} onChange={v => u("arm", v)} options={ARMS} /></Field>
      </div>
      <div style={{ ...S.label, marginBottom: 6 }}>Grasp Points (relative to component)</div>
      <GraspPoints points={params.grasp_points} onChange={v => u("grasp_points", v)} />
      <Field label="Notes" style={{ marginTop: 8 }}>
        <textarea style={{ ...S.textarea, minHeight: 40 }} value={params.notes} onChange={e => u("notes", e.target.value)} placeholder="e.g. Lift PCB gently, check for hidden ribbon cables" />
      </Field>
    </>
  );
}

function PryEditor({ params, onChange, components }) {
  const u = (field, val) => onChange({ ...params, [field]: val });

  return (
    <>
      <div style={S.fieldRow}>
        <Field label="Target Component">
          {components.length > 0 ? (
            <Select value={params.target_component} onChange={v => u("target_component", v)} options={components.map(c => c.id)} placeholder="Select component..." />
          ) : (
            <Input value={params.target_component} onChange={v => u("target_component", v)} placeholder="e.g. bottom_panel" />
          )}
        </Field>
        <Field label="Pry Method"><Select value={params.method} onChange={v => u("method", v)} options={PRY_METHODS} /></Field>
        <Field label="Assigned Arm"><Select value={params.arm} onChange={v => u("arm", v)} options={ARMS} /></Field>
      </div>
      <div style={S.fieldRow}>
        <Field label="Insertion Depth (mm)"><Input type="number" value={params.insertion_depth_mm} onChange={v => u("insertion_depth_mm", v)} step={0.5} /></Field>
        <Field label="Force Limit (N)"><Input type="number" value={params.pry_force_limit_n} onChange={v => u("pry_force_limit_n", v)} step={0.5} /></Field>
        <Field label="Approach Axis"><Select value={params.approach_axis} onChange={v => u("approach_axis", v)} options={APPROACH_AXES} /></Field>
      </div>
      <label style={{ display: "flex", alignItems: "center", gap: 6, fontSize: 11, color: COLORS.textDim, cursor: "pointer", marginBottom: 8 }}>
        <input type="checkbox" checked={params.use_ft300} onChange={e => u("use_ft300", e.target.checked)} /> Use FT300 force feedback
      </label>
      {params.arm === "uf850" && params.use_ft300 && (
        <div style={{ fontSize: 10, color: COLORS.danger, marginBottom: 8, padding: "4px 8px", background: COLORS.dangerDim, borderRadius: 3 }}>
          ✕ FT300 is only on xArm5 — disable or switch arm
        </div>
      )}
      <Field label="Notes" style={{ marginTop: 8 }}>
        <textarea style={{ ...S.textarea, minHeight: 40 }} value={params.notes} onChange={e => u("notes", e.target.value)} placeholder="e.g. Start prying from the front edge near the IO ports" />
      </Field>
    </>
  );
}

const ACTION_EDITORS = { hold: HoldEditor, unscrew: UnscrewEditor, pick_up: PickUpEditor, pry: PryEditor };

// ── Main App ──────────────────────────────────────────────────────────
export default function DeviceConfigBuilder() {
  const [device, setDevice] = useState(defaultDevice());
  const [steps, setSteps] = useState([]);
  const [components, setComponents] = useState([]);
  const [selectedStep, setSelectedStep] = useState(null);
  const [showVisionModal, setShowVisionModal] = useState(false);
  const [showPreview, setShowPreview] = useState(false);
  const [expandedSteps, setExpandedSteps] = useState({});
  const [copied, setCopied] = useState(false);
  const downloadRef = useRef(null);

  const addStep = (type) => {
    const newStep = {
      id: `step_${Date.now()}`,
      order: steps.length + 1,
      action: type,
      label: `${type}_${steps.filter(s => s.action === type).length + 1}`,
      params: DEFAULT_CREATORS[type](),
    };
    setSteps(prev => [...prev, newStep]);
    setSelectedStep(newStep.id);
    setExpandedSteps(prev => ({ ...prev, [newStep.id]: true }));
  };

  const updateStep = (id, params) => {
    setSteps(prev => prev.map(s => s.id === id ? { ...s, params } : s));
  };

  const updateStepLabel = (id, label) => {
    setSteps(prev => prev.map(s => s.id === id ? { ...s, label } : s));
  };

  const removeStep = (id) => {
    setSteps(prev => prev.filter(s => s.id !== id).map((s, i) => ({ ...s, order: i + 1 })));
    if (selectedStep === id) setSelectedStep(null);
  };

  const moveStep = (id, direction) => {
    setSteps(prev => {
      const idx = prev.findIndex(s => s.id === id);
      if ((direction === -1 && idx === 0) || (direction === 1 && idx === prev.length - 1)) return prev;
      const newSteps = [...prev];
      [newSteps[idx], newSteps[idx + direction]] = [newSteps[idx + direction], newSteps[idx]];
      return newSteps.map((s, i) => ({ ...s, order: i + 1 }));
    });
  };

  const toggleExpand = (id) => {
    setExpandedSteps(prev => ({ ...prev, [id]: !prev[id] }));
  };

  const handleVisionImport = (importedComponents) => {
    const normalized = importedComponents.map(c => ({
      id: c.id || c.name || `comp_${Math.random().toString(36).substr(2, 6)}`,
      class: c.class || c.type || c.label || "unknown",
      status: c.status || "present",
      loc: Array.isArray(c.loc)
        ? { x: c.loc[0] || 0, y: c.loc[1] || 0, z: c.loc[2] || 0 }
        : (c.loc || { x: 0, y: 0, z: 0 }),
      bbox: c.bbox || c.bounding_box || null,
      confidence: c.confidence || c.conf || null,
      area: c.area || c.mask_area || null,
    }));
    setComponents(prev => [...prev, ...normalized]);
  };

  const removeComponent = (id) => {
    setComponents(prev => prev.filter(c => c.id !== id));
  };

  // ── Build config object ──
  const buildConfig = useCallback(() => {
    const config = {
      device: {
        class: device.device_class,
        model: device.device_model,
        dimensions_mm: {
          length: device.dimensions.length_mm,
          width: device.dimensions.width_mm,
          height: device.dimensions.height_mm,
        },
        material: device.material,
        fixturing: device.fixturing,
        notes: device.notes,
      },
      components: components.map(c => ({
        id: c.id,
        class: c.class,
        status: c.status,
        position_m: [c.loc.x, c.loc.y, c.loc.z],
        ...(c.confidence ? { confidence: c.confidence } : {}),
        ...(c.area ? { mask_area: c.area } : {}),
      })),
      disassembly_sequence: steps.map(s => ({
        step: s.order,
        label: s.label,
        action: s.action,
        parameters: s.params,
      })),
      metadata: {
        created: new Date().toISOString(),
        hardware: {
          manipulation_arm: "uf850_rg6",
          tooling_arm: "xarm5_ft300_screwdriver_endoscopy",
          global_camera: "intel_realsense_d455",
        },
        framework_version: "1.0",
      },
    };
    return config;
  }, [device, components, steps]);

  const yamlOutput = useCallback(() => {
    const config = buildConfig();
    return `# Device Configuration File\n# Generated by Config Builder\n# ${new Date().toISOString()}\n${toYAML(config).trim()}`;
  }, [buildConfig]);

  const handleExport = () => {
    const yaml = yamlOutput();
    const blob = new Blob([yaml], { type: "text/yaml" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${device.device_class || "device"}_config.yaml`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const handleCopy = () => {
    navigator.clipboard.writeText(yamlOutput()).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    });
  };

  const warnings = [];
  steps.forEach(s => {
    if ((s.action === "unscrew" || s.action === "pry") && s.params.arm === "uf850" && s.params.use_ft300) {
      warnings.push(`Step ${s.order} "${s.label}": FT300 assigned to UF850 but sensor is on xArm5`);
    }
    if (s.action === "hold" && s.params.grip_width_mm > 160) {
      warnings.push(`Step ${s.order} "${s.label}": Grip width ${s.params.grip_width_mm}mm exceeds RG6 max stroke (160mm)`);
    }
    if (s.action === "pick_up" && s.params.grip_width_mm > 160) {
      warnings.push(`Step ${s.order} "${s.label}": Grip width ${s.params.grip_width_mm}mm exceeds RG6 max stroke (160mm)`);
    }
  });

  return (
    <div style={S.app}>
      {/* Header */}
      <div style={S.header}>
        <div>
          <div style={S.headerTitle}>Device Config Builder</div>
          <div style={S.headerSub}>Disassembly Action Configuration Tool</div>
        </div>
        <div style={{ display: "flex", gap: 8, alignItems: "center" }}>
          {warnings.length > 0 && (
            <span style={{ ...S.tag, background: COLORS.warningDim, color: COLORS.warning }}>
              ⚠ {warnings.length} warning{warnings.length > 1 ? "s" : ""}
            </span>
          )}
          <Btn variant="ghost" onClick={() => setShowPreview(!showPreview)}>
            {showPreview ? "Hide" : "Preview"} YAML
          </Btn>
          <Btn variant="success" onClick={handleExport} disabled={steps.length === 0}>
            Export YAML
          </Btn>
        </div>
      </div>

      <div style={S.main}>
        {/* Sidebar */}
        <div style={S.sidebar}>
          {/* Device Info */}
          <div style={S.sidebarSection}>
            <div style={S.sidebarLabel}>Device</div>
            <Input value={device.device_class} onChange={v => setDevice(p => ({ ...p, device_class: v }))} placeholder="e.g. hdd, mini_pc, laptop" style={{ marginBottom: 6 }} />
            <Input value={device.device_model} onChange={v => setDevice(p => ({ ...p, device_model: v }))} placeholder="e.g. WD_Blue_1TB" />
          </div>

          {/* Components */}
          <div style={{ ...S.sidebarSection, flex: 1, overflow: "auto" }}>
            <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 8 }}>
              <div style={S.sidebarLabel}>Components ({components.length})</div>
              <Btn variant="primary" onClick={() => setShowVisionModal(true)} style={{ fontSize: 10, padding: "3px 8px" }}>
                + Import
              </Btn>
            </div>
            {components.length === 0 ? (
              <div style={{ fontSize: 11, color: COLORS.textMuted, padding: "8px 0" }}>
                No components. Import from vision or add manually.
              </div>
            ) : (
              <div style={{ display: "flex", flexWrap: "wrap" }}>
                {components.map(c => (
                  <div key={c.id} style={S.componentTag}>
                    <span style={{ color: COLORS.accent, fontWeight: 600 }}>{c.class}</span>
                    <span style={{ color: COLORS.textDim }}>{c.id}</span>
                    <span onClick={() => removeComponent(c.id)} style={{ cursor: "pointer", color: COLORS.textMuted, marginLeft: 2 }}>✕</span>
                  </div>
                ))}
              </div>
            )}
          </div>

          {/* Add Actions */}
          <div style={S.sidebarSection}>
            <div style={S.sidebarLabel}>Add Action Step</div>
            <div style={{ display: "flex", flexWrap: "wrap", gap: 4 }}>
              {ACTION_TYPES.map(type => (
                <button
                  key={type}
                  onClick={() => addStep(type)}
                  style={{ ...S.tagAction(type), cursor: "pointer", border: "none", padding: "5px 10px", fontSize: 11 }}
                >
                  + {type}
                </button>
              ))}
            </div>
          </div>

          {/* Step List */}
          <div style={{ flex: 1, overflow: "auto", padding: "8px 0" }}>
            <div style={{ ...S.sidebarLabel, padding: "0 16px", marginBottom: 4 }}>
              Sequence ({steps.length} steps)
            </div>
            {steps.map(step => (
              <div
                key={step.id}
                style={{
                  ...S.stepItem,
                  background: selectedStep === step.id ? COLORS.accentGlow : "transparent",
                  borderLeft: selectedStep === step.id ? `2px solid ${COLORS.accent}` : "2px solid transparent",
                }}
                onClick={() => { setSelectedStep(step.id); setExpandedSteps(p => ({ ...p, [step.id]: true })); }}
              >
                <div style={{
                  ...S.stepNum,
                  background: selectedStep === step.id ? COLORS.accent : COLORS.border,
                  color: COLORS.white,
                }}>
                  {step.order}
                </div>
                <div style={{ flex: 1, minWidth: 0 }}>
                  <div style={{ fontSize: 11, fontWeight: 600, color: COLORS.white, overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>
                    {step.label}
                  </div>
                  <div style={{ display: "flex", alignItems: "center", gap: 6, marginTop: 2 }}>
                    <span style={S.tagAction(step.action)}>{step.action}</span>
                    <span style={{ fontSize: 10, color: COLORS.textMuted }}>{step.params.arm}</span>
                  </div>
                </div>
                <div style={{ display: "flex", flexDirection: "column", gap: 1 }}>
                  <span onClick={e => { e.stopPropagation(); moveStep(step.id, -1); }} style={{ cursor: "pointer", fontSize: 9, color: COLORS.textMuted, lineHeight: 1 }}>▲</span>
                  <span onClick={e => { e.stopPropagation(); moveStep(step.id, 1); }} style={{ cursor: "pointer", fontSize: 9, color: COLORS.textMuted, lineHeight: 1 }}>▼</span>
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Main Content */}
        <div style={S.content}>
          {/* Device Details Card */}
          <div style={S.card}>
            <div style={S.cardHeader} onClick={() => setExpandedSteps(p => ({ ...p, _device: !p._device }))}>
              <span style={S.cardTitle}>Device Properties</span>
              <span style={{ color: COLORS.textDim, fontSize: 11 }}>{expandedSteps._device ? "▼" : "▶"}</span>
            </div>
            {expandedSteps._device && (
              <div style={S.cardBody}>
                <div style={S.fieldRow}>
                  <Field label="Length (mm)"><Input type="number" value={device.dimensions.length_mm} onChange={v => setDevice(p => ({ ...p, dimensions: { ...p.dimensions, length_mm: v } }))} /></Field>
                  <Field label="Width (mm)"><Input type="number" value={device.dimensions.width_mm} onChange={v => setDevice(p => ({ ...p, dimensions: { ...p.dimensions, width_mm: v } }))} /></Field>
                  <Field label="Height (mm)"><Input type="number" value={device.dimensions.height_mm} onChange={v => setDevice(p => ({ ...p, dimensions: { ...p.dimensions, height_mm: v } }))} /></Field>
                </div>
                <div style={S.fieldRow}>
                  <Field label="Material"><Input value={device.material} onChange={v => setDevice(p => ({ ...p, material: v }))} placeholder="e.g. aluminium, plastic, mixed" /></Field>
                  <Field label="Fixturing Strategy">
                    <Select value={device.fixturing} onChange={v => setDevice(p => ({ ...p, fixturing: v }))} options={["gripper_only", "passive_jig", "gripper_plus_jig", "adhesive_mat"]} />
                  </Field>
                </div>
                <Field label="Notes">
                  <textarea style={{ ...S.textarea, minHeight: 40 }} value={device.notes} onChange={e => setDevice(p => ({ ...p, notes: e.target.value }))} placeholder="Any device-specific notes..." />
                </Field>
              </div>
            )}
          </div>

          {/* Warnings */}
          {warnings.length > 0 && (
            <div style={{ ...S.card, borderColor: `${COLORS.warning}44` }}>
              <div style={{ padding: 12 }}>
                {warnings.map((w, i) => (
                  <div key={i} style={{ fontSize: 11, color: COLORS.warning, padding: "4px 0", display: "flex", gap: 8, alignItems: "flex-start" }}>
                    <span>⚠</span>
                    <span>{w}</span>
                  </div>
                ))}
              </div>
            </div>
          )}

          {/* Action Steps */}
          {steps.length === 0 ? (
            <div style={S.emptyState}>
              <div style={{ fontSize: 32, marginBottom: 12, opacity: 0.3 }}>⚙</div>
              <div style={{ fontSize: 14, fontWeight: 600, color: COLORS.textDim, marginBottom: 6 }}>No disassembly steps defined</div>
              <div style={{ fontSize: 12, color: COLORS.textMuted, maxWidth: 300 }}>
                Add action steps using the buttons in the sidebar. Import vision data to associate components with actions.
              </div>
            </div>
          ) : (
            steps.map(step => {
              const Editor = ACTION_EDITORS[step.action];
              const isExpanded = expandedSteps[step.id];
              return (
                <div key={step.id} style={{
                  ...S.card,
                  borderColor: selectedStep === step.id ? COLORS.accent : S.card.borderColor,
                  boxShadow: selectedStep === step.id ? `0 0 0 1px ${COLORS.accent}33` : "none",
                }}>
                  <div style={S.cardHeader} onClick={() => toggleExpand(step.id)}>
                    <div style={{ display: "flex", alignItems: "center", gap: 10 }}>
                      <span style={{
                        ...S.stepNum,
                        background: COLORS.accent,
                        color: COLORS.white,
                        width: 22,
                        height: 22,
                        fontSize: 11,
                      }}>
                        {step.order}
                      </span>
                      <span style={S.tagAction(step.action)}>{step.action}</span>
                      <input
                        value={step.label}
                        onChange={e => updateStepLabel(step.id, e.target.value)}
                        onClick={e => e.stopPropagation()}
                        style={{
                          background: "transparent",
                          border: "none",
                          borderBottom: `1px solid transparent`,
                          color: COLORS.white,
                          fontSize: 13,
                          fontWeight: 600,
                          fontFamily: "inherit",
                          outline: "none",
                          padding: "2px 4px",
                          width: 180,
                        }}
                        onFocus={e => (e.target.style.borderBottomColor = COLORS.borderFocus)}
                        onBlur={e => (e.target.style.borderBottomColor = "transparent")}
                      />
                    </div>
                    <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
                      <span style={{ fontSize: 10, color: COLORS.textMuted }}>{step.params.arm}</span>
                      <Btn variant="danger" onClick={e => { e.stopPropagation(); removeStep(step.id); }} style={{ fontSize: 10, padding: "2px 8px" }}>
                        Delete
                      </Btn>
                      <span style={{ color: COLORS.textDim, fontSize: 11 }}>{isExpanded ? "▼" : "▶"}</span>
                    </div>
                  </div>
                  {isExpanded && (
                    <div style={S.cardBody}>
                      <Editor params={step.params} onChange={p => updateStep(step.id, p)} components={components} />
                    </div>
                  )}
                </div>
              );
            })
          )}
        </div>

        {/* YAML Preview Panel */}
        {showPreview && (
          <div style={S.preview}>
            <div style={S.previewHeader}>
              <span style={{ fontSize: 13, fontWeight: 700, color: COLORS.white }}>YAML Preview</span>
              <div style={{ display: "flex", gap: 8 }}>
                <Btn variant={copied ? "success" : "ghost"} onClick={handleCopy}>
                  {copied ? "Copied!" : "Copy"}
                </Btn>
                <Btn variant="ghost" onClick={() => setShowPreview(false)}>✕</Btn>
              </div>
            </div>
            <pre style={S.previewCode}>{yamlOutput()}</pre>
          </div>
        )}
      </div>

      {/* Vision Import Modal */}
      {showVisionModal && (
        <VisionImportModal
          onClose={() => setShowVisionModal(false)}
          onImport={handleVisionImport}
        />
      )}

      {/* Hidden download anchor */}
      <a ref={downloadRef} style={{ display: "none" }} />
    </div>
  );
}
