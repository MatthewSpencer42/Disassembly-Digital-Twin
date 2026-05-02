# Disassembly Skills — Architecture Flowcharts

Each section contains a Mermaid flowchart for one skill showing the success path, every failure point, and how the system recovers.

**To export as an image:** paste any `mermaid` block into **https://mermaid.live**, then click the PNG or SVG download button.

---

## How the Skills Chain Together

```mermaid
flowchart LR
    H["① Object Hold\nUF850 presses down on device"]
    U["② Unscrew × N\nxArm5 removes each screw"]
    P["③ Object Pickup\nUF850 lifts the freed part"]
    F["④ Flip or Flip-Drop\nUF850 flips device over"]
    H2["① Object Hold\nrepeat for other side"]

    H --> U --> P --> F --> H2

    classDef hold fill:#4CAF50,color:#fff,stroke:#388E3C
    classDef unscrew fill:#2196F3,color:#fff,stroke:#1565C0
    classDef pickup fill:#FF9800,color:#fff,stroke:#E65100
    classDef flip fill:#9C27B0,color:#fff,stroke:#6A1B9A

    class H,H2 hold
    class U unscrew
    class P pickup
    class F flip
```

---

## 1. Object Hold Skill

```mermaid
flowchart TD
    START(["START — Object Hold"]) --> A["Camera scans workspace\nand locates the device"]

    A --> B{"Device visible\nin camera?"}
    B -->|No| FAIL1(["FAIL — Abort\nMaster agent retries after fresh camera look"])
    B -->|Yes| C["Open gripper wide\nto avoid knocking anything"]

    C --> D["Move arm to hover point\ndirectly above the device"]
    D --> E{"Path to hover\nreachable?"}

    E -->|No| F["Try Cartesian backup path"]
    F --> G{"Backup path\nworks?"}
    G -->|No| FAIL2(["FAIL — Report failure and stop"])
    G -->|Yes| H

    E -->|Yes| H["Descend one small step at a time\nListening for joint torque spike"]

    H --> I{"Arm settled\nwithin 20 seconds?"}
    I -->|No — keeps shaking| FAIL3(["FAIL — Timeout\nReport failure"])
    I -->|Yes| J{"Torque spike detected\n= contact with surface?"}

    J -->|No — max distance reached| FAIL4(["FAIL — Device may have moved\nor coordinates are stale"])
    J -->|Yes — contact confirmed| K{"Holding strategy?"}

    K -->|Flat press| L["Keep gripper open\nArm presses down gently"]
    K -->|Side clamp| M["Close gripper\naround the object"]

    L --> N["Broadcast IS HELD on ROS topic\nWrite hold-state to disk"]
    M --> N
    N --> SUCCESS(["SUCCESS — Device is secured and held"])

    classDef fail fill:#F44336,color:#fff,stroke:#B71C1C
    classDef success fill:#4CAF50,color:#fff,stroke:#1B5E20
    classDef start fill:#607D8B,color:#fff,stroke:#37474F
    classDef recover fill:#FF9800,color:#fff,stroke:#E65100

    class FAIL1,FAIL2,FAIL3,FAIL4 fail
    class SUCCESS success
    class START start
    class F recover
```

---

## 2. Unscrew Skill

```mermaid
flowchart TD
    START(["START — Unscrew"]) --> A["Receive screw 3D position\nfrom vision system"]

    A --> B{"Screw visible\nin camera?"}
    B -->|No| FAIL1(["FAIL — Abort\nMaster agent re-checks vision before retry"])

    B -->|Yes| C{"Screw within\narm reach?"}
    C -->|No| FAIL2(["FAIL — Out of reach\nAbort immediately"])

    C -->|Yes| D["Lift to safe travel height\nMove directly above screw"]

    D --> E["Descend slowly\nTool camera watches screw head continuously"]
    E --> F["Correct XY position at each step\nto centre screwdriver tip over screw"]
    F --> G{"Contact detected\nby force sensor\nwithin 15 s?"}

    G -->|No — timeout| H["Lift to safe height\nNavigate to bin — drop nothing"]
    H --> FAIL3(["FAIL — Descent timed out"])

    G -->|Yes — contact confirmed| I["Spin motor briefly\nCheck for seating resistance"]

    I --> J{"Resistance spike\nconfirmed — bit is seated?"}

    J -->|No — not seated| K["Retract 5 mm\nStart square spiral search"]
    K --> L["Move outward in growing square\n5 mm start, +5 mm per loop\nup to 15 seconds total"]
    L --> M{"Screw hole found\nduring spiral?"}
    M -->|No — 3 retries exhausted| FAIL4(["FAIL — Spiral search failed\nLift to safety and report"])
    M -->|Yes — hole found| I

    J -->|Yes — seated| N["Run motor continuously\nArm lifts following screw as it rises"]

    N --> O{"Extraction stalled\nno force increase for 3 s?"}
    O -->|Yes| FAIL5(["FAIL — Motor stops, arm retracts\nScrew may be partially in\nMaster agent decides next step"])
    O -->|No| P["Force stabilises\nScrew is fully free"]

    P --> Q["Open gripper\nCarry screw to bin and drop it"]
    Q --> R["Retract arm to safe height"]
    R --> SUCCESS(["SUCCESS — Screw in bin\nReady for next screw"])

    classDef fail fill:#F44336,color:#fff,stroke:#B71C1C
    classDef success fill:#4CAF50,color:#fff,stroke:#1B5E20
    classDef start fill:#607D8B,color:#fff,stroke:#37474F
    classDef recover fill:#9C27B0,color:#fff,stroke:#6A1B9A

    class FAIL1,FAIL2,FAIL3,FAIL4,FAIL5 fail
    class SUCCESS success
    class START start
    class K,L recover
```

---

## 3. Object Pickup Skill

```mermaid
flowchart TD
    START(["START — Object Pickup"]) --> A{"Currently holding\nsomething from\na previous run?"}

    A -->|Yes| B["Release gripper\nRetract 30 cm\nReturn to home position"]
    A -->|No| C

    B --> C{"Arm near\nhome position?"}
    C -->|No — drifted| D["Retract and re-home first"]
    D --> E
    C -->|Yes| E["Move xArm5 out of the way\nto avoid collisions"]

    E --> F["Request fresh vision scan\nWait 2.5 s for it to stabilise"]

    F --> G{"Target part\nfound by ID?"}
    G -->|No — ID changed after reset| H["Search by part label instead"]
    H --> I{"Found by\nlabel?"}
    I -->|No| FAIL1(["FAIL — Part not found\nAbort — operator intervention needed"])
    I -->|Yes| J

    G -->|Yes| J["Open gripper\nMove arm to hover above target"]
    J --> K{"Path to hover\nreachable?"}
    K -->|No| FAIL2(["FAIL — Abort\nMaster agent retries after re-homing"])

    K -->|Yes| L["Close gripper to half-open for descent\nAvoids scraping anything on the way down"]
    L --> M["Descend one step at a time\nMonitoring joint torque"]
    M --> N{"Contact sensed\nat surface?"}

    N -->|Yes| O["Back up 10 mm off the surface"]
    O --> P["Close gripper fully\nwith programmed gripping force"]

    P --> Q{"Gripper stuck\nwhile opening earlier?"}
    Q -->|Yes| R["Retry with maximum opening force\nMark as warning and continue"]
    Q -->|No| S
    R --> S["Retract 30 mm carrying the part"]

    S --> T["Travel to drop zone\nRelease the part"]
    T --> U["Return to home position"]
    U --> SUCCESS(["SUCCESS — Part in drop zone\nArm back at home"])

    classDef fail fill:#F44336,color:#fff,stroke:#B71C1C
    classDef success fill:#4CAF50,color:#fff,stroke:#1B5E20
    classDef start fill:#607D8B,color:#fff,stroke:#37474F
    classDef recover fill:#FF9800,color:#fff,stroke:#E65100

    class FAIL1,FAIL2 fail
    class SUCCESS success
    class START start
    class R recover
```

---

## 4. Object Flip Skill

```mermaid
flowchart TD
    START(["START — Object Flip"]) --> PRE{"IS HELD signal\nreceived?"}

    PRE -->|No| WAIT["Wait up to 2 seconds\nfor late message to arrive"]
    WAIT --> PRE2{"Signal\narrived?"}
    PRE2 -->|No| FAIL1(["FAIL — No object held\nRun Object Hold Skill first"])
    PRE2 -->|Yes| PUB

    PRE -->|Yes| PUB["Publish FLIPPING state"]

    PUB --> A["Lift straight up 10 cm\nClosed-loop — checking position continuously"]
    A --> B{"Lift reached\ntarget height?"}
    B -->|No| FAIL2(["FAIL — Closed-loop lift failed\nReport failure immediately"])
    B -->|Yes| C["Read current wrist joint angle\nChoose rotation direction to avoid cable wrap"]

    C --> D{"Current angle\npositive?"}
    D -->|Yes — rotate minus 180 deg| E["Rotate wrist joint 180 deg\nto opposite pole"]
    D -->|No — rotate plus 180 deg| E

    E --> F{"Rotation path\nfound by planner?"}
    F -->|No| FAIL3(["FAIL — Cannot plan 180 deg rotation\nDevice may need repositioning"])
    F -->|Yes| G["Descend slowly back to table\nMonitoring joint torque"]

    G --> H{"Contact with\nsurface detected?"}
    H -->|No| FAIL4(["FAIL — Tactile descent found nothing\nDevice may have slid away"])
    H -->|Yes| I["Release gripper briefly"]

    I --> J{"Gripper stuck\nwhile opening?"}
    J -->|Yes| K["Retry with maximum opening force"]
    K --> L
    J -->|No| L["Re-grasp with full closing force"]

    L --> M["Publish HOLDING\nBroadcast IS HELD"]
    M --> SUCCESS(["SUCCESS — Device flipped 180 deg\nHold maintained"])

    classDef fail fill:#F44336,color:#fff,stroke:#B71C1C
    classDef success fill:#4CAF50,color:#fff,stroke:#1B5E20
    classDef start fill:#607D8B,color:#fff,stroke:#37474F
    classDef recover fill:#FF9800,color:#fff,stroke:#E65100

    class FAIL1,FAIL2,FAIL3,FAIL4 fail
    class SUCCESS success
    class START start
    class K recover
```

---

## 5. Flip-Drop Skill

```mermaid
flowchart TD
    START(["START — Flip-Drop"]) --> PRE{"IS HELD signal\nreceived?"}

    PRE -->|No| WAIT["Wait up to 2 seconds\nfor late message to arrive"]
    WAIT --> PRE2{"Signal\narrived?"}
    PRE2 -->|No| FAIL1(["FAIL — No object held\nRun Object Hold Skill first"])
    PRE2 -->|Yes| REF

    PRE -->|Yes| REF["Read current end-effector\nposition and orientation as reference"]

    REF --> PUB["Publish FLIPPING state"]
    PUB --> A["Lift straight up 15 cm\nClosed-loop mode"]

    A --> B{"Lift\nsuccessful?"}
    B -->|No| FAIL2(["FAIL — Closed-loop lift failed\nImmediate abort"])
    B -->|Yes| C["Travel sideways to fixed flip zone\na safe area with no obstacles below"]

    C --> D{"Path to flip\nzone clear?"}
    D -->|No| FAIL3(["FAIL — Path blocked\nEXOTica planner returns failure"])
    D -->|Yes| E["Rotate wrist 180 deg\nsame direction logic as Flip Skill"]

    E --> F{"Rotation\npath found?"}
    F -->|No| FAIL4(["FAIL — Rotation planning failed\nImmediate abort"])
    F -->|Yes| G["Rotate wrist back to original orientation\nDouble-flip shakes loose any resting parts"]

    G --> H["Return to original XY position\nstaying 15 cm above original height"]
    H --> I["Descend slowly to table\nStopping on torque contact"]

    I --> J{"Contact\ndetected?"}
    J -->|No| FAIL5(["FAIL — Tactile descent found nothing\nReport failure"])
    J -->|Yes| K["Release gripper briefly\nLet loose items settle"]

    K --> L["Re-grasp with full closing force"]
    L --> M{"Re-grasp\nsuccessful within timeout?"}
    M -->|No| FAIL6(["FAIL — Re-grasp timed out\nCheck if device is still in gripper"])
    M -->|Yes| N["Publish HOLDING\nBroadcast IS HELD"]

    N --> SUCCESS(["SUCCESS — Device flipped and re-grasped\nLoose parts shaken free\nHold maintained"])

    classDef fail fill:#F44336,color:#fff,stroke:#B71C1C
    classDef success fill:#4CAF50,color:#fff,stroke:#1B5E20
    classDef start fill:#607D8B,color:#fff,stroke:#37474F

    class FAIL1,FAIL2,FAIL3,FAIL4,FAIL5,FAIL6 fail
    class SUCCESS success
    class START start
```

---

## 6. Master Agent (Orchestrator)

```mermaid
flowchart TD
    START(["DISASSEMBLY BEGINS"]) --> SEE

    SEE["① SEE\nRead latest vision output\nobject positions, labels, states"] --> THINK

    THINK["② THINK\nSend vision data to LLM\nReason about what is done\nand what still needs doing"] --> PLAN

    PLAN["③ PLAN\nLLM outputs the next action\nas a function call\ne.g. unscrew id=3 label=hex"] --> PARSE{"Action name and\narguments valid?"}

    PARSE -->|No — parse error| CORRECT["Send correction prompt to LLM\nand retry planning"]
    CORRECT --> PLAN

    PARSE -->|Yes| ACT["④ ACT\nCall the appropriate skill\nWait for success or failure result"]

    ACT --> RESULT{"Skill\nresult?"}

    RESULT -->|Success| CHECK{"Disassembly\ncomplete?"}
    RESULT -->|Failure| RECORD["Record failure with full details\nShow in live colour dashboard"]
    RECORD --> THINK

    CHECK -->|No — more steps remain| SEE
    CHECK -->|Yes| DONE(["DISASSEMBLY COMPLETE"])

    classDef phase fill:#2196F3,color:#fff,stroke:#1565C0
    classDef decision fill:#FF9800,color:#fff,stroke:#E65100
    classDef success fill:#4CAF50,color:#fff,stroke:#1B5E20
    classDef start fill:#607D8B,color:#fff,stroke:#37474F
    classDef recover fill:#9C27B0,color:#fff,stroke:#6A1B9A

    class SEE,THINK,PLAN,ACT phase
    class START start
    class DONE success
    class CORRECT,RECORD recover
```

---

## Quick Reference

| Skill | Arm | Happy-path steps | Key recovery mechanism |
|---|---|---|---|
| Object Hold | UF850 | 6 | Cartesian backup path |
| Unscrew | xArm5 | 9 | Square spiral search outward from 5 mm |
| Object Pickup | UF850 | 13 | Label fallback + joint-space retract |
| Object Flip | UF850 | 7 | Wait 2 s for hold signal |
| Flip-Drop | UF850 | 10 | Same hold-wait as Flip Skill |
