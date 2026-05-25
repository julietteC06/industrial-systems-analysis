# Industrial Systems Analysis

A portfolio of technical deep-dives into real industrial systems — understanding architecture, modeling behavior, and analyzing engineering tradeoffs.

## Why this exists

As a technical sales engineer, the core skill is not writing production code — it's the ability to **understand how a system works, explain it clearly, and reason about design decisions**. Each analysis in this repo takes a real industrial product or system and breaks it down:

- What problem does it solve?
- How is it architected? (system-level, subsystem interactions)
- What are the governing physical principles and mathematical models?
- **What are the key engineering tradeoffs, and why were they made?**

## Analyses

| # | System | Key Insight | Domain |
|---|--------|------------|--------|
| 1 | [Aircraft Landing Gear Shock Absorber](./landing-gear-tradeoff/) | The same spring-damper system cannot be simultaneously optimized for landing impact and taxiing comfort — an inherent tradeoff driven by human physiology and airframe constraints | Aerospace / Mechanical |
| 2 | eTras Electrical Thrust Reverser (Safran) | *coming soon* | Aerospace / Electromechanical |
| 3 | DELTA Robot Gripper Orientation Axis | *coming soon* | Robotics / Control Systems |
| 4 | JacXson U70 Aircraft Engine Maintenance Chariot | *coming soon* | Industrial Equipment / Mechatronics |

## Format

Each analysis is a self-contained Markdown document with:
- System context and architecture diagrams (Mermaid)
- Physical modeling and transfer function derivation
- Simulation insights and frequency-domain analysis
- Explicit discussion of design tradeoffs and constraints
- References to real standards and specifications

## Background

These analyses are based on coursework from the Sino-French Engineering Program (中法工程师培养项目) at Beihang University's Industrial Science (Sciences Industrielles) curriculum. The original exercises were designed by aerospace engineering faculty and adapted from real industrial systems in partnership with companies like Safran and Excent.

## Contact

Xingguo Chen — [GitHub @julietteC06](https://github.com/julietteC06)
