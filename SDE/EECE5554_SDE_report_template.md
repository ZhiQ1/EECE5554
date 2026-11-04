# System Design Exercise: Report Template

Copy this file to `EECE5554/SDE/report/`, fill it in, and export it to PDF. Keep every heading, and delete the italic guidance as you write. Tables may grow as long as you need. The worked example, `EECE5554_SDE_example.md`, shows every section filled in.

**Name:**
**Scenario:** S#, title
**Repository:** URL of your EECE5554 repository

## Summary

*At most 150 words: the problem, the design in two sentences, the headline numbers, and the biggest open risk.*

## Design loops

| # | What failed | What changed | Where in the report |
|---|---|---|---|
| 1 | | | |

## 1. Problem Definition

### 1.1 Operational concept

*Users and stakeholders, one normal mission from start to finish, and the operating envelope.*

| Off-nominal event | What the system should do |
|---|---|
| | |

### 1.2 Requirements

*At least eight: all the given ones, and at least three derived ones with the math shown.*

| ID | Requirement | Value | Type | Source or rationale | Verification |
|---|---|---|---|---|---|
| R1 | | | Given | G1 | |

### 1.3 Assumptions and scope

## 2. System Architecture

### 2.1 Functional architecture and pipeline

*Figure 1: every sensor, every processing step, the rate of each data flow, and where the safe stop lives.*

### 2.2 Physical architecture

| Part | What runs on it | How it connects |
|---|---|---|
| | | |

### 2.3 Operating modes

*At least a normal mode, one degraded mode, and a safe stop, with thresholds in numbers.*

| Mode | Entered when | Behavior | Left when |
|---|---|---|---|
| | | | |

### 2.4 ROS 2 nodes and topics

| Node | Package | Publishes | Rate |
|---|---|---|---|
| | | | |

## 3. Sensor Selection

### 3.1 Sensing needs

| Need | From | Quantity | Value |
|---|---|---|---|
| | | | |

### 3.2 Trade study: first sensing function

*Candidates, the must-meet screen, weights with their reasons, datasheet values with sources, judgment scores with one sentence each, the Tool 3 result, the decision, and the runner-up.*

### 3.3 Trade study: second sensing function

### 3.4 Final suite and placement

| Sensor | Where | Coverage and blind zones (Tool 2) |
|---|---|---|
| | | |

### 3.5 Prototype parts list

| Part | Model | Qty | Unit price | Owned | Mass (kg) | Power (W) | Source |
|---|---|---|---|---|---|---|---|
| | | | | | | | |

*Out-of-pocket cost, added mass, and added power, from Tool 9.*

## 4. Localization, Mapping & Navigation

### 4.1 Methods

| Function | Choice | Alternative considered | Why it lost |
|---|---|---|---|
| Localization | | | |
| Mapping | | | |
| Navigation | | | |

### 4.2 Estimator design

| Item | Value |
|---|---|
| States | |
| Process model | |
| Measurements, with rates and 1σ noise | |
| Frames | |
| Start | |
| Outlier rejection | |

### 4.3 Drift and outage analysis

*Tools 4 and 5: the worst gap in absolute fixes, the pose at its start, the error at its end, and the longest fix interval that still meets your requirement.*

### 4.4 Error budget

*The Tool 6 result for your tightest accuracy requirement, in the worst place in the mission.*

| Contributor | 1σ (m) | Precision or accuracy | Source |
|---|---|---|---|
| | | | |

### 4.5 Mapping, planning, control, and the final approach

## 5. System Integration

### 5.1 Frames and calibration

*The TF tree, with the source of every static transform.*

| What | How | How often | Acceptance check |
|---|---|---|---|
| | | | |

### 5.2 Time

### 5.3 Data rates and compute

### 5.4 Power and runtime

*Tool 8: new pack, aged pack, aged pack in the cold, and peak current, against your runtime requirement.*

### 5.5 Verification plan and traceability

| Test | Requirements | What | Pass criterion | Equipment | In the final project? |
|---|---|---|---|---|---|
| T1 | | | | | |

*The Tool 10 result, and what you fixed.*

## 6. Failure Analysis

### 6.1 Failure modes, ranked

*The Tool 11 table, with at least ten failure modes. Mark the rows that an AI review suggested.*

### 6.2 Detection and recovery

| Failure | Detection and threshold | Response | Mode | Leaves when |
|---|---|---|---|---|
| | | | | |

### 6.3 Residual risk

## References

## Appendix A: Tools used

| Tool | Version | What for | Where in the report |
|---|---|---|---|
| | | | |

## Appendix B: AI use log

| # | Tool and version | Part | What I asked, in short | What I used | How I verified it | What I rejected, and why |
|---|---|---|---|---|---|---|
| 1 | | | | | | |

*I used AI tools only as listed in this log. The design decisions, numbers, and text in this report are my own, and I can explain each of them.*

Signed:

## Appendix C: Calculations

*A link to `analysis/sde_analysis.ipynb` in your repository, or the notebook exported to PDF or HTML.*
