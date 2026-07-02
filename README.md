# Architecting a Layered, Resilient GNSS System for Assured Navigation

This repository contains the MATLAB implementation of a layered GNSS spoofing-resilience architecture developed for a civil-aviation-oriented thesis project. The work is centered on a shift from spoofing *detection* alone toward *bounded navigation assurance*: identifying anomalous measurements, keeping the position solution bounded under attack, and reporting an experimental protection metric alongside the recovered solution.

The implementation targets multi-constellation GNSS with GPS, Galileo, BeiDou, and GLONASS, and combines anomaly monitoring, identification, measurement handling, and state estimation in one end-to-end pipeline.

## What This Project Does

The pipeline is organized into five stages:

1. **Stage 0 - OSNMA front end**  
   Partial Galileo OSNMA processing and cryptographic verification primitives. This stage currently provides authentication status input, but it does not yet gate measurements in the main navigation solution.

2. **Stage 1 - Signal anomaly detection**  
   Detects suspicious behaviour using C/N0 variation, pseudorange residual monitoring, clock-consistency monitoring, and AGC support for live hardware mode.

3. **Stage 2 - Satellite identification**  
   Uses RAIM-FDE and inter-constellation consistency to identify likely spoofed signals or constellations.

4. **Stage 3 - Exclusion and gating**  
   Applies classification-driven measurement reweighting and scalar innovation gating to suppress inconsistent measurements.

5. **Stage 4 - Recovery and integrity indication**  
   Runs a centralized multi-constellation EKF with inter-system bias states and computes an experimental horizontal protection level (HPL).

## Key Contributions

- A complete MATLAB pipeline from raw/simulated spoofing scenario to recovered position.
- A centralized 11-state multi-constellation EKF with inter-system bias (ISB) modelling.
- A transmit-time pseudorange measurement model validated on independent u-blox ZED-F9P hardware recordings.
- Five evaluated spoofing scenarios, including single- and dual-constellation attacks.
- Ablation results showing that scalar innovation gating provides most of the immediate protection, while identification and exclusion provide diagnostic and robustness value.
- Experimental HPL generation for integrity-style evaluation.

## Main Findings

- Coordinated spoofing biases can be absorbed by residual-based consistency tests, so **inter-constellation consistency** is stronger than residual RAIM alone for constellation-level identification.
- In the evaluated scenarios, **scalar innovation gating** provides most of the immediate position protection.
- The mitigated pipeline keeps position error bounded under the tested spoofing attacks, while the unmitigated solution can drift by tens to hundreds of metres.
- The final transmit-time WLS model generalizes to independent standalone ZED-F9P recordings with metre-level stability about session mean.

## Scope and Honesty Notes

This repository is a research prototype, not a certified aviation product.

- The HPL is an **experimental covariance-derived metric**, not a certified protection level.
- The 185.2 m and 370.4 m values are used as **descriptive aviation-oriented reference scales**, not compliance claims.
- Stage 0 OSNMA is **partial** in live operation and currently yields status rather than hard measurement admission control.
- The filter is **pseudorange-only** in the main thesis pipeline; Doppler parsing exists but is not the core fused navigation mode.
- The strongest unresolved boundary is a coherent two-against-two constellation attack with no independent trust anchor.

## Pipeline Architecture

Place your architecture figure at:

`docs/pipeline_architecture.png`

Then it will render here:

![Pipeline architecture](docs/pipeline_architecture.png)

## Repository Structure

```text
MATLAB IMPLEMENTATION/
|-- config.m                         # Central configuration: paths, thresholds, scenarios, EKF and integrity settings
|-- main.m                           # Default entry point (Scenario 1, verbose)
|-- run_pipeline.m                   # Full Stage 0-4 pipeline runner for one scenario
|-- run_all_scenarios.m              # Batch execution of all configured spoofing scenarios
|-- run_ablation.m                   # Ablation study runner (full / gate-only / unmitigated)
|-- run_baseline_authentic.m         # Authentic no-attack baseline evaluation
|-- run_real_authentic_validation.m  # Real-hardware WLS/EKF consistency validation
|-- validate_1hz_profile.m           # 1 Hz hardware EKF feasibility validation
|-- generate_all_figures.m           # Regenerates thesis/result figures from saved evidence
|
|-- Data/
|   |-- raw/                         # Raw authentic, spoofed, and hardware captures
|   |-- reference/                   # Reference/supporting data
|   `-- rinex/                       # RINEX observation/navigation/hardware data
|
|-- stage0_osnma/                    # Galileo OSNMA parsing, crypto, and status utilities
|-- stage1_detection/                # Stage 1 anomaly detectors and fusion
|-- stage2_identification/           # RAIM-FDE, inter-constellation checks, classification
|-- stage3_exclusion/                # Exclusion mask, innovation gate, Stage 3 tests
|-- stage4_recovery/                 # EKF, ISB model, HPL, recovery tests and calibration
|-- utils/                           # Shared GNSS utilities, measurement model, spoof injector
|-- tests/                           # Additional supporting tests
|
|-- results/
|   |-- ablation/                    # Saved ablation outputs
|   |-- calibration/                 # Calibration outputs
|   |-- figures/                     # Thesis figures and diagnostics
|   |-- hardware/                    # Saved hardware validation outputs
|   |-- logs/                        # Calibration and audit logs
|   |-- pvt/                         # Saved pipeline outputs
|   `-- simulated_scenarios/         # Saved spoofed scenario datasets
|
`-- docs/                            # Recommended location for thesis PDF and architecture figures
```

## How to Run

### 1. Configure the project

Open [config.m](./config.m) and verify:

- data paths
- `convbin` path
- active constellations
- spoofing scenario settings

### 2. Run one scenario

```matlab
run_pipeline(1, true)
```

or by name:

```matlab
run_pipeline('scenario_4_gps_glonass', true)
```

### 3. Run all scenarios

```matlab
run_all_scenarios
```

### 4. Run authentic baseline

```matlab
run_baseline_authentic
```

### 5. Run hardware validation

```matlab
run_real_authentic_validation
```

### 6. Generate figures from saved results

```matlab
generate_all_figures
```

## Evaluated Spoofing Scenarios

The default configuration includes five drag-off scenarios:

1. `scenario_1_gps` - GPS only
2. `scenario_2_galileo` - Galileo only
3. `scenario_3_beidou` - BeiDou only
4. `scenario_4_gps_glonass` - GPS + GLONASS
5. `scenario_5_gps_galileo` - GPS + Galileo stress case

These scenarios are defined in [config.m](./config.m).

## Hardware Validation

The repository also includes standalone real-hardware validation using u-blox ZED-F9P recordings.

This part of the work supports the claim that the transmit-time WLS measurement model generalizes to independent hardware data. These metrics describe **precision/stability about session mean**, not absolute truth, unless an independent surveyed or RTK reference is introduced.

## Thesis Document

Place the final thesis PDF at:

`docs/thesis.pdf`

Then reference it here:

[Read the full thesis](docs/thesis.pdf)

If you prefer controlled access instead of direct public download, replace that link with your request form or hosted access page.

## Figures and QR Code

The repository can include public-facing assets such as:

- pipeline architecture diagram
- key result figures
- GitHub QR code

For example, the current QR image in the root can be moved to `docs/` and linked here if desired.

## License

This project is intended to be released under the MIT License.

Add the standard `LICENSE` file at the repository root:

`LICENSE`

## Citation

Formal academic references are intentionally kept in the thesis PDF rather than duplicated in the README. If you want others to cite the work, add a short citation block here later with the thesis title, author, institution, and year.
