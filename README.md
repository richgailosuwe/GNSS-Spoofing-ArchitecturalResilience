# Architecting a Layered, Resilient GNSS: A Paradigm Shift from Detection to System Assurance

This repository contains the MATLAB implementation of a layered GNSS spoofing-resilience architecture developed for a civil-aviation-oriented Bachelor's thesis project. The work is centered on a shift from spoofing *detection* alone toward *bounded navigation assurance*: identifying anomalous measurements, keeping the position solution bounded under attack, and reporting an experimental protection metric alongside the recovered solution.

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

## Scope Notes

This repository is a research prototype, not a certified aviation product.

- The HPL is an **experimental covariance-derived metric**, not a certified protection level.
- The 185.2 m and 370.4 m values are used as **descriptive aviation-oriented reference scales**, not compliance claims.
- Stage 0 OSNMA is **partial** in live operation and currently yields status rather than hard measurement admission control.
- The filter is **pseudorange-only** in the main thesis pipeline; Doppler parsing exists but is not the core fused navigation mode.
- The strongest unresolved boundary is a coherent two-against-two constellation attack with no independent trust anchor.

## Pipeline Architecture


<img width="6268" height="4876" alt="gnss_pipeline_architecture2 drawio" src="https://github.com/user-attachments/assets/2d38d367-9637-42d1-9617-4740e8bdce52" />


## Repository Structure

```text
|-- utils/
|   Shared GNSS data readers, orbit/measurement models, coordinate tools, WLS solver, and spoofing injector.
|
|   |-- rinex_read_obs.m
|   |   Reads RINEX observation files and extracts pseudorange, carrier phase, Doppler, and C/N0 fields.
|   |
|   |-- rinex_read_nav.m
|   |   Reads RINEX navigation files and extracts broadcast ephemerides.
|   |
|   |-- sat_position.m
|   |   Computes satellite ECEF position and clock correction for GPS, Galileo, BeiDou, and GLONASS.
|   |
|   |-- corrected_pseudorange.m
|   |   Main corrected measurement model: transmit-time position, clock, Sagnac, ionosphere, troposphere, and group delay.
|   |
|   |-- pseudorange_correct.m
|   |   Legacy correction helper retained for diagnostics and comparison tests.
|   |
|   |-- get_group_delay.m
|   |   Returns GPS TGD or Galileo/BeiDou BGD group-delay correction in metres.
|   |
|   |-- wls_solver.m
|   |   Iterative weighted least-squares solver for snapshot GNSS position and receiver clock.
|   |
|   |-- ecef2lla_simple.m
|   |   Converts ECEF coordinates to WGS84 geodetic latitude, longitude, and altitude.
|   |
|   |-- coord_convert.m
|   |   Central coordinate-conversion utility for ECEF, LLA, ENU, and distance operations.
|   |
|   |-- inject_spoofing.m
|   |   Generates geometry-consistent drag-off spoofed pseudoranges using exact range-difference injection.
|   |
|   |-- ionofree_combination.m
|   |   Computes ionosphere-free dual-frequency pseudorange combinations.
|   |
|   |-- test_corrected_pseudorange.m
|   |   Validates transmit-time and group-delay corrected pseudorange behaviour.
|   |
|   |-- test_group_delay_bgd.m
|   |   Tests group-delay and Galileo BGD band-selection logic.
|   |
|   |-- test_injection_geometry.m
|   |   Verifies that the spoofing injector produces the intended geometric displacement.
|   |
|   |-- test_sagnac_fix.m
|   |   Tests Sagnac correction magnitude and positioning effect.
|   |
|   `-- calibration/
|       Satellite residual-bias calibration helpers.
|       |
|       |-- calibrate_sat_bias.m
|       |   Calibrates satellite-specific residual bias corrections.
|       |
|       `-- apply_sat_bias.m
|           Applies calibrated satellite bias corrections.
|
|-- Data/
|   Input data used by the pipeline.
|
|   |-- raw/
|   |   Raw authentic, spoofed, and hardware captures.
|   |
|   |-- rinex/observation/
|   |   Authentic BUCU RINEX observation files.
|   |
|   |-- rinex/navigation/
|   |   Authentic BUCU RINEX navigation files.
|   |
|   |-- rinex/hardware/
|   |   Hardware RINEX recordings from ZED-F9P sessions.
|   |
|   `-- reference/
|       Supporting reference inputs used by calibration and validation utilities.
|
`-- results/
    Saved outputs, figures, calibrations, and scenario evidence.
    |
    |-- pvt/
    |   Saved pipeline outputs for each scenario and batch summaries.
    |
    |-- ablation/
    |   Saved A/B/C mitigation ablation outputs.
    |
    |-- figures/
    |   Thesis figures and supporting diagnostics.
    |
    |-- hardware/
    |   Saved hardware validation outputs and 1 Hz cross-validation results.
    |
    |-- logs/
    |   Calibration logs, audits, and provenance summaries.
    |
    |-- calibration/
    |   Saved calibration products and intermediate statistics.
    |
    `-- simulated_scenarios/
        Saved spoofed observation sets and scenario-specific evidence.
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

The full PDF of my Bachelor's thesis can be accessed through Microsoft Onedrive, using the link below

[Read the full thesis](https://1drv.ms/b/c/9b41f5afb01ac702/IQD8E81WXcPSQLjELWBqDLKnAY_53BUUrdwkQvsIUrkNwbw?e=3BgyZl)



## License

## License

This project is released under the MIT License, which can be found in [LICENSE](LICENSE) .

