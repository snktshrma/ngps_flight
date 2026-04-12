Short Answer: No, Don't Use It
This repo is fundamentally incompatible with your no-GPS requirement. Here's why, and what you should actually do.

What That Repo Actually Is
PropertyDetailCore designPPP (Precise Point Positioning) + INS tight couplingGNSS dependencyHard — VIO literally requires GNSS for global initializationLast commit2020 — essentially abandonedTested onGround vehicle, Wuhan University campusDependenciesOpenCV 3.4, Ceres 1.14 — outdatedPaperMaster's thesis, not peer-reviewed system
It's a research prototype built around GNSS, not resilient to its absence. Completely wrong tool for your use case.

Your Current Stack (Understood Correctly?)
LightGlue → lat/long  ←── visual geo-localization against map?
       ↓
      UKF  ← fuses absolute position + local VIO
       ↑
   VINS-Fusion → high-rate local odometry
If LightGlue is giving you lat/long by matching against geo-referenced satellite imagery — that's actually a smart no-GPS architecture. The question is whether the UKF fusion layer is the bottleneck.
Is that correct? Or is LightGlue doing something else? This matters a lot for the recommendation.

Problems With Your Current Architecture
1. UKF is a weak fusion layer here

UKF assumes Gaussian noise — LightGlue geo-localization failures are non-Gaussian (multimodal, heavy-tailed)
No mechanism to reject bad geo-localization matches robustly
Latency mismatch between slow LightGlue matches and high-rate VINS not handled optimally

2. Two separate estimators = inconsistency

VINS maintains its own state internally
UKF treats VINS output as a measurement, not a jointly optimized state
Leads to double-counting of IMU information

3. No tight coupling between LightGlue geo-poses and VIO map

Better Architecture: Factor Graph with Absolute Pose Factors
Instead of UKF stitching two systems together, move to a unified factor graph where geo-localization poses are just another factor type:
IMU preintegration factors  ──┐
Visual reprojection factors ──┼── Factor Graph (GTSAM/g2o)
Geo-localization factors    ──┘      ↓
  (from LightGlue matches)      Optimized trajectory
Option 1: GTSAM (Recommended for your setup)

Add GPSFactor or custom AbsolutePoseFactor for your LightGlue lat/long outputs
IMU preintegration is first-class (PreintegratedImuMeasurements)
Sliding window or full batch — your choice
Well-maintained, used in production autonomy systems
Runs well on Orin NX

cpp// Your LightGlue geo-fix becomes just another factor
auto geoFactor = GPSFactor(X(i), gpsPoint, geoNoiseModel);
graph.add(geoFactor);
Option 2: OpenVINS with External Position Plugin

OpenVINS accepts external position measurements natively
Add your LightGlue lat/long as TYPE_LOC_ANCHOR updates
Lighter than GTSAM, easier to tune
Active maintenance, well-documented

Option 3: Kimera-VIO + External Odometry

Best if you also want a 3D map
Supports injection of external pose priors
Heavier compute but Orin NX handles it


Recommended Stack for Your Project
┌─────────────────────────────────────────┐
│  Stereo cameras → feature tracking      │
│  IMU → preintegration                   │
│  LightGlue geo-match → AbsolutePoseFactor│
│         ↓                               │
│    GTSAM sliding window optimizer       │
│    (replaces VINS + UKF entirely)       │
│         ↓                               │
│    6-DoF pose @ 30Hz to flight ctrl     │
└─────────────────────────────────────────┘
Key advantage: When LightGlue fails (texture-poor, clouds, motion blur), the factor just gets skipped — the graph keeps optimizing with whatever factors are valid. No UKF divergence.

Honest Assessment of Your Current Stack
Your LightGlue + VINS + UKF approach is not wrong — it's just architecturally loose. For a research prototype it's fine. For a reliable no-GPS high-altitude system, the failure modes of loose coupling will hurt you.
The single most impactful change: replace UKF with GTSAM and inject both VINS-style visual factors and LightGlue geo-factors into one graph.
What does your LightGlue pipeline actually match against — satellite imagery, a pre-built visual map, or something else? That'll determine exactly how to model the geo-factor noise.