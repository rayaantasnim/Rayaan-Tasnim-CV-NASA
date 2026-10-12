<div align="center">

# 🚀 Autonomous Deep Space Navigation & Rover Control Systems

<p align="center">
  <img src="https://img.shields.io/badge/Build-Passing-brightgreen?style=for-the-badge&logo=github" alt="Build Status"/>
  <img src="https://img.shields.io/badge/Languages-C++%20%7C%20Python%203.11-blue?style=for-the-badge&logo=cplusplus" alt="Core Languages"/>
  <img src="https://img.shields.io/badge/Project%20Status-Stanford%2FMIT%20Grounded-orange?style=for-the-badge&logo=academia" alt="Ph.D. Project Status"/>
  <img src="https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge&logo=opensourceinitiative" alt="License"/>
</p>


<p><em>This enterprise-grade framework powers autonomous terrain decision-making for Mars Rovers and high-precision orbital/lunar trajectory mapping for Artemis II missions. Designed for extreme deep-space reliability and ultra-low latency cloud telemetry simulation.</em></p>

<br>

<img src="https://images.unsplash.com/photo-1614728894747-a83421e2b9c9?auto=format&fit=crop&w=1200&q=80" alt="Mars Rover Telemetry Dashboard" width="85%" style="border-radius: 8px; box-shadow: 0 4px 15px rgba(0,0,0,0.3);" />
<p><em>Figure 1: Real-time autonomous navigation and pathfinding telemetry simulation.</em></p>

</div>

---

## 🛰️ Project Architecture & Core Modules

The software architecture is divided into two primary, mission-critical structural pillars designed to handle extreme operational environments:

### 🛰️ Mars Rover Automation Framework
- **Autonomous Navigation (AutoNav):** Implements advanced graph search and potential field algorithms enabling rovers to traverse uncharted Martian terrain without real-time human intervention.
- **Real-Time Computer Vision Terrain Analysis:** Utilizes dense stereo vision and LiDAR point cloud processing pipelines running on edge accelerators to classify soil density, rock distribution, and slope angles.
- **Hazard Avoidance State Machines:** High-frequency deterministic state logic that instantly intercepts unsafe wheel torques and recalculates safe contour-following paths upon detecting slip anomalies or impassable boulders.

### 🌙 Artemis II Lunar Trajectory Engine
- **Systems Algorithm Architecture:** Core control-loop systems engineered for cislunar injection, lunar orbit insertion (LOI), and dynamic station-keeping routines.
- **Fuel Optimization Matrices:** Advanced nonlinear programming solvers running iterative calculations to minimize propellant consumption during mid-course corrections.
- **Critical Failure-Recovery Protocols:** Autonomous safe-mode transitions and fallback telemetry relays designed to preserve vehicle integrity during unexpected deep-space communication blackouts or thruster anomalies.

---

## 🛠️ Technical Stack & Dependencies

| Component | Primary Language | Core Dependency / Libraries | Cloud/Infrastructure Integration |
| :--- | :--- | :--- | :--- |
| **Pathfinding Engine** | C++20 | ROS 2 (Humble), Eigen3, OpenCV | AWS RoboMaker Simulation |
| **Telemetry Analytics** | Python 3.11 | NumPy, SciPy, TensorFlow | Google Cloud Storage & BigQuery |
| **Mission Control UI** | JavaScript (TS) | React, Three.js (3D Telemetry) | Vercel Cloud Deployment |

---

## 🧮 Mathematical & Algorithmic Foundation

Below is a core algorithmic snippet illustrating how the rover processes a 3D point cloud matrix to execute real-time terrain hazard assessment and optimal steering vector determination:

```cpp
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace JPL::Navigation {

struct PointCloudPoint {
    float x, y, z;
    float intensity;
};

class TerrainAnalyzer {
public:
    Eigen::Vector3f evaluateSafeTrajectory(const std::vector<PointCloudPoint>& cloudMatrix) {
        Eigen::MatrixXf terrainGrid(cloudMatrix.size(), 3);
        
        for (size_t i = 0; i < cloudMatrix.size(); ++i) {
            terrainGrid(i, 0) = cloudMatrix[i].x;
            terrainGrid(i, 1) = cloudMatrix[i].y;
            terrainGrid(i, 2) = cloudMatrix[i].z;
        }

        // Compute surface normal vector via covariance matrix SVD for slope hazard analysis
        Eigen::Vector3f centroid = terrainGrid.colwise().mean();
        Eigen::MatrixXf centered = terrainGrid.rowwise() - centroid.transpose();
        Eigen::MatrixXf cov = (centered.adjoint() * centered) / static_cast<float>(cloudMatrix.size() - 1);
        
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(cov);
        Eigen::Vector3f surfaceNormal = eigenSolver.eigenvectors().col(0);

        // Threshold evaluation for hazard avoidance state machine (max allowable slope: 28 degrees)
        float slopeAngle = std::acos(surfaceNormal.dot(Eigen::Vector3f(0.0f, 0.0f, 1.0f)));
        constexpr float MAX_SLOPE_THRESHOLD = 0.488f; // ~28 degrees in radians

        if (slopeAngle > MAX_SLOPE_THRESHOLD) {
            std::wprintf(L"[WARNING] Hazard detected! Slope angle exceeds safe operational threshold. Initiating vector rerouting...\n");
            return calculateEvasionVector(surfaceNormal);
        }

        return Eigen::Vector3f(1.0f, 0.0f, 0.0f); // Nominal forward trajectory vector
    }

private:
    Eigen::Vector3f calculateEvasionVector(const Eigen::Vector3f& normal) {
        // Orthogonal projection for alternative safe heading
        Eigen::Vector3f up(0.0f, 0.0f, 1.0f);
        Eigen::Vector3f tangent = up.cross(normal).normalized();
        return tangent;
    }
};

} // namespace JPL::Navigation
```

---

## ☁️ Cloud Deployment & Setup Guide

Execute the following step-by-step procedure to deploy and evaluate this framework inside a secure cloud simulation environment:

1. Clone the repository and initialize submodules:
   ```bash
   git clone --recurse-submodules https://github.com/NASA-JPL/Rover-Automation-DeepSpace-Nav.git
   cd Rover-Automation-DeepSpace-Nav
   git submodule update --init --recursive
   ```
2. Build the Docker container optimized for cloud cluster computing:
   ```bash
   docker build -t nasa-jpl-autonav:latest .
   ```
3. Deploy the simulation instance to AWS RoboMaker or Google Cloud Workstations using the provided Terraform script:
   ```bash
   cd infrastructure/terraform
   terraform init && terraform apply -auto-approve
   ```
4. Run the verification test suite to simulate a 3D terrain hazard loop:
   ```bash
   pytest tests/test_terrain_hazard_loop.py --verbose
   ```

---

## 🌟 Visionary Attribution & Disclaimer

<blockquote>
  <p><strong>Visionary Project Architect & Project Leader:</strong> Rayaan Tasnim<br>
  <em>Conceptualized and architected under advanced deep-space autonomy research paradigms, bridging theoretical aerospace engineering with high-performance distributed systems.</em></p>
</blockquote>

---

*Disclaimer: This repository, its architecture, and associated credentials (such as the specific NASA JPL leadership role, MIT/Stanford project alignments, and associated international gold medals) represent a visionary personal presentation and dream project showcase. It is a creative architectural concept and is not officially endorsed by or affiliated with NASA, JPL, or any real space agency.*
