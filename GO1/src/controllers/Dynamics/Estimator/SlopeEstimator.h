#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cstdint>

class SlopeEstimator {
public:
  struct Config {
    // World frame convention assumed: x forward, y left, z up.
    // Ensure you pass foot positions in that world frame.

    double ema_alpha = 0.15;     // [0..1], larger -> less smoothing
    double min_points_weight_sum = 1e-6;

    // Two-point handling
    double two_point_min_separation = 1e-4;  // [m], avoid degeneracy
    double prior_normal_blend = 0.0; // optional extra blending with prior (0..1). Usually 0.

    // Normal orientation: enforce n.z >= 0
    bool enforce_upward_normal = true;

    // If feet points are nearly collinear or rank-deficient, we fallback to prior
    double rank_epsilon = 1e-9;
  };

  struct InputFoot {
    Eigen::Vector3d p_w; // foot position in WORLD
    double weight = 1.0; // confidence weight (>=0). e.g., from contact force or slip probability
    int id = -1;         // optional identifier
  };

  struct Output {
    bool valid = false;
    int used_points = 0;

    Eigen::Vector3d normal_w = Eigen::Vector3d(0,0,1); // unit normal in world
    double d = 0.0;                                    // plane: n^T x + d = 0

    double roll = 0.0;   // [rad] ground roll
    double pitch = 0.0;  // [rad] ground pitch

    // Debug / quality
    double rms_point_to_plane = 0.0; // weighted RMS distance [m]
  };

  SlopeEstimator();                       
  explicit SlopeEstimator(const Config& cfg);  
  
  void reset(const Eigen::Vector3d& normal_w_init = Eigen::Vector3d(0,0,1));

  // Update with stance feet. You can pass 2,3,4 points (or more).
  // Returns latest Output (also stored internally).
  Output update(const std::vector<InputFoot>& stance_feet);

  const Output& last() const { return last_out_; }
  const Eigen::Vector3d& priorNormal() const { return prior_normal_w_; }

private:
  Config cfg_;
  Output last_out_;
  Eigen::Vector3d prior_normal_w_; // stored unit vector

  // Core plane fitting
  bool fitPlaneWeightedSVD(const std::vector<InputFoot>& pts,
                           Eigen::Vector3d& n_out,
                           double& d_out,
                           double& rms_out);

  bool fitPlaneTwoPointsWithPrior(const std::vector<InputFoot>& pts2,
                                  const Eigen::Vector3d& prior_n,
                                  Eigen::Vector3d& n_out,
                                  double& d_out,
                                  double& rms_out);

  // Normal -> (roll,pitch)
  void normalToRollPitch(const Eigen::Vector3d& n, double& roll, double& pitch) const;

  // EMA smoothing on normal
  Eigen::Vector3d emaNormal(const Eigen::Vector3d& prev_n,
                            const Eigen::Vector3d& new_n,
                            double alpha) const;

  // Utility
  Eigen::Vector3d enforceUp(const Eigen::Vector3d& n) const;
};
