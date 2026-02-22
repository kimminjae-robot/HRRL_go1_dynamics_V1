#include "SlopeEstimator.h"
#include <algorithm>
#include <cmath>

SlopeEstimator::SlopeEstimator()
: SlopeEstimator(Config{}) {}

SlopeEstimator::SlopeEstimator(const Config& cfg)
: cfg_(cfg) {
  reset(Eigen::Vector3d(0,0,1));
}

void SlopeEstimator::reset(const Eigen::Vector3d& normal_w_init) {
  prior_normal_w_ = normal_w_init.normalized();
  if (cfg_.enforce_upward_normal) prior_normal_w_ = enforceUp(prior_normal_w_);

  last_out_ = Output{};
  last_out_.valid = true;
  last_out_.normal_w = prior_normal_w_;
  last_out_.d = 0.0;
  normalToRollPitch(last_out_.normal_w, last_out_.roll, last_out_.pitch);
}

SlopeEstimator::Output SlopeEstimator::update(const std::vector<InputFoot>& stance_feet) {
  Output out;
  out.valid = false;
  out.used_points = 0;

  // Filter non-positive weights
  std::vector<InputFoot> pts;
  pts.reserve(stance_feet.size());
  for (const auto& f : stance_feet) {
    if (f.weight > 0.0) pts.push_back(f);
  }

  out.used_points = static_cast<int>(pts.size());
  if (pts.size() < 2) {
    // Not enough points: keep prior
    out.valid = false;
    out.normal_w = prior_normal_w_;
    out.d = last_out_.d;
    normalToRollPitch(out.normal_w, out.roll, out.pitch);
    last_out_ = out;
    return last_out_;
  }

  Eigen::Vector3d n_new(0,0,1);
  double d_new = 0.0;
  double rms_new = 0.0;
  bool ok = false;

  if (pts.size() >= 3) {
    ok = fitPlaneWeightedSVD(pts, n_new, d_new, rms_new);
  } else { // exactly 2
    ok = fitPlaneTwoPointsWithPrior(pts, prior_normal_w_, n_new, d_new, rms_new);
  }

  if (!ok) {
    // fallback to prior
    n_new = prior_normal_w_;
    d_new = last_out_.d;
    rms_new = last_out_.rms_point_to_plane;
  }

  if (cfg_.enforce_upward_normal) n_new = enforceUp(n_new);

  // Optional blend toward prior (rarely needed)
  if (cfg_.prior_normal_blend > 0.0) {
    n_new = emaNormal(n_new, prior_normal_w_, cfg_.prior_normal_blend);
  }

  // EMA smoothing (on normal)
  Eigen::Vector3d n_filt = emaNormal(prior_normal_w_, n_new, cfg_.ema_alpha);

  // Plane offset d should match filtered normal: recompute d from weighted centroid
  // d = -n^T pbar
  double wsum = 0.0;
  Eigen::Vector3d pbar = Eigen::Vector3d::Zero();
  for (const auto& p : pts) {
    wsum += p.weight;
    pbar += p.weight * p.p_w;
  }
  if (wsum < cfg_.min_points_weight_sum) {
    // keep previous
    out.valid = false;
    out.normal_w = prior_normal_w_;
    out.d = last_out_.d;
    normalToRollPitch(out.normal_w, out.roll, out.pitch);
    last_out_ = out;
    return last_out_;
  }
  pbar /= wsum;

  double d_filt = -n_filt.dot(pbar);

  // Output
  out.valid = ok;
  out.normal_w = n_filt;
  out.d = d_filt;
  out.rms_point_to_plane = rms_new; // (rms based on n_new); you can recompute if needed
  normalToRollPitch(out.normal_w, out.roll, out.pitch);

  // update internal state
  prior_normal_w_ = out.normal_w;
  last_out_ = out;
  return last_out_;
}

bool SlopeEstimator::fitPlaneWeightedSVD(const std::vector<InputFoot>& pts,
                                        Eigen::Vector3d& n_out,
                                        double& d_out,
                                        double& rms_out) {
  // Weighted centroid
  double wsum = 0.0;
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  for (const auto& p : pts) {
    wsum += p.weight;
    c += p.weight * p.p_w;
  }
  if (wsum < cfg_.min_points_weight_sum) return false;
  c /= wsum;

  // Weighted covariance
  Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
  for (const auto& p : pts) {
    const Eigen::Vector3d q = p.p_w - c;
    S += p.weight * (q * q.transpose());
  }

  // Check rank/degeneracy
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(S);
  if (es.info() != Eigen::Success) return false;

  const auto& evals = es.eigenvalues();   // ascending
  const auto& evecs = es.eigenvectors();  // columns

  // If smallest eigenvalue is ~0 but second is also ~0 => collinear/degenerate
  if (evals(1) < cfg_.rank_epsilon) {
    return false;
  }

  Eigen::Vector3d n = evecs.col(0); // smallest eigenvalue direction
  n.normalize();

  // plane offset
  double d = -n.dot(c);

  // RMS point-to-plane distance (weighted)
  double num = 0.0;
  double den = 0.0;
  for (const auto& p : pts) {
    double dist = n.dot(p.p_w) + d; // signed
    num += p.weight * dist * dist;
    den += p.weight;
  }
  rms_out = (den > 0.0) ? std::sqrt(num / den) : 0.0;

  n_out = n;
  d_out = d;
  return true;
}

bool SlopeEstimator::fitPlaneTwoPointsWithPrior(const std::vector<InputFoot>& pts2,
                                                const Eigen::Vector3d& prior_n,
                                                Eigen::Vector3d& n_out,
                                                double& d_out,
                                                double& rms_out) {
  if (pts2.size() != 2) return false;

  const Eigen::Vector3d p0 = pts2[0].p_w;
  const Eigen::Vector3d p1 = pts2[1].p_w;
  const Eigen::Vector3d e = p1 - p0;
  const double e_norm = e.norm();
  if (e_norm < cfg_.two_point_min_separation) return false;

  // We want a plane that contains line (p0,p1):
  // normal n must satisfy n · e = 0.
  // And we want n as close as possible to prior_n.
  // Solution: project prior_n onto subspace orthogonal to e:
  // n = prior_n - (prior_n·ê) ê, then normalize.
  const Eigen::Vector3d eh = e / e_norm;
  Eigen::Vector3d n = prior_n - (prior_n.dot(eh)) * eh;
  const double nn = n.norm();
  if (nn < 1e-9) {
    // prior was parallel to e; choose any normal orthogonal to e
    // pick axis not parallel to e
    Eigen::Vector3d a(0,0,1);
    if (std::abs(eh.dot(a)) > 0.9) a = Eigen::Vector3d(0,1,0);
    n = eh.cross(a);
    if (n.norm() < 1e-9) return false;
  }
  n.normalize();

  // For d, use weighted centroid of 2 points
  double wsum = pts2[0].weight + pts2[1].weight;
  if (wsum < cfg_.min_points_weight_sum) return false;
  Eigen::Vector3d c = (pts2[0].weight * p0 + pts2[1].weight * p1) / wsum;

  double d = -n.dot(c);

  // RMS distance (2 points will be ~0 by construction, but compute anyway)
  double num = 0.0;
  double den = 0.0;
  for (const auto& p : pts2) {
    double dist = n.dot(p.p_w) + d;
    num += p.weight * dist * dist;
    den += p.weight;
  }
  rms_out = (den > 0.0) ? std::sqrt(num / den) : 0.0;

  n_out = n;
  d_out = d;
  return true;
}

void SlopeEstimator::normalToRollPitch(const Eigen::Vector3d& n,
                                      double& roll,
                                      double& pitch) const {
  // Using convention:
  // roll  = atan2(n_y, n_z)
  // pitch = atan2(-n_x, sqrt(n_y^2 + n_z^2))
  const double nx = n.x();
  const double ny = n.y();
  const double nz = n.z();
  roll  = std::atan2(ny, nz);
  pitch = std::atan2(-nx, std::sqrt(ny*ny + nz*nz));
}

Eigen::Vector3d SlopeEstimator::emaNormal(const Eigen::Vector3d& prev_n,
                                          const Eigen::Vector3d& new_n,
                                          double alpha) const {
  // Ensure shortest-arc direction (avoid sign flips)
  Eigen::Vector3d nn = new_n;
  if (prev_n.dot(nn) < 0.0) nn = -nn;

  Eigen::Vector3d blended = (1.0 - alpha) * prev_n + alpha * nn;
  double norm = blended.norm();
  if (norm < 1e-12) return prev_n;
  blended /= norm;
  return blended;
}

Eigen::Vector3d SlopeEstimator::enforceUp(const Eigen::Vector3d& n) const {
  return (n.z() >= 0.0) ? n : (-n);
}
