#ifndef OV_MSCKF_UPDATER_MSCKF_CORR_H
#define OV_MSCKF_UPDATER_MSCKF_CORR_H

#include <deque>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

// Forward declarations
namespace ov_type {
class Type;
}
namespace ov_core {
class Feature;
class FeatureInitializer;
struct FeatureInitializerOptions;
}

namespace ov_msckf {

class State;
class UpdaterOptions;

/**
 * @brief Correntropy-based MSCKF updater for OpenVINS.
 *
 * The class keeps the original UpdaterMSCKF pipeline (cleaning,
 * triangulation, null-space projection, chi2 gating, QR compression, EKFUpdate)
 * and optionally adds robust correntropy weighting.  The robust weighting is
 * controlled by UpdaterOptions, so setting use_correntropy=false makes this
 * updater follow the standard MSCKF measurement model.
 *
 * Important implementation detail: robust weighting is applied to H_f, H_x, and
 * the residual before null-space projection.  This preserves the weighted
 * measurement equation W(H_x dx + H_f df - r)=0 that is later projected into the
 * MSCKF null space.
 */
class UpdaterMSCKF_Corr {

public:
  /// Constructor — same signature as the original UpdaterMSCKF
  UpdaterMSCKF_Corr(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options);

  /// Main update function
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  // Correntropy parameter setters
  void set_use_correntropy(bool enabled) { _options.use_correntropy = enabled; }
  void set_window_size(int N) { _options.correntropy_window_size = N; }
  void set_recent_window(int W) { _options.correntropy_recent_window = W; }
  void set_sigma_cam(double s) { _options.correntropy_sigma = s; }
  void set_adaptive_R(bool enabled) { _options.correntropy_adaptive_R = enabled; }

private:
  UpdaterOptions &_options;

  /// Feature triangulation/refinement helper (same as original UpdaterMSCKF)
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

  /// Chi2 95% confidence interval lookup table
  std::map<int, double> chi_squared_table;

  /// Sliding window of accepted per-feature normalized innovation magnitudes
  std::deque<Eigen::VectorXd> past_innov;

  // ---------- Helpers (defined in .cpp) ----------

  double correntropy_weight(double norm_innov, double sigma) const;

  /// Left-multiplies H_f, H_x, and residual rows by sqrt(Co)=sqrt(diag(G)).
  /// Writes normalized innovations to innov_out for optional adaptive-R history.
  void apply_correntropy_to_block(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res_block,
                                  const Eigen::MatrixXd &R_block, Eigen::VectorXd &innov_out) const;

  /// Estimates a scale factor from the sliding window and updates R.
  Eigen::MatrixXd estimate_R(const Eigen::MatrixXd &R_meas_in) const;

  /// Appends a new innovation vector to the window, discarding the oldest if full.
  void push_innovation_history(const Eigen::VectorXd &innov_this_step);
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MSCKF_CORR_H
