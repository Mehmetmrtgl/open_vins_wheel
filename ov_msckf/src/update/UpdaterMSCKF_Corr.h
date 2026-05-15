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
 * Structurally identical to the original UpdaterMSCKF
 * (cleaning + triangulation + gauss-newton + chi2 + QR + EKFUpdate).
 *
 * Additionally implements two mechanisms:
 *  1) Correntropy weight (Co): for feature blocks that pass the chi2 gate,
 *     a weight G is applied based on normalized innovation magnitude.
 *     The Co weight is embedded into H and res via left-multiplication;
 *     QR and EKFUpdate then proceed identically to the original flow.
 *  2) Sliding-window R estimation: a scale factor is estimated from the
 *     normalized innovation statistics of the last N_window steps
 *     and the compressed R matrix is updated accordingly.
 *
 *  - Before the window fills, Co = I -> standard UpdaterMSCKF behavior.
 */
class UpdaterMSCKF_Corr {

public:
    /// Constructor — same signature as the original UpdaterMSCKF
    UpdaterMSCKF_Corr(UpdaterOptions &options,
                      ov_core::FeatureInitializerOptions &feat_init_options);

    /// Main update function
    void update(std::shared_ptr<State> state,
                std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

    // Correntropy parameter setters
    void set_window_size(int N)   { N_window  = N; }
    void set_recent_window(int W) { W_recent  = W; }
    void set_sigma_cam(double s)  { sigma_cam = s; }

private:
    UpdaterOptions &_options;

    /// Feature triangulation/refinement helper (same as original UpdaterMSCKF)
    std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

    /// Chi2 95% confidence interval lookup table
    std::map<int, double> chi_squared_table;

    // ---------- Correntropy parameters ----------
    int    N_window  = 75;
    int    W_recent  = 5;
    double sigma_cam = 10.0;

    /// Sliding window of per-feature normalized innovation magnitudes
    std::deque<Eigen::VectorXd> past_innov;

    // ---------- Helpers (defined in .cpp) ----------

    double correntropy_weight(double norm_innov, double sigma) const;

    /// Left-multiplies H and res per-feature by Co=diag(G).
    /// Does nothing if the window is not yet full (Co = I).
    /// Writes normalized innovations to innov_out for appending to the window.
    void apply_correntropy_to_block(Eigen::MatrixXd &H_block,
                                    Eigen::VectorXd &res_block,
                                    const Eigen::MatrixXd &R_block,
                                    Eigen::VectorXd &innov_out) const;

    /// Estimates a scale factor from the sliding window and updates R.
    Eigen::MatrixXd estimate_R(const Eigen::MatrixXd &R_meas_in) const;

    /// Appends a new innovation vector to the window, discarding the oldest if full.
    void push_innovation_history(const Eigen::VectorXd &innov_this_step);
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MSCKF_CORR_H
