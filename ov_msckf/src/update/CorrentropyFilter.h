#ifndef OV_MSCKF_CORRENTROPY_FILTER_H
#define OV_MSCKF_CORRENTROPY_FILTER_H

#include <Eigen/Dense>
#include <deque>
#include <vector>

namespace ov_msckf {

/**
 * @brief Tuning for CorrentropyFilter.
 */
struct OptionsCorrentropy {

    /// Gaussian kernel bandwidth, in units of NORMALIZED innovation. A residual
    /// this many sigma away keeps weight exp(-1/2) ~ 0.61. Small = aggressive
    /// down-weighting, large = the filter degenerates to a plain Kalman update.
    double kernel_sigma = 1.5;

    /// Floor on the correntropy weight, i.e. cap on how far R may be inflated
    /// (R scales by 1/G, so 1e-3 caps the inflation at 1e3).
    double gain_min = 1e-3;

    /// Samples that must accumulate before the innovation-based noise estimate
    /// activates. Below this the filter applies correntropy weighting only.
    int window = 75;

    /// Most recent samples used for the noise estimate.
    int recent = 5;

    /// Clamp range for the estimated noise scale.
    ///
    /// A floor below 1 lets the estimate TIGHTEN R, which is worth being wary
    /// of when the measurement is a standing model assertion rather than an
    /// independent sensor reading: a small residual is then not evidence that R
    /// is too big, because the update itself is what made it small, and that
    /// closes a loop (small residual -> tighter R -> stronger correction).
    /// This floor is what the urban39 result is most sensitive to, and it does
    /// not point one way: with wheel odometry also running, a 0.5 floor measured
    /// 10.590 m against 12.742 m at 1.0; with the platform update as the only
    /// source, the same two settings gave 13.511 m and 10.697 m. A single clamp
    /// swinging the result by 3 m in both directions is itself the finding —
    /// that regime is not a robust contribution. 1.0 is the default because it
    /// is the setting that cannot manufacture confidence.
    double r_scale_min = 1.0;
    double r_scale_max = 5.0;
};

/**
 * @brief Maximum-correntropy weighting combined with innovation-based noise
 *        estimation, for a linear(ized) Kalman update.
 *
 * This is the fusion of the two mechanisms, kept in one place and independent
 * of any particular sensor:
 *
 *  1) CORRENTROPY. For each measurement channel a Gaussian kernel is evaluated
 *     on the normalized innovation n_i = |res_i| / sqrt(S_ii), where
 *     S = H P H^T + R is the covariance the filter predicts for the residual,
 *
 *         G_i = exp( -n_i^2 / (2 sigma^2) ),      G_i in (0, 1]
 *
 *     so a residual consistent with its own covariance keeps G ~ 1 and a gross
 *     one is pushed toward 0. The reference formulation puts G in the gain,
 *
 *         K = C P H^T (H C P H^T + R)^-1,   C = diag(G_i)
 *
 *     which for a common G factors exactly into the ordinary Kalman gain with
 *     R replaced by R/G. This class therefore applies the weight on the
 *     COVARIANCE side, as the congruence
 *
 *         R <- C^-1/2 R C^-1/2                (R_ii / G_i on the diagonal)
 *
 *     which is the same estimator, generalized to per-channel weights. Two
 *     things are gained over touching the gain directly: a congruence provably
 *     keeps R positive definite for any positive weight, and cross-covariance
 *     terms scale with the diagonal instead of being left behind — R is not
 *     diagonal in general. It also means the chi2 gate and the update see one
 *     and the same R, so a soft-weighted outlier is not also rejected outright.
 *
 *  2) NOISE ESTIMATION. A sliding window of normalized innovations gives the
 *     empirical mean square per channel; under a consistent filter it is 1, so
 *     the departure from 1 is a direct multiplicative correction on R. This is
 *     the bounded form of the classical innovation-covariance estimator
 *     R = mean(y y^T): expressed as a clamped SCALE on the modelled R rather
 *     than as a replacement for it, so a short window cannot collapse R toward
 *     zero or let it run away, and so the structure of R (units, cross-terms,
 *     the model's own knowledge) survives the adaptation.
 *
 * Both mechanisms normalize against S rather than R, which is what makes
 * mean(n^2) = 1 the meaningful target: the residual's covariance is S, not R,
 * so dividing by R alone measures the residual against something it was never
 * distributed as, and the resulting weight and scale answer the wrong question.
 *
 * Channels whose variance is already accounted for by an explicit model (see
 * PlatformMotionModel's gait absorption) can be marked as explained and are
 * passed through untouched: down-weighting structured, correctly modelled
 * motion would fight the model instead of helping it.
 */
class CorrentropyFilter {

public:
    /// Per-channel diagnostics from the last apply()
    struct Channel {
        double norm_innov = 0.0;  ///< |res_i| / sqrt(S_ii) as seen on entry
        double gain = 1.0;        ///< correntropy weight G_i in (0,1]
        double r_scale = 1.0;     ///< clamped innovation-based noise scale
    };

    explicit CorrentropyFilter(const OptionsCorrentropy &opts) : _opts(opts) {}

    /**
     * @brief Weight and adapt the measurement covariance in place.
     * @param R          m x m covariance, adapted in place
     * @param res        m-vector residual for this update
     * @param HPHt       m x m state-uncertainty contribution to the residual
     *                   covariance, i.e. H P H^T with P the marginal covariance
     *                   over the states this update touches. Normalization uses
     *                   S = H P H^T + R taken BEFORE the adaptation, so the
     *                   weight measures the residual against what the filter
     *                   predicted, not against what this call is about to
     *                   decide.
     * @param explained  Optional per-channel flags; a channel marked true keeps
     *                   G = 1 and skips the noise estimate
     */
    void apply(Eigen::MatrixXd &R, const Eigen::VectorXd &res,
               const Eigen::MatrixXd &HPHt,
               const std::vector<bool> &explained = std::vector<bool>());

    /// True once the innovation window has filled (noise estimation active)
    bool active() const { return (int)_window.size() >= _opts.window; }

    /// Diagnostics for channel i of the last apply()
    const Channel &channel(int i) const { return _channels.at(i); }

    /// Number of channels seen by the last apply()
    int dim() const { return (int)_channels.size(); }

    /// Drop all history (measurement dimension changed, filter restarted)
    void reset();

private:
    OptionsCorrentropy _opts;

    /// Sliding window of normalized innovation vectors
    std::deque<Eigen::VectorXd> _window;

    std::vector<Channel> _channels;
};

} // namespace ov_msckf

#endif // OV_MSCKF_CORRENTROPY_FILTER_H
