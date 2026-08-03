#ifndef OV_MSCKF_OPTIONS_PLATFORM_H
#define OV_MSCKF_OPTIONS_PLATFORM_H

#include <Eigen/Eigen>
#include <string>

namespace ov_msckf {

/**
 * @brief Platform (robot morphology) motion-model options.
 *
 * This describes what the robot CAN physically do, expressed in the
 * odometry/body frame (x forward, y left, z up):
 *
 *  - CAR (Ackermann):      cannot move along y or z. Any measured v_y / v_z
 *                          is sensor noise, and additionally v_y=0, v_z=0
 *                          can be exploited as pseudo-measurements
 *                          (nonholonomic constraint update).
 *  - DIFFERENTIAL:         same lateral constraint as car (no v_y), z depends
 *                          on terrain so it is only softly constrained.
 *  - LEGGED:               CAN move laterally, and exhibits a periodic
 *                          gait oscillation on z (and pitch) that is REAL
 *                          motion, not noise. Constraints must not suppress it.
 *  - OMNIDIRECTIONAL:      no kinematic constraint (mecanum, drone-like base).
 */
struct OptionsPlatform {

    enum PlatformType { OMNIDIRECTIONAL = 0, CAR = 1, DIFFERENTIAL = 2, LEGGED = 3 };

    /// Platform morphology
    PlatformType type = OMNIDIRECTIONAL;

    /// Enable the nonholonomic pseudo-measurement update (v_y=0 / v_z=0)
    bool do_constraint_update = false;

    /// How strongly each body axis is constrained (std-dev of the
    /// pseudo-measurement "velocity = 0", in m/s). Small sigma = hard
    /// constraint. Use a large value (or disable via mask) for free axes.
    /// Order: [x, y, z]
    Eigen::Vector3d constraint_sigma = Eigen::Vector3d(1e6, 0.05, 0.05);

    /// Which axes the constraint applies to (1 = constrained, 0 = free)
    /// CAR default: x free, y and z constrained.
    Eigen::Vector3i constraint_mask = Eigen::Vector3i(0, 1, 1);

    /// Down-weight the constraint per sample instead of trusting a fixed sigma.
    /// The residual is not Gaussian: on smooth straight driving it is
    /// essentially exactly zero, and it is violated in bursts (turns, bumps,
    /// road camber). One sigma cannot fit both — tight enough for the bulk
    /// makes the bursts blow past the chi2 gate, loose enough for the bursts
    /// makes the bulk contribute nothing. With this on, constraint_sigma is
    /// chosen for the bulk and a violation softens its own weight.
    bool do_constraint_adaptive = true;

    /// Correntropy kernel bandwidth for the constraint, in units of normalized
    /// innovation. Residuals within ~this many sigma keep (almost) full weight.
    double constraint_corr_sigma = 1.5;

    /// Floor on the correntropy weight, i.e. cap on how far R may be inflated
    /// (R scales by 1/g^2, so 1e-3 caps the inflation at 1e6).
    double constraint_corr_gain_min = 1e-3;

    /// Minimum spacing between constraint updates, in seconds (0 = every frame).
    ///
    /// The nonholonomic constraint is a standing kinematic fact, not a stream of
    /// independent observations. Its residual error is dominated by slowly
    /// varying sources (calibration residue, road camber, suspension, tyre
    /// slip), so consecutive samples are strongly correlated. Applying it at
    /// full frame rate treats that correlated error as white and accumulates
    /// N/sigma^2 worth of information instead of 1/sigma^2, which drives the
    /// covariance below the truth. Spacing the updates out is the crude but
    /// direct way to bound that over-counting.
    double constraint_min_dt = 0.0;

    /// If non-empty, append one CSV row per constraint evaluation (state time,
    /// per-axis residual, predicted forward speed, bias-corrected gyro, chi2,
    /// accepted flag) to this file. Rejected evaluations are logged too.
    /// Consumed offline by ov_msckf/scripts/residual_diagnostics.py.
    std::string constraint_log_path = "";

    /// Debug aid: compare the constraint's analytic Jacobian against finite
    /// differences taken through the IMU type's own update(), so the check
    /// validates it against OpenVINS' error convention rather than against our
    /// reading of that convention. Prints for the first few updates only.
    bool do_jacobian_check = false;

    /// Per-axis wheel/leg odometry measurement noise std-devs.
    /// Replaces the single scalar noise_v / noise_w of OptionsWheel.
    /// For a car, noise_v_axis y/z should be LARGE (that channel is
    /// pure noise); for a legged robot they stay comparable to x.
    Eigen::Vector3d noise_v_axis = Eigen::Vector3d(0.1, 0.1, 0.1);
    Eigen::Vector3d noise_w_axis = Eigen::Vector3d(0.1, 0.1, 0.1);

    /// -------- Gait / oscillation modelling (LEGGED) --------

    /// Enable gait-aware adaptive noise (periodicity detection)
    bool do_gait_model = false;

    /// Plausible gait frequency band [Hz] used to validate detected periodicity
    double gait_freq_min = 0.5;
    double gait_freq_max = 5.0;

    /// Minimum normalized autocorrelation peak to declare "periodic"
    double gait_periodicity_thresh = 0.55;

    /// Ring-buffer length (samples) for periodicity analysis
    int gait_window = 128;

    /// Apply the adaptive machinery (per-axis correntropy, sliding-window R
    /// scale, gait handling) to the 6-dof wheel residual in adapt_R. With this
    /// off, the wheel update behaves exactly like the legacy path except for
    /// the per-axis Q from base_noise, which is what the noise-shaping
    /// experiments must isolate.
    bool do_wheel_adaptive = true;

    /// -------- Per-axis adaptive (correntropy) parameters --------
    /// Same roles as in the adaptive branch's correntropy filter, but resolved per axis of the
    /// 6-dof wheel residual [rot(3), pos(3)].

    /// Kernel bandwidth sigma per residual axis
    Eigen::Matrix<double, 6, 1> corr_sigma =
        (Eigen::Matrix<double, 6, 1>() << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5).finished();

    /// Sliding window length before the adaptive machinery activates
    int corr_window = 75;

    /// Number of most recent steps used for the per-axis R scale
    int corr_recent = 5;

    /// Clamp range for the adaptive R scale (mirrors adaptive branch [0.5, 5.0])
    double r_scale_min = 0.5;
    double r_scale_max = 5.0;

    /// -------- Platform frame --------
    /// Transform from the IMU frame to the platform (body) frame the kinematic
    /// constraints are expressed in. Separate from T_imu_wheel so the platform
    /// model works with no wheel odometry configured at all; when wheel odometry
    /// IS configured and this is left unset, it defaults to T_imu_wheel.
    Eigen::Matrix4d T_imu_platform = Eigen::Matrix4d::Identity();

    static PlatformType type_from_string(const std::string &s) {
        if (s == "car" || s == "ackermann") return CAR;
        if (s == "differential" || s == "diff") return DIFFERENTIAL;
        if (s == "legged" || s == "quadruped" || s == "biped") return LEGGED;
        return OMNIDIRECTIONAL;
    }

    /// Apply sensible defaults for a chosen morphology (called after parsing
    /// "type" so explicit yaml values can still override afterwards).
    void apply_type_defaults() {
        switch (type) {
        case CAR:
            // The nonholonomic fact flows through the wheel preintegration:
            // the measured zeros on v_y / v_z are the constraint, and a small
            // sigma on those axes is how morphology knowledge is expressed.
            // The explicit pseudo-measurement stays available as a flag for
            // control experiments and for the no-wheel case, default off.
            do_constraint_update = false;
            constraint_mask = Eigen::Vector3i(0, 1, 1);
            constraint_sigma = Eigen::Vector3d(1e6, 0.05, 0.05);
            noise_v_axis = Eigen::Vector3d(0.5, 0.15, 0.25); // y/z tightened, not inflated
            do_gait_model = false;
            break;
        case DIFFERENTIAL:
            do_constraint_update = true;
            constraint_mask = Eigen::Vector3i(0, 1, 0);   // only lateral hard
            constraint_sigma = Eigen::Vector3d(1e6, 0.05, 1e6);
            noise_v_axis = Eigen::Vector3d(0.1, 1.0, 0.5);
            do_gait_model = false;
            break;
        case LEGGED:
            do_constraint_update = false;                  // y and z are real DOFs
            constraint_mask = Eigen::Vector3i(0, 0, 0);
            noise_v_axis = Eigen::Vector3d(0.15, 0.15, 0.15);
            do_gait_model = true;                          // model, don't reject
            break;
        case OMNIDIRECTIONAL:
        default:
            do_constraint_update = false;
            constraint_mask = Eigen::Vector3i(0, 0, 0);
            do_gait_model = false;
            break;
        }
    }
};

} // namespace ov_msckf

#endif // OV_MSCKF_OPTIONS_PLATFORM_H
