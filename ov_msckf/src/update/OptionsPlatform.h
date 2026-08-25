#ifndef OV_MSCKF_OPTIONS_PLATFORM_H
#define OV_MSCKF_OPTIONS_PLATFORM_H

#include <Eigen/Eigen>
#include <string>

#include "CorrentropyFilter.h"

namespace ov_msckf {

/**
 * @brief Platform (robot morphology) motion-model options.
 *
 * This describes what the robot CAN physically do, expressed in the platform
 * body frame (x forward, y left, z up), and nothing else. It carries no wheel
 * odometry parameters: the platform model is a self-contained estimator that
 * runs with or without an odometry stream, and the wheel updater owns its own
 * noise model (see OptionsWheel).
 *
 *  - CAR (Ackermann):      cannot slide sideways. v_y = 0 is a measurement the
 *                          morphology supplies at every instant. v_z is NOT
 *                          asserted: suspension travel, pitch under braking
 *                          and road grade violate it continuously.
 *  - DIFFERENTIAL:         same lateral fact, z likewise free over terrain.
 *  - LEGGED:               y and z are real DOFs, but their gait-cycle MEAN is
 *                          zero. Asserted loosely, with the periodic component
 *                          absorbed into the covariance rather than fought.
 *  - OMNIDIRECTIONAL:      no kinematic knowledge (mecanum, drone-like base);
 *                          the platform updater has nothing to contribute.
 */
struct OptionsPlatform {

    enum PlatformType { OMNIDIRECTIONAL = 0, CAR = 1, DIFFERENTIAL = 2, LEGGED = 3 };

    /// Platform morphology
    PlatformType type = OMNIDIRECTIONAL;

    /// Which body axes the morphology pins (1 = the model asserts v_axis = 0).
    /// Order: [x, y, z]
    Eigen::Vector3i meas_mask = Eigen::Vector3i(0, 0, 0);

    /// Std-dev of the "body velocity on this axis is zero" measurement, in m/s.
    /// This is how far the true velocity may depart from zero for reasons the
    /// model does not resolve: tyre slip on y, suspension motion on z, gait
    /// excursion on a legged base. Order: [x, y, z]; entries of unmasked axes
    /// are unused.
    Eigen::Vector3d meas_sigma = Eigen::Vector3d(1e6, 0.05, 0.05);

    /// -------- Slowly varying offset --------
    ///
    /// The morphology says the body velocity on a pinned axis is zero. What the
    /// sensors report on that axis is zero PLUS a slowly varying offset: mount
    /// calibration residue, road camber, suspension trim, a platform frame whose
    /// origin is not quite the true zero-lateral-velocity point. Measured on
    /// urban39 that offset is real — 22.4% of 30 s windows of the lateral
    /// residual fall outside the +/-3 sigma/sqrt(N) band a white residual would
    /// stay inside, and the residual's decorrelation time is far shorter than
    /// the drift, so the structure is in the mean, not the scatter.
    ///
    /// Asserting a hard zero therefore injects that offset into the state a few
    /// thousand times per run. Integrated over urban39 a persistent 0.01 m/s of
    /// lateral error is ~19 m of cross-track, the same order as the whole APE.
    /// So the offset is estimated here and removed before the update, turning
    /// the assertion from "lateral velocity is zero" into "lateral velocity does
    /// not depart from its own slow trend" — which is the part of the kinematic
    /// fact the sensors actually support.

    /// Time constant of the offset estimate, in seconds (0 disables it and
    /// restores the hard-zero assertion). Must be long compared with a turn, so
    /// genuine lateral motion is not absorbed into the offset.
    double bias_tau = 15.0;

    /// How well the offset is believed to be known, in m/s. Added in quadrature
    /// to meas_sigma, because a measurement corrected by an estimate is no more
    /// certain than the estimate.
    double bias_sigma = 0.02;

    /// Minimum spacing between platform updates, in seconds (0 = every frame).
    ///
    /// The morphology is a standing fact, not a stream of independent
    /// observations. Its residual error is dominated by slowly varying sources
    /// (calibration residue, road camber, suspension, tyre slip), so
    /// consecutive samples are strongly correlated. Applying the measurement at
    /// full frame rate treats that correlated error as white and accumulates
    /// N/sigma^2 worth of information instead of 1/sigma^2, which drives the
    /// covariance below the truth. Spacing the updates out bounds that.
    double update_min_dt = 0.2;

    /// Multiplier on the 95% chi2 gate that backstops the correntropy weighting
    /// for violations it cannot absorb (hard skid, sensor fault).
    double chi2_mult = 1.0;

    /// Correntropy weighting + innovation-based noise estimation. This is the
    /// estimator itself, not an option on top of it, so it has no on/off flag:
    /// the parameters below say how sharp the kernel is and how far the noise
    /// estimate may move, and a large kernel_sigma is what "plain Kalman" means
    /// here.
    OptionsCorrentropy corr;

    /// -------- Gait / oscillation modelling --------
    /// Always evaluated. On a wheeled platform no periodicity exists in the
    /// band below and the test simply never fires, so this needs no flag
    /// either; on a legged one it is what keeps the gait from being mistaken
    /// for noise.

    /// Plausible gait frequency band [Hz] used to validate detected periodicity
    double gait_freq_min = 0.5;
    double gait_freq_max = 5.0;

    /// Minimum normalized autocorrelation peak to declare "periodic"
    double gait_periodicity_thresh = 0.55;

    /// Ring-buffer length (samples) for periodicity analysis
    int gait_window = 128;

    /// -------- Platform frame --------
    /// Transform from the platform (body) frame the kinematics are expressed in
    /// to the IMU frame. Same convention as T_imu_wheel:
    ///   T = [R_OtoI | p_OinI ; 0 0 0 1]
    /// Required: the platform model does not fall back to the wheel extrinsics,
    /// because it must mean the same thing whether or not wheel odometry is
    /// configured at all.
    Eigen::Matrix4d T_imu_platform = Eigen::Matrix4d::Identity();

    /// -------- Diagnostics --------

    /// If non-empty, append one CSV row per platform update evaluation (state
    /// time, per-axis residual, predicted forward speed, bias-corrected gyro,
    /// chi2, correntropy weights, accepted flag) to this file. Rejected
    /// evaluations are logged too. Consumed offline by
    /// ov_msckf/scripts/residual_diagnostics.py.
    std::string log_path = "";

    /// Debug aid: compare the analytic Jacobian against finite differences taken
    /// through the IMU type's own update(), so the check validates it against
    /// OpenVINS' error convention rather than against our reading of that
    /// convention. Prints for the first few updates only.
    bool do_jacobian_check = false;

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
            // Lateral only. Asserting v_z at the same tightness moved urban39
            // APE from 10.94 m to 37.30 m with nothing else changed: a car's
            // vertical velocity is nominally zero but violated continuously by
            // suspension travel, pitch and grade transitions.
            meas_mask = Eigen::Vector3i(0, 1, 0);
            // 0.06, close to the 0.040 m/s the lateral residual scatters at on
            // urban39. Loosening it to 0.15 was tried on the theory that the
            // residual's low-frequency bias needed covering, and it made things
            // worse (12.742 m -> 13.024 m APE), because the correntropy kernel
            // works on NORMALIZED innovation: a loose meas_sigma makes every
            // residual look consistent and silently disables the down-weighting
            // (max R inflation collapsed from 526x to 2.8x, chi2 never reached
            // its gate). meas_sigma and corr.kernel_sigma are one setting in two
            // parts — moving either alone changes how robust the update is.
            meas_sigma = Eigen::Vector3d(1e6, 0.06, 1e6);
            update_min_dt = 0.2;
            break;
        case DIFFERENTIAL:
            meas_mask = Eigen::Vector3i(0, 1, 0);
            meas_sigma = Eigen::Vector3d(1e6, 0.05, 1e6);
            update_min_dt = 0.2;
            break;
        case LEGGED:
            // y and z are real DOFs, so the assertion is loose and its job is
            // to anchor the gait-cycle mean. The oscillation itself is detected
            // and absorbed into R rather than treated as an outlier.
            meas_mask = Eigen::Vector3i(0, 1, 1);
            meas_sigma = Eigen::Vector3d(1e6, 0.30, 0.30);
            // Every frame: the periodicity buffers only see rate-limited
            // samples, so throttling to 5 Hz would put the top of the gait band
            // above Nyquist and the oscillation would alias instead of being
            // detected. The gait absorption is what keeps this from
            // over-counting, in place of the spacing a wheeled base needs.
            update_min_dt = 0.0;
            break;
        case OMNIDIRECTIONAL:
        default:
            meas_mask = Eigen::Vector3i(0, 0, 0);
            break;
        }
    }
};

} // namespace ov_msckf

#endif // OV_MSCKF_OPTIONS_PLATFORM_H
