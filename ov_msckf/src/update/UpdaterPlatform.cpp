#include "UpdaterPlatform.h"

#include <boost/math/distributions/chi_squared.hpp>

#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "utils/print.h"

using namespace ov_msckf;
using namespace Eigen;

namespace {
inline Matrix3d skew_x(const Vector3d &v) {
    Matrix3d m;
    m <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return m;
}
} // namespace

UpdaterPlatform::UpdaterPlatform(std::shared_ptr<State> state, const OptionsPlatform &opts)
    : _state(state), _opts(opts), _model(std::make_shared<PlatformMotionModel>(opts)) {

    // Extrinsics: T = [R_OtoI | p_OinI], the platform origin expressed in the
    // IMU frame. Same reading as UpdaterWheel's T_imu_wheel, so the two 4x4
    // yaml blocks cannot silently mean different things.
    const Matrix3d R_OtoI = _opts.T_imu_platform.block<3, 3>(0, 0);
    _R_ItoO = R_OtoI.transpose();
    _p_OinI = _opts.T_imu_platform.block<3, 1>(0, 3);

    for (int dof = 1; dof <= 3; dof++) {
        boost::math::chi_squared chi_squared_dist(dof);
        _chi2_table[dof] = boost::math::quantile(chi_squared_dist, 0.95);
    }

    if (!_opts.log_path.empty()) {
        _log.open(_opts.log_path, std::ios::out | std::ios::trunc);
        if (_log.is_open())
            _log << "t,axis0,res0,axis1,res1,vx_pred,whx,why,whz,chi2,applied,g0,g1,res0_used,bias0" << std::endl;
    }
}

bool UpdaterPlatform::try_update(const Vector3d &w_m) {

    // An omnidirectional base pins nothing, so there is no measurement to make.
    const std::vector<int> &axes = _model->axes();
    if (axes.empty()) return false;
    const int m = (int)axes.size();

    // Rate limit: the morphology is one standing fact whose residual error is
    // strongly autocorrelated, so applying it every frame over-counts it.
    if (_opts.update_min_dt > 0.0) {
        const double t_now = _state->_timestamp;
        if (_last_update_time >= 0.0 && (t_now - _last_update_time) < _opts.update_min_dt)
            return false;
    }

    // ------------------------------------------------------------------
    // Predicted measurement
    //   v_OinO = R_ItoO * ( R_GtoI * v_IinG + skew(w_hat) * p_OinI )
    // ------------------------------------------------------------------
    std::shared_ptr<ov_type::IMU> imu = _state->_imu;

    Matrix3d R_GtoI = imu->Rot();
    Vector3d v_IinG = imu->vel();
    Vector3d b_g = imu->bias_g();
    Vector3d w_hat = w_m - b_g;

    Vector3d v_OinO = _R_ItoO * (R_GtoI * v_IinG + skew_x(w_hat) * _p_OinI);

    // ------------------------------------------------------------------
    // Residual: z_meas (= 0) minus prediction, on the pinned axes only.
    //
    // res_raw is what the morphology's hard zero implies; res is what actually
    // reaches the filter, with the slowly varying offset (calibration residue,
    // camber, suspension trim, platform-origin error) removed. The raw one still
    // drives the model's own estimators, which is what keeps that offset
    // tracked rather than injected.
    // ------------------------------------------------------------------
    VectorXd res_raw(m);
    for (int k = 0; k < m; k++) res_raw(k) = 0.0 - v_OinO(axes[k]);
    VectorXd res = _model->debias(res_raw);

    // ------------------------------------------------------------------
    // Jacobians w.r.t. [theta_GtoI (3), v_IinG (3), b_g (3)]
    //
    // OpenVINS left-quaternion error convention: R_GtoI = exp(-skew(dth)) Rhat
    //   d(R_GtoI * v)/d(dth) =  skew(Rhat * v)
    //   d(v_OinO)/d(v_IinG)  =  R_ItoO * R_GtoI
    //   d(skew(w_hat) p)/d(b_g) = d(-skew(p) w_hat)/d(b_g) = +skew(p)
    //   => d(v_OinO)/d(b_g)  =  R_ItoO * skew(p_OinI)
    //
    // OpenVINS EKFUpdate expects H = d(h)/dx with res = z - h, so H is built as
    // dh/dx directly and res stays -h as computed above.
    // ------------------------------------------------------------------
    Matrix3d dh_dth = _R_ItoO * skew_x(R_GtoI * v_IinG);
    Matrix3d dh_dv = _R_ItoO * R_GtoI;
    Matrix3d dh_dbg = _R_ItoO * skew_x(_p_OinI);

    // ------------------------------------------------------------------
    // Numerical Jacobian check. The perturbation goes through IMU::update(), so
    // this validates the analytic Jacobian against OpenVINS' own error
    // convention instead of against our reading of it (which would be circular).
    // IMU::update takes dx = [theta(3) p(3) v(3) bg(3) ba(3)].
    // ------------------------------------------------------------------
    if (_opts.do_jacobian_check && _jac_checks_done < 5) {
        _jac_checks_done++;
        const double eps = 1e-6;

        auto h_of = [&](const std::shared_ptr<ov_type::IMU> &im) -> Vector3d {
            Vector3d w_local = w_m - im->bias_g();
            return _R_ItoO * (im->Rot() * im->vel() + skew_x(w_local) * _p_OinI);
        };

        Vector3d h0 = h_of(imu);
        MatrixXd Hnum(3, 9);
        const int idx[9] = {0, 1, 2, 6, 7, 8, 9, 10, 11};   // theta, v, bg
        for (int c = 0; c < 9; c++) {
            auto clone = std::dynamic_pointer_cast<ov_type::IMU>(imu->clone());
            VectorXd dxp = VectorXd::Zero(15);
            dxp(idx[c]) = eps;
            clone->update(dxp);
            Hnum.col(c) = (h_of(clone) - h0) / eps;
        }

        MatrixXd Hana(3, 9);
        Hana.block<3, 3>(0, 0) = dh_dth;
        Hana.block<3, 3>(0, 3) = dh_dv;
        Hana.block<3, 3>(0, 6) = dh_dbg;

        auto blk = [&](int c0) {
            double e = (Hana.block<3, 3>(0, c0) - Hnum.block<3, 3>(0, c0)).cwiseAbs().maxCoeff();
            double s = Hana.block<3, 3>(0, c0).cwiseAbs().maxCoeff();
            return std::make_pair(e, s);
        };
        auto th = blk(0), vv = blk(3), bg = blk(6);
        PRINT_INFO("[PLATFORM][JAC] blok mutlak hata / buyukluk:  dtheta %.3e/%.3e   dv %.3e/%.3e   dbg %.3e/%.3e\n",
                   th.first, th.second, vv.first, vv.second, bg.first, bg.second);
        for (int r = 0; r < 3; r++) {
            PRINT_INFO("[PLATFORM][JAC]  dtheta satir%d  ana[%+.5f %+.5f %+.5f]  num[%+.5f %+.5f %+.5f]\n", r,
                       Hana(r, 0), Hana(r, 1), Hana(r, 2), Hnum(r, 0), Hnum(r, 1), Hnum(r, 2));
        }
    }

    std::vector<std::shared_ptr<ov_type::Type>> x_order;
    x_order.push_back(imu->q());
    x_order.push_back(imu->v());
    x_order.push_back(imu->bg());

    MatrixXd H = MatrixXd::Zero(m, 9);
    for (int k = 0; k < m; k++) {
        const int a = axes[k];
        H.block<1, 3>(k, 0) = dh_dth.row(a);
        H.block<1, 3>(k, 3) = dh_dv.row(a);
        H.block<1, 3>(k, 6) = dh_dbg.row(a);
    }

    // ------------------------------------------------------------------
    // The covariance this Kalman step is performed with: the morphology's own
    // sigma, then gait absorption, then the correntropy weight and the
    // innovation-based noise estimate. Applied BEFORE the gate so the gate and
    // the update see one and the same R — gating on the raw R would penalise a
    // violation twice, once through the widened covariance and again through
    // rejection.
    //
    // H P H^T goes in so the weighting normalizes against the residual's own
    // covariance S = H P H^T + R rather than against R alone.
    // ------------------------------------------------------------------
    MatrixXd P_marg = StateHelper::get_marginal_covariance(_state, x_order);
    MatrixXd HPHt = H * P_marg * H.transpose();

    MatrixXd R = _model->base_R();
    _model->adapt_R(R, res, HPHt);

    // ------------------------------------------------------------------
    // Chi2 gate: backstop for violations beyond what the weighting can absorb
    // (hard skid, sensor fault).
    // ------------------------------------------------------------------
    MatrixXd S = HPHt + R;
    double chi2 = res.dot(S.llt().solve(res));
    const double gate = _opts.chi2_mult * _chi2_table[m];

    const bool accepted = (chi2 <= gate);

    // Residual log for offline diagnostics (residual_diagnostics.py). One row
    // per evaluation, rejected rows included: the regression against angular
    // velocity needs exactly the samples the gate throws away. Flushed per row
    // so a SIGINT at bag end does not truncate the file.
    if (_log.is_open()) {
        // res0/res1 stay the RAW residual: the offline whiteness and bias tests
        // are about the underlying physics, not about what the offset estimate
        // has already removed. What the filter actually used is logged
        // alongside, as res0_used and bias0.
        char buf[384];
        const int a0 = axes[0];
        const int a1 = (m > 1) ? axes[1] : -1;
        const double r1 = (m > 1) ? res_raw(1) : 0.0;
        snprintf(buf, sizeof(buf), "%.9f,%d,%.6f,%d,%.6f,%.4f,%.6f,%.6f,%.6f,%.4f,%d,%.4f,%.4f,%.6f,%.6f",
                 _state->_timestamp, a0, res_raw(0), a1, r1, v_OinO(0),
                 w_hat(0), w_hat(1), w_hat(2), chi2, (int)accepted,
                 _model->axis_state(a0).gain,
                 (m > 1) ? _model->axis_state(a1).gain : 1.0,
                 res(0), _model->bias(a0));
        _log << buf << std::endl;
    }

    // The offset estimate and the gait buffers track the motion, not the gate's
    // decisions, so every evaluated residual advances them — and they take the
    // RAW residual, since the offset is exactly what was subtracted out of the
    // other one.
    _model->push_residual(_state->_timestamp, res_raw);

    if (!accepted) {
        PRINT_DEBUG("[PLATFORM] update REJECTED (chi2 %.2f > %.2f) — possible slip/skid\n",
                    chi2, gate);
        return false;
    }

    StateHelper::EKFUpdate(_state, x_order, H, res, R);
    _last_update_time = _state->_timestamp;

    PRINT_DEBUG("[PLATFORM] update applied | res=[%.4f,%.4f] chi2=%.3f G=[%.3f,%.3f]\n",
                res(0), (m > 1) ? res(1) : 0.0, chi2,
                _model->axis_state(axes[0]).gain,
                (m > 1) ? _model->axis_state(axes[1]).gain : 1.0);
    return true;
}
