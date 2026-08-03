#include "UpdaterPlatform.h"

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

bool UpdaterPlatform::try_update(const Vector3d &w_m) {

    if (!_opts.do_constraint_update) return false;

    // Rate limit: the constraint is one standing fact, not a stream of
    // independent observations, so applying it every frame over-counts it.
    if (_opts.constraint_min_dt > 0.0) {
        const double t_now = _state->_timestamp;
        if (_last_update_time >= 0.0 && (t_now - _last_update_time) < _opts.constraint_min_dt)
            return false;
    }

    // Collect the constrained axes
    std::vector<int> axes;
    for (int i = 0; i < 3; i++) {
        if (_opts.constraint_mask(i) == 1) axes.push_back(i);
    }
    if (axes.empty()) return false;
    const int m = (int)axes.size();

    // ------------------------------------------------------------------
    // Predicted measurement
    //   v_OinO = R_ItoO * ( R_GtoI * v_IinG + skew(w_hat) * p_OinI )
    // ------------------------------------------------------------------
    std::shared_ptr<ov_type::IMU> imu = _state->_imu;

    Matrix3d R_GtoI = imu->Rot();
    Vector3d v_IinG = imu->vel();
    Vector3d b_g    = imu->bias_g();
    Vector3d w_hat  = w_m - b_g;

    Vector3d v_OinO = _R_ItoO * (R_GtoI * v_IinG + skew_x(w_hat) * _p_OinI);

    // ------------------------------------------------------------------
    // Residual: z_meas (= 0) minus prediction, on constrained axes only
    // ------------------------------------------------------------------
    VectorXd res(m);
    for (int k = 0; k < m; k++) res(k) = 0.0 - v_OinO(axes[k]);

    // ------------------------------------------------------------------
    // Jacobians w.r.t. [theta_GtoI (3), v_IinG (3), b_g (3)]
    //
    // OpenVINS left-quaternion error convention: R_GtoI = exp(-skew(dth)) Rhat
    //   d(R_GtoI * v)/d(dth) =  skew(Rhat * v)
    //   d(v_OinO)/d(v_IinG)  =  R_ItoO * R_GtoI
    //   d(skew(w_hat) p)/d(b_g) = d(-skew(p) w_hat)/d(b_g) = +skew(p)
    //   => d(v_OinO)/d(b_g)  =  R_ItoO * skew(p_OinI)
    //
    // H rows are minus the prediction Jacobian projected on the axes,
    // because res = 0 - h(x) and H = d(res)/dx... res = -h, dres/dx = -dh/dx.
    // OpenVINS EKFUpdate expects H = d(h)/dx with res = z - h, so we build
    // H = dh/dx directly and keep res = -h as computed above.
    // ------------------------------------------------------------------
    // Sign of the bias Jacobian:
    //   skew(w_hat) * p = -skew(p) * w_hat  and  w_hat = w_m - b_g
    //   => d(h)/d(b_g) = -skew(p_OinI) * (-I) = +skew(p_OinI)
    Matrix3d dh_dth = _R_ItoO * skew_x(R_GtoI * v_IinG);
    Matrix3d dh_dv  = _R_ItoO * R_GtoI;
    Matrix3d dh_dbg = _R_ItoO * skew_x(_p_OinI);

    // ------------------------------------------------------------------
    // Numerical Jacobian check. The perturbation goes through IMU::update(),
    // so this validates the analytic Jacobian against OpenVINS' own error
    // convention instead of against our reading of it (which would be
    // circular). IMU::update takes dx = [theta(3) p(3) v(3) bg(3) ba(3)].
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
    MatrixXd R = MatrixXd::Zero(m, m);
    for (int k = 0; k < m; k++) {
        int a = axes[k];
        H.block<1, 3>(k, 0) = dh_dth.row(a);
        H.block<1, 3>(k, 3) = dh_dv.row(a);
        H.block<1, 3>(k, 6) = dh_dbg.row(a);
        double s = std::max(_opts.constraint_sigma(a), 1e-4);
        R(k, k) = s * s;
    }

    // ------------------------------------------------------------------
    // Adaptive weighting, before the gate so a violation softens itself
    // rather than being thrown away wholesale. The constraint residual is
    // sharply peaked at zero and violated in bursts (turns, bumps, camber),
    // so constraint_sigma is set for the bulk and the correntropy weight
    // inflates R on the bursts. With no model attached this is a no-op and
    // the fixed sigma is used.
    // ------------------------------------------------------------------
    if (_model != nullptr) {
        _model->adapt_constraint_R(R, res, axes);
    }

    // ------------------------------------------------------------------
    // Chi2 gate: backstop for assumptions that are violated beyond what the
    // down-weighting can absorb (hard skid, sensor fault).
    // ------------------------------------------------------------------
    MatrixXd P_marg = StateHelper::get_marginal_covariance(_state, x_order);
    MatrixXd S = H * P_marg * H.transpose() + R;
    double chi2 = res.dot(S.llt().solve(res));
    double gate = (m == 1) ? _chi2_1dof : _chi2_2dof;

    const bool accepted = (chi2 <= gate);

    // Residual log for offline diagnostics (residual_diagnostics.py). One row
    // per evaluation, rejected rows included: the regression against angular
    // velocity needs exactly the samples the gate throws away. Flushed per row
    // so a SIGINT at bag end does not truncate the file.
    if (_log.is_open()) {
        char buf[256];
        const int a0 = axes[0];
        const int a1 = (m > 1) ? axes[1] : -1;
        const double r1 = (m > 1) ? res(1) : 0.0;
        snprintf(buf, sizeof(buf), "%.9f,%d,%.6f,%d,%.6f,%.4f,%.6f,%.6f,%.6f,%.4f,%d",
                 _state->_timestamp, a0, res(0), a1, r1, v_OinO(0),
                 w_hat(0), w_hat(1), w_hat(2), chi2, (int)accepted);
        _log << buf << std::endl;
    }

    if (!accepted) {
        PRINT_DEBUG("[PLATFORM] constraint update REJECTED (chi2 %.2f > %.2f) — possible slip/skid\n",
                    chi2, gate);
        return false;
    }

    StateHelper::EKFUpdate(_state, x_order, H, res, R);
    _last_update_time = _state->_timestamp;

    PRINT_DEBUG("[PLATFORM] nonholonomic update applied | res=[%.4f,%.4f] chi2=%.3f gain=[%.3f,%.3f]\n",
                res(0), (m > 1) ? res(1) : 0.0, chi2,
                (_model != nullptr) ? _model->constraint_gain(axes[0]) : 1.0,
                (_model != nullptr && m > 1) ? _model->constraint_gain(axes[1]) : 1.0);
    return true;
}
