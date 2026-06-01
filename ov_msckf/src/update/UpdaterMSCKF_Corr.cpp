
#include "UpdaterMSCKF_Corr.h"

#include "update/UpdaterHelper.h"
#include "update/UpdaterOptions.h"

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <algorithm>
#include <cmath>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

// =============================================================================
// Constructor
// =============================================================================

UpdaterMSCKF_Corr::UpdaterMSCKF_Corr(UpdaterOptions &options,
                                     ov_core::FeatureInitializerOptions &feat_init_options)
    : _options(options) {

    _options.sigma_pix_sq = std::pow(_options.sigma_pix, 2);

    initializer_feat = std::shared_ptr<ov_core::FeatureInitializer>(
        new ov_core::FeatureInitializer(feat_init_options));

    for (int i = 1; i < 500; i++) {
        boost::math::chi_squared chi_squared_dist(i);
        chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
    }

    _options.correntropy_window_size = std::max(1, _options.correntropy_window_size);
    _options.correntropy_recent_window = std::max(1, _options.correntropy_recent_window);
    _options.correntropy_sigma = std::max(1e-6, _options.correntropy_sigma);
    if (_options.correntropy_min_R_scale > _options.correntropy_max_R_scale) {
        std::swap(_options.correntropy_min_R_scale, _options.correntropy_max_R_scale);
    }

    PRINT_DEBUG("[CorrUpdater] initialized | enabled=%s  N=%d  W=%d  sigma=%.2f  adaptive_R=%s\n",
                _options.use_correntropy ? "true" : "false",
                _options.correntropy_window_size, _options.correntropy_recent_window,
                _options.correntropy_sigma,
                _options.correntropy_adaptive_R ? "true" : "false");
}

// =============================================================================
// correntropy_weight
// =============================================================================

double UpdaterMSCKF_Corr::correntropy_weight(double norm_innov,
                                             double sigma) const {
    return std::exp(-(norm_innov * norm_innov) / (2.0 * sigma * sigma));
}

// =============================================================================
// apply_correntropy_to_block
//
// H_f, H_x, and res_block are the raw outputs of get_feature_jacobian_full.
// Rows are laid out as [u1,v1, u2,v2, ...], so the count is always 2N.
// This function must be called BEFORE nullspace_project_inplace.
// =============================================================================

void UpdaterMSCKF_Corr::apply_correntropy_to_block(
        Eigen::MatrixXd &H_f,
        Eigen::MatrixXd &H_x,
        Eigen::VectorXd &res_block,
        const Eigen::MatrixXd &R_block,
        Eigen::VectorXd &innov_out) const {

    const int n = static_cast<int>(res_block.size());

    // Safety check — soft exit instead of assert. Rows are laid out as
    // [u1,v1,u2,v2,...] before null-space projection.
    if (n <= 0 || (n % 2) != 0 ||
        H_f.rows() != n || H_x.rows() != n ||
        R_block.rows() != n || R_block.cols() != n) {
        innov_out = Eigen::VectorXd();
        return;
    }

    const int num_feat = n / 2;
    innov_out.resize(num_feat);

    for (int i = 0; i < num_feat; i++) {
        const double u_err = res_block(2 * i);
        const double v_err = res_block(2 * i + 1);
        const double norm_pix = std::sqrt(u_err * u_err + v_err * v_err);

        const double r_u = std::sqrt(std::max(R_block(2 * i, 2 * i), 1e-9));
        const double r_v = std::sqrt(std::max(R_block(2 * i + 1, 2 * i + 1), 1e-9));
        const double r_avg = 0.5 * (r_u + r_v);

        innov_out(i) = norm_pix / r_avg;
    }

    if (!_options.use_correntropy) {
        return;
    }

    // Robust weighted least squares: apply sqrt(G) to both sides of the
    // measurement equation. Weight H_f as well, otherwise the null-space
    // projection is built from a different feature Jacobian than the weighted
    // state/residual rows.
    for (int i = 0; i < num_feat; i++) {
        const double G = correntropy_weight(innov_out(i), _options.correntropy_sigma);
        const double sqrt_G = std::sqrt(std::max(G, 1e-12));
        H_f.row(2 * i) *= sqrt_G;
        H_f.row(2 * i + 1) *= sqrt_G;
        H_x.row(2 * i) *= sqrt_G;
        H_x.row(2 * i + 1) *= sqrt_G;
        res_block(2 * i) *= sqrt_G;
        res_block(2 * i + 1) *= sqrt_G;
    }
}

// =============================================================================
// estimate_R
// =============================================================================

Eigen::MatrixXd UpdaterMSCKF_Corr::estimate_R(
        const Eigen::MatrixXd &R_meas_in) const {

    if (!_options.correntropy_adaptive_R ||
        static_cast<int>(past_innov.size()) < _options.correntropy_window_size) {
        return R_meas_in;
    }

    const int n_buf = static_cast<int>(past_innov.size());
    const int W = std::max(1, _options.correntropy_recent_window);
    const int start = std::max(0, n_buf - W);

    double sum_sq = 0.0;
    int count = 0;

    for (int i = start; i < n_buf; i++) {
        const Eigen::VectorXd &v = past_innov[i];
        for (int j = 0; j < v.size(); j++) {
            sum_sq += v(j) * v(j);
            count++;
        }
    }

    if (count == 0) return R_meas_in;

    const double mean_sq = sum_sq / static_cast<double>(count);
    const double scale = std::min(std::max(mean_sq, _options.correntropy_min_R_scale),
                                  _options.correntropy_max_R_scale);

    Eigen::MatrixXd R_new = R_meas_in;
    for (int i = 0; i < R_new.rows(); i++) {
        R_new(i, i) *= scale;
    }

    PRINT_DEBUG("[CorrUpdater] R scale=%.3f (mean_sq=%.3f)\n", scale, mean_sq);
    return R_new;
}

// =============================================================================
// push_innovation_history
// =============================================================================

void UpdaterMSCKF_Corr::push_innovation_history(
        const Eigen::VectorXd &innov_this_step) {
    past_innov.push_back(innov_this_step);
    const int window_size = std::max(1, _options.correntropy_window_size);
    while (static_cast<int>(past_innov.size()) > window_size) {
        past_innov.pop_front();
    }
}

// =============================================================================
// Main update function
// =============================================================================

void UpdaterMSCKF_Corr::update(std::shared_ptr<State> state,
                                std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec) {

    if (feature_vec.empty())
        return;

    boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5;
    rT0 = boost::posix_time::microsec_clock::local_time();

    // ------------------------------------------------------------------
    // 0) Clone timestamps
    // ------------------------------------------------------------------
    std::vector<double> clonetimes;
    for (const auto &clone_imu : state->_clones_IMU) {
        clonetimes.emplace_back(clone_imu.first);
    }

    // ------------------------------------------------------------------
    // 1) Clean feature measurements
    // ------------------------------------------------------------------
    auto it0 = feature_vec.begin();
    while (it0 != feature_vec.end()) {
        (*it0)->clean_old_measurements(clonetimes);

        int ct_meas = 0;
        for (const auto &pair : (*it0)->timestamps) {
            ct_meas += (*it0)->timestamps[pair.first].size();
        }

        if (ct_meas < 2) {
            (*it0)->to_delete = true;
            it0 = feature_vec.erase(it0);
        } else {
            it0++;
        }
    }
    rT1 = boost::posix_time::microsec_clock::local_time();

    // ------------------------------------------------------------------
    // 2) Clone pose vector
    // ------------------------------------------------------------------
    std::unordered_map<size_t,
        std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
    for (const auto &clone_calib : state->_calib_IMUtoCAM) {
        std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
        for (const auto &clone_imu : state->_clones_IMU) {
            Eigen::Matrix<double, 3, 3> R_GtoCi =
                clone_calib.second->Rot() * clone_imu.second->Rot();
            Eigen::Matrix<double, 3, 1> p_CioinG =
                clone_imu.second->pos() - R_GtoCi.transpose() * clone_calib.second->pos();
            clones_cami.insert({clone_imu.first,
                                FeatureInitializer::ClonePose(R_GtoCi, p_CioinG)});
        }
        clones_cam.insert({clone_calib.first, clones_cami});
    }

    // ------------------------------------------------------------------
    // 3) Triangulation + Gauss-Newton refinement
    // ------------------------------------------------------------------
    auto it1 = feature_vec.begin();
    while (it1 != feature_vec.end()) {
        bool success_tri = true;
        if (initializer_feat->config().triangulate_1d) {
            success_tri = initializer_feat->single_triangulation_1d(*it1, clones_cam);
        } else {
            success_tri = initializer_feat->single_triangulation(*it1, clones_cam);
        }

        bool success_refine = true;
        if (initializer_feat->config().refine_features) {
            success_refine = initializer_feat->single_gaussnewton(*it1, clones_cam);
        }

        if (!success_tri || !success_refine) {
            (*it1)->to_delete = true;
            it1 = feature_vec.erase(it1);
            continue;
        }
        it1++;
    }
    rT2 = boost::posix_time::microsec_clock::local_time();

    // ------------------------------------------------------------------
    // Maximum size computation
    // ------------------------------------------------------------------
    size_t max_meas_size = 0;
    for (size_t i = 0; i < feature_vec.size(); i++) {
        for (const auto &pair : feature_vec.at(i)->timestamps) {
            max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
        }
    }

    size_t max_hx_size = state->max_covariance_size();
    for (auto &landmark : state->_features_SLAM) {
        max_hx_size -= landmark.second->size();
    }

    Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
    Eigen::MatrixXd Hx_big  = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
    std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
    std::vector<std::shared_ptr<Type>> Hx_order_big;
    size_t ct_jacob = 0;
    size_t ct_meas  = 0;

    Eigen::VectorXd innov_acc;

    // ------------------------------------------------------------------
    // 4) Per-feature Jacobian + correntropy + null-space + chi2
    // ------------------------------------------------------------------
    auto it2 = feature_vec.begin();
    while (it2 != feature_vec.end()) {

        UpdaterHelper::UpdaterHelperFeature feat;
        feat.featid     = (*it2)->featid;
        feat.uvs        = (*it2)->uvs;
        feat.uvs_norm   = (*it2)->uvs_norm;
        feat.timestamps = (*it2)->timestamps;

        feat.feat_representation = state->_options.feat_rep_msckf;
        if (state->_options.feat_rep_msckf ==
            LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
            feat.feat_representation =
                LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
        }

        if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
            feat.anchor_cam_id          = (*it2)->anchor_cam_id;
            feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
            feat.p_FinA     = (*it2)->p_FinA;
            feat.p_FinA_fej = (*it2)->p_FinA;
        } else {
            feat.p_FinG     = (*it2)->p_FinG;
            feat.p_FinG_fej = (*it2)->p_FinG;
        }

        Eigen::MatrixXd H_f, H_x;
        Eigen::VectorXd res;
        std::vector<std::shared_ptr<Type>> Hx_order;

        // Raw Jacobian — at this point res is laid out as [u1,v1, u2,v2, ...]
        UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

        // ----- IMPORTANT: apply correntropy weight BEFORE null-space projection -----
        // nullspace_project_inplace reduces row count from 2N to 2N-3,
        // destroying the (u,v) pair structure needed by the correntropy step.
        Eigen::MatrixXd R_pre = _options.sigma_pix_sq *
            Eigen::MatrixXd::Identity(res.rows(), res.rows());

        Eigen::VectorXd block_innov;
        apply_correntropy_to_block(H_f, H_x, res, R_pre, block_innov);


        // ----- Null-space projection (original flow) -----
        UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

        // ----- Chi2 test -----
        Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
        Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
        S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
        double chi2 = res.dot(S.llt().solve(res));

        double chi2_check;
        if (res.rows() < 500) {
            chi2_check = chi_squared_table[res.rows()];
        } else {
            boost::math::chi_squared chi_squared_dist(res.rows());
            chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
            PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET,
                          (int)res.rows());
        }

        if (chi2 > _options.chi2_multipler * chi2_check) {
            (*it2)->to_delete = true;
            it2 = feature_vec.erase(it2);
            continue;
        }

        // Accumulate accepted-feature innovations for the sliding window.
        if (block_innov.size() > 0) {
            Eigen::VectorXd merged(innov_acc.size() + block_innov.size());
            if (innov_acc.size() > 0) merged.head(innov_acc.size()) = innov_acc;
            merged.tail(block_innov.size()) = block_innov;
            innov_acc = merged;
        }

        // ----- Stack into large H and res -----
        size_t ct_hx = 0;
        for (const auto &var : Hx_order) {
            if (Hx_mapping.find(var) == Hx_mapping.end()) {
                Hx_mapping.insert({var, ct_jacob});
                Hx_order_big.push_back(var);
                ct_jacob += var->size();
            }
            Hx_big.block(ct_meas, Hx_mapping[var], H_x.rows(), var->size()) =
                H_x.block(0, ct_hx, H_x.rows(), var->size());
            ct_hx += var->size();
        }

        res_big.block(ct_meas, 0, res.rows(), 1) = res;
        ct_meas += res.rows();
        it2++;
    }
    rT3 = boost::posix_time::microsec_clock::local_time();

    // Mark processed features for deletion
    for (size_t f = 0; f < feature_vec.size(); f++) {
        feature_vec[f]->to_delete = true;
    }

    if (ct_meas < 1) {
        return;
    }
    res_big.conservativeResize(ct_meas, 1);
    Hx_big.conservativeResize(ct_meas, ct_jacob);

    // Push innovations to window (single accumulation point)
    if (innov_acc.size() > 0) {
        push_innovation_history(innov_acc);
    }

    // ------------------------------------------------------------------
    // 5) QR measurement compression
    // ------------------------------------------------------------------
    UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
    if (Hx_big.rows() < 1) {
        return;
    }
    rT4 = boost::posix_time::microsec_clock::local_time();

    Eigen::MatrixXd R_big = _options.sigma_pix_sq *
        Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());

    // Scale R using the sliding-window estimate
    R_big = estimate_R(R_big);

    // ------------------------------------------------------------------
    // 6) EKF update
    // ------------------------------------------------------------------
    StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
    rT5 = boost::posix_time::microsec_clock::local_time();

    PRINT_ALL("[CORR-UP]: %.4f sec clean\n",      (rT1 - rT0).total_microseconds() * 1e-6);
    PRINT_ALL("[CORR-UP]: %.4f sec triangulate\n",(rT2 - rT1).total_microseconds() * 1e-6);
    PRINT_ALL("[CORR-UP]: %.4f sec system (%d feats)\n",
              (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
    PRINT_ALL("[CORR-UP]: %.4f sec compress\n",   (rT4 - rT3).total_microseconds() * 1e-6);
    PRINT_ALL("[CORR-UP]: %.4f sec update (%d size)\n",
              (rT5 - rT4).total_microseconds() * 1e-6, (int)res_big.rows());
    PRINT_ALL("[CORR-UP]: %.4f sec total | history=%zu\n",
              (rT5 - rT1).total_microseconds() * 1e-6, past_innov.size());
}
