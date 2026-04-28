// =============================================================================
// DOSYA: UpdaterMSCKF_Corr.h   <<< BU DOSYA HEADER'DIR — .h UZANTISIYLA KAYDET
// =============================================================================
#ifndef OV_MSCKF_UPDATER_MSCKF_CORR_H
#define OV_MSCKF_UPDATER_MSCKF_CORR_H

#include <deque>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

// İleri bildirimler
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
 * @brief Correntropy tabanlı MSCKF güncelleyici (OpenVINS).
 *
 * Yapı olarak orijinal UpdaterMSCKF ile birebir aynıdır
 * (cleaning + triangulation + gauss-newton + chi2 + QR + EKFUpdate).
 *
 * EK olarak iki mekanizma:
 *  1) Correntropy ağırlığı (Co): chi2'yi geçen feature bloklarına
 *     normalize inovasyon büyüklüklerine göre G ağırlığı uygulanır.
 *     Co ağırlığı H ve res'in içine sol-çarpılarak gömülür; QR ve
 *     EKFUpdate orijinal akışla aynı çalışır.
 *  2) Kayan pencere R kestirimi: son N_window adımdaki normalize
 *     inovasyon istatistiklerinden bir ölçek faktörü kestirilir
 *     ve sıkıştırılmış R bu faktörle güncellenir.
 *
 *  - Pencere dolmadan önce Co = I → standart UpdaterMSCKF davranışı.
 */
class UpdaterMSCKF_Corr {

public:
    /// Constructor — orijinal UpdaterMSCKF ile aynı imza
    UpdaterMSCKF_Corr(UpdaterOptions &options,
                      ov_core::FeatureInitializerOptions &feat_init_options);

    /// Ana güncelleme fonksiyonu
    void update(std::shared_ptr<State> state,
                std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

    // Correntropy parametre setterları
    void set_window_size(int N)   { N_window  = N; }
    void set_recent_window(int W) { W_recent  = W; }
    void set_sigma_cam(double s)  { sigma_cam = s; }

private:
    UpdaterOptions &_options;

    /// Feature triangulation/refinement helper (orijinal UpdaterMSCKF gibi)
    std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

    /// Chi2 95% güven aralığı tablosu
    std::map<int, double> chi_squared_table;

    // ---------- Correntropy parametreleri ----------
    int    N_window  = 75;
    int    W_recent  = 5;
    double sigma_cam = 5.0;

    /// Kayan pencere — feature başına normalize inovasyon büyüklükleri
    std::deque<Eigen::VectorXd> past_innov;

    // ---------- Yardımcılar (tanımları .cpp'de) ----------
    double correntropy_weight(double norm_innov, double sigma) const;

    /// H ve res'i feature başına Co=diag(G) ile sol-çarpar.
    /// Pencere dolmadıysa hiçbir şey yapmaz (Co = I).
    /// Pencereye eklenmek üzere normalize inovasyonları innov_out'a yazar.
    void apply_correntropy_to_block(Eigen::MatrixXd &H_block,
                                    Eigen::VectorXd &res_block,
                                    const Eigen::MatrixXd &R_block,
                                    Eigen::VectorXd &innov_out) const;

    /// Kayan pencereden bir ölçek faktörü kestirir, R'yi günceller.
    Eigen::MatrixXd estimate_R(const Eigen::MatrixXd &R_meas_in) const;

    /// Yeni inovasyon vektörünü pencereye ekler, fazlasını atar.
    void push_innovation_history(const Eigen::VectorXd &innov_this_step);
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MSCKF_CORR_H