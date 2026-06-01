/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_MSCKF_UPDATER_OPTIONS_H
#define OV_MSCKF_UPDATER_OPTIONS_H

#include "utils/print.h"

namespace ov_msckf {

/**
 * @brief Struct which stores general updater options
 */
struct UpdaterOptions {

  /// What chi-squared multipler we should apply
  double chi2_multipler = 5;

  /// Noise sigma for our raw pixel measurements
  double sigma_pix = 1;

  /// Covariance for our raw pixel measurements
  double sigma_pix_sq = 1;

  /// Enable correntropy reweighting for MSCKF feature updates
  bool use_correntropy = false;

  /// Normalized innovation sigma used by the correntropy kernel
  double correntropy_sigma = 5.0;

  /// Number of update innovations kept for optional adaptive-R estimation
  int correntropy_window_size = 75;

  /// Number of most-recent innovation batches used by adaptive-R estimation
  int correntropy_recent_window = 5;

  /// Enable innovation-based measurement covariance scaling
  bool correntropy_adaptive_R = false;

  /// Minimum adaptive-R diagonal scale
  double correntropy_min_R_scale = 0.5;

  /// Maximum adaptive-R diagonal scale
  double correntropy_max_R_scale = 5.0;

  /// Nice print function of what parameters we have loaded
  void print() {
    PRINT_DEBUG("    - chi2_multipler: %.1f\n", chi2_multipler);
    PRINT_DEBUG("    - sigma_pix: %.2f\n", sigma_pix);
    PRINT_DEBUG("    - use_correntropy: %s\n", use_correntropy ? "true" : "false");
    PRINT_DEBUG("    - correntropy_sigma: %.2f\n", correntropy_sigma);
    PRINT_DEBUG("    - correntropy_window_size: %d\n", correntropy_window_size);
    PRINT_DEBUG("    - correntropy_recent_window: %d\n", correntropy_recent_window);
    PRINT_DEBUG("    - correntropy_adaptive_R: %s\n", correntropy_adaptive_R ? "true" : "false");
    PRINT_DEBUG("    - correntropy_R_scale: [%.2f, %.2f]\n", correntropy_min_R_scale, correntropy_max_R_scale);
  }
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_OPTIONS_H