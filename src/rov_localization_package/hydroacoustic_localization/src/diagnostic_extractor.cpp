#include <hydroacoustic_localization/diagnostic_extractor.h>

namespace rov_localization {

Metrics DiagnosticExtractor::get_metrics() const {
    Metrics metrics;
    metrics.fitness_score = get_fitness_score_();
    metrics.scan_mismatch_score = get_scan_mismatch_score_();
    metrics.transform_found = matcher_->is_transform_found();
    metrics.result_delta = get_result_delta_();

    return metrics;
}

double DiagnosticExtractor::get_fitness_score_() const {
    if (!matcher_->is_transform_found()) {
        return UNDEFINED;
    }

    return matcher_->getFitnessScore();
}

Transform DiagnosticExtractor::get_result_delta_() const {
    const auto& prior_transform = matcher_->get_prior_transform();
    const auto& current_transform = matcher_->get_current_transform();
    return prior_transform.inverse_transform() * current_transform;
}

double DiagnosticExtractor::get_scan_mismatch_score_() const {
    constexpr float THRESHOLD = 1.0f;
    constexpr int K_NUMBERS = 1;

    if (!matcher_->is_transform_found()) {
        return UNDEFINED;
    }

    const auto& aligned_scan = matcher_->get_aligned_scan();

    pcl::Indices k_indices(K_NUMBERS);
    std::vector<float> k_sqr_distances(K_NUMBERS);
    size_t counter = 0;
    for (const auto& point : aligned_scan) {
        search_tree_.nearestKSearch(point, 1, k_indices, k_sqr_distances);
        if (k_sqr_distances[0] > THRESHOLD) {
            ++counter;
        }
    }

    return static_cast<double>(counter) / (aligned_scan.size());
}

} // end namespace rov_localization
