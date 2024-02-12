#pragma once

#include <memory>

#include "types.h"
#include "mather_shell.h"

namespace rov_localization {

class DiagnosticExtractor {
public:
    DiagnosticExtractor() = default;
    ~DiagnosticExtractor() = default;
    DiagnosticExtractor(const DiagnosticExtractor&) = delete;
    DiagnosticExtractor& operator=(const DiagnosticExtractor&) = delete;
    DiagnosticExtractor(DiagnosticExtractor&&) = delete;
    DiagnosticExtractor& operator=(DiagnosticExtractor&&) = delete;

    explicit DiagnosticExtractor(std::shared_ptr<MatherShell> matcher): matcher_(std::move(matcher)),
                                                                               search_tree_(false) {}

    inline void set_map(std::shared_ptr<PointCloud> point_cloud)
    {
        search_tree_.setInputCloud(std::move(point_cloud));
    }

    [[nodiscard]] Metrics get_metrics() const;

private:
    std::shared_ptr<MatherShell> matcher_;
    SearchTree search_tree_;

    [[nodiscard]] double get_fitness_score_() const;
    [[nodiscard]] double get_scan_mismatch_score_() const;
    [[nodiscard]] Transform get_result_delta_() const;
};
} // end namespace rov_localization
