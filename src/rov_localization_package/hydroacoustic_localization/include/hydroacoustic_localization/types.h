#pragma once

#include <pcl/registration/ndt.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include "transform.h"

namespace rov_localization {

constexpr double UNDEFINED = -1.0;

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using VoxelGrid = pcl::VoxelGrid<Point>;
using NDTSolver = pcl::NormalDistributionsTransform<Point, Point>;
using SearchTree = pcl::search::KdTree<pcl::PointXYZ>;

struct Metrics {
    double fitness_score = UNDEFINED;
    double scan_mismatch_score = UNDEFINED;
    bool transform_found = false;
    Transform result_delta;
};

struct MatherParams {
    float optimization_step = 0.1f;
    float grid_cell_size = 1.0f;
    float optimization_treshold = 0.1f;
    uint optimization_iterations = 20u;
};

struct ManagerParams {
    float filter_cell_size = 1.0f;
    bool filter_enabled = false;
    MatherParams mather_params;
};
} // end namespace rov_localization
