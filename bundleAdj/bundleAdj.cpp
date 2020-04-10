#include "bundleAdj.h"

void BundleAdj::loadMap() {
    for(auto &point:map->mapPoints) {
        mapPointsPos[point] = point->getPosMatx13<double>();
    }
    for(auto &frame:map->frames) {
        auto angleAxis = frame->getAngleAxisWcMatxCV<double>();
        auto t = frame->Tcw.translation();
        frameExtrinsics[frame] = Matx23d(angleAxis(0), angleAxis(1), angleAxis(2), t[0], t[1], t[2]);
        if (cameraIntrinsics.find(frame->camera) == cameraIntrinsics.end()) {
            cameraIntrinsics[frame->camera] = Matx14d(frame->camera->fx, 
                frame->camera->fy, frame->camera->cx, frame->camera->cy);
        }
    }
}

void BundleAdj::bundleAdjustment() {
    ceres::Problem problem;
    for (auto &frameExtrinsic:frameExtrinsics) {
        problem.AddParameterBlock(frameExtrinsic.second.val, 6);
    }
    problem.SetParameterBlockConstant(frameExtrinsics[map->frames.front()].val);
    for (auto &cameraIntrinsic:cameraIntrinsics) {
        problem.AddParameterBlock(cameraIntrinsic.second.val, 4);
    }
    ceres::LossFunction *lossFunction = new ceres::HuberLoss(4);
    // cout<<mapPointsPos.size()<<endl;
    for (auto &mapPointPos:mapPointsPos) {
        for (auto &frame:mapPointPos.first->frames) {
            ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<ReprojectCost, 2, 4, 6, 3>(
                new ReprojectCost(frame.second)
            );
            Camera::Ptr camera;
            problem.AddResidualBlock(
                costFunction,
                lossFunction,
                cameraIntrinsics[frame.first->camera].val,
                frameExtrinsics[frame.first].val,
                mapPointPos.second.val
            );
        }    
    }
    ceres::Solver::Summary summary;
    ceres::Solve(ceres_config, &problem, &summary);
    if (!summary.IsSolutionUsable()) {
        std::cout << "Bundle Adjustment failed." << std::endl;
    } else {
        // Display statistics about the minimization
        std::cout << "Bundle Adjustment statistics (approximated RMSE):\n"
                    << "    #views: " << frameExtrinsics.size() << "\n"
                    << "    #residuals: " << summary.num_residuals << "\n"
                    << "    Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
                    << "    Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
                    << "    Time (s): " << summary.total_time_in_seconds << "\n";
    }
}

void BundleAdj::writeMap() {
    for (auto &mapPointPos:mapPointsPos) {
        mapPointPos.first->setPos(mapPointPos.second);
    }

    for (auto &frameExtrinsic:frameExtrinsics) {
        frameExtrinsic.first->setTcw(frameExtrinsic.second);
    }

    for (auto &cameraIntrinsic:cameraIntrinsics) {
        cameraIntrinsic.first->setIntrinsic(cameraIntrinsic.second);
    }
}

void BundleAdj::clear() {
    map = nullptr;
    cameraIntrinsics.clear();
    frameExtrinsics.clear();
    mapPointsPos.clear();
}

BundleAdj::BundleAdj() {
    ceres_config.minimizer_progress_to_stdout = false;
    ceres_config.logging_type = ceres::SILENT;
        ceres_config.num_threads = 1;
        ceres_config.preconditioner_type = ceres::JACOBI;
        ceres_config.linear_solver_type = ceres::SPARSE_SCHUR;
        ceres_config.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
}

void BundleAdj::operator()(Map::Ptr &map) {
    this->map = map;
    loadMap();
    bundleAdjustment();
    writeMap();
    clear();
}