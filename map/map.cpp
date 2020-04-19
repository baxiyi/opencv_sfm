#include "map.h"

void Map::addFrame(Frame::Ptr frame) {
    if (frame) {
        frames.push_back(frame);
    }
}

void Map::addMapPoint(MapPoint::Ptr point) {
    if (point) {
        mapPoints.push_back(point);
    }
}

void Map::visualCloudViewer() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto point:mapPoints) {
        pcl::PointXYZRGB pointXYZ(point->rgb[0], point->rgb[1], point->rgb[2]);
        pointXYZ.x = point->pos(0);
        pointXYZ.y = point->pos(1);
        pointXYZ.z = point->pos(2);
        cloud->push_back(pointXYZ);
    }

    pcl::visualization::PCLVisualizer viewer("Viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud(cloud, "Triangulated Point Cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Triangulated Point Cloud");
    // viewer.addCoordinateSystem(1.0);

    // int indexFrame = 0;
    // for (auto &frame:frames) {
    //     Eigen::Matrix4f camPose;
    //     auto T_c_w = frame->Tcw.inverse().matrix();
    //     for (int i = 0; i < camPose.rows(); i++) {
    //         for (int j = 0; j < camPose.cols(); j++) {
    //             camPose(i, j) = T_c_w(i, j);
    //         }
    //     }
    //     viewer.addCoordinateSystem(1.0, Eigen::Affine3f(camPose), "cam" + to_string(indexFrame++));
    // }
    // viewer.initCameraParameters();
    while(!viewer.wasStopped()) {
        viewer.spin();
    }
}