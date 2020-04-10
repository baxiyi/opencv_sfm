#ifndef _MAP_H
#define _MAP_H
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "../mapPoint/mapPoint.h"
#include "../frame/frame.h"


class Map {
    public:
        typedef shared_ptr<Map> Ptr;
        list<MapPoint::Ptr> mapPoints;
        list<Frame::Ptr> frames; // 点被观测到的帧

        Map() {}
        void addFrame(Frame::Ptr frame);
        void addMapPoint(MapPoint::Ptr point);
        void visualCloudViewer();
};

#endif