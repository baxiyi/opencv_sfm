#ifndef _MAP_H
#define _MAP_H

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
};

#endif