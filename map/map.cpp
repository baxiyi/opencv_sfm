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