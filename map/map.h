#include "../mapPoint/mapPoint.h"
#include "../frame/frame.h"


class Map {
    public:
        typedef shared_ptr<Map> Ptr;
        list<MapPoint::Ptr> mapPoints;
        list<Frame::Ptr> frames;

        Map() {}
        void addFrame(Frame::Ptr frame);
        void addMapPoint(MapPoint::Ptr point);
};