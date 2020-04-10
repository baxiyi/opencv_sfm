#ifndef _MAPPOINT_H
#define _MAPPOINT_H

#include "../includes.h"
#include "../frame/frame.h"

class MapPoint {
    public:
        typedef shared_ptr<MapPoint> Ptr;
        Mat descriptor;
        list<pair<Frame::Ptr, cv::Point2d>> frames;
        Vector3d pos;
        Vec3b rgb;

        MapPoint();

        MapPoint(const Vector3d &pos, const Mat &descriptor, const Vec3b &rgb) :
            pos(pos), descriptor(descriptor), rgb(rgb) {}
        
        template<typename T>
        cv::Point3_<T> getPosPoint3_CV() const {
            return cv::Point3_<T>(pos(0, 0), pos(1, 0), pos(2, 0));
        }

        template<typename T>
        cv::Matx<T, 1, 3> getPosMatx13() const {
            return cv::Matx<T, 1, 3>(pos(0, 0), pos(1, 0), pos(2, 0));
        }

        template<typename T>
        void setPos(cv::Matx<T, 1, 3> posMatx13) {
            pos(0) = posMatx13(0);
            pos(1) = posMatx13(1);
            pos(2) = posMatx13(2);
        }

        void addObservedFrame(const Frame::Ptr &observedFrame, const cv::Point2d &p_p);
};

#endif