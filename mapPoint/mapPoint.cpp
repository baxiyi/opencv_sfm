#include "mapPoint.h"

void MapPoint::addObservedFrame(const Frame::Ptr &observedFrame, const cv::Point2d &p_p) {
    if (observedFrame) {
        frames.push_back(pair<Frame::Ptr, cv::Point2d>(observedFrame, p_p));
    }
}

