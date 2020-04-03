#include "includes.h"
#include "map/map.h"
#include "frame/frame.h"
#include "opencv2/opencv.hpp"
#include "bundleAdj/bundleAdj.h"
using namespace cv;

class SFM {
    Map::Ptr map;
    cv::Ptr<cv::Feature2D> feature2D;
    cv::Ptr<DescriptorMatcher> matcher;
    struct KeyFrame {
        public:
            typedef shared_ptr<KeyFrame> Ptr;
            Frame::Ptr frame;
            Mat image;
            vector<cv::KeyPoint> keyPoints;
            Mat descriptors;
            unordered_map<int, MapPoint::Ptr> inlinerPoints;

            KeyFrame(const Frame::Ptr &frame, const Mat &image) : frame(frame), image(image) {}
    };

    KeyFrame::Ptr keyFrame1, keyFrame2;

}
