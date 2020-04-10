#ifndef _SFM_H
#define _SFM_H

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

    public:
        SFM(const cv::Ptr<cv::Feature2D> &feature2D, const cv::Ptr<DescriptorMatcher> &matcher) :
            feature2D(feature2D), matcher(matcher), map(new Map) {};
        
        void addImages(const vector<string> &imageDir, Camera::Ptr camera);
        void init(Mat &image1, Mat &image2, Camera::Ptr camera);
        void step(Mat &image, const Camera::Ptr &camera);
        // 添加新帧
        void pushImage(Mat &image, const Camera::Ptr &camera);
        // 保存新帧
        void saveFrame();
        // 检测特征点
        void detectFeatures();
        // 匹配筛选特征点
        void matchFeatures(vector<DMatch> &matches);
        // 转化为齐次坐标，存入map
        void saveMapPoints(const Mat& inlierMask, const Mat &points4D, const vector<DMatch> &matches);
        // 筛选匹配点
        void filtMatches(vector<DMatch> &matches);


};

#endif
