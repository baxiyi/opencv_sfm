#include "sfm.h"
using namespace cv;

void SFM::addImages(const vector<string> &imageDir, Camera::Ptr camera) {
    auto imageDirIter = imageDir.begin();
    Mat image1 = imread(*imageDirIter++);
    Mat image2 = imread(*imageDirIter++);
    init(image1, image2, camera);
    for (; imageDirIter != imageDir.end(); ++imageDirIter) {
        Mat image = imread(*imageDirIter);
        step(image, camera);
        BundleAdj ba;
        ba(map);
    }
}

void SFM::init(Mat &image1, Mat &image2, Camera::Ptr camera) {
    pushImage(image1, camera);
    saveFrame();
    detectFeatures();
    pushImage(image2, camera);
    detectFeatures();
    vector<DMatch> matches;
    matchFeatures(matches);

    vector<Point2f> matchPoints1, matchPoints2;
    for (auto match:matches) {
        matchPoints1.push_back(keyFrame1->keyPoints[match.queryIdx].pt);
        matchPoints2.push_back(keyFrame2->keyPoints[match.trainIdx].pt);
    }
    Mat essentialMatrix, inlierMask;
    double threshold = 1.0;
    essentialMatrix = findEssentialMat(matchPoints1, matchPoints2, 
        keyFrame2->frame->camera->getKMatxCV(), RANSAC, 0.999, threshold, inlierMask);
    
    Mat R, t, points4D;
    recoverPose(essentialMatrix, matchPoints1, matchPoints2, 
        keyFrame2->frame->camera->getKMatxCV(), R, t, threshold, inlierMask, points4D);
    Eigen::Matrix3d R2;
    cv2eigen(R, R2);
    keyFrame2->frame->Tcw = SE3(R2, 
        Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));
    saveMapPoints(inlierMask, points4D, matches);
    saveFrame();
}

void SFM::step(Mat &image, const Camera::Ptr &camera) {
    pushImage(image, camera);
    detectFeatures();
    Mat descriptorMap;
    vector<Point3f> points3D;
    for (MapPoint::Ptr &point:map->mapPoints) {
        if (keyFrame1->frame->isInFrame(point->pos)) {
            points3D.push_back(point->getPostPoint3_CV<float>());
            descriptorMap.push_back(point->descriptor);
        }
    }
    // todo

}

void SFM::pushImage(Mat &image, const Camera::Ptr &camera) {
    Frame::Ptr frame(new Frame(camera, image));
    keyFrame2 = KeyFrame::Ptr(new KeyFrame(frame, image));
}

void SFM::saveFrame() {
    map->addFrame(keyFrame2->frame);
    keyFrame1 = keyFrame2;
}

void SFM::detectFeatures() {
    feature2D->detect(keyFrame2->image, keyFrame2->keyPoints, noArray());
    feature2D->compute(keyFrame2->image, keyFrame2->keyPoints, keyFrame2->descriptors);
}

void SFM::matchFeatures(vector<DMatch> &matches) {
    matcher->match(keyFrame1->descriptors, keyFrame2->descriptors, matches, noArray());
    filtMatches(matches);
}

bool matchComp(DMatch &a, DMatch &b) {
    return a.distance < b.distance;
}
void SFM::filtMatches(vector<DMatch> &matches) {
    auto minMatch = min_element(matches.begin(), matches.end(), matchComp);
    auto minDis = minMatch->distance;
    vector<DMatch> goodMatches;
    for (auto match:matches) {
        if (match.distance <= 4 * minDis) {
            goodMatches.push_back(match);
        }
    }
    matches = goodMatches;
}

void SFM::saveMapPoints(const Mat& inlierMask, const Mat &points4D, const vector<DMatch> &matches) {
    Map::Ptr localMap(new Map);
    if (keyFrame1 && keyFrame2) {
        localMap->addFrame(keyFrame1->frame);
        localMap->addFrame(keyFrame2->frame);
    }
    for (int i = 0; i < points4D.cols; i++) {
        MapPoint::Ptr mapPoint;
        if (!inlierMask.empty() && !inlierMask.at<uint8_t>(i, 0))
            continue;
        Mat descriptor = keyFrame2->descriptors.row(matches[i].trainIdx);
        if (keyFrame1->inlinerPoints.find(matches[i].queryIdx) != keyFrame1->inlinerPoints.end()) {
            mapPoint = keyFrame1->inlinerPoints[matches[i].queryIdx];
            mapPoint->descriptor = descriptor;
            mapPoint->addObservedFrame(keyFrame2->frame, keyFrame2->keyPoints[matches[i].trainIdx].pt);
            keyFrame2->inlinerPoints[matches[i].trainIdx] = mapPoint;
        } else {
            Mat x = points4D.col(i);
            Vec3b rgb;
            if (keyFrame2->image.type() == CV_8UC1) {
                
            }
        }
    }
}



