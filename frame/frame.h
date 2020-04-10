#ifndef _FRAME_H
#define _FRAME_H

#include "../includes.h"
#include "../camera/camera.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

class Frame {
    public:
        typedef shared_ptr<Frame> Ptr;
        SE3 Tcw;
        Camera::Ptr camera;
        int cols, rows;

        Frame() : camera(nullptr) {}
        Frame(const Camera::Ptr &camera, const Mat &image):
            camera(camera), cols(image.cols), rows(image.rows) {}
        ~Frame() {}

        Vector3d getCameraCenterEigen() const {
            return Tcw.inverse().translation();
        };

        cv::Mat getTcwMatCV(int rtype) {
            cv::Mat TcwCV;
            cv::eigen2cv(Tcw.matrix(), TcwCV);
            TcwCV.convertTo(TcwCV, rtype);
            return TcwCV;
        }

        cv::Mat getTcw34MatCV(int rtype) {
            auto TcwCV = getTcwMatCV(rtype);
            Mat Tcw34;
            TcwCV(cv::Range(0, 3), cv::Range(0, 4)).convertTo(Tcw34, rtype);
            return Tcw34;
        }

        template<typename T>
        cv::Matx<T, 1, 3> getAngleAxisWcMatxCV() {
            Sophus::AngleAxisd angleAxis(Tcw.so3().matrix());
            auto axis = angleAxis.angle() * angleAxis.axis();
            cv::Matx<T, 1, 3> angleAxisCV(axis[0], axis[1], axis[2]);
            return angleAxisCV;
        }

        template<typename T>
        void setTcw(Matx<T, 2, 3> angleAxisAndTrans) {
            Tcw.so3() = SO3(angleAxisAndTrans(0, 0), angleAxisAndTrans(0, 1), angleAxisAndTrans(0, 2));
            Tcw.translation() = Vector3d(angleAxisAndTrans(1, 0), angleAxisAndTrans(1, 1), angleAxisAndTrans(1, 3));
        }

        bool isInFrame(const Vector3d &p_world);


};

#endif