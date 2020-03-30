#include "opencv2/opencv.hpp"
#include "../includes.h"

using namespace cv;

class Camera {
    public:
        typedef shared_ptr<Camera> Ptr;
        float fx, fy, cx, cy;
        Camera(float fx, float fy, float cx, float cy):
            fx(fx), fy(fy), cx(cx), cy(cy) {}
        template<typename T>
        void setIntrinsic(Matx<T, 1, 4> intrinsic) {
            fx = intrinsic(0);
            fy = intrinsic(1);
            cx = intrinsic(2);
            cy = intrinsic(3);
        }

        Vector3d worldToCamera(const Vector3d &p_w, const SE3 &T_c_w);
        Vector3d cameraToWorld(const Vector3d &p_c, const SE3 &T_c_w);
        Vector2d cameraToPixel(const Vector3d &p_c);
        Vector3d pixelToCamera(const Vector2d  &p_p, double depth = 1);
        Vector3d pixelToWorld(const Vector2d &p_p, const SE3 &T_c_w, double depth  = 1);
        Vector2d worldToPixel(const Vector3d &p_w, const SE3 &T_c_w);
        Point2f pixelToNormal(const Point2d &p) const;

        float getFocalLength() {
            return (fx + fy) / 2;
        }

        cv::Point2d getPrincipalPoint() {
            return cv::Point2d(cx, cy);
        }

        cv::Matx<float, 3, 3> getKMatxCV() {
            return cv::Matx<float, 3, 3>(fx, 0, cx, 0, fy, cy, 0, 0, 1);
        }

        cv::Mat getKMatCV() {
            return (Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0,  0, 1);
        }
 };