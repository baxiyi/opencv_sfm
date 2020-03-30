
#include "camera.h"

Vector3d Camera::worldToCamera(const Vector3d &p_w, const SE3 &T_c_w) {
    return T_c_w * p_w;
}

Vector3d Camera::cameraToWorld(const Vector3d &p_c, const SE3 &T_c_w) {
    return T_c_w.inverse() * p_c;
}

Vector2d Camera::cameraToPixel(const Vector3d &p_c) {
    return Vector2d(
        fx * p_c(0, 0) / p_c(2, 0) + cx,
        fy * p_c(1, 0) / p_c(2, 0) + cy
    );
}

Vector3d Camera::pixelToCamera(const Vector2d &p_p, double depth) {
    return Vector3d(
        (p_p(0, 0) - cx) * depth / fx,
        (p_p(1, 0) - cy) * depth / fy,
        depth
    );
}

Vector2d Camera::worldToPixel(const Vector3d &p_w, const SE3 &T_c_w) {
    return cameraToPixel(worldToCamera(p_w, T_c_w));
}

Vector3d Camera::pixelToWorld(const Vector2d &p_p, const SE3& T_c_w, double depth) {
    return cameraToWorld(pixelToCamera(p_p, depth), T_c_w);
}

Point2f Camera::pixelToNormal(const Point2d &p) const {
    return Point2f(
        (p.x - cx) / fx,
        (p.y - cy) / fy
    );
}