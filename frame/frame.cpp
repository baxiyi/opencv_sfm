#include "frame.h"

bool Frame::isInFrame(const Vector3d &p_world) {
    Vector3d p_cam = camera->worldToCamera(p_world, Tcw);
    if (p_cam(2, 0) < 0)
        return false;
    Vector2d p_p = camera->worldToPixel(p_world, Tcw);
    return p_p(0, 0) > 0 && p_p(0, 0) < cols 
        && p_p(1, 0) > 0 && p_p(1, 0) < rows;
}