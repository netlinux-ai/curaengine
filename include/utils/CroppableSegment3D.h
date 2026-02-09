// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_CROPPABLESEGMENT3D_H
#define UTILS_CROPPABLESEGMENT3D_H

#include <optional>

#include "geometry/Point3LL.h"


namespace cura
{

/*!
 * The CroppableSegment is a helper to quickly calculate the intersections of a segment with the X and Y planes.
 */
class CroppableSegment3D
{
public:
    CroppableSegment3D(const Point3LL& start, const Point3LL& end);

    const Point3LL& start() const
    {
        return start_;
    }

    const Point3LL& end() const
    {
        return end_;
    }

    void setEnd(const Point3LL& end);

    std::optional<CroppableSegment3D> intersectionWithXLayer(const coord_t layer_start, const coord_t layer_end) const;

    std::optional<CroppableSegment3D> intersectionWithYLayer(const coord_t layer_start, const coord_t layer_end) const;

    std::optional<CroppableSegment3D> intersectionWithZLayer(const coord_t layer_start, const coord_t layer_end) const;

private:
    enum class LayerInsideness
    {
        Below,
        Inside,
        Above
    };

    Point3LL pointAtX(const coord_t x) const;

    Point3LL pointAtY(const coord_t y) const;

    Point3LL pointAtZ(const coord_t z) const;

    std::optional<CroppableSegment3D> intersectionWithLayer(
        const coord_t start_coordinate,
        const coord_t end_coordinate,
        const coord_t layer_start,
        const coord_t layer_end,
        const std::function<Point3LL(const Point3LL& point, const LayerInsideness insideness, const coord_t layer_start, const coord_t layer_end)>& function_crop_point) const;

    static Point3LL croppedPoint(
        const Point3LL& point,
        const LayerInsideness insideness,
        const coord_t layer_start,
        const coord_t layer_end,
        const std::function<Point3LL(const coord_t)>& function_point_at);

    Point3LL croppedPointX(const Point3LL& point, const LayerInsideness insideness, const coord_t layer_start, const coord_t layer_end) const;

    Point3LL croppedPointY(const Point3LL& point, const LayerInsideness insideness, const coord_t layer_start, const coord_t layer_end) const;

    Point3LL croppedPointZ(const Point3LL& point, const LayerInsideness insideness, const coord_t layer_start, const coord_t layer_end) const;

    static LayerInsideness pointIsInside(const coord_t point, const coord_t layer_start, const coord_t layer_end);

private:
    Point3LL direction_;
    Point3LL start_;
    Point3LL end_;
};

} // namespace cura

#endif