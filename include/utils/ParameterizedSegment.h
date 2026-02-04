// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_PARAMETERIZEDSEGMENT_H
#define UTILS_PARAMETERIZEDSEGMENT_H

#include <optional>

#include "utils/Point3D.h"


namespace cura
{

/*!
 * The ParameterizedSegment is a helper to quickly calculate the intersections of a segment with the X and Y planes.
 */
class ParameterizedSegment
{
public:
    ParameterizedSegment(const Point3D& start, const Point3D& end);

    const Point3D& start() const
    {
        return start_;
    }

    const Point3D& end() const
    {
        return end_;
    }

    void setEnd(const Point3D& end);

    std::optional<ParameterizedSegment> intersectionWithXLayer(const double layer_start, const double layer_end) const;

    std::optional<ParameterizedSegment> intersectionWithYLayer(const double layer_start, const double layer_end) const;

    std::optional<ParameterizedSegment> intersectionWithZLayer(const double layer_start, const double layer_end) const;

private:
    enum class LayerInsideness
    {
        Below,
        Inside,
        Above
    };

    Point3D pointAtX(const double x) const;

    Point3D pointAtY(const double y) const;

    Point3D pointAtZ(const double y) const;

    std::optional<ParameterizedSegment> intersectionWithLayer(
        const double start_coordinate,
        const double end_coordinate,
        const double layer_start,
        const double layer_end,
        const std::function<Point3D(const Point3D& point, const LayerInsideness insideness, const double layer_start, const double layer_end)>& function_crop_point) const;

    static Point3D croppedPoint(
        const Point3D& point,
        const LayerInsideness insideness,
        const double layer_start,
        const double layer_end,
        const std::function<Point3D(const double)>& function_point_at);

    Point3D croppedPointX(const Point3D& point, const LayerInsideness insideness, const double layer_start, const double layer_end) const;

    Point3D croppedPointY(const Point3D& point, const LayerInsideness insideness, const double layer_start, const double layer_end) const;

    Point3D croppedPointZ(const Point3D& point, const LayerInsideness insideness, const double layer_start, const double layer_end) const;

    static LayerInsideness pointIsInside(const double point, const double layer_start, const double layer_end);

private:
    Point3D direction_;
    Point3D start_;
    Point3D end_;
};

} // namespace cura

#endif