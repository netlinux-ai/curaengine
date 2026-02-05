// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/CroppableSegment3D.h"


namespace cura
{

CroppableSegment3D::CroppableSegment3D(const Point3D& start, const Point3D& end)
    : direction_(end - start)
    , start_(start)
    , end_(end)
{
}

Point3D CroppableSegment3D::pointAtX(const double x) const
{
    const double factor = (x - start_.x_) / (direction_.x_);
    return Point3D(x, start_.y_ + factor * direction_.y_, start_.z_ + factor * direction_.z_);
}

Point3D CroppableSegment3D::pointAtY(const double y) const
{
    const double factor = (y - start_.y_) / (direction_.y_);
    return Point3D(start_.x_ + factor * direction_.x_, y, start_.z_ + factor * direction_.z_);
}

Point3D CroppableSegment3D::pointAtZ(const double z) const
{
    const double factor = (z - start_.z_) / (direction_.z_);
    return Point3D(start_.x_ + factor * direction_.x_, start_.y_ + factor * direction_.y_, z);
}

std::optional<CroppableSegment3D> CroppableSegment3D::intersectionWithLayer(
    const double start_coordinate,
    const double end_coordinate,
    const double layer_start,
    const double layer_end,
    const std::function<Point3D(const Point3D& point, const LayerInsideness insideness, const double layer_start, const double layer_end)>& function_crop_point) const
{
    const LayerInsideness segment_start_inside = pointIsInside(start_coordinate, layer_start, layer_end);
    const LayerInsideness segment_end_inside = pointIsInside(end_coordinate, layer_start, layer_end);

    if (segment_end_inside == segment_start_inside)
    {
        if (segment_start_inside == LayerInsideness::Inside)
        {
            // Segment is fully inside layer, take it as is
            return *this;
        }

        // Otherwise segment is fully outside, so intersection is empty
        return std::nullopt;
    }

    const Point3D new_segment_start = function_crop_point(start_, segment_start_inside, layer_start, layer_end);
    const Point3D new_segment_end = function_crop_point(end_, segment_end_inside, layer_start, layer_end);

    if ((new_segment_end - new_segment_start).vSize2() < EPSILON * EPSILON)
    {
        return std::nullopt;
    }

    return CroppableSegment3D(new_segment_start, new_segment_end);
}

CroppableSegment3D::LayerInsideness CroppableSegment3D::pointIsInside(const double point, const double layer_start, const double layer_end)
{
    if (point < layer_start)
    {
        return LayerInsideness::Below;
    }

    if (point > layer_end)
    {
        return LayerInsideness::Above;
    }

    return LayerInsideness::Inside;
}

Point3D CroppableSegment3D::croppedPoint(
    const Point3D& point,
    const LayerInsideness insideness,
    const double layer_start,
    const double layer_end,
    const std::function<Point3D(const double)>& function_point_at)
{
    switch (insideness)
    {
    case LayerInsideness::Inside:
        return point;
    case LayerInsideness::Below:
        return function_point_at(layer_start);
    case LayerInsideness::Above:
        return function_point_at(layer_end);
    }

    return Point3D();
}

Point3D CroppableSegment3D::croppedPointX(const Point3D& point, const LayerInsideness insideness, const double layer_start, const double layer_end) const
{
    return croppedPoint(point, insideness, layer_start, layer_end, std::bind(&CroppableSegment3D::pointAtX, this, std::placeholders::_1));
}

Point3D CroppableSegment3D::croppedPointY(const Point3D& point, const LayerInsideness insideness, const double layer_start, const double layer_end) const
{
    return croppedPoint(point, insideness, layer_start, layer_end, std::bind(&CroppableSegment3D::pointAtY, this, std::placeholders::_1));
}

Point3D CroppableSegment3D::croppedPointZ(const Point3D& point, const LayerInsideness insideness, const double layer_start, const double layer_end) const
{
    return croppedPoint(point, insideness, layer_start, layer_end, std::bind(&CroppableSegment3D::pointAtZ, this, std::placeholders::_1));
}

void CroppableSegment3D::setEnd(const Point3D& end)
{
    end_ = end;
    direction_ = end_ - start_;
}

std::optional<CroppableSegment3D> CroppableSegment3D::intersectionWithXLayer(const double layer_start, const double layer_end) const
{
    return intersectionWithLayer(
        start_.x_,
        end_.x_,
        layer_start,
        layer_end,
        std::bind(&CroppableSegment3D::croppedPointX, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

std::optional<CroppableSegment3D> CroppableSegment3D::intersectionWithYLayer(const double layer_start, const double layer_end) const
{
    return intersectionWithLayer(
        start_.y_,
        end_.y_,
        layer_start,
        layer_end,
        std::bind(&CroppableSegment3D::croppedPointY, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

std::optional<CroppableSegment3D> CroppableSegment3D::intersectionWithZLayer(const double layer_start, const double layer_end) const
{
    return intersectionWithLayer(
        start_.z_,
        end_.z_,
        layer_start,
        layer_end,
        std::bind(&CroppableSegment3D::croppedPointZ, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

} // namespace cura