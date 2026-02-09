// Copyright (c) 2025 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/CroppableSegment3LL.h"


namespace cura
{

CroppableSegment3LL::CroppableSegment3LL(const Point3LL& start, const Point3LL& end)
    : direction_(end - start)
    , start_(start)
    , end_(end)
{
}

Point3LL CroppableSegment3LL::pointAtX(const coord_t x) const
{
    const double factor = static_cast<double>(x - start_.x_) / (direction_.x_);
    return Point3LL(x, std::llrint(start_.y_ + factor * direction_.y_), std::llrint(start_.z_ + factor * direction_.z_));
}

Point3LL CroppableSegment3LL::pointAtY(const coord_t y) const
{
    const double factor = static_cast<double>(y - start_.y_) / (direction_.y_);
    return Point3LL(std::llrint(start_.x_ + factor * direction_.x_), y, std::llrint(start_.z_ + factor * direction_.z_));
}

Point3LL CroppableSegment3LL::pointAtZ(const coord_t z) const
{
    const double factor = static_cast<double>(z - start_.z_) / (direction_.z_);
    return Point3LL(std::llrint(start_.x_ + factor * direction_.x_), std::llrint(start_.y_ + factor * direction_.y_), z);
}

std::optional<CroppableSegment3LL> CroppableSegment3LL::intersectionWithLayer(
    const coord_t start_coordinate,
    const coord_t end_coordinate,
    const coord_t layer_start,
    const coord_t layer_end,
    const std::function<Point3LL(const Point3LL& point, const LayerInsideness insideness, const coord_t layer_start, const coord_t layer_end)>& function_crop_point) const
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

    const Point3LL new_segment_start = function_crop_point(start_, segment_start_inside, layer_start, layer_end);
    const Point3LL new_segment_end = function_crop_point(end_, segment_end_inside, layer_start, layer_end);

    if ((new_segment_end - new_segment_start).vSize2() < EPSILON * EPSILON)
    {
        return std::nullopt;
    }

    return CroppableSegment3LL(new_segment_start, new_segment_end);
}

CroppableSegment3LL::LayerInsideness CroppableSegment3LL::pointIsInside(const coord_t point, const coord_t layer_start, const coord_t layer_end)
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

Point3LL CroppableSegment3LL::croppedPoint(
    const Point3LL& point,
    const LayerInsideness insideness,
    const coord_t layer_start,
    const coord_t layer_end,
    const std::function<Point3LL(const coord_t)>& function_point_at)
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

    return Point3LL();
}

Point3LL CroppableSegment3LL::croppedPointX(const Point3LL& point, const LayerInsideness insideness, const coord_t layer_start, const coord_t layer_end) const
{
    return croppedPoint(point, insideness, layer_start, layer_end, std::bind(&CroppableSegment3LL::pointAtX, this, std::placeholders::_1));
}

Point3LL CroppableSegment3LL::croppedPointY(const Point3LL& point, const LayerInsideness insideness, const coord_t layer_start, const coord_t layer_end) const
{
    return croppedPoint(point, insideness, layer_start, layer_end, std::bind(&CroppableSegment3LL::pointAtY, this, std::placeholders::_1));
}

Point3LL CroppableSegment3LL::croppedPointZ(const Point3LL& point, const LayerInsideness insideness, const coord_t layer_start, const coord_t layer_end) const
{
    return croppedPoint(point, insideness, layer_start, layer_end, std::bind(&CroppableSegment3LL::pointAtZ, this, std::placeholders::_1));
}

void CroppableSegment3LL::setEnd(const Point3LL& end)
{
    end_ = end;
    direction_ = end_ - start_;
}

std::optional<CroppableSegment3LL> CroppableSegment3LL::intersectionWithXLayer(const coord_t layer_start, const coord_t layer_end) const
{
    return intersectionWithLayer(
        start_.x_,
        end_.x_,
        layer_start,
        layer_end,
        std::bind(&CroppableSegment3LL::croppedPointX, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

std::optional<CroppableSegment3LL> CroppableSegment3LL::intersectionWithYLayer(const coord_t layer_start, const coord_t layer_end) const
{
    return intersectionWithLayer(
        start_.y_,
        end_.y_,
        layer_start,
        layer_end,
        std::bind(&CroppableSegment3LL::croppedPointY, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

std::optional<CroppableSegment3LL> CroppableSegment3LL::intersectionWithZLayer(const coord_t layer_start, const coord_t layer_end) const
{
    return intersectionWithLayer(
        start_.z_,
        end_.z_,
        layer_start,
        layer_end,
        std::bind(&CroppableSegment3LL::croppedPointZ, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

} // namespace cura