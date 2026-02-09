// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#include "utils/PlanarPolygon3LL.h"

#include <range/v3/algorithm/max.hpp>
#include <range/v3/algorithm/min.hpp>
#include <range/v3/algorithm/minmax_element.hpp>
#include <range/v3/view/transform.hpp>

#include "utils/CroppableSegment3LL.h"


namespace cura
{

PlanarPolygon3LL::PlanarPolygon3LL(std::vector<CroppableSegment3LL>&& segments)
    : segments_(std::move(segments))
{
}

PlanarPolygon3LL::PlanarPolygon3LL(const std::initializer_list<CroppableSegment3LL>& segments)
    : segments_(segments)
{
}

std::optional<PlanarPolygon3LL> PlanarPolygon3LL::cropToLayer(
    const coord_t layer_start,
    const coord_t layer_end,
    const std::function<std::optional<CroppableSegment3LL>(const CroppableSegment3LL&, const coord_t, const coord_t)>& function_intersect_with_layer) const
{
    std::vector<CroppableSegment3LL> new_segments;

    const auto join_segments = [&new_segments](const CroppableSegment3LL& next_segment)
    {
        const coord_t join_distance = (new_segments.back().end() - next_segment.start()).vSize2();
        if (join_distance > EPSILON * EPSILON)
        {
            // Segments are not joined, add a transition segment
            new_segments.push_back(CroppableSegment3LL(new_segments.back().end(), next_segment.start()));
        }
        else
        {
            // Segments are almost joined, so slightly change the end of the previous segment to match
            new_segments.back().setEnd(next_segment.start());
        }
    };

    for (const CroppableSegment3LL& segment : segments_)
    {
        const std::optional<CroppableSegment3LL> cropped_segment = function_intersect_with_layer(segment, layer_start, layer_end);
        if (cropped_segment.has_value())
        {
            if (! new_segments.empty())
            {
                join_segments(*cropped_segment);
            }
            new_segments.push_back(*cropped_segment);
        }
    }

    if (new_segments.size() < 2)
    {
        return std::nullopt;
    }

    // Explicitly close the polygon if not closed yet
    join_segments(new_segments.front());

    return PlanarPolygon3LL(std::move(new_segments));
}

std::optional<PlanarPolygon3LL> PlanarPolygon3LL::cropToXLayer(const coord_t layer_start_x, const coord_t layer_end_x) const
{
    return cropToLayer(layer_start_x, layer_end_x, &CroppableSegment3LL::intersectionWithXLayer);
}

std::optional<PlanarPolygon3LL> PlanarPolygon3LL::cropToYLayer(const coord_t layer_start_y, const coord_t layer_end_y) const
{
    return cropToLayer(layer_start_y, layer_end_y, &CroppableSegment3LL::intersectionWithYLayer);
}

std::optional<PlanarPolygon3LL> PlanarPolygon3LL::cropToZLayer(const coord_t layer_start_z, const coord_t layer_end_z) const
{
    return cropToLayer(layer_start_z, layer_end_z, &CroppableSegment3LL::intersectionWithZLayer);
}

std::tuple<coord_t, coord_t> PlanarPolygon3LL::minmaxX() const
{
    return minmax(
        [](const CroppableSegment3LL& segment)
        {
            return segment.start().x_;
        });
}

std::tuple<coord_t, coord_t> PlanarPolygon3LL::minmaxY() const
{
    return minmax(
        [](const CroppableSegment3LL& segment)
        {
            return segment.start().y_;
        });
}

std::tuple<coord_t, coord_t> PlanarPolygon3LL::minmaxZ() const
{
    return minmax(
        [](const CroppableSegment3LL& segment)
        {
            return segment.start().z_;
        });
}

std::tuple<coord_t, coord_t> PlanarPolygon3LL::minmax(const std::function<coord_t(const CroppableSegment3LL& segment)>& get_coordinate) const
{
    const auto segments_coordinates = segments_ | ranges::views::transform(get_coordinate);
    auto result = ranges::minmax_element(segments_coordinates);
    return std::make_tuple(*result.min, *result.max);
}

} // namespace cura
