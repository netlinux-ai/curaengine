// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher

#ifndef UTILS_PLANARPOLYGON3LL_H
#define UTILS_PLANARPOLYGON3LL_H

#include <execution>

#include <boost/unordered/concurrent_flat_map.hpp>

#include "Coord_t.h"


namespace cura
{

class CroppableSegment3LL;

/*!
 * Represents a polygon that is contained in a plan, but has 3D coordinates. So the plan can be non-axis-aligned.
 * It provides method to be intersected with axis-aligned plans/slices.
 */
class PlanarPolygon3LL
{
public:
    explicit PlanarPolygon3LL(std::vector<CroppableSegment3LL>&& segments);

    explicit PlanarPolygon3LL(const std::initializer_list<CroppableSegment3LL>& segments);

    std::optional<PlanarPolygon3LL> cropToLayer(
        const coord_t layer_start,
        const coord_t layer_end,
        const std::function<std::optional<CroppableSegment3LL>(const CroppableSegment3LL&, const coord_t, const coord_t)>& function_intersect_with_layer) const;

    std::optional<PlanarPolygon3LL> cropToXLayer(const coord_t layer_start_x, const coord_t layer_end_x) const;

    std::optional<PlanarPolygon3LL> cropToYLayer(const coord_t layer_start_y, const coord_t layer_end_y) const;

    std::optional<PlanarPolygon3LL> cropToZLayer(const coord_t layer_start_z, const coord_t layer_end_z) const;

    std::tuple<coord_t, coord_t> minmaxX() const;

    std::tuple<coord_t, coord_t> minmaxY() const;

    std::tuple<coord_t, coord_t> minmaxZ() const;

private:
    std::tuple<coord_t, coord_t> minmax(const std::function<coord_t(const CroppableSegment3LL& segment)>& get_coordinate) const;

private:
    std::vector<CroppableSegment3LL> segments_;
};

} // namespace cura

#endif