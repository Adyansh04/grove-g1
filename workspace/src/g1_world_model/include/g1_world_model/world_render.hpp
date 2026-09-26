#ifndef G1_WORLD_MODEL__WORLD_RENDER_HPP_
#define G1_WORLD_MODEL__WORLD_RENDER_HPP_

/**
 * @file world_render.hpp
 * @brief A picture of the semantic map, saved beside it so it can be looked at without ROS.
 */

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "g1_world_model/grid.hpp"

namespace g1_world_model
{

/// One room as drawn: its label in the room image, where to write it, and what to write.
struct RenderRoom
{
    int         label = 0;
    std::string text;
    double      x = 0.0;  ///< Map frame, m.
    double      y = 0.0;
};

struct RenderObject
{
    Footprint   footprint;
    std::string text;
};

/// The colour of the room at @p index in a list of rooms, as 0..1 RGB: the RViz markers use it too.
[[nodiscard]] cv::Vec3f roomColour(std::size_t index);

/**
 * @brief Where to write each label: on its object if that is clear, else just beside it.
 *
 * In order, each label takes the first spot that overlaps nothing written so far, or the least
 * crowded one, so put big objects first to keep their middles.
 *
 * @param boxes Each label's object, pixels.
 * @param sizes Each label's text, pixels.
 * @param taken What is already written on, such as the room names.
 * @return Each label's top-left corner.
 */
[[nodiscard]] std::vector<cv::Point> placeLabels(
    const std::vector<cv::Rect>& boxes, const std::vector<cv::Size>& sizes,
    std::vector<cv::Rect> taken);

/**
 * @brief The map in grey, each room tinted its colour, object boxes and names.
 *
 * @param cells       CV_8U Cell values on @p geometry.
 * @param room_labels CV_32S room labels on @p geometry, 1-based in the order of @p rooms.
 * @param scale       Pixels per map cell.
 * @return A BGR image, map north up.
 */
[[nodiscard]] cv::Mat renderWorld(
    const cv::Mat& cells, const GridGeometry& geometry, const cv::Mat& room_labels,
    const std::vector<RenderRoom>& rooms, const std::vector<RenderObject>& objects, int scale = 4);

}  // namespace g1_world_model

#endif  // G1_WORLD_MODEL__WORLD_RENDER_HPP_
