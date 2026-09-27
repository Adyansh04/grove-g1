/**
 * @file world_render.cpp
 * @brief The saved picture of the semantic map.
 */

#include "canopy/world_render.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <string>

namespace canopy
{

cv::Vec3f roomColour(std::size_t index)
{
    // Well apart in hue and all readable under white text: Tableau's ten.
    static constexpr std::array<std::array<float, 3>, 10> kPalette{ {
        { 0.12F, 0.47F, 0.71F },
        { 1.00F, 0.50F, 0.05F },
        { 0.17F, 0.63F, 0.17F },
        { 0.84F, 0.15F, 0.16F },
        { 0.58F, 0.40F, 0.74F },
        { 0.55F, 0.34F, 0.29F },
        { 0.89F, 0.47F, 0.76F },
        { 0.50F, 0.50F, 0.50F },
        { 0.74F, 0.74F, 0.13F },
        { 0.09F, 0.75F, 0.81F },
    } };
    const auto& colour = kPalette[index % kPalette.size()];
    return { colour[0], colour[1], colour[2] };
}

std::vector<cv::Point> placeLabels(
    const std::vector<cv::Rect>& boxes, const std::vector<cv::Size>& sizes,
    std::vector<cv::Rect> taken)
{
    constexpr int          kGap = 3;
    std::vector<cv::Point> placed;
    placed.reserve(boxes.size());
    for (std::size_t i = 0; i < boxes.size(); ++i)
    {
        const cv::Rect& box  = boxes[i];
        const cv::Size& size = sizes[i];
        const int       cx   = box.x + (box.width / 2);
        const int       cy   = box.y + (box.height / 2);
        // On it, then above, below, right and left of it.
        const std::array candidates{
            cv::Point(cx - (size.width / 2), cy - (size.height / 2)),
            cv::Point(cx - (size.width / 2), box.y - size.height - kGap),
            cv::Point(cx - (size.width / 2), box.y + box.height + kGap),
            cv::Point(box.x + box.width + kGap, cy - (size.height / 2)),
            cv::Point(box.x - size.width - kGap, cy - (size.height / 2)),
        };
        cv::Point best         = candidates.front();
        int       best_overlap = std::numeric_limits<int>::max();
        for (const cv::Point& corner : candidates)
        {
            const cv::Rect label(corner, size);
            int            overlap = 0;
            for (const cv::Rect& other : taken)
            {
                overlap += (label & other).area();
            }
            if (overlap < best_overlap)
            {
                best_overlap = overlap;
                best         = corner;
            }
            if (overlap == 0)
            {
                break;
            }
        }
        taken.emplace_back(best, size);
        placed.push_back(best);
    }
    return placed;
}

cv::Mat renderWorld(
    const cv::Mat& cells, const GridGeometry& geometry, const cv::Mat& room_labels,
    const std::vector<RenderRoom>& rooms, const std::vector<RenderObject>& objects, int scale)
{
    const int  width  = geometry.width * scale;
    const int  height = geometry.height * scale;
    cv::Mat    image(height, width, CV_8UC3, cv::Scalar(205, 205, 205));
    const bool tinted = room_labels.rows == geometry.height && room_labels.cols == geometry.width;
    const auto bgr    = [](const cv::Vec3f& rgb, float light) {
        const auto mix = [light](float c) {
            return cv::saturate_cast<std::uint8_t>(255.0F * ((c * (1.0F - light)) + light));
        };
        return cv::Vec3b(mix(rgb[2]), mix(rgb[1]), mix(rgb[0]));
    };
    for (int y = 0; y < geometry.height; ++y)
    {
        for (int x = 0; x < geometry.width; ++x)
        {
            cv::Vec3b  colour(205, 205, 205);
            const auto cell = cells.at<std::uint8_t>(y, x);
            if (cell == kOccupied)
            {
                colour = { 40, 40, 40 };
            }
            else if (cell == kFree)
            {
                colour          = { 255, 255, 255 };
                const int label = tinted ? room_labels.at<int>(y, x) : 0;
                if (label > 0 && label <= static_cast<int>(rooms.size()))
                {
                    colour = bgr(roomColour(static_cast<std::size_t>(label - 1)), 0.72F);
                }
            }
            // Row y grows with map y: the image's top row is the map's highest.
            image(cv::Rect(x * scale, (geometry.height - 1 - y) * scale, scale, scale)) =
                cv::Scalar(colour);
        }
    }
    const auto pixel = [&](double map_x, double map_y) {
        return cv::Point2f(
            static_cast<float>((map_x - geometry.origin_x) / geometry.resolution * scale),
            static_cast<float>(
                height - ((map_y - geometry.origin_y) / geometry.resolution * scale)));
    };
    const double font  = 0.4 + (0.1 * scale);
    const auto   write = [&](const std::string& text,
                           const cv::Point&   corner,
                           const cv::Size&    size,
                           double             scale_of_text,
                           int                weight,
                           const cv::Scalar&  colour) {
        const cv::Point origin(corner.x, corner.y + size.height);
        cv::putText(
            image,
            text,
            origin,
            cv::FONT_HERSHEY_SIMPLEX,
            scale_of_text,
            cv::Scalar(255, 255, 255),
            weight + 3,
            cv::LINE_AA);
        cv::putText(
            image,
            text,
            origin,
            cv::FONT_HERSHEY_SIMPLEX,
            scale_of_text,
            colour,
            weight,
            cv::LINE_AA);
    };

    // Room names first: object labels make way for them.
    std::vector<cv::Rect>  taken;
    std::vector<cv::Point> room_corners;
    std::vector<cv::Size>  room_sizes;
    for (const RenderRoom& room : rooms)
    {
        int            baseline = 0;
        const cv::Size size =
            cv::getTextSize(room.text, cv::FONT_HERSHEY_SIMPLEX, font, 2, &baseline);
        const cv::Point2f at = pixel(room.x, room.y);
        room_corners.emplace_back(
            static_cast<int>(at.x) - (size.width / 2),
            static_cast<int>(at.y) - (size.height / 2));
        room_sizes.push_back(size);
        taken.emplace_back(room_corners.back(), size);
    }

    std::vector<std::size_t> order(objects.size());
    std::iota(order.begin(), order.end(), std::size_t{ 0 });
    std::ranges::sort(order, std::greater{}, [&](std::size_t i) {
        return objects[i].footprint.size.prod();
    });
    std::vector<cv::Rect> boxes;
    std::vector<cv::Size> sizes;
    for (const std::size_t i : order)
    {
        const Footprint&       box = objects[i].footprint;
        const double           c   = std::cos(box.yaw);
        const double           s   = std::sin(box.yaw);
        std::vector<cv::Point> corners;
        for (const auto& [u, v] :
             { std::pair{ -0.5, -0.5 }, { 0.5, -0.5 }, { 0.5, 0.5 }, { -0.5, 0.5 } })
        {
            const double du = u * box.size.x();
            const double dv = v * box.size.y();
            corners.emplace_back(
                pixel(box.centre.x() + (du * c) - (dv * s), box.centre.y() + (du * s) + (dv * c)));
        }
        cv::polylines(
            image,
            corners,
            true,
            cv::Scalar(20, 60, 200),
            std::max(1, scale / 2),
            cv::LINE_AA);
        int baseline = 0;
        boxes.push_back(cv::boundingRect(corners));
        sizes.push_back(
            cv::getTextSize(objects[i].text, cv::FONT_HERSHEY_SIMPLEX, 0.6 * font, 1, &baseline));
    }
    const std::vector<cv::Point> labels = placeLabels(boxes, sizes, taken);
    for (std::size_t n = 0; n < order.size(); ++n)
    {
        write(objects[order[n]].text, labels[n], sizes[n], 0.6 * font, 1, cv::Scalar(20, 40, 150));
    }
    for (std::size_t slot = 0; slot < rooms.size(); ++slot)
    {
        write(
            rooms[slot].text,
            room_corners[slot],
            room_sizes[slot],
            font,
            2,
            cv::Scalar(bgr(roomColour(slot), 0.0F)));
    }
    return image;
}

}  // namespace canopy
