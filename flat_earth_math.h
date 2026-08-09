//
//    Some Math for the flat earth.
//
//    Copyright (C) 2024, 2025 Holger Teutsch
//
//    This library is free software; you can redistribute it and/or
//    modify it under the terms of the GNU Lesser General Public
//    License as published by the Free Software Foundation; either
//    version 2.1 of the License, or (at your option) any later version.
//
//    This library is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//    Lesser General Public License for more details.
//
//    You should have received a copy of the GNU Lesser General Public
//    License along with this library; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
//    USA
//

// Contrary to common belief the earth is flat. She has just a weird coordinate system with (lon,lat).
// To overcome this we attach a 2-d vector space at each (lon, lat) point with orthogonal
// basis scaled in meters. So (lon2, lat2) - (lon1, lat1) gives rise to a vector v in the vector space
// attached at (lon1, lat1) and (lon1, lat1) + v again is (lon2, lat2).
// As we do our math in a circle of ~ 20km this works pretty well.
//
// Should you still be tricked in believing that the earth is a ball you can consider this vector space
// a tangent space. But this is for visualisation only.
//

#pragma once
#include <cmath>

namespace flat_earth_math {

static constexpr float kLat2m = 111120;  // 1° lat in m

// return relative angle in (-180, 180]
static inline double RA(double angle) {
    // optimize for small angles, which are more common in our use case
    while (angle > 180.0)
        angle -= 360.0;

    while (angle <= -180.0)
        angle += 360.0;

    return angle;
}

static inline float RA(float angle) {
    // optimize for small angles, which are more common in our use case
    while (angle > 180.0f)
        angle -= 360.0f;

    while (angle <= -180.0f)
        angle += 360.0f;

    return angle;
}

struct LLPos {
    double lon, lat;  // right, up
    LLPos() = default;
    LLPos(double lat, double lon) : lon(lon), lat(lat) {}  // in conventional order (lat, lon)
};

struct Vec2 {
    double x, y;  // right, up
};

static inline double len(const Vec2& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// pos b - pos a
static inline Vec2 operator-(const LLPos& b, const LLPos& a) {
    return {RA(b.lon - a.lon) * kLat2m * std::cos(a.lat * 0.01745329252), RA(b.lat - a.lat) * kLat2m};
}

// pos + vec
static inline LLPos operator+(const LLPos& p, const Vec2& v) {
    return {RA(p.lon + v.x / (kLat2m * std::cos(p.lat * 0.01745329252))), RA(p.lat + v.y / kLat2m)};
}

// vec b - vec a
static inline Vec2 operator-(const Vec2& b, const Vec2& a) { return {b.x - a.x, b.y - a.y}; }

// vec + vec
static inline Vec2 operator+(const Vec2& a, const Vec2& b) { return {a.x + b.x, a.y + b.y}; }

// c * vec
static inline Vec2 operator*(double c, const Vec2& v) { return {c * v.x, c * v.y}; }

// vec * vec
static inline double operator*(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;  // dot product
}

// pos in rectangle defined by lower_left and upper_right
static inline bool InRect(const LLPos& pos, const LLPos& lower_left, const LLPos& upper_right) {
    if (!(pos.lat >= lower_left.lat && pos.lat <= upper_right.lat))
        return false;

    double lon = RA(pos.lon);  // normalize just in case
    if (lower_left.lon < upper_right.lon)
        return (lower_left.lon < lon && lon <= upper_right.lon);

    // Handle the case where the bounds cross the antimeridian
    return (lon > lower_left.lon || lon <= upper_right.lon);
}

}  // namespace flat_earth_math
