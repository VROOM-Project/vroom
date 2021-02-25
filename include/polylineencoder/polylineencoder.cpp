/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2017 Vahan Aghajanyan <vahancho@gmail.com>                       *
*                                                                                 *
*  Permission is hereby granted, free of charge, to any person obtaining a copy   *
*  of this software and associated documentation files (the "Software"), to deal  *
*  in the Software without restriction, including without limitation the rights   *
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      *
*  copies of the Software, and to permit persons to whom the Software is          *
*  furnished to do so, subject to the following conditions:                       *
*                                                                                 *
*  The above copyright notice and this permission notice shall be included in all *
*  copies or substantial portions of the Software.                                *
*                                                                                 *
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     *
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       *
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    *
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         *
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  *
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  *
*  SOFTWARE.                                                                      *
***********************************************************************************/

#include <cinttypes>
#include <cassert>
#include <cmath>
#include <limits>

#include "polylineencoder.h"

static const int    s_chunkSize   = 5;
static const int    s_asciiOffset = 63;
static const int    s_5bitMask    = 0x1f; // 0b11111 = 31
static const int    s_6bitMask    = 0x20; // 0b100000 = 32

namespace gepaf
{

template<int Digits>
PolylineEncoder<Digits>::Point::Point(double latitude, double longitude)
    : m_latitude (std::round(latitude  * Precision::Value) / Precision::Value)
    , m_longitude(std::round(longitude * Precision::Value) / Precision::Value)
{
    assert(latitude <= 90.0 && latitude >= -90.0);
    assert(longitude <= 180.0 && longitude >= -180.0);
}

template<int Digits>
double PolylineEncoder<Digits>::Point::latitude() const
{
    return m_latitude;
}

template<int Digits>
double PolylineEncoder<Digits>::Point::longitude() const
{
    return m_longitude;
}

template<int Digits>
void PolylineEncoder<Digits>::addPoint(double latitude, double longitude)
{
    m_polyline.emplace_back(latitude, longitude);
}

template<int Digits>
std::string PolylineEncoder<Digits>::encode() const
{
    return encode(m_polyline);
}

template<int Digits>
std::string PolylineEncoder<Digits>::encode(double value)
{
    int32_t e5 =
            std::round(value * Precision::Value); // (2)

    e5 <<= 1;                                     // (4)

    if (value < 0) {
        e5 = ~e5;                                 // (5)
    }

    bool hasNextChunk = false;
    std::string result;

    // Split the value into 5-bit chunks and convert each of them to integer
    do {
        int32_t nextChunk = (e5 >> s_chunkSize); // (6), (7) - start from the left 5 bits.
        hasNextChunk = nextChunk > 0;

        int charVar = e5 & s_5bitMask;           // 5-bit mask (0b11111 == 31). Extract the left 5 bits.
        if (hasNextChunk) {
            charVar |= s_6bitMask;               // (8)
        }
        charVar += s_asciiOffset;                // (10)
        result += (char)charVar;                 // (11)

        e5 = nextChunk;
    } while (hasNextChunk);

    return result;
}

template<int Digits>
std::string PolylineEncoder<Digits>::encode(const PolylineEncoder::Polyline &polyline)
{
    std::string result;

    // The first segment: offset from (.0, .0)
    double latPrev = .0;
    double lonPrev = .0;

    for (const auto &point : polyline)
    {
      const auto lat = point.latitude();
      const auto lon = point.longitude();

      // Offset from the previous point
      result.append(encode(lat - latPrev));
      result.append(encode(lon - lonPrev));

      latPrev = lat;
      lonPrev = lon;
    }

    return result;
}

template<int Digits>
double PolylineEncoder<Digits>::decode(const std::string &coords, size_t &i)
{
    assert(i < coords.size());

    int32_t result = 0;
    int shift = 0;
    char c = 0;
    do {
        if (i < coords.size()) {
            c = coords.at(i++);
            c -= s_asciiOffset;      // (10)
            result |= (c & s_5bitMask) << shift;
            shift += s_chunkSize;    // (7)
        } else {
            return std::numeric_limits<double>::quiet_NaN();
        }
    } while (c >= s_6bitMask);

    if (result & 1) {
        result = ~result;        // (5)
    }
    result >>= 1;                // (4)

    // Convert to decimal value with the given precision.
    return result / static_cast<double>(Precision::Value); // (2)
}

template<int Digits>
typename PolylineEncoder<Digits>::Polyline PolylineEncoder<Digits>::decode(const std::string &coords)
{
    PolylineEncoder<Digits>::Polyline polyline;

    size_t i = 0;
    while (i < coords.size())
    {
        double lat = decode(coords, i);
        if (std::isnan(lat) || fabs(lat) > 90.0) {
            // Invalid latitude, implies invalid polyline string.
            polyline.clear();
            break;  // exit while
        }

        double lon = std::numeric_limits<double>::quiet_NaN();
        if (i < coords.size()) {
            lon = decode(coords, i);
        }
        if (std::isnan(lon) || fabs(lon) > 180.0) {
            // Invalid longitude, implies invalid polyline string.
            polyline.clear();
            break;  // exit while
        }

        if (!polyline.empty()) {
            const auto &prevPoint = polyline.back();
            lat += prevPoint.latitude();
            lon += prevPoint.longitude();
        }
        polyline.emplace_back(lat, lon);
    }

    return polyline;
}

template<int Digits>
const typename PolylineEncoder<Digits>::Polyline &PolylineEncoder<Digits>::polyline() const
{
    return m_polyline;
}

template<int Digits>
void PolylineEncoder<Digits>::clear()
{
    m_polyline.clear();
}

} // namespace

