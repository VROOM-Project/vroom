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

#ifndef POLYLINEENCODER_H
#define POLYLINEENCODER_H

#include <string>
#include <vector>

namespace gepaf
{

//! Implements Google's Encoded Polyline Algorithm Format
/*!
    For more details refer to the algorithm definition at
    https://developers.google.com/maps/documentation/utilities/polylinealgorithm

    Default implementation (precision of 5 decimal places) guarantees to conform
    with the results of the Google Interactive Polyline Encoder Utility
    (https://developers.google.com/maps/documentation/utilities/polylineutility)
*/
template<int Digits = 5>
class PolylineEncoder
{
public:
    /// A geodetic point.
    class Point
    {
    public:
        /// Creates a geodetic point with the given coordinates.
        /*!
            Both latitude and longitude will be rounded to a reasonable precision
            of 5 decimal places (default) or to the number of digits specified by the template parameter..
            \param latitude  The latitude in decimal point degrees. The values are bounded by ±90.0°.
            \param longitude The longitude in decimal point degrees. The values are bounded by ±180.0°.
        */
        Point(double latitude, double longitude);

        /// Returns the latitude.
        double latitude() const;

        /// Returns the longitude.
        double longitude() const;

    private:
        double m_latitude { 0.0 };
        double m_longitude{ 0.0 };
    };

    /// The container of geodetic points to be encoded.
    using Polyline = std::vector<Point>;

    //! Adds new point with the given \p latitude and \p longitude for encoding.
    /*!
        Note: both latitude and longitude will be rounded to a reasonable precision
        of 5 decimal places (default) or to the number of digits specified by the
        template parameter.
        \param latitude  The latitude in decimal point degrees. The values are bounded by ±90.0°.
        \param longitude The longitude in decimal point degrees. The values are bounded by ±180.0°.
    */
    void addPoint(double latitude, double longitude);

    //! Encode the polyline according to the defined compression algorithm.
    /*!
    \return The encoded polyline as string.
    */
    std::string encode() const;

    //! Returns the existing polyline.
    const Polyline &polyline() const;

    //! Clears the list of points.
    void clear();

    //! Returns the result of encoding of the given polyline.
    static std::string encode(const Polyline &polyline);

    //! Returns polyline decoded from the given \p coordinates string.
    static Polyline decode(const std::string &coordinates);

    enum Precision
    {
        Value = PolylineEncoder<Digits - 1>::Precision::Value * 10
    };

private:
    //! Encodes a single value according to the compression algorithm.
    static std::string encode(double value);

    //! Decodes the current decimal value out of string.
    static double decode(const std::string &coords, size_t &i);

    //! Store the polyline - the list of points.
    Polyline m_polyline;
};

// A bogus class for compile-time precision calculations.
template<>
class PolylineEncoder<0>
{
public:
    enum Precision
    {
        Value = 1 // 10^0 = 1
    };
};

} // namespace

#endif // POLYLINEENCODER_H
