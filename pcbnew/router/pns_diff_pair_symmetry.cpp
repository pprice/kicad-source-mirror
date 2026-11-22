/*
 * KiRouter - a push-and-(sometimes-)shove PCB router
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "pns_diff_pair_symmetry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <base_units.h>
#include <geometry/seg.h>
#include <geometry/shape.h>
#include <geometry/shape_arc.h>
#include <geometry/shape_circle.h>
#include <geometry/shape_compound.h>
#include <geometry/shape_rect.h>
#include <geometry/shape_segment.h>
#include <geometry/shape_simple.h>
#include <math/util.h>

namespace PNS
{

namespace
{

// The geometry here is worked in millimetres, as doubles, and turned back into the router's
// nanometres at the end.
using PT = VECTOR2D;

/// The straight the centre line keeps either side of a sideways step, where there is room.
constexpr double STEP_STRAIGHT = 0.7;

/// What is cut off the corner where a pair turns a right angle, making it two turns of 45.
constexpr double CHAMFER = 1.0;

/// Two points nearer than this are one.
constexpr double HAIR = 1e-7;

/// A bump's top, and the straight between two bumps, in track widths (Altera, "Agilex 7 Device
/// Family High-Speed Serial Interface Signal Integrity Design Guidelines", 683864, section
/// 1.4.1.1, figures 7 and 8: "E = F = G = 3W", and the gap at a bump under twice the pair's).
constexpr double BUMP_TOP = 3.0;

/// A bend that the next bend takes back within this distance is left to it: the two cancel as
/// near the mismatch as a bump would.  From the span Altera gives a deskew ("<100 mils").
constexpr double TAKEN_BACK = 2.54;

/// A difference in length smaller than this is left as it is.
constexpr double SKEW_TOLERANCE = 0.025;

/// The least straight left between two arcs, where one is left at all.
constexpr double SHORT_STRAIGHT = 0.1;

constexpr double ARC_HAIR = 1e-9;
constexpr double MM_PER_NM = 1e-6;


double dot( const PT& a, const PT& b )
{
    return a.x * b.x + a.y * b.y;
}


double cross( const PT& a, const PT& b )
{
    return a.x * b.y - a.y * b.x;
}


double dist( const PT& a, const PT& b )
{
    return std::hypot( b.x - a.x, b.y - a.y );
}


PT unit( const PT& a, const PT& b )
{
    double d = dist( a, b );
    return PT( ( b.x - a.x ) / d, ( b.y - a.y ) / d );
}


/// The side of a line a positive offset lies on.
PT beside( const PT& d )
{
    return PT( d.y, -d.x );
}


double signOf( double v )
{
    return std::signbit( v ) ? -1.0 : 1.0;
}


PT toMm( const VECTOR2I& p )
{
    return PT( p.x * MM_PER_NM, p.y * MM_PER_NM );
}


VECTOR2I toNm( const PT& p )
{
    return VECTOR2I( KiROUND( p.x / MM_PER_NM ), KiROUND( p.y / MM_PER_NM ) );
}


/// How far a pad's or via's copper reaches from aFrom the way aOut points.
double reach( const SHAPE* aShape, const PT& aFrom, const PT& aOut )
{
    double best = -std::numeric_limits<double>::infinity();

    auto at = [&]( const VECTOR2I& p, double r )
    {
        best = std::max( best, dot( toMm( p ) - aFrom, aOut ) + r );
    };

    switch( aShape->Type() )
    {
    case SH_CIRCLE:
    {
        const SHAPE_CIRCLE* c = static_cast<const SHAPE_CIRCLE*>( aShape );
        at( c->GetCenter(), c->GetRadius() * MM_PER_NM );
        break;
    }

    case SH_RECT:
    {
        const SHAPE_RECT* r = static_cast<const SHAPE_RECT*>( aShape );
        const VECTOR2I    p = r->GetPosition();
        const VECTOR2I    s = r->GetSize();
        at( p, 0.0 );
        at( p + VECTOR2I( s.x, 0 ), 0.0 );
        at( p + VECTOR2I( 0, s.y ), 0.0 );
        at( p + s, 0.0 );
        break;
    }

    case SH_SEGMENT:
    {
        const SHAPE_SEGMENT* s = static_cast<const SHAPE_SEGMENT*>( aShape );
        at( s->GetSeg().A, s->GetWidth() / 2.0 * MM_PER_NM );
        at( s->GetSeg().B, s->GetWidth() / 2.0 * MM_PER_NM );
        break;
    }

    case SH_SIMPLE:
    {
        const SHAPE_SIMPLE* s = static_cast<const SHAPE_SIMPLE*>( aShape );

        for( int i = 0; i < s->PointCount(); i++ )
            at( s->CPoint( i ), 0.0 );

        break;
    }

    case SH_COMPOUND:
        for( const SHAPE* sub : static_cast<const SHAPE_COMPOUND*>( aShape )->Shapes() )
            best = std::max( best, reach( sub, aFrom, aOut ) );

        break;

    default:
    {
        const BOX2I box = aShape->BBox();
        at( box.GetOrigin(), 0.0 );
        at( box.GetOrigin() + VECTOR2I( box.GetWidth(), 0 ), 0.0 );
        at( box.GetOrigin() + VECTOR2I( 0, box.GetHeight() ), 0.0 );
        at( box.GetEnd(), 0.0 );
        break;
    }
    }

    return best;
}


/// One end of a stretch, worked out.
struct END
{
    /// The P line's pad first.
    PT pads[2];

    /// Half way between the pads.
    PT mid;

    /// The way the pair leaves the end: square to the line through its pads.
    PT out;

    /// How far from the pads the lines run straight before they turn in.
    double lead = 0.0;

    /// Where the lines have closed to their place beside the centre line.
    PT closed;
};


std::optional<END> makeEnd( const DP_SYMMETRY_END& aEnd, const PT& aToward, double aHalf, double aWidth,
                            double aExtra )
{
    END e;
    e.pads[0] = toMm( aEnd.anchor[0] );
    e.pads[1] = toMm( aEnd.anchor[1] );

    const double apart = dist( e.pads[0], e.pads[1] );

    if( apart < HAIR )
        return std::nullopt;

    const PT along = unit( e.pads[1], e.pads[0] );
    e.mid = ( e.pads[0] + e.pads[1] ) * 0.5;

    if( !aEnd.copper[0] || !aEnd.copper[1] )
    {
        // Free track ends: the pair already runs there at its gap, and goes on as it was.
        const double size = aEnd.out.EuclideanNorm();

        if( size < HAIR )
            return std::nullopt;

        e.out = aEnd.out * ( 1.0 / size );

        if( std::abs( dot( e.out, along ) ) > 1e-3 || std::abs( apart / 2.0 - aHalf ) > 0.005 )
            return std::nullopt;

        e.lead = 0.0;
        e.closed = e.mid;
        return e;
    }

    const PT     square( -along.y, along.x );
    const double side = dot( square, aToward - e.mid );

    if( std::abs( side ) < 1e-6 )
        return std::nullopt;

    e.out = square * signOf( side );

    // Off the pad's copper, and a track's width more
    e.lead = std::max( reach( aEnd.copper[0], e.pads[0], e.out ), reach( aEnd.copper[1], e.pads[1], e.out ) )
             + aWidth + aExtra;

    if( e.lead <= 0.0 )
        return std::nullopt;

    // The lines turn in (or out) at 45 degrees: as far along as across
    e.closed = e.mid + e.out * ( e.lead + std::abs( apart / 2.0 - aHalf ) );
    return e;
}


/// The plain way between two ends with no corner of the router's to keep: straight, a sideways
/// step at 45 degrees between ends that face each other, or one turn between ends at an angle,
/// a right angle cut into two of 45.
std::optional<std::vector<PT>> plainWay( const END& a, const END& b )
{
    const double step = 0.5;
    const PT     across = b.closed - a.closed;
    const double along = dot( across, a.out );
    const double aside = dot( across, beside( a.out ) );

    if( dot( a.out, b.out ) < -0.999 )
    {
        if( along <= HAIR )
            return std::nullopt;

        if( std::abs( aside ) < 1e-6 )
            return std::vector<PT>{ a.closed, b.closed };

        double first, second;

        if( along >= std::abs( aside ) + 2.0 * STEP_STRAIGHT )
        {
            double room = along - std::abs( aside ) - 2.0 * STEP_STRAIGHT;
            first = STEP_STRAIGHT + room * step;
            second = STEP_STRAIGHT + room * ( 1.0 - step );
        }
        else
        {
            first = second = std::min( STEP_STRAIGHT, along / 4.0 );
        }

        return std::vector<PT>{ a.closed, a.closed + a.out * first, b.closed + b.out * second, b.closed };
    }

    const double turn = cross( a.out, b.out );

    if( std::abs( turn ) < 1e-6 )
        return std::nullopt;

    const double p = cross( across, b.out ) / turn;
    const double q = cross( across, a.out ) / turn;

    if( p <= HAIR || q <= HAIR )
        return std::nullopt;

    const PT corner = a.closed + a.out * p;

    if( std::abs( dot( a.out, b.out ) ) > 0.5 )
        return std::vector<PT>{ a.closed, corner, b.closed };

    const double cut = std::min( CHAMFER, std::min( p, q ) / 2.0 );
    return std::vector<PT>{ a.closed, corner - a.out * cut, corner - b.out * cut, b.closed };
}


/// A stretch of the router's centre line: half way between two straights of its lines that run
/// side by side at the pair's distance.
struct MIDLINE
{
    PT a;
    PT b;
    PT dir;
};


bool sameWay( const PT& u, const PT& v )
{
    return std::abs( cross( u, v ) ) < 1e-3 && dot( u, v ) > 0.0;
}


std::vector<MIDLINE> midlines( const SHAPE_LINE_CHAIN& aP, const SHAPE_LINE_CHAIN& aN, double aHalf )
{
    struct FOUND
    {
        int     index;
        double  t;
        MIDLINE m;
    };

    std::vector<FOUND> found;
    const double       tolerance = 0.1 * 2.0 * aHalf;

    for( int i = 0; i < aP.SegmentCount(); i++ )
    {
        if( aP.IsArcSegment( i ) )
            continue;

        const PT     pa = toMm( aP.CSegment( i ).A );
        const PT     pb = toMm( aP.CSegment( i ).B );
        const double length = dist( pa, pb );

        if( length < HAIR )
            continue;

        const PT u = unit( pa, pb );
        const PT left( -u.y, u.x );

        for( int j = 0; j < aN.SegmentCount(); j++ )
        {
            if( aN.IsArcSegment( j ) )
                continue;

            const PT na = toMm( aN.CSegment( j ).A );
            const PT nb = toMm( aN.CSegment( j ).B );

            if( dist( na, nb ) < HAIR || !sameWay( u, unit( na, nb ) ) )
                continue;

            const double d = dot( left, na - pa );

            if( std::abs( std::abs( d ) - 2.0 * aHalf ) > tolerance )
                continue;

            const double t0 = dot( na - pa, u );
            const double t1 = dot( nb - pa, u );
            const double lo = std::max( 0.0, std::min( t0, t1 ) );
            const double hi = std::min( length, std::max( t0, t1 ) );

            if( hi - lo < 1e-4 )
                continue;

            MIDLINE m;
            m.a = pa + u * lo + left * ( d / 2.0 );
            m.b = pa + u * hi + left * ( d / 2.0 );
            m.dir = u;
            found.push_back( { i, lo, m } );
        }
    }

    std::sort( found.begin(), found.end(),
               []( const FOUND& x, const FOUND& y )
               {
                   return x.index != y.index ? x.index < y.index : x.t < y.t;
               } );

    // Stretches on one line are one
    std::vector<MIDLINE> out;

    for( const FOUND& f : found )
    {
        if( !out.empty() && sameWay( out.back().dir, f.m.dir )
            && std::abs( dot( beside( out.back().dir ), f.m.a - out.back().a ) ) < 1e-3 )
        {
            if( dot( f.m.b - out.back().b, out.back().dir ) > 0.0 )
                out.back().b = f.m.b;

            continue;
        }

        out.push_back( f.m );
    }

    return out;
}


/// Where two lines, each a point and a way, cross.
std::optional<PT> meet( const PT& p, const PT& u, const PT& q, const PT& v )
{
    const double c = cross( u, v );

    if( std::abs( c ) < 1e-9 )
        return std::nullopt;

    return p + u * ( cross( q - p, v ) / c );
}


/// The corners of the pair's centre line, from where the lines have closed at one end to where
/// they open at the other: the router's corners where its lines run side by side, the first and
/// last straight moved to run out from the middle of each end's pads.
std::optional<std::vector<PT>> centreRoute( const END& a, const END& b, const std::vector<MIDLINE>& aMids,
                                            double aHalf )
{
    struct WAY
    {
        PT   at;
        PT   dir;
        bool mid;
    };

    const PT bWay = b.out * -1.0;

    size_t first = 0;
    size_t last = aMids.size();

    if( last > first && sameWay( aMids[first].dir, a.out ) )
        first++;

    if( last > first && sameWay( aMids[last - 1].dir, bWay ) )
        last--;

    std::vector<WAY> ways;
    ways.push_back( { a.mid, a.out, false } );

    for( size_t k = first; k < last; k++ )
        ways.push_back( { aMids[k].a, aMids[k].dir, true } );

    ways.push_back( { b.mid, bWay, false } );

    if( ways.size() == 2 )
        return plainWay( a, b );

    std::vector<PT> route = { a.closed };

    for( size_t k = 0; k + 1 < ways.size(); k++ )
    {
        std::optional<PT> corner = meet( ways[k].at, ways[k].dir, ways[k + 1].at, ways[k + 1].dir );

        if( !corner )
            return std::nullopt;

        // A corner far from where the router's lines turn is no corner of theirs
        const double nearEnough = std::max( 1.0, 8.0 * aHalf );

        if( ways[k].mid && dist( *corner, aMids[first + k - 1].b ) > nearEnough )
            return std::nullopt;

        if( ways[k + 1].mid && dist( *corner, aMids[first + k].a ) > nearEnough )
            return std::nullopt;

        route.push_back( *corner );
    }

    route.push_back( b.closed );

    // A line set off to the inside of a corner turns short of it, by half the pair's distance
    // times the tangent of half the turn.  Each straight (route segment k, on ways[k]) runs the way
    // its line does and is long enough for both its ends' turns, or the lines would double back.
    auto turnAt = [&]( size_t j )
    {
        return std::acos( std::clamp( dot( ways[j].dir, ways[j + 1].dir ), -1.0, 1.0 ) );
    };

    for( size_t k = 0; k + 1 < route.size(); k++ )
    {
        const PT     d = route[k + 1] - route[k];
        const double length = d.EuclideanNorm();
        double       need = 0.0;

        if( k > 0 )
            need += aHalf * std::tan( turnAt( k - 1 ) / 2.0 );

        if( k + 1 < ways.size() )
            need += aHalf * std::tan( turnAt( k ) / 2.0 );

        if( dot( d, ways[k].dir ) < need - 1e-4 )
            return std::nullopt;

        if( length > HAIR && dot( d, ways[k].dir ) < 0.99 * length )
            return std::nullopt;
    }

    return route;
}


/// A line set off to one side of another, its corners mitred, so the two are the same distance
/// apart all the way.
std::optional<std::vector<PT>> offset( const std::vector<PT>& aLine, double aBy )
{
    std::vector<PT> out;
    out.reserve( aLine.size() );

    for( size_t i = 0; i < aLine.size(); i++ )
    {
        std::optional<PT> before, after;

        if( i > 0 )
            before = beside( unit( aLine[i - 1], aLine[i] ) );

        if( i + 1 < aLine.size() )
            after = beside( unit( aLine[i], aLine[i + 1] ) );

        const PT n1 = before ? *before : ( after ? *after : PT( 0, 0 ) );
        const PT n2 = after ? *after : ( before ? *before : PT( 0, 0 ) );
        PT       m = n1 + n2;
        double   size = m.EuclideanNorm();

        if( size < 1e-3 )
            return std::nullopt;

        m = m * ( 1.0 / size );
        out.push_back( aLine[i] + m * ( aBy / dot( m, n1 ) ) );
    }

    return out;
}


/// A line with no point said twice, and none that is no corner.
std::vector<PT> plain( const std::vector<PT>& aPoints )
{
    std::vector<PT> out;
    out.reserve( aPoints.size() );

    for( const PT& p : aPoints )
    {
        if( !out.empty() && dist( out.back(), p ) < HAIR )
            continue;

        if( out.size() >= 2 )
        {
            const PT d1 = unit( out[out.size() - 2], out.back() );
            const PT d2 = unit( out.back(), p );

            if( std::abs( cross( d1, d2 ) ) < 1e-9 && dot( d1, d2 ) > 0.0 )
                out.pop_back();
        }

        out.push_back( p );
    }

    return out;
}


/// A pair's two lines, P's first, and the centre line they were set off from.
struct DRAWN
{
    std::vector<PT> lines[2];
    std::vector<PT> centre;

    /// How far each line is set off from the centre, signed.
    double by[2] = { 0.0, 0.0 };
};


std::optional<DRAWN> draw( const END& a, const END& b, const std::vector<PT>& aRoute, double aHalf )
{
    // The P line keeps one side of the centre line from end to end
    const double side = signOf( dot( a.pads[0] - a.mid, beside( a.out ) ) );

    if( signOf( dot( b.pads[0] - b.mid, beside( b.out * -1.0 ) ) ) != side )
        return std::nullopt;

    std::vector<PT> whole = { a.mid };
    whole.insert( whole.end(), aRoute.begin(), aRoute.end() );
    whole.push_back( b.mid );
    whole.erase( std::unique( whole.begin(), whole.end(),
                              []( const PT& x, const PT& y )
                              {
                                  return dist( x, y ) < HAIR;
                              } ),
                 whole.end() );

    if( whole.size() < 2 )
        return std::nullopt;

    DRAWN drawn;
    drawn.by[0] = side * aHalf;
    drawn.by[1] = -side * aHalf;

    for( int i = 0; i < 2; i++ )
    {
        std::optional<std::vector<PT>> setOff = offset( whole, drawn.by[i] );

        if( !setOff )
            return std::nullopt;

        std::vector<PT> points = { a.pads[i], a.pads[i] + a.out * a.lead };
        points.insert( points.end(), setOff->begin() + 1, setOff->end() - 1 );
        points.push_back( b.pads[i] + b.out * b.lead );
        points.push_back( b.pads[i] );
        drawn.lines[i] = plain( points );
    }

    drawn.centre = plain( whole );
    return drawn;
}


// Arcs at a line's corners.  Two arcs share the straight between them in proportion to what each
// asks, so a wide arc beside a small one keeps its radius (a pair's outer line beside its inner
// one, about one centre), and what they leave of a straight is nothing or a whole track.

struct PIECE
{
    bool   arc = false;
    PT     from;
    PT     mid;
    PT     to;
    double length = 0.0;
};


std::vector<double> turnsOf( const std::vector<PT>& aPoints )
{
    std::vector<double> out;

    for( size_t i = 0; i + 2 < aPoints.size(); i++ )
    {
        const PT d1 = unit( aPoints[i], aPoints[i + 1] );
        const PT d2 = unit( aPoints[i + 1], aPoints[i + 2] );
        out.push_back( std::acos( std::clamp( dot( d1, d2 ), -1.0, 1.0 ) ) );
    }

    return out;
}


/// How much of the straight on either side each corner's arc takes, for corners that ask for
/// aRadii.  An arc takes at most half of a straight that ends the line.
std::vector<double> cutsOf( const std::vector<PT>& aPoints, const std::vector<double>& aRadii )
{
    if( aPoints.size() < 3 )
        return {};

    const std::vector<double> turns = turnsOf( aPoints );
    std::vector<double>       straights;

    for( size_t i = 0; i + 1 < aPoints.size(); i++ )
        straights.push_back( dist( aPoints[i], aPoints[i + 1] ) );

    const size_t        last = turns.size() - 1;
    std::vector<double> cuts( turns.size() );

    for( size_t i = 0; i < turns.size(); i++ )
    {
        const double radius = i < aRadii.size() ? aRadii[i] : 0.0;
        double       cut = ( radius <= 0.0 || M_PI - turns[i] < 1e-3 ) ? 0.0 : radius * std::tan( turns[i] / 2.0 );

        if( i == 0 )
            cut = std::min( cut, straights[0] / 2.0 );

        if( i == last )
            cut = std::min( cut, straights[last + 1] / 2.0 );

        cuts[i] = cut;
    }

    // Each straight between two corners: by how much its two arcs must shrink to fit on it
    std::vector<double> fits;

    for( size_t s = 1; s <= last; s++ )
    {
        fits.push_back( cuts[s - 1] + cuts[s] > straights[s] ? straights[s] / ( cuts[s - 1] + cuts[s] ) : 1.0 );
    }

    for( size_t i = 0; i < cuts.size(); i++ )
    {
        const double before = i > 0 ? fits[i - 1] : 1.0;
        const double after = i < last ? fits[i] : 1.0;
        cuts[i] *= std::min( before, after );
    }

    // An arc too short to be a track of its own is a sharp corner, and what an arc leaves of a
    // straight is a whole track or nothing.  Tightening an arc only lengthens the straights
    // beside it, so this settles.
    auto arcLength = [&]( size_t i, double cut )
    {
        return cut / std::tan( turns[i] / 2.0 ) * turns[i];
    };

    for( size_t round = 0; round <= 2 * cuts.size() + 1; round++ )
    {
        bool changed = false;

        for( size_t i = 0; i < cuts.size(); i++ )
        {
            if( cuts[i] > 0.0 && arcLength( i, cuts[i] ) < SHORT_STRAIGHT - ARC_HAIR )
            {
                cuts[i] = 0.0;
                changed = true;
            }
        }

        for( size_t s = 0; s < straights.size(); s++ )
        {
            const double before = s > 0 ? cuts[s - 1] : 0.0;
            const double after = s < cuts.size() ? cuts[s] : 0.0;
            const double left = straights[s] - before - after;

            if( ( before > 0.0 || after > 0.0 ) && left > ARC_HAIR && left < SHORT_STRAIGHT - ARC_HAIR )
            {
                const double less = SHORT_STRAIGHT - left;

                if( before > 0.0 && after > 0.0 )
                {
                    cuts[s - 1] = std::max( before - less / 2.0, 0.0 );
                    cuts[s] = std::max( after - less / 2.0, 0.0 );
                }
                else if( before > 0.0 )
                {
                    cuts[s - 1] = std::max( before - less, 0.0 );
                }
                else
                {
                    cuts[s] = std::max( after - less, 0.0 );
                }

                changed = true;
            }
        }

        if( !changed )
            break;
    }

    return cuts;
}


/// The arc at the corner aPoints[i + 1] that takes aCut off each straight beside it.
std::optional<PIECE> arcAt( const std::vector<PT>& aPoints, size_t i, double aCut )
{
    const PT     corner = aPoints[i + 1];
    const PT     d1 = unit( aPoints[i], corner );
    const PT     d2 = unit( corner, aPoints[i + 2] );
    const double turn = std::acos( std::clamp( dot( d1, d2 ), -1.0, 1.0 ) );

    if( aCut < ARC_HAIR || turn < 1e-9 )
        return std::nullopt;

    const double radius = aCut / std::tan( turn / 2.0 );
    const PT     inward = d2 - d1;
    const double size = inward.EuclideanNorm();
    const double reachIn = radius / std::cos( turn / 2.0 ) - radius;

    PIECE piece;
    piece.arc = true;
    piece.from = corner - d1 * aCut;
    piece.to = corner + d2 * aCut;
    piece.mid = corner + inward * ( reachIn / size );
    piece.length = radius * turn;
    return piece;
}


/// A line as it is laid, each corner with the arc its own radius asks for.
std::vector<PIECE> roundedEach( const std::vector<PT>& aPoints, const std::vector<double>& aRadii )
{
    std::vector<PIECE> out;

    auto straight = [&]( const PT& from, const PT& to )
    {
        if( dist( from, to ) >= 1e-7 )
        {
            PIECE piece;
            piece.from = from;
            piece.to = to;
            piece.length = dist( from, to );
            out.push_back( piece );
        }
    };

    if( aPoints.size() < 3
        || std::all_of( aRadii.begin(), aRadii.end(),
                        []( double r )
                        {
                            return r <= 0.0;
                        } ) )
    {
        for( size_t i = 0; i + 1 < aPoints.size(); i++ )
            straight( aPoints[i], aPoints[i + 1] );

        return out;
    }

    const std::vector<double> cuts = cutsOf( aPoints, aRadii );
    PT                        here = aPoints[0];

    for( size_t i = 0; i < cuts.size(); i++ )
    {
        const PT             corner = aPoints[i + 1];
        std::optional<PIECE> arc = arcAt( aPoints, i, cuts[i] );

        if( !arc )
        {
            straight( here, corner );
            here = corner;
            continue;
        }

        if( dist( here, arc->from ) < 1e-7 )
            arc->from = here;
        else
            straight( here, arc->from );

        out.push_back( *arc );
        here = arc->to;
    }

    straight( here, aPoints.back() );
    return out;
}


double lengthEach( const std::vector<PT>& aPoints, const std::vector<double>& aRadii )
{
    double total = 0.0;

    for( const PIECE& piece : roundedEach( aPoints, aRadii ) )
        total += piece.length;

    return total;
}


/// How far along a line, as laid, each of its corners is left behind: to the end of the corner's
/// arc, or to the corner itself where it is sharp.
std::vector<double> exitsOf( const std::vector<PT>& aPoints, const std::vector<double>& aRadii )
{
    const std::vector<double> cuts = cutsOf( aPoints, aRadii );

    std::vector<std::pair<PT, double>> ends = { { aPoints[0], 0.0 } };

    for( const PIECE& piece : roundedEach( aPoints, aRadii ) )
        ends.push_back( { piece.to, ends.back().second + piece.length } );

    std::vector<double> out;
    size_t              from = 0;

    for( size_t k = 0; k < cuts.size(); k++ )
    {
        const PT corner = aPoints[k + 1];
        PT       leave = corner;

        if( cuts[k] > ARC_HAIR )
            leave = corner + unit( corner, aPoints[k + 2] ) * cuts[k];

        for( size_t i = from; i < ends.size(); i++ )
        {
            if( dist( ends[i].first, leave ) < 1e-7 )
            {
                from = i;
                break;
            }
        }

        out.push_back( ends[from].second );
    }

    return out;
}


/// A corner of the centre line: where it is, which way it turns and the radius of its arc.
struct BEND
{
    PT     at;
    PT     inward;
    double halfTurn = 0.0;
    double radius = 0.0;

    /// Whether p is this corner set off aHalf to one side.
    bool Is( const PT& p, double aHalf ) const
    {
        const double reachOut = aHalf / std::cos( halfTurn );
        const PT     d = p - at;

        return std::abs( d.EuclideanNorm() - reachOut ) < 1e-6
               && std::abs( cross( d, inward ) ) < 1e-6 * std::max( reachOut, 1.0 );
    }

    bool Inside( const PT& p ) const { return dot( p - at, inward ) > 0.0; }
};


std::vector<BEND> centreCorners( const std::vector<PT>& aCentre, double aRound )
{
    std::vector<BEND> out;

    if( aCentre.size() < 3 )
        return out;

    const std::vector<double> cuts = cutsOf( aCentre, std::vector<double>( aCentre.size() - 2, aRound ) );

    for( size_t i = 0; i + 2 < aCentre.size(); i++ )
    {
        const PT     d1 = unit( aCentre[i], aCentre[i + 1] );
        const PT     d2 = unit( aCentre[i + 1], aCentre[i + 2] );
        const double turn = std::acos( std::clamp( dot( d1, d2 ), -1.0, 1.0 ) );

        if( turn < 1e-6 )
            continue;

        const PT inward = d2 - d1;

        BEND bend;
        bend.at = aCentre[i + 1];
        bend.inward = inward * ( 1.0 / inward.EuclideanNorm() );
        bend.halfTurn = turn / 2.0;
        bend.radius = aRound > 0.0 ? cuts[i] / std::tan( turn / 2.0 ) : 0.0;
        out.push_back( bend );
    }

    return out;
}


/// Each line's corner radii, so the two lines' arcs share their centres: at a corner of the
/// centre line the inner line's arc is tighter by half the distance between the lines and the
/// outer's wider.  A corner of one line alone, where it turns in from its pad, gets aRound itself.
void concentric( const DRAWN& aDrawn, double aRound, std::vector<double> aRadii[2] )
{
    const std::vector<BEND> corners = centreCorners( aDrawn.centre, aRound );

    for( int i = 0; i < 2; i++ )
    {
        const std::vector<PT>& line = aDrawn.lines[i];
        const double           half = std::abs( aDrawn.by[i] );

        aRadii[i].clear();

        for( size_t j = 1; j + 1 < line.size(); j++ )
        {
            const BEND* found = nullptr;

            for( const BEND& c : corners )
            {
                if( c.Is( line[j], half ) )
                {
                    found = &c;
                    break;
                }
            }

            if( !found )
                aRadii[i].push_back( aRound );
            else if( found->radius <= 0.0 )
                aRadii[i].push_back( 0.0 );
            else if( found->Inside( line[j] ) )
                aRadii[i].push_back( std::max( found->radius - half, 0.0 ) );
            else
                aRadii[i].push_back( found->radius + half );
        }
    }
}


/// Whether a straight of a line from a to b runs beside the centre line at aBy from it.
bool alongCentre( const std::vector<PT>& aCentre, const PT& a, const PT& b, double aBy )
{
    if( dist( a, b ) < HAIR )
        return false;

    const PT u = unit( a, b );

    for( size_t k = 0; k + 1 < aCentre.size(); k++ )
    {
        const PT& w0 = aCentre[k];
        const PT& w1 = aCentre[k + 1];

        if( dist( w0, w1 ) < HAIR )
            continue;

        const PT     v = unit( w0, w1 );
        const double sb = dot( b - w0, v );
        const double sa = dot( a - w0, v );

        if( std::abs( cross( u, v ) ) < 1e-6 && dot( u, v ) > 0.0
            && std::abs( dot( a - w0, beside( v ) ) - aBy ) < 1e-6 && sb > 0.0 && sa < dist( w0, w1 ) )
        {
            return true;
        }
    }

    return false;
}


/// One bump's extra length on a line, aHeight high with sharp 45 degree sides: its two sides are
/// each sqrt(2) times as long as the straight they replace.
double bumpExtra( double aHeight )
{
    return 2.0 * aHeight * ( M_SQRT2 - 1.0 );
}


/// Bumps of one pair's size: BUMP_TOP widths across the top, under the gap high, their sides at
/// 45 degrees as Altera draws them, several small rather than one large (Microchip).
class BUMPER
{
public:
    BUMPER( double aWidth, double aGap, const std::function<bool( int, const std::vector<PT>& )>& aClear ) :
            m_width( aWidth ),
            m_top( BUMP_TOP * aWidth ),
            m_most( aGap * 0.95 ),
            m_clear( aClear )
    {
        m_mostExtra = bumpExtra( m_most );
    }

    struct PUT
    {
        int    bumps = 0;
        double added = 0.0;
        size_t points = 0;
    };

    /// Bumps on the straight of line aShort from its point j to the next, away from the other
    /// line (aBy is the line's side of the centre), as many as fit and are needed to add aWant.
    PUT Put( std::vector<PT> aLines[2], std::vector<double> aRadii[2], int aShort, size_t j, double aWant,
             double aBy ) const
    {
        std::vector<PT>& line = aLines[aShort];

        if( j + 1 >= line.size() )
            return {};

        const std::vector<double> cuts = cutsOf( line, aRadii[aShort] );
        const PT                  a = line[j];
        const PT                  b = line[j + 1];
        const double              leave = j == 0 ? 0.0 : cuts[j - 1];
        const double              arrive = j < cuts.size() ? cuts[j] : 0.0;
        const double              room = dist( a, b ) - leave - arrive - 2.0 * m_width;
        const double              span = 2.0 * m_most + m_top;
        const size_t              need = (size_t) std::ceil( aWant / m_mostExtra );

        // From the head of the straight, or as near it as keeps clear: slid on along it a
        // quarter of a millimetre at a time
        for( double slide = 0.0; room - slide >= span; slide += 0.25 )
        {
            const size_t fit = (size_t) std::floor( ( room - slide - span ) / ( span + m_top ) ) + 1;
            const size_t n = std::min( fit, need );
            std::vector<PT> points = shape( a, b, leave + m_width + slide, n, std::min( aWant, n * m_mostExtra ),
                                            aBy );

            if( !m_clear || m_clear( aShort, points ) )
            {
                const double before = lengthEach( line, aRadii[aShort] );
                line.insert( line.begin() + j + 1, points.begin(), points.end() );
                aRadii[aShort].insert( aRadii[aShort].begin() + j, points.size(), 0.0 );

                PUT put;
                put.bumps = (int) n;
                put.added = lengthEach( line, aRadii[aShort] ) - before;
                put.points = points.size();
                return put;
            }
        }

        return {};
    }

private:
    /// The corners of n bumps on the straight from a to b, the first aFrom along it, that add
    /// aWant between them, away from the other line.
    std::vector<PT> shape( const PT& a, const PT& b, double aFrom, size_t n, double aWant, double aBy ) const
    {
        const double height = std::min( m_most, aWant / ( n * bumpExtra( 1.0 ) ) );
        const PT     u = unit( a, b );
        const PT     away = beside( u ) * signOf( aBy );

        std::vector<PT> points;
        PT              here = a + u * aFrom;

        for( size_t i = 0; i < n; i++ )
        {
            if( i > 0 )
                here = here + u * m_top;

            const PT up = here + u * height + away * height;
            const PT over = up + u * m_top;
            const PT down = over + u * height - away * height;

            points.push_back( here );
            points.push_back( up );
            points.push_back( over );
            points.push_back( down );
            here = down;
        }

        return points;
    }

    double m_width;
    double m_top;
    double m_most;
    double m_mostExtra;

    std::function<bool( int, const std::vector<PT>& )> m_clear;
};


/// Where a bend has left one line longer than the other, bumps on the shorter line just after it
/// put the difference back.  A bend the next takes back within TAKEN_BACK is left.  aInitial is
/// how much longer P already is where the stretch starts: it goes back on the straights before
/// the first bend.  What is left at the end goes on any straight with room.
int deskew( const DRAWN& aDrawn, std::vector<PT> aLines[2], std::vector<double> aRadii[2], double aRound,
            double aWidth, double aGap, double aInitial,
            const std::function<bool( int, const std::vector<PT>& )>& aClear )
{
    const std::vector<BEND> bends = centreCorners( aDrawn.centre, aRound );
    const BUMPER            bumper( aWidth, aGap, aClear );
    int                     bumps = 0;

    // Where a bend is on a line: the index of the line's point there
    auto place = [&]( const BEND& aBend, int i ) -> std::optional<size_t>
    {
        for( size_t j = 1; j + 1 < aLines[i].size(); j++ )
        {
            if( aBend.Is( aLines[i][j], std::abs( aDrawn.by[i] ) ) )
                return j;
        }

        return std::nullopt;
    };

    // Two lines leave a bend at different places along the pair where its corners are sharp: the
    // outer line's mitre lies further on along the way in and further back along the way out.
    // Measured at their own corners the lines would be compared at different places, so the
    // line whose corner is the further back along the way out is counted on to the other's:
    // how much further P has to go, at the bend whose points are j0 on P and j1 on N.  Arcs
    // about one centre end level with each other, and this is nothing.
    auto behind = [&]( size_t j0, size_t j1 ) -> double
    {
        auto exitPoint = [&]( int i, size_t j )
        {
            const std::vector<double> cuts = cutsOf( aLines[i], aRadii[i] );
            const PT                  corner = aLines[i][j];
            return corner + unit( corner, aLines[i][j + 1] ) * cuts[j - 1];
        };

        const PT way = unit( aLines[0][j0], aLines[0][j0 + 1] );
        return dot( exitPoint( 1, j1 ) - exitPoint( 0, j0 ), way );
    };

    // Bumps on the shorter line's straights beside the centre line, in order, until the point
    // aUntil gives: what was put back
    auto fill = [&]( double aSkew, const std::function<size_t()>& aUntil ) -> double
    {
        const int short_ = aSkew > 0.0 ? 1 : 0;
        double    left = std::abs( aSkew );
        size_t    j = 0;

        while( left >= SKEW_TOLERANCE && j < std::min( aUntil(), aLines[short_].size() - 1 ) )
        {
            if( alongCentre( aDrawn.centre, aLines[short_][j], aLines[short_][j + 1], aDrawn.by[short_] ) )
            {
                BUMPER::PUT put = bumper.Put( aLines, aRadii, short_, j, left, aDrawn.by[short_] );
                bumps += put.bumps;
                left -= put.added;
                j += put.points;
            }

            j++;
        }

        return std::abs( aSkew ) - left;
    };

    if( std::abs( aInitial ) >= SKEW_TOLERANCE )
    {
        const int short_ = aInitial > 0.0 ? 1 : 0;

        fill( aInitial,
              [&]() -> size_t
              {
                  if( bends.empty() )
                      return std::numeric_limits<size_t>::max();

                  return place( bends.front(), short_ ).value_or( std::numeric_limits<size_t>::max() );
              } );
    }

    for( size_t k = 0; k < bends.size(); k++ )
    {
        std::optional<size_t> j0 = place( bends[k], 0 );
        std::optional<size_t> j1 = place( bends[k], 1 );

        if( !j0 || !j1 )
            continue;

        // How far apart the lines are in length as they leave this bend
        const std::vector<double> exits0 = exitsOf( aLines[0], aRadii[0] );
        const std::vector<double> exits1 = exitsOf( aLines[1], aRadii[1] );
        const double              skew = exits0[*j0 - 1] - exits1[*j1 - 1] + behind( *j0, *j1 ) + aInitial;

        if( std::abs( skew ) < SKEW_TOLERANCE )
            continue;

        // The next bend takes it back soon enough: leave it to that
        if( k + 1 < bends.size() )
        {
            std::optional<size_t> n0 = place( bends[k + 1], 0 );
            std::optional<size_t> n1 = place( bends[k + 1], 1 );

            if( n0 && n1 )
            {
                const double after = exits0[*n0 - 1] - exits1[*n1 - 1] + behind( *n0, *n1 ) + aInitial;

                if( std::abs( after ) < std::abs( skew ) && dist( bends[k].at, bends[k + 1].at ) <= TAKEN_BACK )
                    continue;
            }
        }

        const int short_ = skew > 0.0 ? 1 : 0;
        bumps += bumper.Put( aLines, aRadii, short_, short_ == 0 ? *j0 : *j1, std::abs( skew ),
                             aDrawn.by[short_] )
                         .bumps;
    }

    // Left over at the end: on any straight with room, the nearest the start first
    const double left = lengthEach( aLines[0], aRadii[0] ) - lengthEach( aLines[1], aRadii[1] ) + aInitial;

    if( std::abs( left ) >= SKEW_TOLERANCE )
    {
        fill( left,
              []() -> size_t
              {
                  return std::numeric_limits<size_t>::max();
              } );
    }

    return bumps;
}


SHAPE_LINE_CHAIN chainOf( const std::vector<PT>& aLine, const std::vector<double>& aRadii )
{
    SHAPE_LINE_CHAIN out;

    for( const PIECE& piece : roundedEach( aLine, aRadii ) )
    {
        if( out.PointCount() == 0 )
            out.Append( toNm( piece.from ) );

        if( piece.arc )
            out.Append( SHAPE_ARC( toNm( piece.from ), toNm( piece.mid ), toNm( piece.to ), 0 ) );
        else
            out.Append( toNm( piece.to ) );
    }

    return out;
}


SHAPE_LINE_CHAIN chainOf( const std::vector<PT>& aPoints )
{
    SHAPE_LINE_CHAIN out;

    for( const PT& p : aPoints )
        out.Append( toNm( p ) );

    return out;
}


/// The nearest the two lines' centres come to each other.
double nearestBetween( const SHAPE_LINE_CHAIN& aP, const SHAPE_LINE_CHAIN& aN )
{
    double best = std::numeric_limits<double>::max();

    for( int i = 0; i < aP.SegmentCount(); i++ )
    {
        for( int j = 0; j < aN.SegmentCount(); j++ )
            best = std::min( best, (double) aP.CSegment( i ).Distance( aN.CSegment( j ) ) );
    }

    return best * MM_PER_NM;
}

} // namespace


std::optional<DP_SYMMETRY_RESULT> SymmetricPair( const SHAPE_LINE_CHAIN& aP, const SHAPE_LINE_CHAIN& aN,
                                                 const DP_SYMMETRY_END& aStart, const DP_SYMMETRY_END& aEnd,
                                                 const DP_SYMMETRY_PARAMS& aParams )
{
    if( aParams.width <= 0 || aParams.gap <= 0 || aP.PointCount() < 2 || aN.PointCount() < 2 )
        return std::nullopt;

    const double width = aParams.width * MM_PER_NM;
    const double gap = aParams.gap * MM_PER_NM;
    const double half = ( width + gap ) / 2.0;
    const double round = aParams.radius * MM_PER_NM;

    const std::vector<MIDLINE> mids = midlines( aP, aN, half );

    const PT startMid = ( toMm( aStart.anchor[0] ) + toMm( aStart.anchor[1] ) ) * 0.5;
    const PT endMid = ( toMm( aEnd.anchor[0] ) + toMm( aEnd.anchor[1] ) ) * 0.5;
    const PT towardStart = mids.empty() ? endMid : mids.front().b;
    const PT towardEnd = mids.empty() ? startMid : mids.back().a;

    // Where the pads are nearer than the pair, the lines meet them nearer too
    const double closest = std::min( { 2.0 * half, dist( toMm( aStart.anchor[0] ), toMm( aStart.anchor[1] ) ),
                                       dist( toMm( aEnd.anchor[0] ), toMm( aEnd.anchor[1] ) ) } );

    auto bumpClear = [&]( int aLine, const std::vector<PT>& aPoints )
    {
        return !aParams.bumpClear || aParams.bumpClear( aLine, chainOf( aPoints ) );
    };

    // The lines run on further from their pads until a way keeps clear
    for( double extra : { 0.0, 1.0, 2.0, 4.0 } )
    {
        std::optional<END> a = makeEnd( aStart, towardStart, half, width, extra * width );
        std::optional<END> b = makeEnd( aEnd, towardEnd, half, width, extra * width );

        if( !a || !b )
            return std::nullopt;

        std::optional<std::vector<PT>> route = centreRoute( *a, *b, mids, half );

        if( !route )
            return std::nullopt;

        std::optional<DRAWN> drawn = draw( *a, *b, *route, half );

        if( !drawn )
            return std::nullopt;

        std::vector<PT>     lines[2] = { drawn->lines[0], drawn->lines[1] };
        std::vector<double> radii[2];

        if( round > 0.0 )
        {
            concentric( *drawn, round, radii );
        }
        else
        {
            for( int i = 0; i < 2; i++ )
                radii[i].assign( lines[i].size() >= 2 ? lines[i].size() - 2 : 0, 0.0 );
        }

        DP_SYMMETRY_RESULT result;
        result.bumps = deskew( *drawn, lines, radii, round, width, gap, aParams.skew * MM_PER_NM, bumpClear );
        result.p = chainOf( lines[0], radii[0] );
        result.n = chainOf( lines[1], radii[1] );

        if( result.p.PointCount() < 2 || result.n.PointCount() < 2 )
            return std::nullopt;

        // Arcs are measured on their chords, which cut inside by up to the arc accuracy
        const double slack = ( result.p.ArcCount() || result.n.ArcCount() ) ? ARC_HIGH_DEF * MM_PER_NM : 0.001;

        if( nearestBetween( result.p, result.n ) < closest - slack )
            continue;

        // Free ends are kept exactly where the router left them
        if( result.p.CPoint( 0 ) != aStart.anchor[0] || result.n.CPoint( 0 ) != aStart.anchor[1]
            || result.p.CLastPoint() != aEnd.anchor[0] || result.n.CLastPoint() != aEnd.anchor[1] )
        {
            return std::nullopt;
        }

        if( !aParams.linesClear || aParams.linesClear( result.p, result.n ) )
            return result;
    }

    return std::nullopt;
}

} // namespace PNS
