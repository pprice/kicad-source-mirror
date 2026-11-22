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

    // Where the router ran its coupled lines straight at the pads, past where they turn out of
    // them (as it does when the pair is wider than the pins), the straight is moved over to meet
    // the end's own straight just beyond its lead
    auto turnBetween = []( const PT& u, const PT& v )
    {
        return std::acos( std::clamp( dot( u, v ), -1.0, 1.0 ) );
    };

    auto meetBeyond = [&]( WAY& aMid, const WAY& aEnd, const END& aEndOf, const PT& aOut )
    {
        if( !aMid.mid || std::abs( cross( aMid.dir, aEnd.dir ) ) < 1e-6 )
            return;

        std::optional<PT> corner = meet( aMid.at, aMid.dir, aEnd.at, aEnd.dir );

        if( !corner )
            return;

        const double need = aHalf * std::tan( turnBetween( aMid.dir, aEnd.dir ) / 2.0 ) + 0.005;

        if( dot( *corner - aEndOf.closed, aOut ) < need )
            aMid.at = aEndOf.closed + aOut * need;
    };

    meetBeyond( ways[1], ways[0], a, a.out );
    meetBeyond( ways[ways.size() - 2], ways.back(), b, b.out );

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


/// A line with no point said twice, and none that is no corner but those in aKeep.
std::vector<PT> plain( const std::vector<PT>& aPoints, const std::vector<PT>& aKeep = {} )
{
    std::vector<PT> out;
    out.reserve( aPoints.size() );

    auto kept = [&]( const PT& p )
    {
        return std::any_of( aKeep.begin(), aKeep.end(),
                            [&]( const PT& k )
                            {
                                return dist( k, p ) < HAIR;
                            } );
    };

    for( const PT& p : aPoints )
    {
        if( !out.empty() && dist( out.back(), p ) < HAIR )
            continue;

        if( out.size() >= 2 && !kept( out.back() ) )
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

    /// How far each line is set off from the centre, signed, where the pair keeps its own size.
    double by[2] = { 0.0, 0.0 };

    /// How far the lines are set off from the centre at each size they take.
    std::vector<double> halves;

    /// Each line's runs of one width: where each run after the first begins, and every run's width.
    std::vector<PT>     partFrom[2];
    std::vector<double> partWidth[2];

    /// Corners left sharp: where a line steps from one size to another.
    std::vector<PT> sharp;
};


/// The size of a pair over a run of it: its lines' width, and how far each is set off from the
/// centre line.
struct SIZE
{
    double width = 0.0;
    double half = 0.0;
};


/// A run of the centre line, measured along it from its start, where the pair has one size.
struct ZONE
{
    double from = 0.0;
    double to = 0.0;
    SIZE   size;
};


/// The pair's whole centre line: from the middle of one end's pads to the other's.
std::vector<PT> wholeOf( const END& a, const END& b, const std::vector<PT>& aRoute )
{
    std::vector<PT> whole = { a.mid };
    whole.insert( whole.end(), aRoute.begin(), aRoute.end() );
    whole.push_back( b.mid );
    whole.erase( std::unique( whole.begin(), whole.end(),
                              []( const PT& x, const PT& y )
                              {
                                  return dist( x, y ) < HAIR;
                              } ),
                 whole.end() );
    return whole;
}


/// How far along a line each of its points is.
std::vector<double> runningLengths( const std::vector<PT>& aLine )
{
    std::vector<double> out = { 0.0 };

    for( size_t k = 1; k < aLine.size(); k++ )
        out.push_back( out.back() + dist( aLine[k - 1], aLine[k] ) );

    return out;
}


/// The point aS along a line, and which of its straights it is on.
PT pointAlong( const std::vector<PT>& aLine, const std::vector<double>& aAlong, double aS, size_t* aStraight = nullptr )
{
    size_t k = 0;

    while( k + 2 < aLine.size() && aAlong[k + 1] < aS )
        k++;

    if( aStraight )
        *aStraight = k;

    const double length = aAlong[k + 1] - aAlong[k];
    const double t = length > HAIR ? std::clamp( ( aS - aAlong[k] ) / length, 0.0, 1.0 ) : 0.0;
    return aLine[k] + ( aLine[k + 1] - aLine[k] ) * t;
}


/// Which side of the centre line the P line keeps from end to end: +1 or -1, or 0 where its pads
/// are on different sides at the two ends.
double sideOf( const END& a, const END& b )
{
    const double side = signOf( dot( a.pads[0] - a.mid, beside( a.out ) ) );

    if( signOf( dot( b.pads[0] - b.mid, beside( b.out * -1.0 ) ) ) != side )
        return 0.0;

    return side;
}


/// The two lines set off from aWhole, a run of one size after another (aZones, in order, each
/// beginning where or after the last ended).  Between two runs a line steps across at 45 degrees
/// as a track of the narrower width.  aOwn is the pair's own size, which its bumps go on.
std::optional<DRAWN> draw( const END& a, const END& b, const std::vector<PT>& aWhole, double aSide,
                           const std::vector<ZONE>& aZones, const SIZE& aOwn )
{
    if( aWhole.size() < 2 || aSide == 0.0 || aZones.empty() )
        return std::nullopt;

    const std::vector<double> along = runningLengths( aWhole );

    DRAWN drawn;
    drawn.by[0] = aSide * aOwn.half;
    drawn.by[1] = -aSide * aOwn.half;

    for( const ZONE& zone : aZones )
    {
        if( std::none_of( drawn.halves.begin(), drawn.halves.end(),
                          [&]( double h )
                          {
                              return std::abs( h - zone.size.half ) < HAIR;
                          } ) )
        {
            drawn.halves.push_back( zone.size.half );
        }
    }

    for( int i = 0; i < 2; i++ )
    {
        const double sideOfLine = i == 0 ? aSide : -aSide;

        std::vector<PT> points = { a.pads[i], a.pads[i] + a.out * a.lead };

        for( size_t z = 0; z < aZones.size(); z++ )
        {
            const ZONE&     zone = aZones[z];
            const bool      first = z == 0;
            const bool      last = z + 1 == aZones.size();
            std::vector<PT> run;

            run.push_back( first ? aWhole.front() : pointAlong( aWhole, along, zone.from ) );

            for( size_t k = 1; k + 1 < aWhole.size(); k++ )
            {
                if( ( first || along[k] > zone.from + HAIR ) && ( last || along[k] < zone.to - HAIR ) )
                    run.push_back( aWhole[k] );
            }

            run.push_back( last ? aWhole.back() : pointAlong( aWhole, along, zone.to ) );

            std::optional<std::vector<PT>> setOff = offset( run, sideOfLine * zone.size.half );

            if( !setOff || setOff->size() < 2 )
                return std::nullopt;

            // Where one run steps to the next, the wider run's end is where the narrower width
            // takes over, on the step itself
            if( !first )
            {
                const ZONE& before = aZones[z - 1];
                const PT    from = setOff->front();
                const PT    to = points.back();

                drawn.sharp.push_back( from );
                drawn.sharp.push_back( to );
                drawn.partFrom[i].push_back( before.size.width > zone.size.width ? to : from );
            }

            drawn.partWidth[i].push_back( zone.size.width );
            points.insert( points.end(), setOff->begin() + ( first ? 1 : 0 ), setOff->end() - ( last ? 1 : 0 ) );
        }

        points.push_back( b.pads[i] + b.out * b.lead );
        points.push_back( b.pads[i] );
        drawn.lines[i] = plain( points, drawn.partFrom[i] );

        // A step between two runs of the same width is no change of width
        for( size_t k = drawn.partFrom[i].size(); k-- > 0; )
        {
            if( std::abs( drawn.partWidth[i][k] - drawn.partWidth[i][k + 1] ) < HAIR )
            {
                drawn.partFrom[i].erase( drawn.partFrom[i].begin() + k );
                drawn.partWidth[i].erase( drawn.partWidth[i].begin() + k + 1 );
            }
        }
    }

    drawn.centre = plain( aWhole );
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

        aRadii[i].clear();

        for( size_t j = 1; j + 1 < line.size(); j++ )
        {
            const BEND* found = nullptr;
            double      half = 0.0;

            for( const BEND& c : corners )
            {
                for( double h : aDrawn.halves )
                {
                    if( c.Is( line[j], h ) )
                    {
                        found = &c;
                        half = h;
                        break;
                    }
                }

                if( found )
                    break;
            }

            const bool sharp = std::any_of( aDrawn.sharp.begin(), aDrawn.sharp.end(),
                                            [&]( const PT& p )
                                            {
                                                return dist( p, line[j] ) < HAIR;
                                            } );

            if( sharp )
                aRadii[i].push_back( 0.0 );
            else if( !found )
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


/// A line in runs of one width: a new run begins at each of aFrom, a corner of the line.
std::vector<DP_SYMMETRY_PART> partsOf( const std::vector<PT>& aLine, const std::vector<double>& aRadii,
                                       const std::vector<PT>& aFrom, const std::vector<double>& aWidth )
{
    std::vector<DP_SYMMETRY_PART> out( 1 );
    out.back().width = KiROUND( aWidth.front() / MM_PER_NM );

    for( const PIECE& piece : roundedEach( aLine, aRadii ) )
    {
        const size_t next = out.size() - 1;

        if( next < aFrom.size() && dist( piece.from, aFrom[next] ) < 1e-6 && out.back().line.PointCount() > 1 )
        {
            out.emplace_back();
            out.back().width = KiROUND( aWidth[next + 1] / MM_PER_NM );
        }

        SHAPE_LINE_CHAIN& chain = out.back().line;

        if( chain.PointCount() == 0 )
            chain.Append( toNm( piece.from ) );

        if( piece.arc )
            chain.Append( SHAPE_ARC( toNm( piece.from ), toNm( piece.mid ), toNm( piece.to ), 0 ) );
        else
            chain.Append( toNm( piece.to ) );
    }

    return out;
}


/// How finely, and in what steps first, the place where a rule stops is looked for.
constexpr double STEP_FOUND_TO = 0.005;
constexpr double STEP_LOOKED = 0.1;


/**
 * Where along aWhole the pair steps from aZone, the size the rules give it at one end (the start,
 * or the end where aAtEnd), to aOwn, its own size: the first place from that end where a track of
 * the wider size, at the wider size's place beside the centre, is given another size by the rules
 * (aInZone tells, d along the centre from that end).  Measured from aWhole's start: where the
 * wider size's lines end or begin, then where the narrower's do.  The step keeps off the end's
 * lead and the centre's corners by aMargin: a narrow end runs on narrow further, a wide end
 * stops short.  A narrow end runs on aPush further still, where the pair would not keep clear
 * at its own width so near.  Nothing where a wide end has no room.
 */
std::optional<std::pair<double, double>> stepFrom( const std::vector<PT>& aWhole, bool aAtEnd, const SIZE& aZone,
                                                   const SIZE& aOwn, double aMargin, double aPush,
                                                   const std::function<bool( double )>& aInZone )
{
    const std::vector<double> along = runningLengths( aWhole );
    const double              total = along.back();
    const bool                narrow = aZone.width < aOwn.width
                                       || ( aZone.width == aOwn.width && aZone.half < aOwn.half );
    const double              across = std::abs( aZone.half - aOwn.half );

    if( aWhole.size() < 3 )
        return std::nullopt;

    // The lead, from the pads to where the lines have closed, is the first straight from the end
    const double lead = aAtEnd ? total - along[along.size() - 2] : along[1];
    const double last = total - ( aAtEnd ? along[1] : total - along[along.size() - 2] );

    double inside = lead;
    double outside = -1.0;

    for( double d = lead; d <= last; d += STEP_LOOKED )
    {
        if( !aInZone( d ) )
        {
            outside = d;
            break;
        }

        inside = d;
    }

    if( outside < 0.0 )
        return std::nullopt;

    while( outside - inside > STEP_FOUND_TO && outside > lead )
    {
        const double mid = ( inside + outside ) / 2.0;

        if( aInZone( mid ) )
            inside = mid;
        else
            outside = mid;
    }

    // From the end: where the wider lines are, and the narrower, either side of the step
    double wide = narrow ? outside + aPush : inside;
    double thin = narrow ? wide - across : wide + across;

    // Distances from the end of the corners of the centre line, the lead's own included
    std::vector<double> corners;

    for( size_t k = 1; k + 1 < aWhole.size(); k++ )
        corners.push_back( aAtEnd ? total - along[k] : along[k] );

    for( int tries = 0; tries < 64; tries++ )
    {
        const double lo = std::min( wide, thin ) - aMargin;
        const double hi = std::max( wide, thin ) + aMargin;
        double       shift = 0.0;

        for( double c : corners )
        {
            if( c > lo && c < hi )
                shift = narrow ? std::max( shift, c - lo ) : std::min( shift, c - hi );
        }

        if( lo < lead )
            shift = narrow ? std::max( shift, lead - lo ) : -1e9;

        if( shift == 0.0 )
        {
            if( hi > last )
                return std::nullopt;

            if( aAtEnd )
                return std::make_pair( total - wide, total - thin );

            return std::make_pair( wide, thin );
        }

        if( !narrow && std::min( wide, thin ) + shift - aMargin < lead )
            return std::nullopt;

        wide += shift;
        thin += shift;
    }

    return std::nullopt;
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

    auto sizeOf = []( const std::pair<int, int>& aRule )
    {
        return SIZE{ aRule.first * MM_PER_NM, ( aRule.first + aRule.second ) * MM_PER_NM / 2.0 };
    };

    const std::pair<int, int> ownRule( aParams.width, aParams.gap );
    const SIZE                own = sizeOf( ownRule );

    // The size the rules give each end.  Where one end is given the pair's own size and the other
    // another, the pair takes the other's size at that end, up to where the rule stops.
    std::optional<std::pair<int, int>> endRule[2];

    if( aParams.sizeAt )
    {
        auto ruleAt = [&]( const DP_SYMMETRY_END& aEnd )
        {
            const std::pair<int, int> p = aParams.sizeAt( 0, aEnd.anchor[0], aParams.width );
            const std::pair<int, int> n = aParams.sizeAt( 1, aEnd.anchor[1], aParams.width );
            return std::make_pair( std::min( p.first, n.first ), std::min( p.second, n.second ) );
        };

        const std::pair<int, int> atStart = ruleAt( aStart );
        const std::pair<int, int> atEnd = ruleAt( aEnd );

        if( atStart != atEnd && atStart == ownRule && atEnd.first > 0 && atEnd.second > 0 )
            endRule[1] = atEnd;
        else if( atStart != atEnd && atEnd == ownRule && atStart.first > 0 && atStart.second > 0 )
            endRule[0] = atStart;
    }

    const SIZE endSize[2] = { endRule[0] ? sizeOf( *endRule[0] ) : own, endRule[1] ? sizeOf( *endRule[1] ) : own };

    const std::vector<MIDLINE> mids = midlines( aP, aN, half );

    const PT startMid = ( toMm( aStart.anchor[0] ) + toMm( aStart.anchor[1] ) ) * 0.5;
    const PT endMid = ( toMm( aEnd.anchor[0] ) + toMm( aEnd.anchor[1] ) ) * 0.5;
    const PT towardStart = mids.empty() ? endMid : mids.front().b;
    const PT towardEnd = mids.empty() ? startMid : mids.back().a;

    // Where the pads are nearer than the pair, the lines meet them nearer too
    const double closest = std::min( { 2.0 * half, 2.0 * endSize[0].half, 2.0 * endSize[1].half,
                                       dist( toMm( aStart.anchor[0] ), toMm( aStart.anchor[1] ) ),
                                       dist( toMm( aEnd.anchor[0] ), toMm( aEnd.anchor[1] ) ) } );

    auto bumpClear = [&]( int aLine, const std::vector<PT>& aPoints )
    {
        return !aParams.bumpClear || aParams.bumpClear( aLine, chainOf( aPoints ) );
    };

    // A step in size keeps clear of the centre line's arcs, and of its sharp corners by a hair
    const double margin = round > 0.0 ? round : 0.02;

    // The lines run on further from their pads until a way keeps clear; a necked end runs on
    // narrow further, where the pair at its own width would not keep clear so near
    const bool necked = ( endRule[0] && endSize[0].width < own.width ) || ( endRule[1] && endSize[1].width < own.width );

    std::vector<std::pair<double, double>> attempts;

    for( double push : { 0.0, 0.5, 1.0, 2.0, 4.0 } )
    {
        for( double extra : { 0.0, 1.0, 2.0, 4.0 } )
        {
            if( push == 0.0 || necked )
                attempts.emplace_back( push, extra );
        }
    }

    for( const std::pair<double, double>& attempt : attempts )
    {
        const double push = attempt.first;
        const double extra = attempt.second;

        std::optional<END> a = makeEnd( aStart, towardStart, endSize[0].half, endSize[0].width,
                                        extra * endSize[0].width );
        std::optional<END> b = makeEnd( aEnd, towardEnd, endSize[1].half, endSize[1].width,
                                        extra * endSize[1].width );

        if( !a || !b )
            return std::nullopt;

        std::optional<std::vector<PT>> route = centreRoute( *a, *b, mids, half );

        if( !route )
            continue;

        const std::vector<PT> whole = wholeOf( *a, *b, *route );
        const double          side = sideOf( *a, *b );

        if( side == 0.0 )
            return std::nullopt;

        // Each end's size, up to where its rule stops, then the pair's own
        std::vector<ZONE> zones;
        std::optional<std::pair<double, double>> steps[2];

        for( int e = 0; e < 2; e++ )
        {
            if( !endRule[e] )
                continue;

            const SIZE& wider = endSize[e].width > own.width ? endSize[e] : own;

            auto inZone = [&]( double d )
            {
                const std::vector<double> along = runningLengths( whole );
                size_t                    k = 0;
                const PT c = pointAlong( whole, along, e == 1 ? along.back() - d : d, &k );
                const PT n = beside( unit( whole[k], whole[k + 1] ) );

                for( int i = 0; i < 2; i++ )
                {
                    const PT at = c + n * ( ( i == 0 ? side : -side ) * wider.half );

                    if( aParams.sizeAt( i, toNm( at ), KiROUND( wider.width / MM_PER_NM ) ) != *endRule[e] )
                        return false;
                }

                return true;
            };

            steps[e] = stepFrom( whole, e == 1, endSize[e], own, margin, push, inZone );
        }

        // An end that cannot step back to the pair's own size where its rule stops keeps it
        // all the way, as the router laid it
        if( ( endRule[0] && !steps[0] ) || ( endRule[1] && !steps[1] ) )
            continue;

        auto stepAt = [&]( int e, bool aFirstOf )
        {
            // Along from the start: the zone at the start ends at its own size's line, the
            // zone after begins at the other's
            const auto& [wideAt, thinAt] = *steps[e];
            const bool zoneNarrow = endSize[e].width < own.width
                                    || ( endSize[e].width == own.width && endSize[e].half < own.half );
            const double zoneLine = zoneNarrow ? thinAt : wideAt;
            const double ownLine = zoneNarrow ? wideAt : thinAt;
            return aFirstOf ? zoneLine : ownLine;
        };

        const double total = runningLengths( whole ).back();

        if( steps[0] )
            zones.push_back( { 0.0, stepAt( 0, true ), endSize[0] } );

        zones.push_back( { steps[0] ? stepAt( 0, false ) : 0.0, steps[1] ? stepAt( 1, false ) : total, own } );

        if( steps[1] )
            zones.push_back( { stepAt( 1, true ), total, endSize[1] } );

        bool ordered = true;

        for( size_t z = 0; z < zones.size(); z++ )
        {
            if( zones[z].to < zones[z].from - HAIR || ( z > 0 && zones[z].from < zones[z - 1].to - HAIR ) )
                ordered = false;
        }

        if( !ordered )
            continue;

        std::optional<DRAWN> drawn = draw( *a, *b, whole, side, zones, own );

        if( !drawn )
            continue;

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
            continue;

        bool split = true;

        for( int i = 0; i < 2; i++ )
        {
            result.parts[i] = partsOf( lines[i], radii[i], drawn->partFrom[i], drawn->partWidth[i] );

            if( result.parts[i].size() != drawn->partWidth[i].size() )
                split = false;
        }

        if( !split )
            continue;

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

        if( !aParams.linesClear || aParams.linesClear( result.parts ) )
            return result;
    }

    return std::nullopt;
}

} // namespace PNS
