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

#ifndef PNS_DIFF_PAIR_SYMMETRY_H
#define PNS_DIFF_PAIR_SYMMETRY_H

#include <functional>
#include <optional>
#include <vector>

#include <math/vector2d.h>
#include <geometry/shape_line_chain.h>

class SHAPE;

namespace PNS
{

/**
 * One end of a stretch of a differential pair: the two pads or vias the lines leave or meet,
 * the P line's first, or two free track ends where the pair goes on.
 */
struct DP_SYMMETRY_END
{
    VECTOR2I anchor[2];

    /// The copper of each pad or via, or both null for free track ends.
    const SHAPE* copper[2] = { nullptr, nullptr };

    /// At free track ends, the way the pair runs from them into the stretch.
    VECTOR2D out;

    /**
     * At free track ends, the tracks' width, where they may be laid at another size than the
     * pair's (necked down out of a part's pins): the pair then joins them wherever their ends lie.
     * 0 where the ends are the pair's own, at its gap.
     */
    int width = 0;
};


/// A run of one line at one width.
struct DP_SYMMETRY_PART
{
    SHAPE_LINE_CHAIN line;
    int              width = 0;
};


struct DP_SYMMETRY_PARAMS
{
    int width = 0;
    int gap = 0;

    /// Radius of the arcs on the pair's centre line; 0 keeps every corner sharp.
    int radius = 0;

    /// How much longer the P line already is than the N line where the stretch starts.
    double skew = 0.0;

    /// Whether the corners of the bumps put on one line (0 for P, 1 for N) keep clear.
    std::function<bool( int, const SHAPE_LINE_CHAIN& )> bumpClear;

    /// Whether the two lines as drawn, each in runs of one width, keep clear of everything around them.
    std::function<bool( const std::vector<DP_SYMMETRY_PART> aParts[2] )> linesClear;
};


struct DP_SYMMETRY_RESULT
{
    SHAPE_LINE_CHAIN p;
    SHAPE_LINE_CHAIN n;

    /// Each line in runs of one width, from the start: one run where the pair keeps its size.
    std::vector<DP_SYMMETRY_PART> parts[2];

    /// How many deskewing bumps went in.
    int bumps = 0;
};


/**
 * Redraw a stretch of a pair the router has laid as two lines set off either side of one centre
 * line, so the gap holds round every corner and both lines open by the same amount each way where
 * they leave or meet pads further apart (or nearer) than the pair.
 *
 * The centre line keeps the router's corners where its lines run coupled.  At a pad end each line
 * leaves its pad square to the line through both pads, clears the pad's copper and a track's
 * width more, then turns in at 45 degrees to its place beside the centre.  With a radius the
 * centre line's corners are arcs, and the two lines' arcs share their centres.  A bend leaves the
 * outer line the longer; that goes back into the inner line in small bumps just after the bend,
 * each three track widths across the top, under twice the gap high, with 45 degree sides, placed
 * where it keeps clear (slid along the straight until it does), as Altera (683864, section
 * 1.4.1.1), Microchip (DS50003612B, section 3.2.1.2) and TI (SPRAAR7J, section 2.3) ask.  A bend
 * the next bend takes back within 2.54 mm is left to it.
 *
 * Free ends keep their points exactly, so a stretch joins the one before or after it.  Where they
 * are the ends of tracks laid at another size (a pair necked down out of a part's pins, laid by
 * hand), the lines open or close from them to the pair's place at 45 degrees, as tracks of the
 * narrower width, and take the pair's width there.  Each line comes back in runs of one width.
 *
 * @return the two lines, or nothing where no symmetric pair can be drawn there that keeps clear.
 */
std::optional<DP_SYMMETRY_RESULT> SymmetricPair( const SHAPE_LINE_CHAIN& aP, const SHAPE_LINE_CHAIN& aN,
                                                 const DP_SYMMETRY_END& aStart, const DP_SYMMETRY_END& aEnd,
                                                 const DP_SYMMETRY_PARAMS& aParams );

} // namespace PNS

#endif // PNS_DIFF_PAIR_SYMMETRY_H
