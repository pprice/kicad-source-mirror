/*
 * This program source code file is part of KiCad, a free EDA CAD application.
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
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>

#include <kiid.h>
#include <wx/string.h>

class BOARD;
class BOARD_COMMIT;
class BOARD_ITEM;
class PCB_TRACK;

enum class TRACK_CORNER_STYLE
{
    /// Each corner between two straight tracks becomes an arc
    ARCS,

    /// Each arc becomes straight tracks that turn at 45 degrees
    MITERED_45
};


struct TRACK_CORNERS_RESULT
{
    /// Corners made arcs, or arcs made corners
    int m_changed = 0;

    /// Each corner or arc left as it was, and why
    std::vector<wxString> m_left;

    std::vector<BOARD_ITEM*> m_added;
    std::vector<BOARD_ITEM*> m_modified;
    std::vector<KIID>        m_removed;
};


/**
 * Turns the corners of tracks into arcs, or arcs into 45 degree corners, keeping every clearance
 * KiCad's router knows.
 *
 * The tracks given are taken as runs: tracks and arcs joined end to end on one layer, of one width,
 * with nothing else where they meet.
 *
 * A corner becomes an arc sized by \a aFactor of the most it could be: each straight between two
 * corners is shared half and half, so at 1 the two corners of a 45 degree bend meet in the middle
 * of its diagonal as one arc, and at 0.5 half of each side is curved.  \a aMaxRadius, above 0,
 * makes it a fillet of that radius, and a 45 degree chamfer too short for one at each corner is
 * rounded as one corner.  Corners that turn side by side, as a bus does, have one radius, the
 * tightest's with a factor, so they are one curve repeated.  An arc that
 * would come nearer another net than its rules allow, and nearer than the corner was, is made
 * smaller until it does not.
 *
 * An arc becomes the straight tracks round its outside that turn at 45 degrees, made wider where
 * that comes too near something.  What is left between two arcs is nothing or at least 0.1 mm.
 *
 * The board's rules engine must be ready, as for the router.  Changes are staged in \a aCommit,
 * which is not pushed.
 */
TRACK_CORNERS_RESULT ConvertTrackCorners( BOARD* aBoard, const std::vector<PCB_TRACK*>& aTracks,
                                          TRACK_CORNER_STYLE aStyle, double aFactor,
                                          int aMaxRadius, BOARD_COMMIT& aCommit );
