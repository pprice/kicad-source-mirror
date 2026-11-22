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
 * with nothing else where they meet.  A corner becomes the widest arc up to \a aMaxRadius that fits
 * along both its tracks (two corners share the straight between them) and comes no nearer any other
 * net than its rules allow.  An arc becomes the straight tracks round its outside that turn at 45
 * degrees, made wider where that comes too near something.  What is left between two arcs is nothing
 * or at least 0.1 mm.
 *
 * The board's rules engine must be ready, as for the router.  Changes are staged in \a aCommit,
 * which is not pushed.
 */
TRACK_CORNERS_RESULT ConvertTrackCorners( BOARD* aBoard, const std::vector<PCB_TRACK*>& aTracks,
                                          TRACK_CORNER_STYLE aStyle, int aMaxRadius,
                                          BOARD_COMMIT& aCommit );
