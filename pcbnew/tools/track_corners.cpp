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

#include "track_corners.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <tuple>

#include <board.h>
#include <board_commit.h>
#include <footprint.h>
#include <pad.h>
#include <pcb_track.h>
#include <geometry/shape_arc.h>
#include <router/pns_arc.h>
#include <router/pns_kicad_iface.h>
#include <router/pns_node.h>
#include <router/pns_segment.h>


namespace
{

const double PI = 3.14159265358979323846;

/// What is left of a straight between two arcs: nothing, or at least this
const double MIN_STRAIGHT = pcbIUScale.mmToIU( 0.1 );

/// A straight shorter than this is none
const double NOTHING = pcbIUScale.mmToIU( 0.001 );

/// The narrowest arc worth laying; under it a corner stays sharp
const double MIN_RADIUS = pcbIUScale.mmToIU( 0.1 );


/// The router's board, to check new copper against, with nothing to draw on
class CHECK_IFACE : public PNS_KICAD_IFACE
{
public:
    CHECK_IFACE() { SetView( nullptr ); }

    void EraseView() override {}
    bool IsAnyLayerVisible( const PNS_LAYER_RANGE& aLayer ) const override { return true; }
    bool IsItemVisible( const PNS::ITEM* aItem ) const override { return true; }
    void HideItem( PNS::ITEM* aItem ) override {}

    void DisplayItem( const PNS::ITEM* aItem, int aClearance, bool aEdit = false,
                      int aFlags = 0 ) override {}

    void DisplayPathLine( const SHAPE_LINE_CHAIN& aLine, int aImportance ) override {}
    void DisplayRatline( const SHAPE_LINE_CHAIN& aRatline, PNS::NET_HANDLE aNet ) override {}
    void Commit() override {}
};


/// One piece of a run, its ends in the run's order
struct PIECE
{
    bool       m_arc = false;
    VECTOR2D   m_a;
    VECTOR2D   m_b;
    VECTOR2D   m_mid;               ///< an arc's
    PCB_TRACK* m_from = nullptr;    ///< the track it was, or comes from
    bool       m_changed = false;

    double Length() const { return ( m_b - m_a ).EuclideanNorm(); }
};


/// Tracks and arcs joined end to end, nothing else where they meet
struct RUN
{
    std::vector<PCB_TRACK*> m_items;
    std::vector<PIECE>      m_pieces;
    std::vector<PNS::ITEM*> m_inWorld;     ///< its copper in the router's world, as it now is
    bool                    m_changed = false;
};


VECTOR2D unit( const VECTOR2D& aV )
{
    double n = aV.EuclideanNorm();
    return n > 0 ? aV / n : VECTOR2D( 0, 0 );
}


double dot( const VECTOR2D& aA, const VECTOR2D& aB )
{
    return aA.x * aB.x + aA.y * aB.y;
}


double cross( const VECTOR2D& aA, const VECTOR2D& aB )
{
    return aA.x * aB.y - aA.y * aB.x;
}


VECTOR2D rotate( const VECTOR2D& aV, double aAngle )
{
    double c = std::cos( aAngle );
    double s = std::sin( aAngle );
    return VECTOR2D( aV.x * c - aV.y * s, aV.x * s + aV.y * c );
}


VECTOR2I toI( const VECTOR2D& aV )
{
    return VECTOR2I( KiROUND( aV.x ), KiROUND( aV.y ) );
}


wxString where( const VECTOR2D& aAt, PCB_TRACK* aTrack )
{
    return wxString::Format( wxS( "at %.3f, %.3f mm on %s" ), pcbIUScale.IUTomm( aAt.x ),
                             pcbIUScale.IUTomm( aAt.y ), aTrack->GetNetname() );
}


/// Says what new copper comes too near
wxString describe( const PNS::OBSTACLE& aObstacle )
{
    BOARD_ITEM* hit = aObstacle.m_item->Parent();

    if( !hit && aObstacle.m_item->ParentPadVia() )
        hit = aObstacle.m_item->ParentPadVia()->Parent();

    wxString what = wxString( wxS( "a " ) ) + wxString( aObstacle.m_item->KindStr() );

    if( hit && hit->Type() == PCB_VIA_T )
        what = wxS( "a via" );
    else if( hit && hit->Type() == PCB_TRACE_T )
        what = wxS( "a track" );
    else if( hit && hit->Type() == PCB_ARC_T )
        what = wxS( "an arc" );
    else if( hit && hit->Type() == PCB_PAD_T )
    {
        PAD* pad = static_cast<PAD*>( hit );
        what = wxString::Format( wxS( "%s pad %s" ), pad->GetParentFootprint()->GetReference(),
                                 pad->GetNumber() );
    }
    else if( hit )
    {
        what = hit->GetFriendlyName();
    }

    if( BOARD_CONNECTED_ITEM* connected = dynamic_cast<BOARD_CONNECTED_ITEM*>( hit ) )
        what << wxS( " on " ) << connected->GetNetname();

    return wxS( "too near " ) + what;
}


/// The tangent points and middle of the arc of radius aRadius in the corner at aV between the
/// directions aU (back along the first track) and aW (on along the second)
struct FILLET
{
    VECTOR2D m_t1, m_mid, m_t2;
    double   m_tangent = 0;
};


FILLET fillet( const VECTOR2D& aV, const VECTOR2D& aU, const VECTOR2D& aW, double aRadius )
{
    double   alpha = std::acos( std::clamp( dot( aU, aW ), -1.0, 1.0 ) );
    VECTOR2D bisector = unit( aU + aW );
    FILLET   f;

    f.m_tangent = aRadius / std::tan( alpha / 2 );
    f.m_t1 = aV + aU * f.m_tangent;
    f.m_t2 = aV + aW * f.m_tangent;

    VECTOR2D centre = aV + bisector * ( aRadius / std::sin( alpha / 2 ) );
    f.m_mid = centre - bisector * aRadius;
    return f;
}


/// The points round the outside of an arc, from its start S through centre C by aTheta, where
/// straight tracks turning at 45 degrees (or in even steps near it) meet
std::vector<VECTOR2D> miterPoints( const VECTOR2D& aC, const VECTOR2D& aS, double aTheta )
{
    int    k = std::max( 1, (int) std::lround( std::abs( aTheta ) / ( PI / 4 ) ) );
    double out = 1.0 / std::cos( aTheta / ( 2.0 * k ) );

    std::vector<VECTOR2D> points;

    for( int j = 0; j < k; ++j )
        points.push_back( aC + rotate( aS - aC, ( j + 0.5 ) * aTheta / k ) * out );

    return points;
}


class CORNERS
{
public:
    CORNERS( BOARD* aBoard, TRACK_CORNER_STYLE aStyle, double aFactor, int aMaxRadius ) :
            m_board( aBoard ),
            m_style( aStyle ),
            m_factor( aFactor > 0 ? std::min( aFactor, 1.0 ) : 1.0 ),
            m_maxRadius( aMaxRadius )
    {
        m_iface.SetBoard( aBoard );
        m_iface.SyncWorld( &m_world );

        for( PCB_TRACK* track : aBoard->Tracks() )
        {
            if( track->Type() == PCB_VIA_T )
                continue;

            m_ends[ key( track->GetStart() ) ].push_back( track );
            m_ends[ key( track->GetEnd() ) ].push_back( track );
        }
    }

    TRACK_CORNERS_RESULT Run( const std::vector<PCB_TRACK*>& aTracks, BOARD_COMMIT& aCommit )
    {
        std::set<PCB_TRACK*> chosen;

        for( PCB_TRACK* track : aTracks )
        {
            if( track->Type() == PCB_VIA_T )
                continue;

            if( track->IsLocked() )
            {
                m_result.m_left.push_back( wxS( "a locked track " )
                                           + where( track->GetStart(), track ) );
                continue;
            }

            chosen.insert( track );
        }

        std::set<PCB_TRACK*> done;
        std::vector<RUN>     runs;

        for( PCB_TRACK* track : aTracks )
        {
            if( !chosen.count( track ) || done.count( track ) )
                continue;

            RUN run = buildRun( track, chosen );
            done.insert( run.m_items.begin(), run.m_items.end() );

            for( PCB_TRACK* item : run.m_items )
            {
                if( PNS::ITEM* inWorld = m_world.FindItemByParent( item ) )
                    run.m_inWorld.push_back( inWorld );
            }

            runs.push_back( std::move( run ) );
        }

        if( m_style == TRACK_CORNER_STYLE::ARCS )
            sizeArcs( runs );

        // Round after round, each run's new copper put into the world as it is made: where two
        // tracks turn side by side the outer can only follow once the inner has turned
        std::vector<wxString> locked = m_result.m_left;

        for( int pass = 0; pass < 6; ++pass )
        {
            int before = m_result.m_changed;
            m_result.m_left = locked;

            for( RUN& run : runs )
            {
                int runBefore = m_result.m_changed;

                if( m_style == TRACK_CORNER_STYLE::ARCS )
                    this->round( run );
                else
                    miter( run );

                if( m_result.m_changed > runBefore )
                {
                    run.m_changed = true;
                    relay( run );
                }
            }

            if( m_result.m_changed == before )
                break;
        }

        for( RUN& run : runs )
        {
            if( run.m_changed )
                apply( run, aCommit );
        }

        return m_result;
    }

private:
    static std::pair<int, int> key( const VECTOR2I& aP ) { return { aP.x, aP.y }; }

    /// The one other chosen track that carries a run on through aP, if nothing else is there
    PCB_TRACK* onward( PCB_TRACK* aTrack, const VECTOR2I& aP, const std::set<PCB_TRACK*>& aChosen )
    {
        auto it = m_ends.find( key( aP ) );

        if( it == m_ends.end() )
            return nullptr;

        PCB_TRACK* next = nullptr;
        int        others = 0;

        for( PCB_TRACK* t : it->second )
        {
            if( t == aTrack || t->GetLayer() != aTrack->GetLayer()
                || t->GetNetCode() != aTrack->GetNetCode() )
            {
                continue;
            }

            ++others;
            next = t;
        }

        if( others != 1 || !aChosen.count( next ) || next->GetWidth() != aTrack->GetWidth() )
            return nullptr;

        for( PCB_TRACK* t : m_board->Tracks() )
        {
            if( t->Type() == PCB_VIA_T && t->GetNetCode() == aTrack->GetNetCode()
                && t->IsOnLayer( aTrack->GetLayer() ) && t->HitTest( aP ) )
            {
                return nullptr;
            }
        }

        for( FOOTPRINT* fp : m_board->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
            {
                if( pad->GetNetCode() == aTrack->GetNetCode() && pad->IsOnLayer( aTrack->GetLayer() )
                    && pad->HitTest( aP, 0, aTrack->GetLayer() ) )
                {
                    return nullptr;
                }
            }
        }

        return next;
    }

    static PIECE pieceOf( PCB_TRACK* aTrack, bool aReversed )
    {
        PIECE p;
        p.m_from = aTrack;
        p.m_a = aReversed ? aTrack->GetEnd() : aTrack->GetStart();
        p.m_b = aReversed ? aTrack->GetStart() : aTrack->GetEnd();

        if( aTrack->Type() == PCB_ARC_T )
        {
            p.m_arc = true;
            p.m_mid = static_cast<PCB_ARC*>( aTrack )->GetMid();
        }

        return p;
    }

    RUN buildRun( PCB_TRACK* aStart, const std::set<PCB_TRACK*>& aChosen )
    {
        // Back from the start's start to the run's first track, then forwards to its last
        PCB_TRACK* first = aStart;
        VECTOR2I   firstEnd = aStart->GetStart();
        std::set<PCB_TRACK*> seen = { aStart };

        while( PCB_TRACK* prev = onward( first, firstEnd, aChosen ) )
        {
            if( seen.count( prev ) )
                break;

            seen.insert( prev );
            firstEnd = prev->GetStart() == firstEnd ? prev->GetEnd() : prev->GetStart();
            first = prev;
        }

        RUN        run;
        PCB_TRACK* cur = first;
        VECTOR2I   from = firstEnd;
        seen.clear();

        while( cur && !seen.count( cur ) )
        {
            seen.insert( cur );
            bool reversed = cur->GetStart() != from;
            run.m_items.push_back( cur );
            run.m_pieces.push_back( pieceOf( cur, reversed ) );

            VECTOR2I to = reversed ? cur->GetStart() : cur->GetEnd();
            PCB_TRACK* next = onward( cur, to, aChosen );
            from = to;
            cur = next;
        }

        return run;
    }

    /// Puts a run's copper into the router's world as its pieces now are
    void relay( RUN& aRun )
    {
        PCB_TRACK* like = aRun.m_items.front();
        int        layer = m_iface.GetPNSLayerFromBoardLayer( like->GetLayer() );

        for( PNS::ITEM* item : aRun.m_inWorld )
            m_world.Remove( item );

        aRun.m_inWorld.clear();

        for( const PIECE& piece : aRun.m_pieces )
        {
            if( piece.m_arc )
            {
                SHAPE_ARC shape( toI( piece.m_a ), toI( piece.m_mid ), toI( piece.m_b ),
                                 like->GetWidth() );
                auto      arc = std::make_unique<PNS::ARC>( shape, like->GetNet() );
                arc->SetLayer( layer );
                PNS::ITEM* added = arc.get();

                if( m_world.Add( std::move( arc ), true ) )
                    aRun.m_inWorld.push_back( added );
            }
            else
            {
                auto seg = std::make_unique<PNS::SEGMENT>( SEG( toI( piece.m_a ), toI( piece.m_b ) ),
                                                           like->GetNet() );
                seg->SetWidth( like->GetWidth() );
                seg->SetLayer( layer );
                PNS::ITEM* added = seg.get();

                if( m_world.Add( std::move( seg ), true ) )
                    aRun.m_inWorld.push_back( added );
            }
        }
    }

    /// Whether a point lies on a piece's copper
    static bool onCopper( const PIECE& aPiece, const VECTOR2I& aP, int aWidth )
    {
        if( aPiece.m_arc )
        {
            SHAPE_ARC shape( toI( aPiece.m_a ), toI( aPiece.m_mid ), toI( aPiece.m_b ), aWidth );
            return shape.Collide( aP, 0 );
        }

        return SEG( toI( aPiece.m_a ), toI( aPiece.m_b ) ).Distance( aP ) <= aWidth / 2;
    }

    /**
     * Where other copper of the net joins some of these pieces: as KiCad joins them, a track's end
     * or a via's centre that lies on their copper.
     */
    std::vector<VECTOR2I> joinsOn( const std::vector<const PIECE*>& aPieces, const RUN& aRun,
                                   PCB_TRACK* aLike )
    {
        std::vector<VECTOR2I> joins;

        auto consider =
                [&]( const VECTOR2I& aP )
                {
                    for( const PIECE* piece : aPieces )
                    {
                        if( onCopper( *piece, aP, aLike->GetWidth() ) )
                        {
                            joins.push_back( aP );
                            return;
                        }
                    }
                };

        for( PCB_TRACK* t : m_board->Tracks() )
        {
            if( t->GetNetCode() != aLike->GetNetCode() || !t->IsOnLayer( aLike->GetLayer() )
                || std::find( aRun.m_items.begin(), aRun.m_items.end(), t ) != aRun.m_items.end() )
            {
                continue;
            }

            if( t->Type() == PCB_VIA_T )
            {
                consider( t->GetPosition() );
            }
            else
            {
                consider( t->GetStart() );
                consider( t->GetEnd() );
            }
        }

        return joins;
    }

    /// Whether every join still lies on some of the new copper
    static bool keeps( const std::vector<VECTOR2I>& aJoins, const std::vector<PIECE>& aCopper,
                       int aWidth )
    {
        for( const VECTOR2I& join : aJoins )
        {
            bool held = false;

            for( const PIECE& piece : aCopper )
            {
                if( onCopper( piece, join, aWidth ) )
                {
                    held = true;
                    break;
                }
            }

            if( !held )
                return false;
        }

        return true;
    }

    /// A piece as the router's item, on the board layer of aLike
    std::unique_ptr<PNS::ITEM> routerItem( const PIECE& aPiece, PCB_TRACK* aLike )
    {
        int layer = m_iface.GetPNSLayerFromBoardLayer( aLike->GetLayer() );

        if( aPiece.m_arc )
        {
            SHAPE_ARC shape( toI( aPiece.m_a ), toI( aPiece.m_mid ), toI( aPiece.m_b ),
                             aLike->GetWidth() );
            auto      arc = std::make_unique<PNS::ARC>( shape, aLike->GetNet() );
            arc->SetLayer( layer );
            return arc;
        }

        auto seg = std::make_unique<PNS::SEGMENT>( SEG( toI( aPiece.m_a ), toI( aPiece.m_b ) ),
                                                   aLike->GetNet() );
        seg->SetWidth( aLike->GetWidth() );
        seg->SetLayer( layer );
        return seg;
    }

    /**
     * The gap between two shapes, edge to edge, to the nanometre: the least clearance at which
     * they touch.  Collide's own figure is not the least for an arc, which reports the last of
     * the points it tries.
     */
    static int gap( const SHAPE* aA, const SHAPE* aB )
    {
        int lo = 0;
        int hi = pcbIUScale.mmToIU( 2 );

        if( !aA->Collide( aB, hi ) )
            return std::numeric_limits<int>::max();

        if( aA->Collide( aB, 0 ) )
            return 0;

        while( hi - lo > 1 )
        {
            int mid = lo + ( hi - lo ) / 2;

            if( aA->Collide( aB, mid ) )
                hi = mid;
            else
                lo = mid;
        }

        return hi;
    }

    /**
     * What new copper comes too near: other nets' copper within its clearance, as KiCad's check
     * counts it, unless the copper it replaces (aOld) was no further from it.  A corner laid at
     * its clearance from a neighbour keeps it when rounded or mitred, however tight.
     */
    std::optional<wxString> clash( const PIECE& aPiece, PCB_TRACK* aLike,
                                   const std::vector<PIECE>& aOld = {} )
    {
        int                        layer = m_iface.GetPNSLayerFromBoardLayer( aLike->GetLayer() );
        std::unique_ptr<PNS::ITEM> item = routerItem( aPiece, aLike );

        // The router's own checks allow a hair under the clearance; KiCad's check does not
        PNS::COLLISION_SEARCH_OPTIONS opts;
        opts.m_useClearanceEpsilon = false;

        PNS::NODE::OBSTACLES obstacles;
        m_world.QueryColliding( item.get(), obstacles, opts );

        std::vector<std::unique_ptr<PNS::ITEM>> old;

        for( const PIECE& piece : aOld )
            old.push_back( routerItem( piece, aLike ) );

        for( const PNS::OBSTACLE& obstacle : obstacles )
        {
            const SHAPE* theirs = obstacle.m_item->Shape( layer );

            if( !theirs || old.empty() )
                return describe( obstacle );

            int now = gap( item->Shape( layer ), theirs );
            int before = std::numeric_limits<int>::max();

            for( const std::unique_ptr<PNS::ITEM>& o : old )
                before = std::min( before, gap( o->Shape( layer ), theirs ) );

            if( now + 1 < before )
                return describe( obstacle );
        }

        return std::nullopt;
    }

    /**
     * The corner between straights i and i + 1, if there is one: aU back along the first, aW on
     * along the second, aAlpha the angle inside it.  Straight on or doubling back is none.
     */
    static bool cornerAt( const std::vector<PIECE>& aP, size_t aI, VECTOR2D& aU, VECTOR2D& aW,
                          double& aAlpha )
    {
        if( aI + 1 >= aP.size() || aP[aI].m_arc || aP[aI + 1].m_arc || aP[aI].Length() == 0
            || aP[aI + 1].Length() == 0 )
        {
            return false;
        }

        aU = unit( aP[aI].m_a - aP[aI].m_b );
        aW = unit( aP[aI + 1].m_b - aP[aI + 1].m_a );
        aAlpha = std::acos( std::clamp( dot( aU, aW ), -1.0, 1.0 ) );
        return aAlpha < PI - 0.01 && aAlpha > 0.01;
    }

    static std::tuple<int, int, int> cornerKey( int aLayer, const VECTOR2D& aV )
    {
        VECTOR2I v = toI( aV );
        return { aLayer, v.x, v.y };
    }

    /**
     * A bend: corners a to b of a run (corner i lies between pieces i and i + 1), rounded as one
     * arc in the corner where its first and last straights would meet.  One corner is a bend of
     * its own; the two corners of a short 45 degree chamfer are one bend, as a fillet rounds it.
     */
    struct BEND
    {
        size_t   m_a = 0;
        size_t   m_b = 0;
        VECTOR2D m_v;               ///< where the first and last straights meet
        VECTOR2D m_u;               ///< back along the first
        VECTOR2D m_w;               ///< on along the last
        double   m_alpha = 0;       ///< the angle inside
        double   m_leadIn = 0;      ///< from m_v back to the first corner
        double   m_leadOut = 0;     ///< from m_v on to the last corner
    };

    /// The angle the run turns through at corner i, signed
    static double turnAt( const std::vector<PIECE>& aP, size_t aI )
    {
        VECTOR2D in = aP[aI].m_b - aP[aI].m_a;
        VECTOR2D out = aP[aI + 1].m_b - aP[aI + 1].m_a;
        return std::atan2( cross( in, out ), dot( in, out ) );
    }

    static BEND makeBend( const std::vector<PIECE>& aP, size_t aA, size_t aB )
    {
        BEND bend;
        bend.m_a = aA;
        bend.m_b = aB;
        bend.m_u = unit( aP[aA].m_a - aP[aA].m_b );
        bend.m_w = unit( aP[aB + 1].m_b - aP[aB + 1].m_a );
        bend.m_alpha = std::acos( std::clamp( dot( bend.m_u, bend.m_w ), -1.0, 1.0 ) );
        bend.m_v = aP[aA].m_b;

        if( aA != aB )
        {
            // The first straight carried on until it meets the last one carried back
            VECTOR2D from = aP[aA].m_b;
            VECTOR2D ahead = -bend.m_u;
            double   s = cross( aP[aB + 1].m_a - from, bend.m_w ) / cross( ahead, bend.m_w );

            bend.m_v = from + ahead * s;
            bend.m_leadIn = ( bend.m_v - aP[aA].m_b ).EuclideanNorm();
            bend.m_leadOut = ( bend.m_v - aP[aB + 1].m_a ).EuclideanNorm();
        }

        return bend;
    }

    /// Whether corners i and i + 1 turn the same way, together less than a half turn
    static bool chamferAt( const std::vector<PIECE>& aP, size_t aI )
    {
        VECTOR2D u, w;
        double   a;

        if( !cornerAt( aP, aI, u, w, a ) || !cornerAt( aP, aI + 1, u, w, a ) )
            return false;

        double t1 = turnAt( aP, aI );
        double t2 = turnAt( aP, aI + 1 );
        return t1 * t2 > 0 && std::abs( t1 ) + std::abs( t2 ) < PI - 0.05;
    }

    /// A run's bends: each corner, and each chamfer whose corners were found to be one bend
    std::vector<BEND> bendsOf( const std::vector<PIECE>& aP, int aLayer ) const
    {
        std::vector<BEND> bends;
        VECTOR2D          u, w;
        double            a;

        for( size_t i = 0; i + 1 < aP.size(); ++i )
        {
            if( !cornerAt( aP, i, u, w, a ) )
                continue;

            if( m_merged.count( cornerKey( aLayer, aP[i].m_b ) ) && chamferAt( aP, i ) )
            {
                bends.push_back( makeBend( aP, i, i + 1 ) );
                ++i;
            }
            else
            {
                bends.push_back( makeBend( aP, i, i ) );
            }
        }

        return bends;
    }

    /// A bend's place among the others, to find which turn side by side
    struct AT
    {
        VECTOR2D v;
        VECTOR2D b;               ///< the bisector, towards the centre
        double   half = 0;        ///< half the angle inside
        double   most = 0;        ///< the widest radius its straights allow
        double   least = 0;       ///< the narrowest that keeps its arc on its straights
        double   chamfer = 0;     ///< the straight between a chamfer's corners
        double   fillets = 0;     ///< the most that two fillets of the radius take of it
        int      layer = 0;
        VECTOR2D key;             ///< its first corner
        long     du = 0;
        long     dw = 0;
    };

    /**
     * Bends that turn the same way with their points on one line along their bisector, close
     * together: tracks turning side by side.  Each group nearest the centre first.
     */
    static std::vector<std::vector<size_t>> sideBySide( const std::vector<AT>& aAll )
    {
        const double ONE_LINE = pcbIUScale.mmToIU( 0.02 );
        const double SIDE_BY_SIDE = pcbIUScale.mmToIU( 1.5 );

        std::map<std::tuple<int, long, long>, std::vector<size_t>> byTurn;
        std::vector<std::vector<size_t>>                           groups;

        for( size_t k = 0; k < aAll.size(); ++k )
            byTurn[{ aAll[k].layer, aAll[k].du, aAll[k].dw }].push_back( k );

        for( auto& [turn, ks] : byTurn )
        {
            const VECTOR2D b = aAll[ks.front()].b;

            auto across = [&]( size_t k ) { return cross( b, aAll[k].v ); };
            auto along = [&]( size_t k ) { return dot( b, aAll[k].v ); };

            std::sort( ks.begin(), ks.end(),
                       [&]( size_t x, size_t y ) { return across( x ) < across( y ); } );

            std::vector<std::vector<size_t>> lines;

            for( size_t k = 0; k < ks.size(); ++k )
            {
                if( k == 0 || across( ks[k] ) - across( ks[k - 1] ) > ONE_LINE )
                    lines.emplace_back();

                lines.back().push_back( ks[k] );
            }

            for( std::vector<size_t>& line : lines )
            {
                std::sort( line.begin(), line.end(),
                           [&]( size_t x, size_t y ) { return along( x ) > along( y ); } );

                size_t first = 0;

                while( first < line.size() )
                {
                    size_t last = first + 1;

                    while( last < line.size()
                           && along( line[last - 1] ) - along( line[last] ) <= SIDE_BY_SIDE )
                    {
                        ++last;
                    }

                    groups.emplace_back( line.begin() + first, line.begin() + last );
                    first = last;
                }
            }
        }

        return groups;
    }

    static AT placeOf( const BEND& aBend, const std::vector<PIECE>& aP, int aLayer )
    {
        AT at;
        at.v = aBend.m_v;
        at.b = unit( aBend.m_u + aBend.m_w );
        at.half = aBend.m_alpha / 2;
        at.layer = aLayer;
        at.key = aP[aBend.m_a].m_b;
        at.du = std::lround( std::atan2( aBend.m_u.y, aBend.m_u.x ) * 1800 / PI );
        at.dw = std::lround( std::atan2( aBend.m_w.y, aBend.m_w.x ) * 1800 / PI );
        return at;
    }

    /**
     * The radius each bend of the runs is to have, before any is made.
     *
     * With a fillet's radius, a 45 degree chamfer too short for a fillet at each of its corners is
     * one bend, rounded where its straights would meet.
     *
     * The most a bend can have takes half of each straight it shares with another bend, and all but
     * a short lead of one that ends the run.  Each bend has the fillet's radius, or the factor's
     * share of its most; bends side by side have one radius, so they are one curve repeated (the
     * gap between them widens a little round the bend, and never narrows).  A bend with less room
     * than that has what it has.
     */
    void sizeArcs( const std::vector<RUN>& aRuns )
    {
        if( m_maxRadius > 0 )
        {
            std::vector<AT> chamfers;

            for( const RUN& run : aRuns )
            {
                const std::vector<PIECE>& p = run.m_pieces;
                int                       layer = run.m_items.front()->GetLayer();

                for( size_t i = 0; i + 2 < p.size(); ++i )
                {
                    if( !chamferAt( p, i ) )
                        continue;

                    AT at = placeOf( makeBend( p, i, i + 1 ), p, layer );
                    at.chamfer = p[i + 1].Length();
                    at.fillets = m_maxRadius * ( std::tan( std::abs( turnAt( p, i ) ) / 2 )
                                                 + std::tan( std::abs( turnAt( p, i + 1 ) ) / 2 ) );
                    chamfers.push_back( at );
                    ++i;
                }
            }

            for( const AT& at : chamfers )
            {
                if( at.chamfer < at.fillets )
                    m_merged.insert( cornerKey( at.layer, at.key ) );
            }
        }

        std::vector<AT> all;

        for( const RUN& run : aRuns )
        {
            const std::vector<PIECE>& p = run.m_pieces;
            size_t                    n = p.size();
            int                       layer = run.m_items.front()->GetLayer();
            std::vector<BEND>         bends = bendsOf( p, layer );

            for( size_t k = 0; k < bends.size(); ++k )
            {
                const BEND& bend = bends[k];
                size_t      first = bend.m_a;
                size_t      last = bend.m_b + 1;

                double in = p[first].Length();

                if( k > 0 && bends[k - 1].m_b + 1 == first )
                    in /= 2;
                else if( first == 0 )
                    in = std::max( 0.0, in - MIN_STRAIGHT );

                double out = p[last].Length();

                if( k + 1 < bends.size() && bends[k + 1].m_a == last )
                    out /= 2;
                else if( last + 1 == n )
                    out = std::max( 0.0, out - MIN_STRAIGHT );

                AT at = placeOf( bend, p, layer );
                at.most = std::min( bend.m_leadIn + in, bend.m_leadOut + out )
                          * std::tan( bend.m_alpha / 2 );
                at.least = std::max( bend.m_leadIn, bend.m_leadOut ) * std::tan( bend.m_alpha / 2 );
                all.push_back( at );
            }
        }

        for( const std::vector<size_t>& group : sideBySide( all ) )
        {
            // One radius for them all: the fillet's, else the most the tightest allows; a track
            // with less room than that has what it has
            double radius = std::numeric_limits<double>::max();

            if( m_maxRadius > 0 )
            {
                radius = m_maxRadius;
            }
            else
            {
                for( size_t k : group )
                    radius = std::min( radius, m_factor * all[k].most );
            }

            for( size_t k : group )
            {
                const AT& at = all[k];
                double    r = std::min( radius, at.most );

                if( at.least <= at.most )
                    r = std::max( r, at.least );

                m_radius[cornerKey( at.layer, at.key )] = r;
            }
        }
    }

    /// Each corner between two straights becomes an arc
    void round( RUN& aRun )
    {
        std::vector<PIECE>& p = aRun.m_pieces;
        PCB_TRACK*          like = aRun.m_items.front();
        size_t              n = p.size();

        int               layer = like->GetLayer();
        std::vector<BEND> bends = bendsOf( p, layer );
        size_t            m = bends.size();

        // Each bend's tangent length from where its straights meet, as wanted
        std::vector<double> t( m, 0 );

        for( size_t k = 0; k < m; ++k )
        {
            const BEND& bend = bends[k];
            auto        it = m_radius.find( cornerKey( layer, p[bend.m_a].m_b ) );
            double      radius = it != m_radius.end()
                                    ? it->second
                                    : m_factor
                                         * std::min( p[bend.m_a].Length(), p[bend.m_b + 1].Length() )
                                         / 2 * std::tan( bend.m_alpha / 2 );

            t[k] = radius / std::tan( bend.m_alpha / 2 );
        }

        // The bend that ends on each straight, and the one that starts on it
        std::vector<int> ending( n, -1 ), starting( n, -1 ), within( n, -1 );

        for( size_t k = 0; k < m; ++k )
        {
            starting[bends[k].m_a] = (int) k;
            ending[bends[k].m_b + 1] = (int) k;

            for( size_t j = bends[k].m_a + 1; j <= bends[k].m_b; ++j )
                within[j] = (int) k;
        }

        // Each straight shared between the bends at its ends, by what they want of it
        for( int pass = 0; pass < 4; ++pass )
        {
            for( size_t j = 0; j < n; ++j )
            {
                int    e = ending[j];
                int    s = starting[j];
                double before = e >= 0 ? std::max( 0.0, t[e] - bends[e].m_leadOut ) : 0;
                double after = s >= 0 ? std::max( 0.0, t[s] - bends[s].m_leadIn ) : 0;
                double length = p[j].Length();
                double total = before + after;

                if( total <= 0 )
                    continue;

                double scale = 1.0;

                if( total > length )
                    scale = length / total;

                // Nothing left between, or at least the least straight: a hair left is taken up
                // by the arcs, more is made the least straight
                double left = length - total * scale;

                if( left > NOTHING && left < MIN_STRAIGHT )
                {
                    if( left < MIN_STRAIGHT / 2 )
                        scale = length / total;
                    else
                        scale = length > MIN_STRAIGHT ? ( length - MIN_STRAIGHT ) / total
                                                      : length / total;
                }

                if( scale != 1.0 )
                {
                    if( e >= 0 )
                        t[e] = bends[e].m_leadOut + before * scale;

                    if( s >= 0 )
                        t[s] = bends[s].m_leadIn + after * scale;
                }
            }
        }

        // The widest arc that keeps its clearances and everything joined to its tracks, sharp
        // where there is no room
        std::vector<FILLET> fillets( m );
        std::vector<bool>   made( m, false );

        for( size_t k = 0; k < m; ++k )
        {
            const BEND& bend = bends[k];
            double      half = std::tan( bend.m_alpha / 2 );
            double      radius = t[k] * half;
            double      least = std::max( bend.m_leadIn, bend.m_leadOut ) * half;
            VECTOR2D    at = p[bend.m_a].m_b;

            // Each outer straight's far part is the other bend's to check
            std::vector<const PIECE*> around;

            for( size_t j = bend.m_a; j <= bend.m_b + 1; ++j )
                around.push_back( &p[j] );

            std::vector<VECTOR2I> joins = joinsOn( around, aRun, like );

            auto fits =
                    [&]( double aRadius, wxString* aWhy )
                    {
                        FILLET f = fillet( bend.m_v, bend.m_u, bend.m_w, aRadius );
                        PIECE  before, arc, after;
                        before.m_a = p[bend.m_a].m_a;
                        before.m_b = f.m_t1;
                        arc.m_arc = true;
                        arc.m_a = f.m_t1;
                        arc.m_mid = f.m_mid;
                        arc.m_b = f.m_t2;
                        after.m_a = f.m_t2;
                        after.m_b = p[bend.m_b + 1].m_b;

                        if( !keeps( joins, { before, arc, after }, like->GetWidth() ) )
                        {
                            if( aWhy )
                                *aWhy = wxS( "another track or a via joins its tracks there" );

                            return false;
                        }

                        // The corner, or the chamfer, it takes the place of
                        std::vector<PIECE> old;
                        PIECE              in, out;
                        in.m_a = f.m_t1;
                        in.m_b = p[bend.m_a].m_b;
                        old.push_back( in );

                        for( size_t j = bend.m_a + 1; j <= bend.m_b; ++j )
                            old.push_back( p[j] );

                        out.m_a = p[bend.m_b + 1].m_a;
                        out.m_b = f.m_t2;
                        old.push_back( out );

                        std::optional<wxString> why = clash( arc, like, old );

                        if( why && aWhy )
                            *aWhy = wxS( "an arc would come " ) + *why;

                        return !why;
                    };

            wxString why;

            if( radius < std::max( MIN_RADIUS, least ) )
            {
                m_result.m_left.push_back( wxS( "the corner " ) + where( at, like )
                                           + wxS( ": its tracks are too short for an arc" ) );
                continue;
            }

            if( !fits( radius, &why ) )
            {
                double lo = least, hi = radius;

                for( int step = 0; step < 14; ++step )
                {
                    double mid = ( lo + hi ) / 2;

                    if( fits( mid, nullptr ) )
                        lo = mid;
                    else
                        hi = mid;
                }

                if( lo < MIN_RADIUS || !fits( lo, nullptr ) )
                {
                    m_result.m_left.push_back( wxS( "the corner " ) + where( at, like ) + wxS( ": " )
                                               + why );
                    continue;
                }

                radius = lo;
            }

            fillets[k] = fillet( bend.m_v, bend.m_u, bend.m_w, radius );
            made[k] = true;
        }

        // Laid out as pieces: each straight cut back to its arcs' tangent points, a chamfer made
        // one arc gone
        std::vector<PIECE> out;

        for( size_t j = 0; j < n; ++j )
        {
            if( within[j] >= 0 && made[within[j]] )
                continue;

            PIECE piece = p[j];
            int   e = ending[j];
            int   s = starting[j];

            if( e >= 0 && made[e] )
            {
                piece.m_a = fillets[e].m_t2;
                piece.m_changed = true;
            }

            if( s >= 0 && made[s] )
            {
                piece.m_b = fillets[s].m_t1;
                piece.m_changed = true;
            }

            if( piece.Length() > NOTHING || piece.m_arc )
                out.push_back( piece );

            if( s >= 0 && made[s] )
            {
                PIECE arc;
                arc.m_arc = true;
                arc.m_a = fillets[s].m_t1;
                arc.m_mid = fillets[s].m_mid;
                arc.m_b = fillets[s].m_t2;
                arc.m_changed = true;
                out.push_back( arc );
                ++m_result.m_changed;
            }
        }

        joinUp( out );
        mergeArcs( out );
        p = out;
    }

    static VECTOR2D centreOf( const PIECE& aArc )
    {
        SHAPE_ARC shape( toI( aArc.m_a ), toI( aArc.m_mid ), toI( aArc.m_b ), 0 );
        return shape.GetCenter();
    }

    /// The angle an arc turns through, signed
    static double sweepOf( const PIECE& aArc, const VECTOR2D& aCentre )
    {
        VECTOR2D s = aArc.m_a - aCentre;
        VECTOR2D m = aArc.m_mid - aCentre;
        return 2 * std::atan2( cross( s, m ), dot( s, m ) );
    }

    /// Two new arcs that meet with one centre and radius, the two halves of a bend, become one
    static void mergeArcs( std::vector<PIECE>& aPieces )
    {
        const double       SAME = pcbIUScale.mmToIU( 0.002 );
        std::vector<PIECE> merged;

        for( const PIECE& piece : aPieces )
        {
            if( !merged.empty() && piece.m_arc && merged.back().m_arc
                && ( piece.m_changed || merged.back().m_changed ) )
            {
                PIECE&   last = merged.back();
                VECTOR2D c1 = centreOf( last );
                VECTOR2D c2 = centreOf( piece );
                double   r1 = ( last.m_a - c1 ).EuclideanNorm();
                double   r2 = ( piece.m_a - c2 ).EuclideanNorm();
                double   sweep = sweepOf( last, c1 ) + sweepOf( piece, c2 );

                if( ( c1 - c2 ).EuclideanNorm() < SAME && std::abs( r1 - r2 ) < SAME
                    && std::abs( sweep ) < PI - 0.01 )
                {
                    last.m_mid = c1 + rotate( last.m_a - c1, sweep / 2 );
                    last.m_b = piece.m_b;
                    last.m_changed = true;
                    continue;
                }
            }

            merged.push_back( piece );
        }

        aPieces = merged;
    }

    /// Each piece starts exactly where the one before ends: a straight of next to nothing taken out
    /// between two arcs leaves their ends a hair apart
    static void joinUp( std::vector<PIECE>& aPieces )
    {
        for( size_t k = 1; k < aPieces.size(); ++k )
        {
            if( aPieces[k].m_a != aPieces[k - 1].m_b )
            {
                aPieces[k].m_a = aPieces[k - 1].m_b;
                aPieces[k].m_changed = true;
            }
        }
    }

    /// Each arc becomes straights round its outside, turning at 45 degrees
    void miter( RUN& aRun )
    {
        std::vector<PIECE>& p = aRun.m_pieces;
        PCB_TRACK*          like = aRun.m_items.front();
        std::vector<PIECE>  out;

        for( size_t i = 0; i < p.size(); ++i )
        {
            if( !p[i].m_arc )
            {
                out.push_back( p[i] );
                continue;
            }

            SHAPE_ARC shape( toI( p[i].m_a ), toI( p[i].m_mid ), toI( p[i].m_b ), 0 );
            VECTOR2D  c = shape.GetCenter();
            VECTOR2D  s = p[i].m_a;
            VECTOR2D  e = p[i].m_b;
            double    half = std::atan2( cross( s - c, p[i].m_mid - c ), dot( s - c, p[i].m_mid - c ) );
            double    theta = 2 * half;

            bool straightBefore = !out.empty() && !out.back().m_arc;
            bool straightAfter = i + 1 < p.size() && !p[i + 1].m_arc;

            // What joins the arc, or the straights either side that a wider corner cuts back
            std::vector<const PIECE*> around = { &p[i] };

            if( straightBefore )
                around.push_back( &out.back() );

            if( straightAfter )
                around.push_back( &p[i + 1] );

            std::vector<VECTOR2I> joins = joinsOn( around, aRun, like );

            // The arc as it is, then wider or narrower in the same corner where that comes too
            // near something; a corner of the two straights is needed to move it
            std::vector<double> scales = { 1.0 };

            if( std::abs( theta ) < PI - 0.1 && straightBefore && straightAfter )
                scales.insert( scales.end(), { 1.3, 1.7, 2.2, 0.7, 0.5 } );

            double   r = ( s - c ).EuclideanNorm();
            VECTOR2D corner = c + rotate( s - c, half ) / std::cos( half );
            double   alpha = PI - std::abs( theta );
            VECTOR2D toCentre = unit( c - corner );
            VECTOR2D back = unit( s - corner );
            VECTOR2D on = unit( e - corner );

            std::optional<std::vector<PIECE>> chosen;
            VECTOR2D chosenStart, chosenEnd;
            wxString why;

            for( double scale : scales )
            {
                VECTOR2D c2 = c, s2 = s, e2 = e;

                if( scale != 1.0 )
                {
                    double r2 = r * scale;
                    double tangent = r2 / std::tan( alpha / 2 );

                    if( tangent > ( out.back().m_a - corner ).EuclideanNorm() - MIN_STRAIGHT
                        || tangent > ( p[i + 1].m_b - corner ).EuclideanNorm() - MIN_STRAIGHT )
                    {
                        continue;
                    }

                    c2 = corner + toCentre * ( r2 / std::sin( alpha / 2 ) );
                    s2 = corner + back * tangent;
                    e2 = corner + on * tangent;
                }

                std::vector<VECTOR2D> points = miterPoints( c2, s2, theta );
                std::vector<PIECE>    pieces;
                VECTOR2D              at = s2;

                points.push_back( e2 );

                for( const VECTOR2D& pt : points )
                {
                    PIECE seg;
                    seg.m_a = at;
                    seg.m_b = pt;
                    seg.m_changed = true;
                    at = pt;

                    if( seg.Length() > NOTHING )
                        pieces.push_back( seg );
                }

                // Everything that joined the old copper still on the new
                std::vector<PIECE> copper = pieces;

                if( straightBefore )
                {
                    PIECE before;
                    before.m_a = out.back().m_a;
                    before.m_b = s2;
                    copper.push_back( before );
                }

                if( straightAfter )
                {
                    PIECE after;
                    after.m_a = e2;
                    after.m_b = p[i + 1].m_b;
                    copper.push_back( after );
                }

                if( !keeps( joins, copper, like->GetWidth() ) )
                {
                    if( why.IsEmpty() )
                        why = wxS( "lose another track or a via that joins it" );

                    continue;
                }

                bool               clear = true;
                std::vector<PIECE> check = pieces;

                // A narrower corner lengthens the straights either side towards it
                if( scale < 1.0 )
                {
                    PIECE before, after;
                    before.m_a = s;
                    before.m_b = s2;
                    after.m_a = e2;
                    after.m_b = e;
                    check.push_back( before );
                    check.push_back( after );
                }

                // What it takes the place of: the arc and the straights either side
                std::vector<PIECE> old = { p[i] };

                for( const PIECE* piece : around )
                    old.push_back( *piece );

                for( const PIECE& seg : check )
                {
                    if( std::optional<wxString> hit = clash( seg, like, old ) )
                    {
                        if( why.IsEmpty() )
                            why = wxS( "come " ) + *hit;

                        clear = false;
                        break;
                    }
                }

                if( clear )
                {
                    chosen = pieces;
                    chosenStart = s2;
                    chosenEnd = e2;
                    break;
                }
            }

            if( !chosen )
            {
                m_result.m_left.push_back( wxS( "the arc " ) + where( p[i].m_mid, like )
                                           + wxS( ": 45 degree corners would " ) + why );
                out.push_back( p[i] );
                continue;
            }

            if( straightBefore && out.back().m_b != chosenStart )
            {
                out.back().m_b = chosenStart;
                out.back().m_changed = true;
            }

            for( const PIECE& seg : *chosen )
                out.push_back( seg );

            if( straightAfter && p[i + 1].m_a != chosenEnd )
            {
                p[i + 1].m_a = chosenEnd;
                p[i + 1].m_changed = true;
            }

            ++m_result.m_changed;
        }

        // Straights that now run on in one line become one
        std::vector<PIECE> merged;

        for( const PIECE& piece : out )
        {
            if( !merged.empty() && !piece.m_arc && !merged.back().m_arc
                && ( piece.m_changed || merged.back().m_changed ) )
            {
                PIECE&   last = merged.back();
                VECTOR2D d1 = last.m_b - last.m_a;
                VECTOR2D d2 = piece.m_b - piece.m_a;

                if( std::abs( cross( unit( d1 ), unit( d2 ) ) ) < 1e-6 && dot( d1, d2 ) > 0 )
                {
                    last.m_b = piece.m_b;
                    last.m_changed = true;

                    if( !last.m_from )
                        last.m_from = piece.m_from;

                    continue;
                }
            }

            merged.push_back( piece );
        }

        joinUp( merged );
        p = merged;
    }

    /// Lays a changed run: a track kept as it was, a straight that comes from one moved, the rest new
    void apply( RUN& aRun, BOARD_COMMIT& aCommit )
    {
        PCB_TRACK*           like = aRun.m_items.front();
        std::set<PCB_TRACK*> used;

        for( PIECE& piece : aRun.m_pieces )
        {
            bool fromStraight = piece.m_from && piece.m_from->Type() == PCB_TRACE_T;

            if( piece.m_from && !piece.m_changed && !used.count( piece.m_from ) )
            {
                used.insert( piece.m_from );
                continue;
            }

            if( !piece.m_arc && fromStraight && !used.count( piece.m_from ) )
            {
                used.insert( piece.m_from );
                aCommit.Modify( piece.m_from );
                piece.m_from->SetStart( toI( piece.m_a ) );
                piece.m_from->SetEnd( toI( piece.m_b ) );
                m_result.m_modified.push_back( piece.m_from );
                continue;
            }

            PCB_TRACK* track;

            if( piece.m_arc )
            {
                SHAPE_ARC shape( toI( piece.m_a ), toI( piece.m_mid ), toI( piece.m_b ), 0 );
                track = new PCB_ARC( m_board, &shape );
            }
            else
            {
                track = new PCB_TRACK( m_board );
                track->SetStart( toI( piece.m_a ) );
                track->SetEnd( toI( piece.m_b ) );
            }

            track->SetLayer( like->GetLayer() );
            track->SetWidth( like->GetWidth() );
            track->SetNet( like->GetNet() );
            track->SetHasSolderMask( like->HasSolderMask() );
            track->SetLocalSolderMaskMargin( like->GetLocalSolderMaskMargin() );
            aCommit.Add( track );
            m_result.m_added.push_back( track );
        }

        for( PCB_TRACK* item : aRun.m_items )
        {
            if( used.count( item ) )
                continue;

            m_result.m_removed.push_back( item->m_Uuid );
            aCommit.Remove( item );
        }
    }

    BOARD*                                             m_board;
    TRACK_CORNER_STYLE                                 m_style;
    double                                             m_factor;
    int                                                m_maxRadius;
    std::map<std::tuple<int, int, int>, double>        m_radius;    ///< by layer and first corner
    std::set<std::tuple<int, int, int>>                m_merged;    ///< chamfers that are one bend
    CHECK_IFACE                                        m_iface;
    PNS::NODE                                          m_world;
    std::map<std::pair<int, int>, std::vector<PCB_TRACK*>> m_ends;
    TRACK_CORNERS_RESULT                               m_result;
};

} // namespace


TRACK_CORNERS_RESULT ConvertTrackCorners( BOARD* aBoard, const std::vector<PCB_TRACK*>& aTracks,
                                          TRACK_CORNER_STYLE aStyle, double aFactor,
                                          int aMaxRadius, BOARD_COMMIT& aCommit )
{
    CORNERS corners( aBoard, aStyle, aFactor, aMaxRadius );
    return corners.Run( aTracks, aCommit );
}
