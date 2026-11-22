/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <gal/color4d.h>
#include <gal/graphics_abstraction_layer.h>
#include <kiplatform/ui.h>
#include <layer_ids.h>
#include <wx/settings.h>
#include <wx/filefn.h>

#include <algorithm>
#include <cmath>
#include <deque>

#include <font/font.h>
#include <paths.h>

#include "preview_items/preview_utils.h"
#include "router_status_view_item.h"
#include "gr_text.h"

using namespace KIGFX;


const BOX2I ROUTER_STATUS_VIEW_ITEM::ViewBBox() const
{
    // We can't tell the text's extents without a view to know the scale, so the box is generous:
    // a metre each way of the cursor. The readout is an item of the view's own index now, not
    // of the preview group, and the maximum box is no place in an index.
    const int reach = 1000000000;

    return BOX2I( m_pos - VECTOR2I( reach, reach ), VECTOR2I( 2 * reach, 2 * reach ) );
}

std::vector<int> ROUTER_STATUS_VIEW_ITEM::ViewGetLayers() const
{
    return { LAYER_UI_START, LAYER_UI_START + 1 };
}

/**
 * The interface face, Atkinson Hyperlegible Next, read from the fork's own font files so the
 * readout matches the panels round it; KiCad's stroke font where they are missing.
 */
static KIFONT::FONT* readoutFont( bool aBold )
{
    static const std::vector<wxString> files = []()
    {
        std::vector<wxString> found;
        const wxString        folder = PATHS::GetStockDataPath() + wxS( "/resources/fonts/" );

        for( const wxString& weight : { wxS( "Regular" ), wxS( "Bold" ) } )
        {
            wxString file = folder + wxS( "AtkinsonHyperlegibleNext-" ) + weight + wxS( ".ttf" );

            if( wxFileExists( file ) )
                found.push_back( file );
        }

        return found;
    }();

    if( files.empty() )
        return KIFONT::FONT::GetFont();

    return KIFONT::FONT::GetFont( wxS( "Atkinson Hyperlegible Next" ), aBold, false, &files );
}


/// A rectangle with round corners, as a polygon (GAL has no rounded rectangle).
static std::deque<VECTOR2D> roundedBox( const VECTOR2D& aA, const VECTOR2D& aB, double aRadius )
{
    const double left = std::min( aA.x, aB.x ), right = std::max( aA.x, aB.x );
    const double top = std::min( aA.y, aB.y ), bottom = std::max( aA.y, aB.y );
    const double r = std::min( { aRadius, ( right - left ) / 2, ( bottom - top ) / 2 } );

    const VECTOR2D centres[4] = { { right - r, top + r }, { right - r, bottom - r },
                                  { left + r, bottom - r }, { left + r, top + r } };
    std::deque<VECTOR2D> pts;

    for( int corner = 0; corner < 4; ++corner )
    {
        for( int step = 0; step <= 6; ++step )
        {
            const double a = M_PI * ( -0.5 + 0.5 * corner + step / 12.0 );
            pts.emplace_back( centres[corner].x + r * cos( a ), centres[corner].y + r * sin( a ) );
        }
    }

    return pts;
}


void ROUTER_STATUS_VIEW_ITEM::ViewDraw( int aLayer, KIGFX::VIEW* aView ) const
{
    // Continuity's readout at the cursor: the message (a net, or a pair's two) in bold, then
    // quiet labels and their values, then a hint; a raised box with round corners and a hairline.
    KIGFX::GAL* gal = aView->GetGAL();
    bool        drawingBox = ( aLayer == LAYER_UI_START );

    gal->Save();
    gal->Scale( { 1., 1. } );

    const KIFONT::METRICS&    metrics = KIFONT::METRICS::Default();
    KIGFX::PREVIEW::TEXT_DIMS dims = KIGFX::PREVIEW::GetConstantGlyphHeight( gal, -1 );
    KIFONT::FONT*             regular = readoutFont( false );
    KIFONT::FONT*             bold = readoutFont( true );

    // Everything in units of the constant glyph height, so the box keeps its size on screen; a
    // third larger than KiCad's own cursor text, which is small to read beside a track.
    const VECTOR2D glyph = VECTOR2D( dims.GlyphSize ) * 1.35;
    const double   u = glyph.y;
    const VECTOR2I titleSize = KiROUND( glyph * 1.05 );
    const VECTOR2I rowSize = KiROUND( glyph * 0.9 );
    const VECTOR2I hintSize = KiROUND( glyph * 0.8 );
    const double   pad = u * 0.75;
    const double   titleLine = u * 1.55;
    const double   rowLine = u * 1.35;
    const double   hintLine = u * 1.25;
    const double   labelGap = u * 1.1;

    auto width = [&]( const wxString& aText, KIFONT::FONT* aFont, const VECTOR2I& aSize )
    {
        return aText.IsEmpty() ? 0 : GRTextWidth( aText, aFont, aSize, dims.StrokeWidth, false, false, metrics );
    };

    int labelWidth = 0, valueWidth = 0;

    for( const ROW& row : m_rows )
    {
        labelWidth = std::max( labelWidth, width( row.label, regular, rowSize ) );
        valueWidth = std::max( valueWidth, width( row.value, bold, rowSize ) );
    }

    double contentWidth = std::max( { (double) width( m_status, bold, titleSize ),
                                      (double) width( m_hint, regular, hintSize ),
                                      m_rows.empty() ? 0.0 : labelWidth + labelGap + valueWidth } );
    double contentHeight = ( m_status.IsEmpty() ? 0 : titleLine ) + rowLine * m_rows.size()
                           + ( m_hint.IsEmpty() ? 0 : hintLine );

    // Up and to the right of the cursor, clear of the track being laid. A view flipped left to
    // right runs world x the other way: lay out along sx.
    const double   sx = gal->IsFlippedX() ? -1.0 : 1.0;
    const VECTOR2D size( contentWidth + 2 * pad, contentHeight + 2 * pad - ( titleLine - u * 1.2 ) );
    const VECTOR2D origin( GetPosition().x + sx * u * 1.6, GetPosition().y - size.y - u * 1.6 );

    if( drawingBox )
    {
        const VECTOR2D corner( origin.x + sx * size.x, origin.y + size.y );
        std::deque<VECTOR2D> box = roundedBox( origin, corner, u * 0.6 );

        gal->SetIsFill( true );
        gal->SetIsStroke( false );
        gal->SetFillColor( COLOR4D( wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE ) ).WithAlpha( 0.94 ) );
        gal->DrawPolygon( box );

        box.push_back( box.front() );
        gal->SetIsFill( false );
        gal->SetIsStroke( true );
        gal->SetLineWidth( gal->GetScreenWorldMatrix().GetScale().x * 1.0 );
        gal->SetStrokeColor( m_problem ? COLOR4D( 1.0, 0.36, 0.75, 1.0 )
                                       : COLOR4D( wxSystemSettings::GetColour( wxSYS_COLOUR_3DLIGHT ) ) );
        gal->DrawPolyline( box );
        gal->Restore();
        return;
    }

    const COLOR4D ink = wxSystemSettings::GetColour( wxSYS_COLOUR_BTNTEXT );
    const COLOR4D quiet = wxSystemSettings::GetColour( wxSYS_COLOUR_INACTIVECAPTIONTEXT );
    const COLOR4D problem( 1.0, 0.36, 0.75, 1.0 );   // Continuity's --c-problem, #ff5cbf

    TEXT_ATTRIBUTES attrs;
    attrs.m_Halign = GR_TEXT_H_ALIGN_LEFT;
    attrs.m_Valign = GR_TEXT_V_ALIGN_TOP;
    attrs.m_Mirrored = gal->IsFlippedX();   // Read the right way round in a flipped view
    attrs.m_StrokeWidth = dims.StrokeWidth;

    auto draw = [&]( const wxString& aText, KIFONT::FONT* aFont, const VECTOR2I& aSize,
                     const COLOR4D& aColour, double aX, double aY )
    {
        attrs.m_Size = aSize;
        attrs.m_Bold = aFont == bold;
        gal->SetIsFill( true );
        gal->SetIsStroke( true );
        gal->SetFillColor( aColour );
        gal->SetStrokeColor( aColour );
        aFont->Draw( gal, aText, VECTOR2I( KiROUND( aX ), KiROUND( aY ) ), attrs, metrics );
    };

    const double x = origin.x + sx * pad;
    double       y = origin.y + pad;

    if( !m_status.IsEmpty() )
    {
        draw( m_status, bold, titleSize, m_problem ? problem : ink, x, y );
        y += titleLine;
    }

    for( const ROW& row : m_rows )
    {
        draw( row.label, regular, rowSize, quiet, x, y );
        draw( row.value, bold, rowSize, row.problem ? problem : ink, x + sx * ( labelWidth + labelGap ), y );
        y += rowLine;
    }

    if( !m_hint.IsEmpty() )
        draw( m_hint, regular, hintSize, quiet, x, y );

    gal->Restore();
}

