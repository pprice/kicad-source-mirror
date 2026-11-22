/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2004 Jean-Pierre Charras, jaen-pierre.charras@gipsa-lab.inpg.com
 * Copyright (C) 2011 Wayne Stambaugh <stambaughw@verizon.net>
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

#include <widgets/msgpanel.h>

#include <algorithm>

#include <wx/dcscreen.h>
#include <wx/dcclient.h>
#include <wx/settings.h>
#include <wx/toplevel.h>

#include <advanced_config.h>
#include <kiid.h>
#include <math/util.h>

#include <widgets/ui_common.h>


BEGIN_EVENT_TABLE( EDA_MSG_PANEL, wxPanel )
    EVT_DPI_CHANGED( EDA_MSG_PANEL::OnDPIChanged )
    EVT_SIZE( EDA_MSG_PANEL::OnSize )
    EVT_PAINT( EDA_MSG_PANEL::OnPaint )
END_EVENT_TABLE()


EDA_MSG_PANEL::EDA_MSG_PANEL( wxWindow* aParent, int aId, const wxPoint& aPosition,
                              const wxSize& aSize, long style, const wxString &name ) :
    wxPanel( aParent, aId, aPosition, aSize, style, name )
{
    SetFont( KIUI::GetStatusFont( this ) );
    SetBackgroundColour( wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE ) );

    // informs wx not to paint the background itself as we will paint it later in erase()
    SetBackgroundStyle( wxBG_STYLE_PAINT );

    m_last_x = 0;

    updateFontSize();

    InvalidateBestSize();
}


EDA_MSG_PANEL::~EDA_MSG_PANEL()
{
}


void EDA_MSG_PANEL::updateFontSize()
{
    wxFont font = KIUI::GetControlFont( this );
    GetTextExtent( wxT( "W" ), &m_fontSize.x, &m_fontSize.y, 0, 0, &font );
}


// Continuity, as the mock lays it out: a small quiet label over a large value, with room above
// and below. In units of the control font's height.
static constexpr double LABEL_TOP = 0.55;
static constexpr double VALUE_TOP = 1.55;
static constexpr double PANEL_HEIGHT = 3.5;

// Across, in widths of a "W": where the first item starts and the room between two items, the
// hairline in the middle of it.
static constexpr double FIRST_X = 1.4;
static constexpr double ITEM_GAP = 3.0;
static constexpr double LABEL_SCALE = 0.9;
static constexpr double VALUE_SCALE = 1.3;


wxSize EDA_MSG_PANEL::DoGetBestSize() const
{
    return wxSize( wxDefaultCoord, KiROUND( PANEL_HEIGHT * m_fontSize.y ) );
}


wxSize EDA_MSG_PANEL::DoGetBestClientSize() const
{
    return wxPanel::DoGetBestClientSize();
}


void EDA_MSG_PANEL::OnDPIChanged( wxDPIChangedEvent& aEvent )
{
    updateFontSize();
    InvalidateBestSize();

    aEvent.Skip();
}


void EDA_MSG_PANEL::rebuildItems()
{
    m_last_x = 0;

    for( MSG_PANEL_ITEM& item : m_Items )
        updateItemPos( item );
}


void EDA_MSG_PANEL::OnSize( wxSizeEvent& aEvent )
{
    rebuildItems();
}


void EDA_MSG_PANEL::OnPaint( wxPaintEvent& aEvent )
{
    wxPaintDC dc( this );

    erase( &dc );

    dc.SetBackground( wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE ) );
    dc.SetBackgroundMode( wxSOLID );
    dc.SetTextBackground( wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE ) );
    dc.SetFont( KIUI::GetControlFont( this ) );

    for( const MSG_PANEL_ITEM& item : m_Items )
        showItem( dc, item );

    // A hairline between items, as the mock has.
    dc.SetPen( wxPen( wxSystemSettings::GetColour( wxSYS_COLOUR_3DLIGHT ), 1 ) );
    const int top = KiROUND( LABEL_TOP * m_fontSize.y );
    const int bottom = GetClientSize().y - top;

    for( size_t i = 1; i < m_Items.size(); ++i )
    {
        const int x = m_Items[i].m_X - KiROUND( ITEM_GAP / 2 * m_fontSize.x );
        dc.DrawLine( x, top, x, bottom );
    }

    // And across it, above and below, as the mock parts it from the editor and the status bar.
    const wxSize size = GetClientSize();
    dc.DrawLine( 0, 0, size.x, 0 );
    dc.DrawLine( 0, size.y - 1, size.x, size.y - 1 );

    aEvent.Skip();
}


void EDA_MSG_PANEL::updateItemPos( MSG_PANEL_ITEM& item )
{
    wxString text;
    wxString upperText = item.GetUpperText();
    wxString lowerText = item.GetLowerText();
    wxSize   drawSize = GetClientSize();

    /* Don't put the first message a window client position 0.  Offset by
     * one 'W' character width. */
    if( m_last_x == 0 )
        m_last_x = KiROUND( FIRST_X * m_fontSize.x );

    item.m_X = m_last_x;

    item.m_UpperY = KiROUND( LABEL_TOP * m_fontSize.y );
    item.m_LowerY = KiROUND( VALUE_TOP * m_fontSize.y );
    wxUnusedVar( drawSize );

    // The label is drawn quiet and the value bold (showItem): each measured in its own face.
    wxClientDC dc( this );
    dc.SetFont( labelFont() );
    int width = dc.GetTextExtent( upperText ).x;
    dc.SetFont( valueFont() );
    width = std::max( width, dc.GetTextExtent( lowerText ).x );

    text.Append( ' ', item.GetPadding() );
    width += dc.GetTextExtent( text ).x;

    m_last_x += width;

    // Room either side of the hairline between items.
    m_last_x += KiROUND( ITEM_GAP * m_fontSize.x );
}


void EDA_MSG_PANEL::AppendMessage( const wxString& aUpperText, const wxString& aLowerText, int aPadding )
{
    MSG_PANEL_ITEM item;

    item.m_UpperText = aUpperText;
    item.m_LowerText = aLowerText;

    updateItemPos( item );
    m_Items.push_back( item );

    Refresh();
}


void EDA_MSG_PANEL::showItem( wxDC& aDC, const MSG_PANEL_ITEM& aItem )
{
    // Continuity's "important first": the value bold in the ink, its label quiet above it. The
    // ink drops to the quiet colour when the window isn't active.
    wxTopLevelWindow* tlw = dynamic_cast<wxTopLevelWindow*>( wxGetTopLevelParent( this ) );
    const bool        active = !tlw || tlw->IsActive();
    const wxColour    quiet = wxSystemSettings::GetColour( wxSYS_COLOUR_INACTIVECAPTIONTEXT );
    const wxColour    ink = wxSystemSettings::GetColour( wxSYS_COLOUR_WINDOWTEXT );

    if( !aItem.m_UpperText.IsEmpty() )
    {
        aDC.SetFont( labelFont() );
        aDC.SetTextForeground( quiet );
        aDC.DrawText( aItem.m_UpperText, aItem.m_X, aItem.m_UpperY );
    }

    if( !aItem.m_LowerText.IsEmpty() )
    {
        wxColour colour = active ? ink : quiet;

        if( active && m_valueColourer )
        {
            wxColour own = m_valueColourer( aItem.m_UpperText, aItem.m_LowerText );

            if( own.IsOk() )
                colour = own;
        }

        aDC.SetFont( valueFont() );
        aDC.SetTextForeground( colour );
        aDC.DrawText( aItem.m_LowerText, aItem.m_X, aItem.m_LowerY );
    }
}


wxFont EDA_MSG_PANEL::labelFont()
{
    return KIUI::GetControlFont( this ).Scaled( LABEL_SCALE );
}


wxFont EDA_MSG_PANEL::valueFont()
{
    return KIUI::GetControlFont( this ).Scaled( VALUE_SCALE ).Bold();
}


void EDA_MSG_PANEL::EraseMsgBox()
{
   m_Items.clear();
   m_last_x = 0;
   Refresh();
}


void EDA_MSG_PANEL::erase( wxDC* aDC )
{
    wxPen   pen;
    wxBrush brush;

    wxSize  size  = GetClientSize();
    wxColour color = wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE );

    pen.SetColour( color );

    brush.SetColour( color );
    brush.SetStyle( wxBRUSHSTYLE_SOLID );

    aDC->SetPen( pen );
    aDC->SetBrush( brush );
    aDC->DrawRectangle( 0, 0, size.x, size.y );
}


std::optional<wxString> GetMsgPanelDisplayUuid( const KIID& aKiid )
{
    const static int        showUuids = ADVANCED_CFG::GetCfg().m_MsgPanelShowUuids;
    std::optional<wxString> uuid;

    if( showUuids > 0 )
    {
        uuid = aKiid.AsString();

        if( showUuids == 2 )
            uuid = uuid->SubString( 0, 7 );
    }

    return uuid;
}
