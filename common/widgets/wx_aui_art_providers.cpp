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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <wx/aui/aui.h>
#include <wx/aui/framemanager.h>
#include <wx/aui/auibook.h>
#include <wx/bitmap.h>
#include <wx/dc.h>
#include <wx/settings.h>
#include <wx/fontenum.h>
#include <wx/image.h>

#include <map>

#include <kiplatform/ui.h>
#include <pgm_base.h>
#include <settings/common_settings.h>
#include <widgets/panel_notebook_base.h>
#include <widgets/wx_aui_art_providers.h>
#include <gal/color4d.h>

/// a moved toward b by t (0 to 1).
static wxColour blend( const wxColour& a, const wxColour& b, double t )
{
    auto mix = [t]( unsigned char x, unsigned char y )
    {
        return static_cast<unsigned char>( x + ( y - x ) * t + 0.5 );
    };

    return wxColour( mix( a.Red(), b.Red() ), mix( a.Green(), b.Green() ),
                     mix( a.Blue(), b.Blue() ) );
}


/// The surface toolbars, tabs and pane captions sit on.
static wxColour artFace()
{
    return wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE );
}


/// What is acted on: the highlight, which dark mode makes Continuity's gold.
static wxColour artAccent()
{
    return wxSystemSettings::GetColour( wxSYS_COLOUR_HIGHLIGHT );
}


/// A hairline: the surface moved a little toward the ink.
static wxColour artRule()
{
    return blend( artFace(), wxSystemSettings::GetColour( wxSYS_COLOUR_BTNTEXT ), 0.12 );
}


/**
 * The icon drawn in one colour, its light and dark kept as strength so the drawing still reads:
 * the tool in use shows in the accent.  Kept per bitmap, since a toolbar repaints often.
 */
static wxBitmap tinted( const wxBitmap& aBitmap, const wxColour& aColour )
{
    // Each entry holds its source too, so the data its key points at stays alive and no other
    // bitmap can take that address while the entry is kept.
    static std::map<std::pair<const void*, wxUint32>, std::pair<wxBitmap, wxBitmap>> cache;

    const auto key = std::make_pair( static_cast<const void*>( aBitmap.GetRefData() ),
                                     static_cast<wxUint32>( aColour.GetRGB() ) );

    if( auto it = cache.find( key ); it != cache.end() )
        return it->second.second;

    wxImage image = aBitmap.ConvertToImage();

    if( !image.HasAlpha() )
        image.InitAlpha();

    unsigned char* rgb = image.GetData();
    const int      count = image.GetWidth() * image.GetHeight();

    for( int i = 0; i < count; ++i )
    {
        const double lum = ( 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2] ) / 255.0;
        const double k = 0.55 + 0.45 * lum;

        rgb[0] = static_cast<unsigned char>( aColour.Red() * k );
        rgb[1] = static_cast<unsigned char>( aColour.Green() * k );
        rgb[2] = static_cast<unsigned char>( aColour.Blue() * k );
        rgb += 3;
    }

    wxBitmap out( image );
    out.SetScaleFactor( aBitmap.GetScaleFactor() );

    if( cache.size() > 512 )
        cache.clear();

    cache[key] = std::make_pair( aBitmap, out );
    return out;
}


#if wxCHECK_VERSION( 3, 3, 0 )
wxSize WX_AUI_TOOLBAR_ART::GetToolSize( wxReadOnlyDC& aDc, wxWindow* aWindow,
                                        const wxAuiToolBarItem& aItem )
#else
wxSize WX_AUI_TOOLBAR_ART::GetToolSize( wxDC& aDc, wxWindow* aWindow,
                                        const wxAuiToolBarItem& aItem )
#endif
{
    // Based on the upstream wxWidgets implementation, but simplified for our application
    int size = aWindow->FromDIP( Pgm().GetCommonSettings()->m_Appearance.toolbar_icon_size );

    int width = size;
    int height = size;

    if( ( m_flags & wxAUI_TB_TEXT ) && !aItem.GetLabel().empty() )
    {
        aDc.SetFont( m_font );
        int tx, ty;

        if( m_textOrientation == wxAUI_TBTOOL_TEXT_BOTTOM )
        {
            aDc.GetTextExtent( wxT( "ABCDHgj" ), &tx, &ty );
            height += ty;

            if( !aItem.GetLabel().empty() )
            {
                aDc.GetTextExtent( aItem.GetLabel(), &tx, &ty );
                width = wxMax( width, tx + aWindow->FromDIP( 6 ) );
            }
        }
        else if( m_textOrientation == wxAUI_TBTOOL_TEXT_RIGHT )
        {
            width += aWindow->FromDIP( 3 ); // space between left border and bitmap
            width += aWindow->FromDIP( 3 ); // space between bitmap and text

            if( !aItem.GetLabel().empty() )
            {
                aDc.GetTextExtent( aItem.GetLabel(), &tx, &ty );
                width += tx;
                height = wxMax( height, ty );
            }
        }
    }

    if( aItem.HasDropDown() )
    {
        int dropdownWidth = GetElementSize( wxAUI_TBART_DROPDOWN_SIZE );
        width += dropdownWidth + aWindow->FromDIP( 4 );
    }

    // Room round each icon, so the rounded ground of a hovered or active tool has a margin.
    const int room = aWindow->FromDIP( 4 );

    return wxSize( width + room, height + room );
}


void WX_AUI_TOOLBAR_ART::DrawButton( wxDC& aDc, wxWindow* aWindow, const wxAuiToolBarItem& aItem,
                                     const wxRect& aRect )
{
    // Based on upstream implementation
    int bmpX = 0, bmpY = 0;
    int textX = 0, textY = 0;

    const wxBitmap& bmp = aItem.GetCurrentBitmapFor( aWindow );
    const wxSize    bmpSize = bmp.IsOk() ? bmp.GetLogicalSize() : wxSize( 0, 0 );

    if( ( m_flags & wxAUI_TB_TEXT ) && !aItem.GetLabel().empty() )
    {
        aDc.SetFont( m_font );

        int textWidth = 0, textHeight = 0;
        int tx, ty;

        aDc.GetTextExtent( wxT( "ABCDHgj" ), &tx, &textHeight );
        aDc.GetTextExtent( aItem.GetLabel(), &textWidth, &ty );

        if( m_textOrientation == wxAUI_TBTOOL_TEXT_BOTTOM )
        {
            bmpX = aRect.x + ( aRect.width / 2 ) - ( bmpSize.x / 2 );

            bmpY = aRect.y + ( ( aRect.height - textHeight ) / 2 ) - ( bmpSize.y / 2 );

            textX = aRect.x + ( aRect.width / 2 ) - ( textWidth / 2 ) + 1;
            textY = aRect.y + aRect.height - textHeight - 1;
        }
        else if( m_textOrientation == wxAUI_TBTOOL_TEXT_RIGHT )
        {
            bmpX = aRect.x + aWindow->FromDIP( 3 );

            bmpY = aRect.y + ( aRect.height / 2 ) - ( bmpSize.y / 2 );

            textX = bmpX + aWindow->FromDIP( 3 ) + bmpSize.x;
            textY = aRect.y + ( aRect.height / 2 ) - ( textHeight / 2 );
        }
    }
    else
    {
        bmpX = aRect.x + ( aRect.width / 2 ) - ( bmpSize.x / 2 );
        bmpY = aRect.y + ( aRect.height / 2 ) - ( bmpSize.y / 2 );
    }

    // Flat, as Continuity's page draws a tool: nothing at rest, a faint lift under the pointer,
    // and the tool in use with its icon in the accent on a faint ground of it. Rounded, and never
    // a bar down one edge.
    const int  state = aItem.GetState();
    const bool enabled = !( state & wxAUI_BUTTON_STATE_DISABLED );
    const bool checked = enabled && ( state & wxAUI_BUTTON_STATE_CHECKED );
    const bool hover = enabled && ( ( state & wxAUI_BUTTON_STATE_HOVER ) || aItem.IsSticky() );
    const bool pressed = enabled && ( state & wxAUI_BUTTON_STATE_PRESSED );

    const wxColour face = artFace();
    const wxColour ink = wxSystemSettings::GetColour( wxSYS_COLOUR_BTNTEXT );
    const wxColour accent = artAccent();
    wxColour       fill;

    if( pressed )
        fill = blend( face, ink, 0.16 );
    else if( checked )
        fill = blend( face, accent, hover ? 0.26 : 0.17 );
    else if( hover )
        fill = blend( face, ink, 0.09 );

    if( fill.IsOk() )
    {
        wxRect r = aRect;
        r.Deflate( aWindow->FromDIP( 1 ) );
        aDc.SetPen( *wxTRANSPARENT_PEN );
        aDc.SetBrush( wxBrush( fill ) );
        aDc.DrawRoundedRectangle( r, aWindow->FromDIP( 6 ) );
    }

    if( bmp.IsOk() )
    {
        if( checked )
            aDc.DrawBitmap( tinted( bmp, accent ), bmpX, bmpY, true );
        else
            aDc.DrawBitmap( bmp, bmpX, bmpY, true );
    }

    // set the item's text color based on if it is disabled
    aDc.SetTextForeground( wxSystemSettings::GetColour( wxSYS_COLOUR_BTNTEXT ) );

    if( aItem.GetState() & wxAUI_BUTTON_STATE_DISABLED )
    {
        aDc.SetTextForeground( wxSystemSettings::GetColour( wxSYS_COLOUR_GRAYTEXT ) );
    }

    if( ( m_flags & wxAUI_TB_TEXT ) && !aItem.GetLabel().empty() )
    {
        aDc.DrawText( aItem.GetLabel(), textX, textY );
    }
}


void WX_AUI_TOOLBAR_ART::DrawBackground( wxDC& aDc, wxWindow* aWindow, const wxRect& aRect )
{
    aDc.SetPen( *wxTRANSPARENT_PEN );
    aDc.SetBrush( wxBrush( artFace() ) );
    aDc.DrawRectangle( aRect );
}


void WX_AUI_TOOLBAR_ART::DrawPlainBackground( wxDC& aDc, wxWindow* aWindow, const wxRect& aRect )
{
    DrawBackground( aDc, aWindow, aRect );
}


void WX_AUI_TOOLBAR_ART::DrawSeparator( wxDC& aDc, wxWindow* aWindow, const wxRect& aRect )
{
    aDc.SetPen( wxPen( artRule(), aWindow->FromDIP( 1 ) ) );

    if( aRect.width < aRect.height )
    {
        const int x = aRect.x + aRect.width / 2;
        const int inset = aRect.height / 4;
        aDc.DrawLine( x, aRect.y + inset, x, aRect.GetBottom() - inset );
    }
    else
    {
        const int y = aRect.y + aRect.height / 2;
        const int inset = aRect.width / 4;
        aDc.DrawLine( aRect.x + inset, y, aRect.GetRight() - inset, y );
    }
}


void WX_AUI_TOOLBAR_ART::DrawGripper( wxDC& aDc, wxWindow* aWindow, const wxRect& aRect )
{
    DrawBackground( aDc, aWindow, aRect );
}


void WX_AUI_TOOLBAR_ART::saturateHighlightColor()
{
#ifdef __WXOSX__
    // Use a slightly stronger highlight colour over grey toolbar backgrounds
    KIGFX::COLOR4D highlight( m_highlightColour );
    m_highlightColour = highlight.Saturate( 0.6 ).ToColour();
#endif
}


void WX_AUI_TOOLBAR_ART::UpdateColoursFromSystem()
{
    wxAuiDefaultToolBarArt::UpdateColoursFromSystem();
    saturateHighlightColor();
}


class ToolbarCommandCapture : public wxEvtHandler
{
public:
    ToolbarCommandCapture() { m_lastId = 0; }
    int GetCommandId() const { return m_lastId; }

    bool ProcessEvent( wxEvent& evt ) override
    {
        if( evt.GetEventType() == wxEVT_MENU )
        {
            m_lastId = evt.GetId();
            return true;
        }

        if( GetNextHandler() )
            return GetNextHandler()->ProcessEvent( evt );

        return false;
    }

private:
    int m_lastId;
};


int WX_AUI_TOOLBAR_ART::ShowDropDown( wxWindow* wnd, const wxAuiToolBarItemArray& items )
{
    wxMenu menuPopup;
    bool   skipNextSeparator = true;

    size_t i, count = items.GetCount();
    for( i = 0; i < count; ++i )
    {
        wxAuiToolBarItem& item = items.Item( i );

        if( item.GetKind() == wxITEM_SEPARATOR )
        {
            if( !skipNextSeparator )
            {
                menuPopup.AppendSeparator();
                skipNextSeparator = true;
            }
        }
        else if( item.GetKind() == wxITEM_NORMAL || item.GetKind() == wxITEM_CHECK || item.GetKind() == wxITEM_RADIO )
        {
            wxString text = item.GetShortHelp();

            if( text.empty() )
                text = item.GetLabel();

            if( text.empty() )
                text = wxT( " " );

            wxString firstLine = text.BeforeFirst( '\n' );
            wxString accel;
            wxString label = firstLine.BeforeFirst( '\t', &accel );

            text = label;

            if( !accel.empty() )
            {
                // Remove brackets from accelerator string so it's recognized
                if( accel.starts_with( "(" ) && accel.ends_with( ")" ) )
                    accel = accel.Mid( 1, accel.size() - 2 );

                text << "\t" << accel;
            }

            bool       checked = item.GetState() & wxAUI_BUTTON_STATE_CHECKED;
            wxItemKind menuKind = wxITEM_NORMAL;

            if( ( item.GetKind() == wxITEM_CHECK || item.GetKind() == wxITEM_RADIO ) && checked )
                menuKind = static_cast<wxItemKind>( item.GetKind() );

            wxMenuItem* m = new wxMenuItem( &menuPopup, item.GetId(), text, item.GetShortHelp(), menuKind );

            if( !m->IsCheckable() )
                m->SetBitmap( item.GetBitmapBundle() );

            menuPopup.Append( m );

            if( m->IsCheckable() )
                m->Check( checked );

            skipNextSeparator = false;
        }
    }

    // find out where to put the popup menu of window items
    wxPoint pt = ::wxGetMousePosition();
    pt = wnd->ScreenToClient( pt );

    // find out the screen coordinate at the bottom of the tab ctrl
    wxRect cli_rect = wnd->GetClientRect();
    pt.y = cli_rect.y + cli_rect.height;

    ToolbarCommandCapture* cc = new ToolbarCommandCapture;
    wnd->PushEventHandler( cc );
    wnd->PopupMenu( &menuPopup, pt );
    int command = cc->GetCommandId();
    wnd->PopEventHandler( true );

    return command;
}


WX_AUI_DOCK_ART::WX_AUI_DOCK_ART() :
        wxAuiDefaultDockArt()
{
#if defined( _WIN32 )
    // Use normal control font, wx likes to use "small"
    m_captionFont = *wxNORMAL_FONT;

    // Increase the box the caption rests in size a bit
    m_captionSize = ( wxNORMAL_FONT->GetPointSize() * 7 ) / 4 + 6;
#endif

    SetColour( wxAUI_DOCKART_ACTIVE_CAPTION_TEXT_COLOUR,
               wxSystemSettings::GetColour( wxSYS_COLOUR_BTNTEXT ) );
    SetColour( wxAUI_DOCKART_INACTIVE_CAPTION_TEXT_COLOUR,
               wxSystemSettings::GetColour( wxSYS_COLOUR_BTNTEXT ) );

    // Panes and their captions on the one surface, hairlines between them: no raised title bars.
    SetColour( wxAUI_DOCKART_BACKGROUND_COLOUR, artFace() );
    SetColour( wxAUI_DOCKART_ACTIVE_CAPTION_COLOUR, artFace() );
    SetColour( wxAUI_DOCKART_INACTIVE_CAPTION_COLOUR, artFace() );
    SetColour( wxAUI_DOCKART_SASH_COLOUR, artFace() );
    SetColour( wxAUI_DOCKART_BORDER_COLOUR, artRule() );
    SetColour( wxAUI_DOCKART_GRIPPER_COLOUR, artRule() );
    if( wxFontEnumerator::IsValidFacename( wxS( "Atkinson Hyperlegible Next" ) ) )
        m_captionFont.SetFaceName( wxS( "Atkinson Hyperlegible Next" ) );

    m_captionFont = m_captionFont.Bold();

    // Turn off the ridiculous looking gradient
    m_gradientType = wxAUI_GRADIENT_NONE;
}

#if wxCHECK_VERSION( 3, 3, 0 )
WX_AUI_TAB_ART::WX_AUI_TAB_ART() :
        wxAuiDefaultTabArt()
{
    SetColour( artFace() );
    SetActiveColour( artFace() );
}


void WX_AUI_TAB_ART::DrawBackground( wxDC& dc, wxWindow* wnd, const wxRect& rect )
{
    dc.SetPen( *wxTRANSPARENT_PEN );
    dc.SetBrush( wxBrush( artFace() ) );
    dc.DrawRectangle( rect );

    dc.SetPen( wxPen( artRule(), wnd->FromDIP( 1 ) ) );
    dc.DrawLine( rect.x, rect.GetBottom(), rect.GetRight() + 1, rect.GetBottom() );
}


int WX_AUI_TAB_ART::DrawPageTab( wxDC& dc, wxWindow* wnd, wxAuiNotebookPage& page,
                                 const wxRect& rect )
{
    int extent = wxAuiDefaultTabArt::DrawPageTab( dc, wnd, page, rect );

    if( page.active )
    {
        // The tab in front: underlined in the accent, the whole width of its caption.
        const int thick = wnd->FromDIP( 2 );
        const int inset = wnd->FromDIP( 6 );
        wxRect    tab = page.rect;

        dc.SetPen( *wxTRANSPARENT_PEN );
        dc.SetBrush( wxBrush( artAccent() ) );
        dc.DrawRectangle( tab.x + inset, tab.GetBottom() - thick + 1, tab.width - 2 * inset, thick );
    }

    return extent;
}


void WX_AUI_TAB_ART::SetSizingInfo( const wxSize& tabCtrlSize, size_t tabCount, wxWindow* wnd )
{
    // SetSizingInfo is called after adding pages.
    // Remove the close button from tabs of unclosable panels.
    PANEL_NOTEBOOK_BASE* panel = dynamic_cast<PANEL_NOTEBOOK_BASE*>( wnd );

    if( panel && !panel->GetClosable() )
    {
        if( wxAuiNotebook* notebook = dynamic_cast<wxAuiNotebook*>( panel->GetParent() ) )
        {
            for( wxAuiTabCtrl* ctrl : notebook->GetAllTabCtrls() )
            {
                for( size_t i = 0; i < ctrl->GetPageCount(); i++ )
                {
                    wxAuiNotebookPage& page = ctrl->GetPage( i );

                    if( page.window == wnd )
                        page.buttons.clear();
                }
            }
        }
    }

    wxAuiDefaultTabArt::SetSizingInfo( tabCtrlSize, tabCount, wnd );
}
#else
void WX_AUI_TAB_ART::DrawTab( wxDC& dc, wxWindow* wnd, const wxAuiNotebookPage& page, const wxRect& in_rect,
                              int close_button_state, wxRect* out_tab_rect, wxRect* out_button_rect,
                              int* x_extent )
{
    PANEL_NOTEBOOK_BASE* panel = dynamic_cast<PANEL_NOTEBOOK_BASE*>( page.window );

    if( panel && !panel->GetClosable() )
        close_button_state = wxAUI_BUTTON_STATE_HIDDEN;

    return wxAuiGenericTabArt::DrawTab( dc, wnd, page, in_rect, close_button_state, out_tab_rect,
                                        out_button_rect, x_extent );
}
#endif
