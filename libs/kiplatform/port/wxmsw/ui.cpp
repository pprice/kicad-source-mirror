/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2020 Ian McInerney <Ian.S.McInerney at ieee.org>
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

#include <windows.h>
#include <commctrl.h>

#ifdef _MSC_VER
#pragma comment( lib, "comctl32.lib" )   // SetWindowSubclass, for FlattenDropDown()
#endif

#include <kiplatform/ui.h>

#include <algorithm>

#include <wx/bmpcbox.h>
#include <wx/control.h>
#include <wx/ctrlsub.h>
#include <wx/dcgraph.h>
#include <wx/dcmemory.h>
#include <wx/settings.h>
#include <wx/cursor.h>
#include <wx/dataview.h>
#include <wx/dialog.h>
#include <wx/display.h>
#include <wx/eventfilter.h>
#include <wx/nonownedwnd.h>
#include <wx/utils.h>
#include <wx/window.h>
#include <wx/msw/registry.h>


bool KIPLATFORM::UI::IsDarkTheme()
{
#if wxCHECK_VERSION( 3, 3, 0 )
    wxSystemAppearance appearance = wxSystemSettings::GetAppearance();
    return appearance.IsDark();
#else
    wxColour bg = wxSystemSettings::GetColour( wxSYS_COLOUR_WINDOW );

    // Weighted W3C formula
    double brightness = ( bg.Red() / 255.0 ) * 0.299 +
        ( bg.Green() / 255.0 ) * 0.587 +
        ( bg.Blue() / 255.0 ) * 0.117;

    return brightness < 0.5;
#endif
}


wxColour KIPLATFORM::UI::GetDialogBGColour()
{
    return wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE );
}


void KIPLATFORM::UI::GetInfoBarColours( wxColour& aFGColour, wxColour& aBGColour )
{
    aBGColour = wxSystemSettings::GetColour( wxSYS_COLOUR_INFOBK );
    aFGColour = wxSystemSettings::GetColour( wxSYS_COLOUR_INFOTEXT );
}


wxWindow* KIPLATFORM::UI::CreateCanvasOverlay( wxWindow* aCanvas )
{
    return nullptr;
}


void KIPLATFORM::UI::ForceFocus( wxWindow* aWindow )
{
    aWindow->SetFocus();
}


bool KIPLATFORM::UI::IsWindowActive( wxWindow* aWindow )
{
    if(! aWindow )
    {
	    return false;
    }

    return ( aWindow->GetHWND() == GetForegroundWindow() );
}


void KIPLATFORM::UI::StabilizeWindowPosition( wxWindow* aWindow )
{
    // Not needed on this platform
}


wxPoint KIPLATFORM::UI::GetRestorablePosition( wxWindow* aWindow )
{
    return aWindow->GetPosition();
}


void KIPLATFORM::UI::ReparentModal( wxNonOwnedWindow* aWindow )
{
    // Not needed on this platform
}


void KIPLATFORM::UI::ReparentWindow( wxNonOwnedWindow* aWindow, wxTopLevelWindow* aParent )
{
    // Not needed on this platform (used only on macOS for child window ordering)
}


void KIPLATFORM::UI::FixupCancelButtonCmdKeyCollision( wxWindow *aWindow )
{
    // Not needed on this platform
}


bool KIPLATFORM::UI::IsStockCursorOk( wxStockCursor aCursor )
{
    switch( aCursor )
    {
    case wxCURSOR_BULLSEYE:
    case wxCURSOR_HAND:
    case wxCURSOR_ARROW:
        return true;
    default:
        return false;
    }
}


void KIPLATFORM::UI::LargeChoiceBoxHack( wxChoice* aChoice )
{
    // Not implemented
}


void KIPLATFORM::UI::EllipsizeChoiceBox( wxChoice* aChoice )
{
    // Not implemented
}


double KIPLATFORM::UI::GetPixelScaleFactor( const wxWindow* aWindow )
{
    return aWindow->GetContentScaleFactor();
}


double KIPLATFORM::UI::GetContentScaleFactor( const wxWindow* aWindow )
{
    return aWindow->GetDPIScaleFactor();
}


double KIPLATFORM::UI::GetDisplayRefreshRate( const wxWindow* aWindow )
{
    // wx reports the hardware's default rate as 0
    return wxDisplay( aWindow ).GetCurrentMode().refresh;
}


wxSize KIPLATFORM::UI::GetUnobscuredSize( const wxWindow* aWindow )
{
    return aWindow->GetClientSize();
}


void KIPLATFORM::UI::SetOverlayScrolling( const wxWindow* aWindow, bool overlay )
{
    // Not implemented
}


bool KIPLATFORM::UI::AllowIconsInMenus()
{
    return true;
}


bool KIPLATFORM::UI::SupportsKeyState()
{
    return true;
}


wxKeyEvent KIPLATFORM::UI::GetNativePopupDismissalEvent()
{
    return wxKeyEvent();
}


bool KIPLATFORM::UI::PrepareChoicePopup( wxWindow* )
{
    return true;
}


/*
 * A pointer of its own, for driving the editor unseen (tools/kicad_shot.py in Continuity).
 *
 * With KICAD_POSTED_POINTER set, the pointer is where the last mouse message sent to one of our
 * windows put it, and warping it moves only that: the real pointer is never read or moved. Mouse
 * messages posted to a window that is off every screen then drive the tools as a hand would.
 */
namespace
{
bool postedPointer()
{
    static const bool posted = wxGetEnv( wxS( "KICAD_POSTED_POINTER" ), nullptr );
    return posted;
}


wxPoint s_postedAt;


class POSTED_POINTER_FILTER : public wxEventFilter
{
public:
    int FilterEvent( wxEvent& aEvent ) override
    {
        const wxEventType type = aEvent.GetEventType();

        if( type == wxEVT_MOTION || type == wxEVT_LEFT_DOWN || type == wxEVT_LEFT_UP
                || type == wxEVT_RIGHT_DOWN || type == wxEVT_RIGHT_UP )
        {
            if( wxWindow* window = dynamic_cast<wxWindow*>( aEvent.GetEventObject() ) )
                s_postedAt = window->ClientToScreen( static_cast<wxMouseEvent&>( aEvent ).GetPosition() );        }

        return Event_Skip;
    }
};
}


wxPoint KIPLATFORM::UI::GetMousePosition()
{
    if( postedPointer() )
    {
        // Added on first use and never removed: a filter destroyed while added asserts.
        static POSTED_POINTER_FILTER* filter = nullptr;

        if( !filter )
        {
            filter = new POSTED_POINTER_FILTER();
            wxEvtHandler::AddFilter( filter );
        }

        return s_postedAt;
    }

    return wxGetMousePosition();
}


bool KIPLATFORM::UI::WarpPointer( wxWindow* aWindow, int aX, int aY )
{
    if( postedPointer() )
    {
        s_postedAt = aWindow->ClientToScreen( wxPoint( aX, aY ) );
        return true;
    }

    aWindow->WarpPointer( aX, aY );
    return true;
}


void KIPLATFORM::UI::ImmControl( wxWindow* aWindow, bool aEnable )
{
    if ( !aEnable )
    {
        ImmAssociateContext( aWindow->GetHWND(), NULL );
    }
    else
    {
        ImmAssociateContextEx( aWindow->GetHWND(), 0, IACE_DEFAULT );
    }
}


void KIPLATFORM::UI::ImeNotifyCancelComposition( wxWindow* aWindow )
{
    const HIMC himc = ImmGetContext( aWindow->GetHWND() );
    ImmNotifyIME( himc, NI_COMPOSITIONSTR, CPS_CANCEL, 0 );
    ImmReleaseContext( aWindow->GetHWND(), himc );
}


bool KIPLATFORM::UI::InfiniteDragPrepareWindow( wxWindow* aWindow )
{
    return true;
}


void KIPLATFORM::UI::InfiniteDragReleaseWindow()
{
    // Not needed on this platform
}


void KIPLATFORM::UI::SetFloatLevel( wxWindow* aWindow )
{
}

void KIPLATFORM::UI::ReleaseChildWindow( wxNonOwnedWindow* aWindow )
{
    // Not needed on this platform
}


void KIPLATFORM::UI::AllowNetworkFileSystems( wxDialog* aDialog )
{
    // Not needed on Windows - file dialogs show network filesystems by default
}


int KIPLATFORM::UI::GetColumnWidth( const wxDataViewColumn* aColumn )
{
    return aColumn->WXGetSpecifiedWidth();
}


void KIPLATFORM::UI::CancelPendingScroll( wxDataViewCtrl* aCtrl )
{
    // Not needed on this platform; the native control does not race on model resets.
}


void KIPLATFORM::UI::SetWMClass( wxWindow* aWindow, const wxString& aClass )
{
    // WM_CLASS is an X11/Wayland concept; taskbar identity on Windows is driven by the
    // AppUserModelID (see KIPLATFORM::ENV::SetAppDetailsForWindow).
}


void KIPLATFORM::UI::GetNativeWindowHandles( wxWindow* aWindow, void** aHandle, void** aDisplay,
                                              int* aDisplayType )
{
    *aHandle = nullptr;
    *aDisplay = nullptr;

    if( aDisplayType )
        *aDisplayType = NATIVE_DISPLAY_TYPE::DISPLAY_WIN32;

    if( aWindow )
        *aHandle = static_cast<void*>( aWindow->GetHWND() );
}


void KIPLATFORM::UI::PrepareForExternalRendering( wxWindow* aWindow )
{
    // The GPU API owns Win32 surfaces natively
}


void KIPLATFORM::UI::KeepAboveExternalRendering( wxWindow* aChild )
{
    // Child windows are native and clip their parent's surface
}


void KIPLATFORM::UI::RebindOnRealize( wxWindow* aWindow )
{
    // Win32 windows keep their HWND when reparented
}


namespace
{
const wchar_t FLAT_HOVER[] = L"KiCadFlatDropDownHover";


/// The drop-down as Continuity's mock draws it, into a bitmap the size of the control.
void paintFlatDropDown( HWND aHwnd, HDC aHdc, wxWindow* aCtrl )
{
    RECT rc;
    ::GetClientRect( aHwnd, &rc );

    const int w = rc.right - rc.left;
    const int h = rc.bottom - rc.top;

    if( w <= 0 || h <= 0 )
        return;

    const bool enabled = aCtrl->IsEnabled();
    const bool hover = ::GetPropW( aHwnd, FLAT_HOVER ) != nullptr;
    const bool open = ::SendMessageW( aHwnd, CB_GETDROPPEDSTATE, 0, 0 ) != 0;

    wxColour ground = aCtrl->GetParent() ? aCtrl->GetParent()->GetBackgroundColour()
                                         : wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE );
    wxColour fill = ground.ChangeLightness( 112 );
    wxColour edge = ( hover || open ) ? wxSystemSettings::GetColour( wxSYS_COLOUR_INACTIVECAPTIONTEXT )
                                      : wxSystemSettings::GetColour( wxSYS_COLOUR_3DLIGHT );
    wxColour ink = enabled ? wxSystemSettings::GetColour( wxSYS_COLOUR_BTNTEXT )
                           : wxSystemSettings::GetColour( wxSYS_COLOUR_GRAYTEXT );
    wxColour quiet = wxSystemSettings::GetColour( wxSYS_COLOUR_INACTIVECAPTIONTEXT );

    wxBitmap   buffer( w, h, 24 );
    wxMemoryDC mdc( buffer );
    mdc.SetBackground( wxBrush( ground ) );
    mdc.Clear();

    {
        wxGCDC       gdc( mdc );
        const double radius = aCtrl->FromDIP( 6 );

        gdc.SetPen( wxPen( edge, 1 ) );
        gdc.SetBrush( wxBrush( fill ) );
        gdc.DrawRoundedRectangle( 0, 0, w, h, radius );

        // The chevron, quiet, at the right.
        const int cx = w - aCtrl->FromDIP( 14 );
        const int cy = h / 2;
        const int a = aCtrl->FromDIP( 4 );
        const int b = aCtrl->FromDIP( 2 );
        gdc.SetPen( wxPen( quiet, std::max( 1, aCtrl->FromDIP( 1 ) ) ) );
        gdc.DrawLine( cx - a, cy - b, cx, cy + b );
        gdc.DrawLine( cx, cy + b, cx + a, cy - b );
    }

    // What is chosen: a layer's swatch first where it has one, then its text.
    int      x = aCtrl->FromDIP( 10 );
    wxString text;

    if( wxItemContainerImmutable* items = dynamic_cast<wxItemContainerImmutable*>( aCtrl ) )
        text = items->GetStringSelection();

    if( wxBitmapComboBox* bitmaps = dynamic_cast<wxBitmapComboBox*>( aCtrl ) )
    {
        int sel = bitmaps->GetSelection();

        if( sel != wxNOT_FOUND )
        {
            wxBitmap bmp = bitmaps->GetItemBitmap( sel );

            if( bmp.IsOk() )
            {
                mdc.DrawBitmap( bmp, x, ( h - bmp.GetHeight() ) / 2, true );
                x += bmp.GetWidth() + aCtrl->FromDIP( 6 );
            }
        }
    }

    // In the control's own font, as Windows drew it: a memory DC would size a wxFont for 96 DPI.
    HDC   mhdc = (HDC) mdc.GetHDC();
    HFONT font = (HFONT) ::SendMessageW( aHwnd, WM_GETFONT, 0, 0 );
    HGDIOBJ oldFont = font ? ::SelectObject( mhdc, font ) : nullptr;

    ::SetBkMode( mhdc, TRANSPARENT );
    ::SetTextColor( mhdc, RGB( ink.Red(), ink.Green(), ink.Blue() ) );

    RECT textRect = { x, 0, w - aCtrl->FromDIP( 24 ), h };
    ::DrawTextW( mhdc, text.wc_str(), -1, &textRect,
                 DT_SINGLELINE | DT_VCENTER | DT_LEFT | DT_END_ELLIPSIS | DT_NOPREFIX );

    if( oldFont )
        ::SelectObject( mhdc, oldFont );

    ::BitBlt( aHdc, 0, 0, w, h, mhdc, 0, 0, SRCCOPY );
}


LRESULT CALLBACK flatDropDownProc( HWND aHwnd, UINT aMsg, WPARAM aWParam, LPARAM aLParam,
                                   UINT_PTR aId, DWORD_PTR aData )
{
    wxWindow* ctrl = reinterpret_cast<wxWindow*>( aData );

    switch( aMsg )
    {
    case WM_NCDESTROY:
        ::RemovePropW( aHwnd, FLAT_HOVER );
        ::RemoveWindowSubclass( aHwnd, flatDropDownProc, aId );
        break;

    case WM_ERASEBKGND:
        return 1;

    case WM_PAINT:
    {
        PAINTSTRUCT ps;
        HDC         hdc = ::BeginPaint( aHwnd, &ps );
        paintFlatDropDown( aHwnd, hdc, ctrl );
        ::EndPaint( aHwnd, &ps );
        return 0;
    }

    case WM_PRINTCLIENT:
        paintFlatDropDown( aHwnd, (HDC) aWParam, ctrl );
        return 0;

    case WM_MOUSEMOVE:
        if( !::GetPropW( aHwnd, FLAT_HOVER ) )
        {
            ::SetPropW( aHwnd, FLAT_HOVER, (HANDLE) 1 );

            TRACKMOUSEEVENT track = { sizeof( TRACKMOUSEEVENT ), TME_LEAVE, aHwnd, 0 };
            ::TrackMouseEvent( &track );
            ::InvalidateRect( aHwnd, nullptr, FALSE );
        }

        break;

    case WM_MOUSELEAVE:
        ::RemovePropW( aHwnd, FLAT_HOVER );
        ::InvalidateRect( aHwnd, nullptr, FALSE );
        break;

    // The combo box paints its focus and its open state straight onto the screen, outside
    // WM_PAINT: let it, then paint over it.
    case WM_SETFOCUS:
    case WM_KILLFOCUS:
    case WM_ENABLE:
    case WM_LBUTTONDOWN:
    case WM_LBUTTONUP:
    case WM_KEYDOWN:
    case CB_SETCURSEL:
    {
        LRESULT result = ::DefSubclassProc( aHwnd, aMsg, aWParam, aLParam );
        ::InvalidateRect( aHwnd, nullptr, FALSE );
        ::UpdateWindow( aHwnd );
        return result;
    }

    default:
        break;
    }

    return ::DefSubclassProc( aHwnd, aMsg, aWParam, aLParam );
}
} // namespace


void KIPLATFORM::UI::FlattenDropDown( wxWindow* aCtrl )
{
    if( !aCtrl || !aCtrl->GetHWND() )
        return;

    HWND hwnd = (HWND) aCtrl->GetHWND();

    // Only a list that cannot be typed into: an editable combo box has a text field inside it.
    if( ( ::GetWindowLongW( hwnd, GWL_STYLE ) & 0x3 ) != CBS_DROPDOWNLIST )
        return;

    ::SetWindowSubclass( hwnd, flatDropDownProc, 1, reinterpret_cast<DWORD_PTR>( aCtrl ) );
    ::InvalidateRect( hwnd, nullptr, TRUE );
}


int KIPLATFORM::UI::FlatDropDownExtraWidth( wxWindow* aCtrl )
{
    if( !aCtrl || !aCtrl->GetHWND() )
        return 0;

    DWORD_PTR data = 0;

    if( !::GetWindowSubclass( (HWND) aCtrl->GetHWND(), flatDropDownProc, 1, &data ) )
        return 0;

    // The pill's 10 before the text and 24 for the chevron, against Windows' own few pixels
    // before the text and its arrow button.
    return aCtrl->FromDIP( 16 );
}
