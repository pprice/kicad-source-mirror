/*
* This program source code file is part of KiCad, a free EDA CAD application.
*
* Copyright (C) 2020 Mark Roszko <mark.roszko@gmail.com>
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

#include <kiplatform/app.h>

#include <wx/app.h>
#include <wx/log.h>
#include <wx/string.h>
#include <wx/window.h>
#include <wx/eventfilter.h>
#include <wx/pen.h>
#include <wx/toplevel.h>
#if wxCHECK_VERSION( 3, 3, 0 )
#include <wx/msw/darkmode.h>
#endif

#include <windows.h>
#include <strsafe.h>
#include <config.h>
#include <versionhelpers.h>
#include <iostream>
#include <cstdio>
#include <io.h>

#if defined( _MSC_VER )
#include <werapi.h>     // issues on msys2
#endif

#ifdef _WIN32
extern "C"
{
    // So there exists this malware called Nahimic by A-Volute, which is marketed as an audio enhancement
    // software. In reality it's an aggressive form of malware that injects itself wildly into every process
    // on the system for god knows what reason. It even includes a tracking/analytics package, <insert tinfoil hat>
    // Our problem is this garbage basically bugs out OpenGL (why an audio driver does that, who knows, its made by morons)
    // And then we get issues reported both in our issue tracker and sentry reports as a result
    // At least these malware authors were nice to include a dumb "disable" trick where it checks if the exe is exporting
    // a symbol called NoHotPatch, so here we are.
    // Hopefully this works and stops the bug reports. Apparently the worst part is this malware aggressively gets reinstalled
    // by awful low-tier motherboard vendors like MSI, Alienware and others who bundled it into their driver packages
    // and distributed it over Windows Update
    // Did I mention they clearly had issues with other apps so instead of fixing their malware, they blacklisted a hundred common
    // apps and even some games in their own config? Obviously kicad isn't on that blacklist :(
    // This malware seems to no longer be distributed as Nahimic and replaced with "Sonar" by SteelSeries.
    // Time will tell if it's the same garbage, I'm not volunteering to install it.
    __declspec(dllexport) void NoHotPatch()
    {
        // this is a intentionally empty function
        return;
    }
}
#endif

#if wxCHECK_VERSION( 3, 3, 0 )
// Continuity's palette (src/ui/src/styles/tokens.css in pprice/continuity): surfaces of matte
// black, white ink and a quieter grey, rules in dark grey, and gold for what is acted on.
namespace CONTINUITY_PALETTE
{
    static const wxColour surface( 15, 15, 15 );          // --surface
    static const wxColour raised( 27, 27, 27 );           // --surface-raised
    static const wxColour rule( 48, 48, 48 );             // --rule
    static const wxColour edge( 116, 116, 111 );          // --edge
    static const wxColour ink( 243, 243, 238 );           // --ink
    static const wxColour quiet( 162, 162, 157 );         // --ink-quiet
    static const wxColour accent( 226, 184, 77 );         // --accent
    static const wxColour onAccent( 17, 17, 17 );         // --on-accent
    static const wxColour accentFaint( 58, 51, 35 );      // gold at a tenth over the surface
}


class KICAD_DARK_MODE_SETTINGS : public wxDarkModeSettings
{
public:
    wxColour GetColour( wxSystemColour index ) override
    {
        using namespace CONTINUITY_PALETTE;

        switch( index )
        {
        // As the mock has it (Phil: "black panels with thin grey border", where every surface
        // was raised grey): the window's chrome, its panels and its bars all on the page's
        // black, parted by rules; only what floats or is picked from is raised.
        case wxSYS_COLOUR_WINDOW:
        case wxSYS_COLOUR_LISTBOX:
        case wxSYS_COLOUR_APPWORKSPACE:
        case wxSYS_COLOUR_BTNFACE:
        case wxSYS_COLOUR_ACTIVECAPTION:
        case wxSYS_COLOUR_INACTIVECAPTION:
        case wxSYS_COLOUR_MENUBAR:
            return surface;

        case wxSYS_COLOUR_MENU:
        case wxSYS_COLOUR_INFOBK:
            return raised;

        case wxSYS_COLOUR_3DLIGHT:
        case wxSYS_COLOUR_BTNHIGHLIGHT:
        case wxSYS_COLOUR_BTNSHADOW:
        case wxSYS_COLOUR_GRIDLINES:
        case wxSYS_COLOUR_SCROLLBAR:
            return rule;

        case wxSYS_COLOUR_BTNTEXT:
        case wxSYS_COLOUR_CAPTIONTEXT:
        case wxSYS_COLOUR_INFOTEXT:
        case wxSYS_COLOUR_LISTBOXTEXT:
        case wxSYS_COLOUR_LISTBOXHIGHLIGHTTEXT:
        case wxSYS_COLOUR_MENUTEXT:
        case wxSYS_COLOUR_WINDOWTEXT:
            return ink;

        case wxSYS_COLOUR_INACTIVECAPTIONTEXT:
            return quiet;

        case wxSYS_COLOUR_GRAYTEXT:
            return edge;

        case wxSYS_COLOUR_LISTBOXHIGHLIGHT:
            return accentFaint;

        case wxSYS_COLOUR_HIGHLIGHT:
        case wxSYS_COLOUR_HOTLIGHT:
            return accent;

        case wxSYS_COLOUR_HIGHLIGHTTEXT:
            return onAccent;

        default: return wxDarkModeSettings::GetColour( index );
        }
    }

    wxColour GetMenuColour( wxMenuColour which ) override
    {
        using namespace CONTINUITY_PALETTE;

        switch( which )
        {
        case wxMenuColour::StandardFg: return ink;
        // wx paints only the menu bar with this (its drop-downs are the system's): on the
        // window's black, or it is "just another grey line" (Phil, 2026-11-22).
        case wxMenuColour::StandardBg: return surface;
        case wxMenuColour::DisabledFg: return edge;
        case wxMenuColour::HotBg:      return rule;
        default:                       return wxDarkModeSettings::GetMenuColour( which );
        }
    }

    wxPen GetBorderPen() override
    {
        return wxPen( CONTINUITY_PALETTE::rule );
    }
};


// Windows 11 draws each frame's title bar in a colour of its own unless asked: every top-level
// window is given the raised surface, white ink and a dark grey border as it is created.
// dwmapi is loaded by name, so nothing has to link it.
class TITLE_BAR_FILTER : public wxEventFilter
{
public:
    int FilterEvent( wxEvent& aEvent ) override
    {
        if( aEvent.GetEventType() == wxEVT_CREATE )
        {
            if( wxTopLevelWindow* tlw = wxDynamicCast( aEvent.GetEventObject(), wxTopLevelWindow ) )
                Paint( tlw->GetHWND() );
        }

        return Event_Skip;
    }

private:
    static void Paint( WXHWND aHwnd )
    {
        using SET_ATTRIBUTE = HRESULT( WINAPI* )( HWND, DWORD, LPCVOID, DWORD );
        static SET_ATTRIBUTE setAttribute = []() -> SET_ATTRIBUTE
        {
            HMODULE dwm = ::LoadLibraryW( L"dwmapi.dll" );
            return dwm ? reinterpret_cast<SET_ATTRIBUTE>( ::GetProcAddress( dwm, "DwmSetWindowAttribute" ) )
                       : nullptr;
        }();

        if( !setAttribute || !aHwnd )
            return;

        auto ref = []( const wxColour& c ) -> COLORREF { return RGB( c.Red(), c.Green(), c.Blue() ); };
        const COLORREF caption = ref( CONTINUITY_PALETTE::raised );
        const COLORREF text = ref( CONTINUITY_PALETTE::ink );
        const COLORREF border = ref( CONTINUITY_PALETTE::rule );

        // DWMWA_BORDER_COLOR, DWMWA_CAPTION_COLOR, DWMWA_TEXT_COLOR: Windows 11 and later.
        setAttribute( (HWND) aHwnd, 34, &border, sizeof( border ) );
        setAttribute( (HWND) aHwnd, 35, &caption, sizeof( caption ) );
        setAttribute( (HWND) aHwnd, 36, &text, sizeof( text ) );
    }
};
#endif


bool KIPLATFORM::APP::Init()
{
#if defined( _MSC_VER ) && defined( DEBUG )
    // wxWidgets turns on leak dumping in debug but its "flawed" and will falsely dump
    // for half a hour _CRTDBG_ALLOC_MEM_DF is the usual default for MSVC.
    _CrtSetDbgFlag( _CRTDBG_ALLOC_MEM_DF );
#endif

#if defined( DEBUG )
    // undo wxwidgets trying to hide errors
    SetErrorMode( 0 );
#else
    SetErrorMode( SEM_FAILCRITICALERRORS | SEM_NOOPENFILEERRORBOX );
#endif

    // remove CWD from the dll search paths
    // just the smallest of security tweaks as we do load DLLs on demand
    SetDllDirectory( wxT( "" ) );

    // Moves the CWD to the end of the search list for spawning processes
    SetSearchPathMode( BASE_SEARCH_PATH_ENABLE_SAFE_SEARCHMODE | BASE_SEARCH_PATH_PERMANENT );

    // In order to support GUI and CLI
    // Let's attach to console when it's possible, or allocate if requested.
    AttachConsole( wxGetEnv( wxS( "KICAD_ALLOC_CONSOLE" ), nullptr ) );

    // It may be useful to log up to traces in a console, but in Release builds the log level changes to Info
    // Also we have to force the active target to stderr or else it goes to the void
    bool forceLog = wxGetEnv( wxS( "KICAD_FORCE_CONSOLE_TRACE" ), nullptr );

    if( forceLog )
    {
        wxLog::EnableLogging( true );
#ifndef DEBUG
        wxLog::SetLogLevel( wxLOG_Trace );
#endif
        const int fd = _fileno( stderr );
        wxLog::SetActiveTarget( fd >= 0 && !_isatty( fd ) ? new wxLogStderr( stderr, wxConvUTF8 )
                                                       : new wxLogStderr );
    }

    return true;
}


void KIPLATFORM::APP::EnableDarkMode( bool aForce )
{
#if wxCHECK_VERSION( 3, 3, 0 )
    if( wxTheApp->MSWEnableDarkMode( aForce ? wxApp::DarkMode_Always : wxApp::DarkMode_Auto,
                                     new KICAD_DARK_MODE_SETTINGS() ) )
    {
        // Kept for the life of the process: a filter destroyed while still added asserts.
        static TITLE_BAR_FILTER* titleBars = nullptr;

        if( !titleBars )
        {
            titleBars = new TITLE_BAR_FILTER();
            wxEvtHandler::AddFilter( titleBars );
        }
    }
#endif
}


bool KIPLATFORM::APP::AttachConsole( bool aTryAlloc )
{
    if( ::AttachConsole( ATTACH_PARENT_PROCESS ) || ( aTryAlloc && ::AllocConsole() ) )
    {
        #if !defined( __MINGW32__ ) // These redirections create problems on mingw:
                                    // Nothing is printed to the console

        if( ::GetStdHandle( STD_INPUT_HANDLE ) != INVALID_HANDLE_VALUE )
        {
            freopen( "CONIN$", "r", stdin );
            setvbuf( stdin, NULL, _IONBF, 0 );
        }

        if( ::GetStdHandle( STD_OUTPUT_HANDLE ) != INVALID_HANDLE_VALUE )
        {
            freopen( "CONOUT$", "w", stdout );
            setvbuf( stdout, NULL, _IONBF, 0 );
        }

        if( ::GetStdHandle( STD_ERROR_HANDLE ) != INVALID_HANDLE_VALUE )
        {
            freopen( "CONOUT$", "w", stderr );
            setvbuf( stderr, NULL, _IONBF, 0 );
        }
        #endif

        std::ios::sync_with_stdio( true );

        std::wcout.clear();
        std::cout.clear();
        std::wcerr.clear();
        std::cerr.clear();
        std::wcin.clear();
        std::cin.clear();

        return true;
    }

    return false;
}


bool KIPLATFORM::APP::IsOperatingSystemUnsupported()
{
#if defined( PYTHON_VERSION_MAJOR ) && ( ( PYTHON_VERSION_MAJOR == 3 && PYTHON_VERSION_MINOR >= 8 ) \
             || PYTHON_VERSION_MAJOR > 3 )
    // Python 3.8 switched to Windows 8+ API, we do not support Windows 7 and will not
    // attempt to hack around it. A normal user will never get here because the Python DLL
    // is missing dependencies - and because it is not dynamically loaded, KiCad will not even
    // start without patching Python or its WinAPI dependency. This is just to create a nag dialog
    // for those who run patched Python and prevent them from submitting bug reports.
    return !IsWindows8OrGreater();
#else
    return false;
#endif
}


bool KIPLATFORM::APP::RegisterApplicationRestart( const wxString& aCommandLine )
{
    // Command line arguments with spaces require quotes.
    wxString restartCmd = wxS( "\"" ) + aCommandLine + wxS( "\"" );

    // Ensure we don't exceed the maximum allowable size
    if( restartCmd.length() > RESTART_MAX_CMD_LINE - 1 )
    {
        return false;
    }

    HRESULT hr = S_OK;

    hr = ::RegisterApplicationRestart( restartCmd.wc_str(), RESTART_NO_PATCH );

    return SUCCEEDED( hr );
}


bool KIPLATFORM::APP::UnregisterApplicationRestart()
{
    // Note, this isn't required to be used on Windows if you are just closing the program
    return SUCCEEDED( ::UnregisterApplicationRestart() );
}


bool KIPLATFORM::APP::SupportsShutdownBlockReason()
{
    return true;
}


void KIPLATFORM::APP::RemoveShutdownBlockReason( wxWindow* aWindow )
{
    // Destroys any block reason that may have existed
    ShutdownBlockReasonDestroy( aWindow->GetHandle() );
}


void KIPLATFORM::APP::SetShutdownBlockReason( wxWindow* aWindow, const wxString& aReason )
{
    // Sets up the pretty message on the shutdown page on why it's being "blocked"
    // This is used in conjunction with handling WM_QUERYENDSESSION (wxCloseEvent)
    // ShutdownBlockReasonCreate does not block by itself

    ShutdownBlockReasonDestroy( aWindow->GetHandle() ); // Destroys any existing or nonexisting reason

    ShutdownBlockReasonCreate( aWindow->GetHandle(), aReason.wc_str() );
}


void KIPLATFORM::APP::ForceTimerMessagesToBeCreatedIfNecessary()
{
    // Taken from https://devblogs.microsoft.com/oldnewthing/20191108-00/?p=103080
    MSG msg;
    PeekMessage( &msg, nullptr, WM_TIMER, WM_TIMER, PM_NOREMOVE );
}


void KIPLATFORM::APP::AddDynamicLibrarySearchPath( const wxString& aPath )
{
    SetDllDirectory( aPath.c_str() );
}


bool KIPLATFORM::APP::SetCurrentThreadPriority( bool aRaise )
{
    return SetThreadPriority( GetCurrentThread(),
                              aRaise ? THREAD_PRIORITY_ABOVE_NORMAL : THREAD_PRIORITY_BELOW_NORMAL )
           != 0;
}


void KIPLATFORM::APP::TerminateWithCode( int aExitCode )
{
    TerminateProcess( GetCurrentProcess(), static_cast<UINT>( aExitCode ) );

    // Not reached unless the call is refused
    ExitProcess( static_cast<UINT>( aExitCode ) );
}
