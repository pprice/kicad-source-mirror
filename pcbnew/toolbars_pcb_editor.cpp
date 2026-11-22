/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2012 Jean-Pierre Charras, jp.charras at wanadoo.fr
 * Copyright (C) 2012 SoftPLC Corporation, Dick Hollenbeck <dick@softplc.com>
 * Copyright (C) 2012 Wayne Stambaugh <stambaughw@gmail.com>
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

#include <memory>
#include <string>

#include <advanced_config.h>
#include <api/api_plugin_manager.h>
#include <bitmaps.h>
#include <board.h>
#include <board_design_settings.h>
#include <eda_text.h>
#include <footprint.h>
#include <pcb_field.h>
#include <kiface_base.h>
#include <kiplatform/ui.h>
#include <macros.h>
#include <pcb_edit_frame.h>
#include <pcb_field.h>
#include <pcb_layer_box_selector.h>
#include <pcbnew_id.h>
#include <pcbnew_settings.h>
#include <eda_text.h>
#include <pcb_field.h>
#include <pgm_base.h>
#include <settings/common_settings.h>
#include <router/pns_routing_settings.h>
#include <router/router_tool.h>
#include <settings/color_settings.h>
#include <tool/action_menu.h>
#include <tool/action_toolbar.h>
#include <tool/actions.h>
#include <tool/common_tools.h>
#include <tool/tool_manager.h>
#include <tool/ui/toolbar_context_menu_registry.h>
#include <tools/pcb_actions.h>
#include <tools/pcb_selection_tool.h>
#include <widgets/appearance_controls.h>
#include <widgets/pcb_design_block_pane.h>
#include <widgets/layer_box_selector.h>
#include <widgets/layer_presentation.h>
#include <widgets/pcb_properties_panel.h>
#include <widgets/net_inspector_panel.h>
#include <widgets/pcb_search_pane.h>
#include <widgets/wx_aui_utils.h>
#include <wx/wupdlock.h>
#include <wx/combobox.h>
#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/timer.h>
#include <connectivity/connectivity_data.h>
#include <pcb_marker.h>
#include <toolbars_pcb_editor.h>
#include <settings/settings_manager.h>
#include <eda_text.h>
#include <tools/pcb_control.h>
#include <pcb_draw_panel_gal.h>
#include <drawing_sheet/ds_proxy_view_item.h>

/* Data to build the layer pair indicator button */
static wxBitmapBundle LayerPairBitmap;


/**
 * Continuity's problem chips (docs/kicad-look.md, "Important first"): how many connections are
 * still to route and how many markers the last check left, each a pill in the problem colour,
 * and nothing at all at zero. A click opens the check.
 */
class PROBLEM_CHIPS : public wxControl
{
public:
    PROBLEM_CHIPS( wxWindow* aParent, PCB_EDIT_FRAME* aFrame ) :
            wxControl( aParent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE ),
            m_frame( aFrame ),
            m_timer( this )
    {
        SetBackgroundStyle( wxBG_STYLE_PAINT );
        SetCursor( wxCursor( wxCURSOR_HAND ) );
        SetToolTip( _( "Open the design rules check" ) );

        wxFont font = GetFont();
        font.SetWeight( wxFONTWEIGHT_BOLD );
        SetFont( font );

        // Room for both chips at three digits, so the toolbar never has to lay itself out again.
        wxClientDC dc( this );
        dc.SetFont( GetFont() );
        wxSize text = dc.GetTextExtent( chipText( 999, _( "unrouted" ) ) + chipText( 999, _( "from the check" ) ) );
        SetMinSize( wxSize( text.x + FromDIP( 40 ), text.y + FromDIP( 12 ) ) );
        SetSize( GetMinSize() );

        Bind( wxEVT_PAINT, &PROBLEM_CHIPS::onPaint, this );
        Bind( wxEVT_LEFT_UP, &PROBLEM_CHIPS::onClick, this );
        Bind( wxEVT_TIMER, [this]( wxTimerEvent& ) { recount(); } );
        m_timer.Start( 1000 );
    }

    ~PROBLEM_CHIPS() override { m_timer.Stop(); }

private:
    static wxString chipText( unsigned aCount, const wxString& aWhat )
    {
        return wxString::Format( wxS( "%u %s" ), aCount, aWhat );
    }

    void recount()
    {
        BOARD* board = m_frame ? m_frame->GetBoard() : nullptr;

        if( !board || !IsShownOnScreen() )
            return;

        unsigned unrouted = board->GetConnectivity() ? board->GetConnectivity()->GetUnconnectedCount( true ) : 0;
        unsigned markers = 0;

        for( PCB_MARKER* marker : board->Markers() )
        {
            if( !marker->IsExcluded() && marker->GetSeverity() != RPT_SEVERITY_EXCLUSION )
                ++markers;
        }

        if( unrouted != m_unrouted || markers != m_markers )
        {
            m_unrouted = unrouted;
            m_markers = markers;
            Refresh();
        }
    }

    void onClick( wxMouseEvent& aEvent )
    {
        if( ( m_unrouted || m_markers ) && m_frame )
            m_frame->GetToolManager()->RunAction( PCB_ACTIONS::runDRC );
    }

    void onPaint( wxPaintEvent& )
    {
        wxAutoBufferedPaintDC dc( this );
        dc.SetBackground( wxBrush( wxSystemSettings::GetColour( wxSYS_COLOUR_BTNFACE ) ) );
        dc.Clear();

        std::unique_ptr<wxGraphicsContext> gc( wxGraphicsContext::Create( dc ) );

        if( !gc )
            return;

        // As the mock draws them: a hairline pill in the problem colour, the count in it, no fill.
        const wxColour problem( 255, 92, 191 );   // Continuity's --c-problem
        const double   line = FromDIP( 1.25 );

        gc->SetFont( GetFont(), problem );

        const wxSize size = GetClientSize();
        const double padX = FromDIP( 8 ), gap = FromDIP( 6 ), height = size.y - FromDIP( 8 );
        const auto   chips = { std::make_pair( m_unrouted, _( "unrouted" ) ),
                               std::make_pair( m_markers, _( "from the check" ) ) };

        // The chips stand at the toolbar's right end, so they are drawn from the control's right.
        double x = size.x - FromDIP( 4 );

        for( const auto& [count, what] : chips )
        {
            if( !count )
                continue;

            double tw, th;
            gc->GetTextExtent( chipText( count, what ), &tw, &th );
            x -= tw + 2 * padX + gap;
        }

        x += gap;

        for( const auto& [count, what] : chips )
        {
            if( !count )
                continue;

            wxString text = chipText( count, what );
            double   tw, th;
            gc->GetTextExtent( text, &tw, &th );

            const double w = tw + 2 * padX;
            const double y = ( size.y - height ) / 2;

            gc->SetPen( gc->CreatePen( wxGraphicsPenInfo( problem, line ) ) );
            gc->SetBrush( *wxTRANSPARENT_BRUSH );
            gc->DrawRoundedRectangle( x + line / 2, y + line / 2, w - line, height - line, FromDIP( 4 ) );
            gc->DrawText( text, x + padX, y + ( height - th ) / 2 );
            x += w + gap;
        }
    }

    PCB_EDIT_FRAME* m_frame;
    wxTimer         m_timer;
    unsigned        m_unrouted = 0;
    unsigned        m_markers = 0;
};


void PCB_EDIT_FRAME::PrepareLayerIndicator( bool aForceRebuild )
{
    COLOR4D    top_color, bottom_color, background_color;
    bool       change = aForceRebuild;

    int icon_size = Pgm().GetCommonSettings()->m_Appearance.toolbar_icon_size;

    if( m_prevIconVal.previous_icon_size != icon_size )
    {
        m_prevIconVal.previous_icon_size = icon_size;
        change = true;
    }

    top_color = GetColorSettings()->GetColor( GetScreen()->m_Route_Layer_TOP );

    if( m_prevIconVal.previous_Route_Layer_TOP_color != top_color )
    {
        m_prevIconVal.previous_Route_Layer_TOP_color = top_color;
        change = true;
    }

    bottom_color = GetColorSettings()->GetColor( GetScreen()->m_Route_Layer_BOTTOM );

    if( m_prevIconVal.previous_Route_Layer_BOTTOM_color != bottom_color )
    {
        m_prevIconVal.previous_Route_Layer_BOTTOM_color = bottom_color;
        change = true;
    }

    if( change || !LayerPairBitmap.IsOk() )
    {
        LayerPairBitmap = LAYER_PRESENTATION::CreateLayerPairIcon( top_color, bottom_color, icon_size );

        if( m_tbTopAux )
        {
            m_tbTopAux->SetToolBitmap( PCB_ACTIONS::selectLayerPair, LayerPairBitmap );
            m_tbTopAux->Refresh();
        }
    }
}


ACTION_TOOLBAR_CONTROL PCB_ACTION_TOOLBAR_CONTROLS::trackWidth( "control.PCBTrackWidth",
                                                                _( "Track width selector" ),
                                                                _( "Control to select the track width" ),
                                                                { FRAME_PCB_EDITOR } );
ACTION_TOOLBAR_CONTROL PCB_ACTION_TOOLBAR_CONTROLS::viaDiameter( "control.PCBViaDia",
                                                                 _( "Via diameter selector" ),
                                                                 _( "Control to select the via diameter" ),
                                                                 { FRAME_PCB_EDITOR } );
ACTION_TOOLBAR_CONTROL PCB_ACTION_TOOLBAR_CONTROLS::viaStack( "control.PCBViaStack", _( "Microvia stack selector" ),
                                                              _( "Control to select the microvia stack preset" ),
                                                              { FRAME_PCB_EDITOR } );
ACTION_TOOLBAR_CONTROL PCB_ACTION_TOOLBAR_CONTROLS::currentVariant( "control.PCBCurrentVariant",
                                                                    _( "Current variant" ),
                                                                    _( "Control to select the current variant" ),
                                                                    { FRAME_PCB_EDITOR } );
ACTION_TOOLBAR_CONTROL PCB_ACTION_TOOLBAR_CONTROLS::problems( "control.PCBProblems",
                                                              _( "Problems" ),
                                                              _( "Connections still to route and markers the "
                                                                 "last check left" ),
                                                              { FRAME_PCB_EDITOR } );


std::optional<TOOLBAR_CONFIGURATION> PCB_EDIT_TOOLBAR_SETTINGS::DefaultToolbarConfig( TOOLBAR_LOC aToolbar )
{
    TOOLBAR_CONFIGURATION config;

    // clang-format off
    switch( aToolbar )
    {
    // Continuity: the toolbars are cut to the mock's (prototypes/kicad-look/mock.html in
    // pprice/continuity): what is used while placing and routing, one row along the top with the
    // track, via and layer choices in it. Everything else is in the menus, and Preferences,
    // Toolbars puts any of it back.
    case TOOLBAR_LOC::LEFT:
        config.AppendAction( ACTIONS::toggleGrid )
              .WithContextMenu(
                  []( TOOL_MANAGER* aMgr ) -> std::unique_ptr<ACTION_MENU>
                  {
                      PCB_SELECTION_TOOL* selTool = aMgr->GetTool<PCB_SELECTION_TOOL>();
                      std::unique_ptr<ACTION_MENU> menu =
                              std::make_unique<ACTION_MENU>( false, selTool );

                      menu->Add( ACTIONS::gridProperties );
                      menu->Add( ACTIONS::gridOrigin );

                      return menu;
                  } )
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Units" ) )
                            .AddAction( ACTIONS::millimetersUnits )
                            .AddAction( ACTIONS::inchesUnits )
                            .AddAction( ACTIONS::milsUnits ) )
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Crosshair modes" ) )
                            .AddAction( ACTIONS::cursorSmallCrosshairs )
                            .AddAction( ACTIONS::cursorFullCrosshairs )
                            .AddAction( ACTIONS::cursor45Crosshairs ) )
              .AppendAction( PCB_ACTIONS::showRatsnest )
              .AppendAction( ACTIONS::highContrastMode );

        config.AppendSeparator()
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Zone display" ) )
                            .AddAction( PCB_ACTIONS::zoneDisplayFilled )
                            .AddAction( PCB_ACTIONS::zoneDisplayOutline ) )
              .AppendAction( PCB_ACTIONS::padDisplayMode )
              .AppendAction( PCB_ACTIONS::trackDisplayMode );

        break;

    case TOOLBAR_LOC::RIGHT:
        config.AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Selection modes" ) )
                            .AddAction( ACTIONS::selectSetRect )
                            .AddAction( ACTIONS::selectSetLasso ) )
              .AppendAction( PCB_ACTIONS::localRatsnestTool );

        config.AppendSeparator()
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Track routing tools" ) )
                            .AddAction( PCB_ACTIONS::routeSingleTrack )
                            .AddAction( PCB_ACTIONS::routeDiffPair )
                            .AddContextMenu(
                                []( TOOL_MANAGER* aMgr ) -> std::unique_ptr<ACTION_MENU>
                                {
                                    PCB_SELECTION_TOOL* selTool = aMgr->GetTool<PCB_SELECTION_TOOL>();
                                    std::unique_ptr<ACTION_MENU> menu =
                                            std::make_unique<ACTION_MENU>( false, selTool );

                                    menu->Add( PCB_ACTIONS::routerHighlightMode, ACTION_MENU::CHECK );
                                    menu->Add( PCB_ACTIONS::routerShoveMode, ACTION_MENU::CHECK );
                                    menu->Add( PCB_ACTIONS::routerWalkaroundMode, ACTION_MENU::CHECK );
                                    menu->AppendSeparator();
                                    menu->Add( PCB_ACTIONS::routerSettingsDialog );

                                    return menu;
                                } ) )
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Track tuning tools" ) )
                            .AddAction( PCB_ACTIONS::tuneSingleTrack )
                            .AddAction( PCB_ACTIONS::tuneDiffPair )
                            .AddAction( PCB_ACTIONS::tuneSkew ) )
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Via tools" ) )
                            .AddAction( PCB_ACTIONS::drawVia )
                            .AddAction( PCB_ACTIONS::placeViaStack )
                            .AddAction( PCB_ACTIONS::drawViaStitchArea ) );

        config.AppendSeparator()
              .AppendAction( PCB_ACTIONS::drawZone )
              .WithContextMenu(
                  []( TOOL_MANAGER* aMgr ) -> std::unique_ptr<ACTION_MENU>
                  {
                      PCB_SELECTION_TOOL* selTool = aMgr->GetTool<PCB_SELECTION_TOOL>();
                      std::unique_ptr<ACTION_MENU> menu =
                              std::make_unique<ACTION_MENU>( false, selTool );

                      menu->Add( PCB_ACTIONS::zoneFillAll );
                      menu->Add( PCB_ACTIONS::zoneUnfillAll );
                      menu->AppendSeparator();
                      menu->Add( PCB_ACTIONS::zonesManager );

                      return menu;
                  } )
              .AppendAction( PCB_ACTIONS::drawRuleArea );

        config.AppendSeparator()
              .AppendAction( PCB_ACTIONS::drawLine )
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Arc" ) )
                            .AddAction( PCB_ACTIONS::drawArcCenter )
                            .AddAction( PCB_ACTIONS::drawArcStartEndMid )
                            .AddAction( PCB_ACTIONS::drawArcStartEndCenter )
                            .AddAction( PCB_ACTIONS::drawArcTangent )
                            .AddAction( PCB_ACTIONS::drawArcStartDirEnd )
                            .AddAction( PCB_ACTIONS::drawEllipseArc )
                            .AddContextMenu(
                                []( TOOL_MANAGER* aMgr ) -> std::unique_ptr<ACTION_MENU>
                                {
                                    PCB_SELECTION_TOOL* selTool = aMgr->GetTool<PCB_SELECTION_TOOL>();
                                    std::unique_ptr<ACTION_MENU> menu =
                                            std::make_unique<ACTION_MENU>( false, selTool );

                                    menu->Add( ACTIONS::pointEditorArcKeepCenter, ACTION_MENU::CHECK );
                                    menu->Add( ACTIONS::pointEditorArcKeepEndpoint, ACTION_MENU::CHECK );
                                    menu->Add( ACTIONS::pointEditorArcKeepRadius, ACTION_MENU::CHECK );

                                    return menu;
                                } ) )
              .AppendAction( PCB_ACTIONS::drawRectangle )
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Circle" ) )
                            .AddAction( PCB_ACTIONS::drawCircle )
                            .AddAction( PCB_ACTIONS::drawEllipse ) )
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Text objects" ) )
                            .AddAction( PCB_ACTIONS::placeText )
                            .AddAction( PCB_ACTIONS::drawTextBox ) )
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "Dimension objects" ) )
                            .AddAction( PCB_ACTIONS::drawOrthogonalDimension )
                            .AddAction( PCB_ACTIONS::drawAlignedDimension )
                            .AddAction( PCB_ACTIONS::drawCenterDimension )
                            .AddAction( PCB_ACTIONS::drawRadialDimension )
                            .AddAction( PCB_ACTIONS::drawLeader ) );

        config.AppendSeparator()
              .AppendAction( ACTIONS::measureTool )
              .AppendGroup( TOOLBAR_GROUP_CONFIG( _( "PCB origins and points" ) )
                            .AddAction( ACTIONS::gridSetOrigin )
                            .AddAction( PCB_ACTIONS::drillOrigin ) );

        break;

    case TOOLBAR_LOC::TOP_MAIN:
        if( Kiface().IsSingle() )
        {
            config.AppendAction( ACTIONS::doNew );
            config.AppendAction( ACTIONS::open );
        }

        config.AppendAction( ACTIONS::save );

        config.AppendSeparator()
              .AppendAction( ACTIONS::undo )
              .AppendAction( ACTIONS::redo );

        config.AppendSeparator()
              .AppendAction( ACTIONS::find )
              .AppendAction( ACTIONS::zoomInCenter )
              .AppendAction( ACTIONS::zoomOutCenter )
              .AppendAction( ACTIONS::zoomFitScreen );

        config.AppendSeparator()
              .AppendAction( PCB_ACTIONS::showNetInspector )
              .AppendAction( PCB_ACTIONS::runDRC )
              .AppendAction( ACTIONS::show3DViewer )
              .AppendControl( ACTION_TOOLBAR_CONTROLS::ipcScripting );

        config.AppendSeparator()
              .AppendControl( PCB_ACTION_TOOLBAR_CONTROLS::trackWidth )
              .AppendControl( PCB_ACTION_TOOLBAR_CONTROLS::viaDiameter )
              .AppendControl( ACTION_TOOLBAR_CONTROLS::layerSelector );

        // The chips at the toolbar's right end, as the mock has them.
        config.AppendSpacer( -1 );
        config.AppendControl( PCB_ACTION_TOOLBAR_CONTROLS::problems );

        break;

    case TOOLBAR_LOC::TOP_AUX:
        // Continuity: no second row; its choices that are used are on the one above.
        return std::nullopt;
    }

    // clang-format on
    return config;
}


void PCB_EDIT_FRAME::configureToolbars()
{
    PCB_BASE_EDIT_FRAME::configureToolbars();

    // Box to display and choose track widths
    auto trackWidthSelectorFactory =
            [this]( ACTION_TOOLBAR* aToolbar )
            {
                if( !m_SelTrackWidthBox )
                {
                    m_SelTrackWidthBox = new wxChoice( aToolbar, ID_AUX_TOOLBAR_PCB_TRACK_WIDTH,
                                                       wxDefaultPosition, wxDefaultSize, 0, nullptr );
                }

                m_SelTrackWidthBox->SetToolTip( _( "Select the default width for new tracks. Note that this "
                                                   "width can be overridden by the board minimum width, or by "
                                                   "the width of an existing track if the 'Use Existing Track "
                                                   "Width' feature is enabled." ) );

                UpdateTrackWidthSelectBox( m_SelTrackWidthBox, true, true );

                aToolbar->Add( m_SelTrackWidthBox );
            };

    RegisterCustomToolbarControlFactory( PCB_ACTION_TOOLBAR_CONTROLS::trackWidth, trackWidthSelectorFactory );


    // Box to display and choose vias diameters
    auto viaDiaSelectorFactory =
            [this]( ACTION_TOOLBAR* aToolbar )
            {
                if( !m_SelViaSizeBox )
                {
                    m_SelViaSizeBox = new wxChoice( aToolbar, ID_AUX_TOOLBAR_PCB_VIA_SIZE,
                                                    wxDefaultPosition, wxDefaultSize, 0, nullptr );
                }

                UpdateViaSizeSelectBox( m_SelViaSizeBox, true, true );
                aToolbar->Add( m_SelViaSizeBox );
            };

    RegisterCustomToolbarControlFactory( PCB_ACTION_TOOLBAR_CONTROLS::viaDiameter, viaDiaSelectorFactory );

    // Box to display and choose via stack presets
    auto viaStackSelectorFactory = [this]( ACTION_TOOLBAR* aToolbar )
    {
        if( !m_SelViaStackBox )
        {
            m_SelViaStackBox = new wxChoice( aToolbar, ID_AUX_TOOLBAR_PCB_VIA_STACK, wxDefaultPosition, wxDefaultSize,
                                             0, nullptr );
        }

        m_SelViaStackBox->SetToolTip( _( "Select the microvia stack preset placed by the "
                                         "microvia stack tool." ) );

        UpdateViaStackSelectBox( m_SelViaStackBox );
        aToolbar->Add( m_SelViaStackBox );
    };

    RegisterCustomToolbarControlFactory( PCB_ACTION_TOOLBAR_CONTROLS::viaStack, viaStackSelectorFactory );

    // Variant selection drop down control on main tool bar
    auto variantSelectionCtrlFactory =
            [this]( ACTION_TOOLBAR* aToolbar )
            {
                if( !m_CurrentVariantCtrl )
                {
                    m_CurrentVariantCtrl = new wxChoice( aToolbar, ID_AUX_TOOLBAR_PCB_VARIANT_SELECT,
                                                         wxDefaultPosition, wxDefaultSize, 0, nullptr );
                }

                m_CurrentVariantCtrl->SetToolTip( _( "Select the current variant to display and edit." ) );

                UpdateVariantSelectionCtrl();

                aToolbar->Add( m_CurrentVariantCtrl );
            };

    RegisterCustomToolbarControlFactory( PCB_ACTION_TOOLBAR_CONTROLS::currentVariant, variantSelectionCtrlFactory );

    // IPC/Scripting plugin control
    // TODO (ISM): Clean this up to make IPC actions just normal tool actions to get rid of this entire
    // control
    auto pluginControlFactory =
            [this]( ACTION_TOOLBAR* aToolbar )
            {
                bool haveApiPlugins = Pgm().GetCommonSettings()->m_Api.enable_server
                                        && !Pgm().GetPluginManager().GetActionsForScope( PluginActionScope() ).empty();

                if( haveApiPlugins )
                {
                    aToolbar->AddScaledSeparator( aToolbar->GetParent() );
                    AddApiPluginTools( aToolbar );
                }
            };

    RegisterCustomToolbarControlFactory( ACTION_TOOLBAR_CONTROLS::ipcScripting, pluginControlFactory );

    auto problemChipsFactory =
            [this]( ACTION_TOOLBAR* aToolbar )
            {
                aToolbar->Add( new PROBLEM_CHIPS( aToolbar, this ) );
            };

    RegisterCustomToolbarControlFactory( PCB_ACTION_TOOLBAR_CONTROLS::problems, problemChipsFactory );
}


void PCB_EDIT_FRAME::ClearToolbarControl( int aId )
{
    PCB_BASE_EDIT_FRAME::ClearToolbarControl( aId );

    switch( aId )
    {
    case ID_AUX_TOOLBAR_PCB_TRACK_WIDTH:    m_SelTrackWidthBox = nullptr;   break;
    case ID_AUX_TOOLBAR_PCB_VIA_SIZE:       m_SelViaSizeBox = nullptr;      break;
    case ID_AUX_TOOLBAR_PCB_VIA_STACK: m_SelViaStackBox = nullptr; break;
    case ID_AUX_TOOLBAR_PCB_VARIANT_SELECT: m_CurrentVariantCtrl = nullptr; break;
    }
}


void PCB_EDIT_FRAME::UpdateVariantSelectionCtrl()
{
    if( !m_CurrentVariantCtrl )
        return;

    if( !GetBoard() )
        return;

    wxArrayString variantNames = GetBoard()->GetVariantNamesForUI();

    m_CurrentVariantCtrl->Set( variantNames );

    int selectionIndex = 0;
    wxString currentVariant = GetBoard()->GetCurrentVariant();

    if( !currentVariant.IsEmpty() )
    {
        int foundIndex = m_CurrentVariantCtrl->FindString( currentVariant );

        if( foundIndex != wxNOT_FOUND )
            selectionIndex = foundIndex;
    }

    if( m_CurrentVariantCtrl->GetCount() > 0 )
        m_CurrentVariantCtrl->SetSelection( selectionIndex );
}


void PCB_EDIT_FRAME::SetCurrentVariant( const wxString& aVariantName )
{
    GetBoard()->SetCurrentVariant( aVariantName );
    UpdateVariantSelectionCtrl();

    if( PCB_DRAW_PANEL_GAL* canvas = dynamic_cast<PCB_DRAW_PANEL_GAL*>( GetCanvas() ) )
    {
        DS_PROXY_VIEW_ITEM* ds = canvas->GetDrawingSheet();

        if( ds )
        {
            wxString currentVariant = GetBoard()->GetCurrentVariant();
            wxString variantDesc = GetBoard()->GetVariantDescription( currentVariant );

            ds->SetVariantName( currentVariant.ToStdString() );
            ds->SetVariantDesc( variantDesc.ToStdString() );
        }
    }

    // Refresh the view and properties panel to show the new variant state
    UpdateProperties();

    GetCanvas()->GetView()->UpdateAllItemsConditionally(
            [&]( KIGFX::VIEW_ITEM* aItem ) -> int
            {
                if( EDA_TEXT* text = dynamic_cast<EDA_TEXT*>( aItem ) )
                {
                    if( text->HasTextVars() )
                    {
                        text->ClearRenderCache();
                        text->ClearBoundingBoxCache();
                        return KIGFX::GEOMETRY | KIGFX::REPAINT;
                    }
                }

                if( PCB_FIELD* field = dynamic_cast<PCB_FIELD*>( aItem ) )
                {
                    if( FOOTPRINT* fp = field->GetParentFootprint() )
                    {
                        if( !fp->GetVariants().empty() )
                        {
                            field->ClearRenderCache();
                            field->ClearBoundingBoxCache();
                            return KIGFX::GEOMETRY | KIGFX::REPAINT;
                        }
                    }
                }

                return 0;
            } );

    GetCanvas()->Refresh();

    Update3DView( true, GetPcbNewSettings()->m_Display.m_Live3DRefresh );
    m_toolManager->ProcessEvent( EVENTS::SelectedItemsModified );
}


void PCB_EDIT_FRAME::onVariantSelected( wxCommandEvent& aEvent )
{
    if( !m_CurrentVariantCtrl )
        return;

    int selection = m_CurrentVariantCtrl->GetSelection();

    if( selection == wxNOT_FOUND || selection == 0 )
    {
        // "<Default>" selected - clear the current variant
        SetCurrentVariant( wxEmptyString );
    }
    else
    {
        wxString selectedVariant = m_CurrentVariantCtrl->GetString( selection );
        SetCurrentVariant( selectedVariant );
    }
}


static wxString ComboBoxUnits( EDA_UNITS aUnits, double aValue, bool aIncludeLabel = true )
{
    wxString      text;
    const wxChar* format;

    switch( aUnits )
    {
    default:
        wxASSERT_MSG( false, wxT( "Invalid unit" ) );
        KI_FALLTHROUGH;
    case EDA_UNITS::UNSCALED: format = wxT( "%.0f" ); break;
    case EDA_UNITS::MM:       format = wxT( "%.3f" ); break;
    case EDA_UNITS::MILS:     format = wxT( "%.2f" ); break;
    case EDA_UNITS::INCH:     format = wxT( "%.5f" ); break;
    }

    text.Printf( format, EDA_UNIT_UTILS::UI::ToUserUnit( pcbIUScale, aUnits, aValue ) );

    if( aIncludeLabel )
        text += EDA_UNIT_UTILS::GetText( aUnits, EDA_DATA_TYPE::DISTANCE );

    return text;
}


void PCB_EDIT_FRAME::UpdateTrackWidthSelectBox( wxChoice* aTrackWidthSelectBox, bool aShowNetclass,
                                                bool aShowEdit )
{
    if( aTrackWidthSelectBox == nullptr )
        return;

    EDA_UNITS primaryUnit;
    EDA_UNITS secondaryUnit;

    GetUnitPair( primaryUnit, secondaryUnit );

    wxString msg;

    aTrackWidthSelectBox->Clear();

    if( aShowNetclass )
        aTrackWidthSelectBox->Append( _( "Track: use netclass width" ) );

    for( unsigned ii = 1; ii < GetDesignSettings().m_TrackWidthList.size(); ii++ )
    {
        int size = GetDesignSettings().m_TrackWidthList[ii];

        msg.Printf( _( "Track: %s (%s)" ), ComboBoxUnits( primaryUnit, size ),
                                           ComboBoxUnits( secondaryUnit, size ) );

        aTrackWidthSelectBox->Append( msg );
    }

    if( aShowEdit )
    {
        aTrackWidthSelectBox->Append( wxT( "---" ) );
        aTrackWidthSelectBox->Append( _( "Edit Pre-defined Sizes..." ) );
    }

    if( GetDesignSettings().GetTrackWidthIndex() >= (int) GetDesignSettings().m_TrackWidthList.size() )
        GetDesignSettings().SetTrackWidthIndex( 0 );

    // GetDesignSettings().GetTrackWidthIndex() can be < 0 if no board loaded
    // So in this case select the first select box item available (use netclass)
    aTrackWidthSelectBox->SetSelection( std::max( 0, GetDesignSettings().GetTrackWidthIndex() ) );
}


void PCB_EDIT_FRAME::UpdateViaSizeSelectBox( wxChoice* aViaSizeSelectBox, bool aShowNetclass,
                                             bool aShowEdit )
{
    if( aViaSizeSelectBox == nullptr )
        return;

    aViaSizeSelectBox->Clear();

    COMMON_TOOLS* cmnTool   = m_toolManager->GetTool<COMMON_TOOLS>();
    EDA_UNITS primaryUnit   = GetUserUnits();
    EDA_UNITS secondaryUnit = EDA_UNITS::MILS;

    if( EDA_UNIT_UTILS::IsImperialUnit( primaryUnit ) )
        secondaryUnit = cmnTool ? cmnTool->GetLastMetricUnits() : EDA_UNITS::MM;
    else
        secondaryUnit = cmnTool ? cmnTool->GetLastImperialUnits() : EDA_UNITS::MILS;

    if( aShowNetclass )
        aViaSizeSelectBox->Append( _( "Via: use netclass sizes" ) );

    for( unsigned ii = 1; ii < GetDesignSettings().m_ViasDimensionsList.size(); ii++ )
    {
        VIA_DIMENSION viaDimension = GetDesignSettings().m_ViasDimensionsList[ii];
        wxString      msg, priStr, secStr;

        double diam = viaDimension.m_Diameter;
        double hole = viaDimension.m_Drill;

        if( hole > 0 )
        {
            priStr = ComboBoxUnits( primaryUnit, diam, false ) + wxT( " / " )
                        + ComboBoxUnits( primaryUnit, hole, true );
            secStr = ComboBoxUnits( secondaryUnit, diam, false ) + wxT( " / " )
                        + ComboBoxUnits( secondaryUnit, hole, true );
        }
        else
        {
            priStr = ComboBoxUnits( primaryUnit, diam, true );
            secStr = ComboBoxUnits( secondaryUnit, diam, true );
        }

        msg.Printf( _( "Via: %s (%s)" ), priStr, secStr );

        aViaSizeSelectBox->Append( msg );
    }

    if( aShowEdit )
    {
        aViaSizeSelectBox->Append( wxT( "---" ) );
        aViaSizeSelectBox->Append( _( "Edit Pre-defined Sizes..." ) );
    }

    if( GetDesignSettings().GetViaSizeIndex() >= (int) GetDesignSettings().m_ViasDimensionsList.size() )
        GetDesignSettings().SetViaSizeIndex( 0 );

    // GetDesignSettings().GetViaSizeIndex() can be < 0 if no board loaded
    // So in this case select the first select box item available (use netclass)
    aViaSizeSelectBox->SetSelection( std::max( 0, GetDesignSettings().GetViaSizeIndex() ) );
}


void PCB_EDIT_FRAME::UpdateViaStackSelectBox( wxChoice* aViaStackSelectBox )
{
    if( aViaStackSelectBox == nullptr )
        return;

    aViaStackSelectBox->Clear();

    const std::vector<VIA_STACK_PRESET>& presets = GetDesignSettings().m_ViaStackPresets;

    if( presets.empty() )
        aViaStackSelectBox->Append( _( "Microvia stack: none defined" ) );

    for( const VIA_STACK_PRESET& preset : presets )
        aViaStackSelectBox->Append( wxString::Format( _( "Microvia stack: %s" ), preset.m_Name ) );

    aViaStackSelectBox->Append( wxT( "---" ) );
    aViaStackSelectBox->Append( _( "Edit Pre-defined Microvia Stacks..." ) );

    int idx = GetDesignSettings().GetViaStackIndex();

    if( idx < 0 || idx >= (int) presets.size() )
    {
        idx = 0;
        GetDesignSettings().SetViaStackIndex( 0 );
    }

    aViaStackSelectBox->SetSelection( presets.empty() ? 0 : idx );
}


void PCB_EDIT_FRAME::ReCreateLayerBox( bool aForceResizeToolbar )
{
    // Continuity: the layer box may be on the main toolbar, with no aux toolbar at all.
    if( m_SelLayerBox == nullptr )
        return;

    m_SelLayerBox->SetToolTip( _( "+/- to switch" ) );
    m_SelLayerBox->Resync();

    if( aForceResizeToolbar )
        UpdateToolbarControlSizes();
}


void PCB_EDIT_FRAME::ToggleLayersManager()
{
    PCBNEW_SETTINGS* settings      = GetPcbNewSettings();
    wxAuiPaneInfo&   layersManager = m_auimgr.GetPane( AppearancePanelName() );
    wxAuiPaneInfo&   selectionFilter = m_auimgr.GetPane( "SelectionFilter" );

    // show auxiliary Vertical layers and visibility manager toolbar
    m_ShowLayerManagerTools = layersManager.IsShown();

    m_ShowLayerManagerTools = !m_ShowLayerManagerTools;

    layersManager.Show( m_ShowLayerManagerTools );
    selectionFilter.Show( m_ShowLayerManagerTools );

    if( m_ShowLayerManagerTools )
    {
        SetAuiPaneSize( m_auimgr, layersManager, settings->m_AuiPanels.right_panel_width, -1 );
    }
    else
    {
        settings->m_AuiPanels.right_panel_width = m_appearancePanel->GetSize().x;
        m_auimgr.Update();
    }
}


void PCB_EDIT_FRAME::ToggleNetInspector()
{
    PCBNEW_SETTINGS* settings          = GetPcbNewSettings();
    wxAuiPaneInfo&   netInspectorPanel = m_auimgr.GetPane( NetInspectorPanelName() );

    m_ShowNetInspector = netInspectorPanel.IsShown();

    m_ShowNetInspector = !m_ShowNetInspector;

    netInspectorPanel.Show( m_ShowNetInspector );

    if( m_ShowNetInspector )
    {
        SetAuiPaneSize( m_auimgr, netInspectorPanel, settings->m_AuiPanels.net_inspector_width, -1 );
        m_netInspectorPanel->OnShowPanel();
    }
    else
    {
        m_netInspectorPanel->SaveSettings();
        settings->m_AuiPanels.net_inspector_width = m_netInspectorPanel->GetSize().x;
        m_auimgr.Update();
    }
}


void PCB_EDIT_FRAME::TogglePane( const wxString& aName )
{
    wxAuiPaneInfo& pane = m_auimgr.GetPane( aName );

    if( !pane.IsOk() )
        return;

    pane.Show( !pane.IsShown() );
    m_auimgr.Update();
}


void PCB_EDIT_FRAME::ToggleSearch()
{
    PCBNEW_SETTINGS* settings = GetPcbNewSettings();

    // Ensure m_ShowSearch is up to date (the pane can be closed outside the menu)
    m_ShowSearch = m_auimgr.GetPane( SearchPaneName() ).IsShown();

    m_ShowSearch = !m_ShowSearch;

    wxAuiPaneInfo& searchPaneInfo = m_auimgr.GetPane( SearchPaneName() );
    searchPaneInfo.Show( m_ShowSearch );

    if( m_ShowSearch )
    {
        searchPaneInfo.Direction( settings->m_AuiPanels.search_panel_dock_direction );

        if( settings->m_AuiPanels.search_panel_dock_direction == wxAUI_DOCK_TOP
            || settings->m_AuiPanels.search_panel_dock_direction == wxAUI_DOCK_BOTTOM )
        {
            SetAuiPaneSize( m_auimgr, searchPaneInfo,
                            -1, settings->m_AuiPanels.search_panel_height );
        }
        else if( settings->m_AuiPanels.search_panel_dock_direction == wxAUI_DOCK_LEFT
                 || settings->m_AuiPanels.search_panel_dock_direction == wxAUI_DOCK_RIGHT )
        {
            SetAuiPaneSize( m_auimgr, searchPaneInfo,
                            settings->m_AuiPanels.search_panel_width, -1 );
        }
        m_searchPane->FocusSearch();
        m_searchPane->RefreshSearch();
    }
    else
    {
        settings->m_AuiPanels.search_panel_height = m_searchPane->GetSize().y;
        settings->m_AuiPanels.search_panel_width = m_searchPane->GetSize().x;
        settings->m_AuiPanels.search_panel_dock_direction = searchPaneInfo.dock_direction;
        m_auimgr.Update();
        GetCanvas()->SetFocus();
    }
}


void PCB_EDIT_FRAME::ReCreateAuxiliaryToolbar()
{
    UpdateTrackWidthSelectBox( m_SelTrackWidthBox, true, true );
    UpdateViaSizeSelectBox( m_SelViaSizeBox, true, true );
    UpdateViaStackSelectBox( m_SelViaStackBox );
}


void PCB_EDIT_FRAME::OnUpdateSelectTrackWidth( wxUpdateUIEvent& aEvent )
{
    if( aEvent.GetId() == ID_AUX_TOOLBAR_PCB_TRACK_WIDTH )
    {
        BOARD_DESIGN_SETTINGS& bds = GetDesignSettings();
        int                    sel;

        if( bds.UseCustomTrackViaSize() )
            sel = wxNOT_FOUND;
        else
            // if GetTrackWidthIndex() < 0, display the "use netclass" option
            sel = std::max( 0, bds.GetTrackWidthIndex() );

        if( m_SelTrackWidthBox->GetSelection() != sel )
            m_SelTrackWidthBox->SetSelection( sel );
    }
}


void PCB_EDIT_FRAME::OnUpdateSelectViaSize( wxUpdateUIEvent& aEvent )
{
    if( aEvent.GetId() == ID_AUX_TOOLBAR_PCB_VIA_SIZE )
    {
        BOARD_DESIGN_SETTINGS& bds = GetDesignSettings();
        int                    sel = 0;

        if( bds.UseCustomTrackViaSize() )
            sel = wxNOT_FOUND;
        else
            // if GetViaSizeIndex() < 0, display the "use netclass" option
            sel = std::max( 0, bds.GetViaSizeIndex() );

        if( m_SelViaSizeBox->GetSelection() != sel )
            m_SelViaSizeBox->SetSelection( sel );
    }
}


void PCB_EDIT_FRAME::ToggleLibraryTree()
{
    PCBNEW_SETTINGS* cfg = GetPcbNewSettings();

    wxCHECK( cfg, /* void */ );

    wxAuiPaneInfo& db_library_pane = m_auimgr.GetPane( DesignBlocksPaneName() );

    db_library_pane.Show( !db_library_pane.IsShown() );

    if( db_library_pane.IsShown() )
    {
        if( db_library_pane.IsFloating() )
        {
            db_library_pane.FloatingSize( cfg->m_AuiPanels.design_blocks_panel_float_width,
                                          cfg->m_AuiPanels.design_blocks_panel_float_height );
            m_auimgr.Update();
        }
        else
        {
            // SetAuiPaneSize also updates m_auimgr
            SetAuiPaneSize( m_auimgr, db_library_pane, cfg->m_AuiPanels.design_blocks_panel_docked_width, -1 );
        }
    }
    else
    {
        if( db_library_pane.IsFloating() )
        {
            cfg->m_AuiPanels.design_blocks_panel_float_width = db_library_pane.floating_size.x;
            cfg->m_AuiPanels.design_blocks_panel_float_height = db_library_pane.floating_size.y;
        }
        else
        {
            cfg->m_AuiPanels.design_blocks_panel_docked_width = m_designBlocksPane->GetSize().x;
        }

        m_auimgr.Update();
    }
}
