/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2001-2009, RedWolf Design GmbH, http://www.clonk.de/
 * Copyright (c) 2013, The OpenClonk Team and contributors
 *
 * Distributed under the terms of the ISC license; see accompanying file
 * "COPYING" for details.
 *
 * "Clonk" is a registered trademark of Matthes Bender, used with permission.
 * See accompanying file "TRADEMARK" for details.
 *
 * To redistribute this file separately, substitute the full license texts
 * for the above references.
 */

/* Editor windows using Qt*/

#include <C4Include.h>
#include <C4ConsoleQtState.h>
#include <C4Console.h>
#include <C4ConsoleGUI.h>
#include <C4Texture.h>
#include <C4Landscape.h>
#include <C4Version.h>

#include <C4ConsoleQt.h>

// todo:
// * viewport subwindow hack causes alt+tab entry to disappear
// * script window inputs
// * object properties
// * object creator
// * missing translation strings
// * missing tooltips
// * mattex pictures in dropdown
// * mat drawing cursor size preview in active viewports
// * proper viewport focus when clicked
// * crash when viewport closes on player elimination
// -----------------------------------------------

void C4ConsoleGUI::Execute() { state->Execute(); }

void C4ConsoleGUI::SetCursor(C4ConsoleGUI::Cursor cursor)
{

}

void C4ConsoleGUI::RecordingEnabled()
{
	if (Active) state->SetRecording(true); // TODO this is never reset. Noone uses it anyway...
}

void C4ConsoleGUI::ShowAboutWithCopyright(StdStrBuf &copyright)
{
	QMessageBox::about(state->window.get(), QString(LoadResStr("IDS_MENU_ABOUT")), QString(copyright.getData()));
}

bool C4ConsoleGUI::UpdateModeCtrls(int iMode)
{
	if (!Active) return false;
	state->SetEditCursorMode(iMode);
	return true;
}

void C4ConsoleGUI::AddNetMenu()
{
	if (Active) state->SetNetEnabled(true);
}

void C4ConsoleGUI::ClearNetMenu()
{
	if (Active) state->ClearNetMenu();
}

void C4ConsoleGUI::AddNetMenuItemForPlayer(int32_t index, StdStrBuf &text)
{
	if (Active) state->AddNetMenuItem(index, text.getData());
}

void C4ConsoleGUI::ClearPlayerMenu()
{
	if (Active) state->ClearPlayerMenu();
}

void C4ConsoleGUI::SetInputFunctions(std::list<const char*> &functions)
{
	if (Active) state->SetInputFunctions(functions);
}

C4Window* C4ConsoleGUI::CreateConsoleWindow(C4AbstractApp *application)
{
	if (!state->CreateConsoleWindow(application)) return NULL;
#ifdef USE_WIN32_WINDOWS
	hWindow = reinterpret_cast<HWND>(state->window->winId());
	renderwnd = hWindow;
#else
	TODO
#endif
	Active = true;
	EnableControls(fGameOpen);
	return this;
}

void C4ConsoleGUI::Out(const char* message)
{
	// Log text: Add to log window
	if (state->window.get())
	{
		// Append text
		state->ui.logView->insertPlainText(QString(message) + "\n");
		// Scroll to end to display it
		QScrollBar *sb = state->ui.logView->verticalScrollBar();
		if (sb) sb->setValue(sb->maximum());
		state->Redraw();
	}
}

bool C4ConsoleGUI::ClearLog()
{
	// Empty log window
	if (!Active) return false;
	state->ui.logView->clear();
	return true;
}

void C4ConsoleGUI::DisplayInfoText(InfoTextType type, StdStrBuf& text)
{
	QLabel *target = NULL;
	switch (type)
	{
	case CONSOLE_Cursor: target = state->status_cursor; break;
	case CONSOLE_FrameCounter: target = state->status_framecounter; break;
	case CONSOLE_TimeFPS: target = state->status_timefps; break;
	}
	if (!target) return;
	target->setText(text.getData());
}

void C4ConsoleGUI::SetCaptionToFileName(const char* file_name) { /* This is never even called? */ }

bool C4ConsoleGUI::FileSelect(StdStrBuf *sFilename, const char * szFilter, DWORD dwFlags, bool fSave)
{
	// Prepare filters from double-zero-terminated list to ";;"-separated list in Qt format
	QString filter="", selected_filter, filename;
	QStringList filenames; bool has_multi = (dwFlags & OpenFileFlags::OFN_ALLOWMULTISELECT);
	if (szFilter)
	{
		while (*szFilter)
		{
			if (filter.length() > 0) filter.append(";;");
			filter.append(szFilter);
			szFilter += strlen(szFilter) + 1;
			if (*szFilter)
			{
				filter.append(" (");
				filter.append(szFilter);
				filter.append(")");
				szFilter += strlen(szFilter) + 1;
			}
			if (selected_filter.length() <= 0) selected_filter = filter;
		}
	}
#ifdef USE_WIN32_WINDOWS
	// cwd backup
	size_t l = GetCurrentDirectoryW(0, 0);
	std::unique_ptr<wchar_t []> wd(new wchar_t[l]);
	GetCurrentDirectoryW(l, wd.get());
#endif
	// Show dialogue
	if (fSave)
		filename = QFileDialog::getSaveFileName(state->window.get(), LoadResStr("IDS_DLG_SAVE"), QString(), filter, &selected_filter);
	else if (!has_multi)
		filename = QFileDialog::getOpenFileName(state->window.get(), LoadResStr("IDS_DLG_OPEN"), QString(), filter, &selected_filter);
	else
		filenames = QFileDialog::getOpenFileNames(state->window.get(), LoadResStr("IDS_DLG_OPEN"), QString(), filter, &selected_filter);
#ifdef USE_WIN32_WINDOWS
	// Restore cwd; may have been changed in open/save dialogue
	SetCurrentDirectoryW(wd.get());
#endif
	// Process multi vs single file select
	if (has_multi)
	{
		// Multi-select: Return double-zero-terminated string list
		if (!filenames.length()) return false;
		for (auto fn : filenames)
		{
			sFilename->Append(fn.toUtf8());
			sFilename->AppendChar('\0');
		}
		return true;
	}
	// Cancelled?
	if (filename.length() <= 0) return false;
	// File selected!
	sFilename->Copy(filename.toUtf8());
	sFilename->AppendChar('\0');
	return true;
}

void C4ConsoleGUI::AddMenuItemForPlayer(C4Player  *player, StdStrBuf& player_text)
{
	// Add "new viewport for X" to window menu
	if (Active) state->AddPlayerViewportMenuItem(player->Number, player_text.getData());
}

void C4ConsoleGUI::AddKickPlayerMenuItem(C4Player *player, StdStrBuf& player_text, bool enabled)
{
	// Add "kick X" to player menu
	if (Active) state->AddKickPlayerMenuItem(player->Number, player_text.getData(), enabled);
}

void C4ConsoleGUI::ClearViewportMenu()
{
	// Remove all "new viewport for X" entries from window menu
	if (Active) state->ClearViewportMenu();
}

bool C4ConsoleGUI::Message(const char *message, bool query)
{
	// Show a message through Qt
	if (query)
	{
		auto result = QMessageBox::question(state->window.get(), C4ENGINECAPTION, message, QMessageBox::StandardButton::Ok | QMessageBox::StandardButton::Cancel);
		return (result == QMessageBox::StandardButton::Ok);
	}
	else
	{
		QMessageBox::information(state->window.get(), C4ENGINECAPTION, message, QMessageBox::StandardButton::Ok);
		return true;
	}
}

void C4ConsoleGUI::DoEnableControls(bool fEnable)
{
	if (!Active) return;
	state->SetEnabled(fEnable);
	state->SetLandscapeMode(::Landscape.Mode); // initial setting
}

bool C4ConsoleGUI::DoUpdateHaltCtrls(bool fHalt)
{
	// Reflect halt state in play/pause buttons
	if (!Active) return false;
	state->ui.actionPlay->setChecked(!fHalt);
	state->ui.actionPause->setChecked(fHalt);
	return true;
}

bool C4ConsoleGUI::PropertyDlgOpen() { /* Always open */ return true; }
void C4ConsoleGUI::PropertyDlgClose() { /* Always open */ }

void C4ConsoleGUI::PropertyDlgUpdate(C4EditCursorSelection &rSelection, bool force_function_update)
{
	if (Active) state->PropertyDlgUpdate(rSelection, force_function_update);
}

bool C4ConsoleGUI::ToolsDlgOpen(class C4ToolsDlg *dlg) { /* Always open */ return true; }
void C4ConsoleGUI::ToolsDlgClose() { /* Always open */ }

void C4ConsoleGUI::ToolsDlgInitMaterialCtrls(class C4ToolsDlg *dlg)
{
	// All foreground materials
	assert(Active);
	if (!Active) return;
	state->ui.foregroundMatTexComboBox->addItem(QString(C4TLS_MatSky));
	const C4TexMapEntry *entry; int32_t i = 0;
	while ((entry = ::TextureMap.GetEntry(i++)))
	{
		if (!entry->isNull())
		{
			const char *material_name = entry->GetMaterialName();
			if (strcmp(material_name, "Vehicle") && strcmp(material_name, "HalfVehicle"))
			{
				state->ui.foregroundMatTexComboBox->addItem(QString(FormatString("%s-%s", material_name, entry->GetTextureName()).getData()));
			}
		}
	}
	auto width = 130; /* The ToolBar randomly resizes the control */
	state->ui.foregroundMatTexComboBox->view()->setMinimumWidth(width);
	state->ui.foregroundMatTexComboBox->setFixedWidth(width);
	// Background materials: True background materials first; then the "funny" stuff
	state->ui.backgroundMatTexComboBox->addItem(QString(C4TLS_MatSky));
	i = 0;
	while ((entry = ::TextureMap.GetEntry(i++)))
	{
		if (!entry->isNull())
		{
			const char *material_name = entry->GetMaterialName();
			C4Material *mat = entry->GetMaterial();
			if (strcmp(material_name, "Vehicle") && strcmp(material_name, "HalfVehicle") && mat->Density == C4M_Background)
			{
				state->ui.backgroundMatTexComboBox->addItem(QString(FormatString("%s-%s", material_name, entry->GetTextureName()).getData()));
			}
		}
	}
	state->ui.backgroundMatTexComboBox->addItem(QString("----------"));
	i = 0;
	while ((entry = ::TextureMap.GetEntry(i++)))
	{
		if (!entry->isNull())
		{
			const char *material_name = entry->GetMaterialName();
			C4Material *mat = entry->GetMaterial();
			if (strcmp(material_name, "Vehicle") && strcmp(material_name, "HalfVehicle") && mat->Density != C4M_Background)
			{
				state->ui.backgroundMatTexComboBox->addItem(QString(FormatString("%s-%s", material_name, entry->GetTextureName()).getData()));
			}
		}
	}
	state->ui.backgroundMatTexComboBox->view()->setMinimumWidth(width);
	state->ui.backgroundMatTexComboBox->setFixedWidth(width);
	// Select current materials
	state->SetMaterial(dlg->Material);
	state->SetTexture(dlg->Texture);
	state->SetBackMaterial(dlg->BackMaterial);
	state->SetBackTexture(dlg->BackTexture);
	state->UpdateMatTex();
	state->UpdateBackMatTex();

}

void C4ConsoleGUI::ToolsDlgSelectTexture(C4ToolsDlg *dlg, const char *texture) { if (!Active) return; state->SetTexture(texture); }
void C4ConsoleGUI::ToolsDlgSelectMaterial(C4ToolsDlg *dlg, const char *material) { if (!Active) return; state->SetMaterial(material); }
void C4ConsoleGUI::ToolsDlgSelectBackTexture(C4ToolsDlg *dlg, const char *texture) { if (!Active) return; state->SetBackTexture(texture); }
void C4ConsoleGUI::ToolsDlgSelectBackMaterial(C4ToolsDlg *dlg, const char *material) { if (!Active) return; state->SetBackMaterial(material); }

#ifdef USE_WIN32_WINDOWS
void C4ConsoleGUI::Win32KeepDialogsFloating(HWND hwnd) { /* Dialogues float nicely */ }
bool C4ConsoleGUI::Win32DialogMessageHandling(MSG *msg) { return false; /* message handling done through Qt (somehow?) */ }
void C4ConsoleGUI::UpdateMenuText(HMENU hMenu) { /* Translation done through QTranslator */ }
#endif

 void C4ConsoleGUI::AddViewport(C4ViewportWindow *cvp)
{
	// Add surrounding widget for viewport
	state->AddViewport(cvp);
}

 void C4ConsoleGUI::OnObjectSelectionChanged(class C4EditCursorSelection &selection)
 {
	 // selection changed (through other means than creator or object list view)
	 // reflect selection change in dialogues
	 state->SetObjectSelection(selection);
 }

void C4ToolsDlg::UpdateToolCtrls()
{
	// Set selected drawing tool
	if (::Console.Active) ::Console.state->SetDrawingTool(Tool);
}

void C4ToolsDlg::UpdateTextures() { /* Textures are done with materials */ }
void C4ToolsDlg::NeedPreviewUpdate() { /* No preview */}

void C4ToolsDlg::InitGradeCtrl()
{
	// Update current grade
	if (::Console.Active) ::Console.state->ui.drawSizeSlider->setValue(Grade);
}

bool C4ToolsDlg::PopMaterial()
{
	// Show material selection
	if (!::Console.Active) return false;
	::Console.state->ui.foregroundMatTexComboBox->setFocus();
	::Console.state->ui.foregroundMatTexComboBox->showPopup();
	return true;
}

bool C4ToolsDlg::PopTextures()
{
	// Show texture selection
	if (!::Console.Active) return false;
	::Console.state->ui.foregroundMatTexComboBox->setFocus();
	::Console.state->ui.foregroundMatTexComboBox->showPopup();
	return true;
}

void C4ToolsDlg::UpdateIFTControls() { /* not using IFT */ }

void C4ToolsDlg::UpdateLandscapeModeCtrls()
{
	// Update button down states for landscape mode
	if (::Console.Active) ::Console.state->SetLandscapeMode(::Landscape.Mode);
}


void C4ToolsDlg::EnableControls() { /* Handled internally by tool selection */ }

#include "C4ConsoleGUICommon.h"
