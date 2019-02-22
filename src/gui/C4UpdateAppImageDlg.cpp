/*
 * OpenClonk, https://www.openclonk.org
 *
 * Copyright (c) 2019, The OpenClonk Team and contributors
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

#include "C4Include.h"
#include "gui/C4UpdateAppImageDlg.h"
#include "game/C4Application.h"

using AppImageUpdaterBridge::AppImageDeltaRevisioner;

AppImageUpdateProgressDialog::AppImageUpdateProgressDialog(C4GUI::Screen *screen)
{
	dialog = std::make_unique<C4GUI::ProgressDialog>("Downloading update...", "Update", 100, 0, C4GUI::Ico_Ex_Update);
	revisioner = std::make_unique<AppImageUpdaterBridge::AppImageDeltaRevisioner>();
	connect(&*revisioner, &AppImageDeltaRevisioner::progress, this, &AppImageUpdateProgressDialog::progress);
	connect(&*revisioner, &AppImageDeltaRevisioner::error, this, &AppImageUpdateProgressDialog::error);
	connect(&*revisioner, &AppImageDeltaRevisioner::finished, this, &AppImageUpdateProgressDialog::finished);
	connect(&*revisioner, &AppImageDeltaRevisioner::logger, this, &AppImageUpdateProgressDialog::logger);
	revisioner->setShowLog(true);
	revisioner->setAppImage(Application.argv0);
	revisioner->start();
	dialog->Show(screen, false);
}

AppImageUpdateProgressDialog::~AppImageUpdateProgressDialog()
{
}
