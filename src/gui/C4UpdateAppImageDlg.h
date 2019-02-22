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

#ifndef INC_C4UpdateAppImageDlg
#define INC_C4UpdateAppImageDlg

#include "C4Include.h" // needed for automoc
#include "gui/C4Gui.h"
#include <QObject>
#include <AppImageUpdaterBridge>

// Class to receive update progress signals via Qt
class AppImageUpdateProgressDialog : public QObject
{
	Q_OBJECT

	std::unique_ptr<AppImageUpdaterBridge::AppImageDeltaRevisioner> revisioner;
	std::unique_ptr<C4GUI::ProgressDialog> dialog;

public:
	AppImageUpdateProgressDialog(C4GUI::Screen *screen);
	virtual ~AppImageUpdateProgressDialog();

public slots:
	void progress(int percent, qint64 br, qint64 bt, double speed, QString unit)
	{
		LogF("AppImageUpdate: Progress %d", percent);
		dialog->SetProgress(percent);
	}

	void error(short ecode) 
	{
		auto error = AppImageUpdaterBridge::AppImageDeltaRevisioner::errorCodeToString(ecode);
		LogF("AppImageUpdate: error %s", error.toUtf8().constData());
		dialog->Close(false);
	}

	void finished(QJsonObject newVersion, QString oldAppImagePath)
	{
		LogF("AppImageUpdate: finished");
		dialog->Close(true);
	}

	void logger(QString msg, QString appimage)
	{
		LogF("AppImageUpdate: %s (%s)", msg.toUtf8().constData(), appimage.toUtf8().constData());
	}
};

#endif // INC_C4UpdateAppImageDlg
