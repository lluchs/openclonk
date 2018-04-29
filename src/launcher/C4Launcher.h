/*
 * OpenClonk, https://www.openclonk.org
 *
 * Copyright (c) 2018, The OpenClonk Team and contributors
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


#ifndef INC_C4Launcher
#define INC_C4Launcher

#ifdef WITH_LAUNCHER

// Avoid some name conflicts
#undef LineFeed
#undef new
#undef delete
#include <QDialog>
#include <QNetworkAccessManager>
#include "ui_C4LauncherWindow.h"

class C4Launcher
{
public:
	int Run();

private:
	//class C4LauncherWindow *window;
};

class QLabel;

class C4LauncherGroup : public QObject
{
	Q_OBJECT
public:
	C4LauncherGroup(QObject *parent, QLabel *statusLabel, QString name, QString hash);

	const QString GetName() const { return name; }

	// JSON (de-)serialization
	void Read(const QJsonObject& json);
	void Write(QJsonObject& json) const;

	// CheckFile reads the group file's signature.
	void ReadFileSignature();

	bool NeedsUpdate() const;

	// Download downloads and replaces the group.
	void Download(QNetworkAccessManager& qnam);

private slots:
	void downloadReadyRead();
	void downloadFinished();

signals:
	void DownloadFinished();

private:
	QString FilePath() const;
	void UpdateStatusLabel();

	bool failed{false};
	QString name;
	QString hash, prevHash;
	QByteArray signature, prevSignature;

	class QNetworkReply *reply{nullptr};
	std::unique_ptr<class QFile> downloadFile;
	QLabel *statusLabel;
};

class C4LauncherWindow : public QDialog
{
	Q_OBJECT

public:
	explicit C4LauncherWindow(QWidget *parent = nullptr);
	virtual ~C4LauncherWindow() { }

private slots:
	void RequestGroupsList();
	void GroupsListFinished();
	void GroupUpdateFinished();

private:
	QString GroupsStatusFilePath() const;
	void ReadGroupsStatus();
	void WriteGroupsStatus() const;
	void UpdateGroups();

	Ui::LauncherWindow ui;

	QNetworkAccessManager qnam;
	class QNetworkReply *reply;

	QVector<C4LauncherGroup*> groups;
};

#endif // WITH_LAUNCHER


#endif //INC_C4Launcher
