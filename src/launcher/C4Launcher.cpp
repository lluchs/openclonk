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

#include "C4Include.h"
#include "C4Version.h"
#include "launcher/C4Launcher.h"

#include <QApplication>
#include <QFile>
#include <QLabel>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QNetworkReply>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

int C4Launcher::Run()
{
	static int fake_argc = 1;
	static const char *fake_argv[] = { "openclonk" };
	QApplication app(fake_argc, const_cast<char**>(fake_argv));

	C4LauncherWindow window;
	window.show();
	app.exec();
	return window.result();
}

C4LauncherGroup::C4LauncherGroup(QObject *parent, QLabel *statusLabel, QString name, QString hash)
	: QObject(parent)
	, statusLabel(statusLabel)
	, name(name), hash(hash)
{
	statusLabel->setText("?");
}

void C4LauncherGroup::Read(const QJsonObject& json)
{
	prevHash = json["hash"].toString();
	prevSignature = QByteArray::fromBase64(json["signature"].toString().toUtf8());
	UpdateStatusLabel();
}

void C4LauncherGroup::Write(QJsonObject& json) const
{
	json["hash"] = hash;
	json["signature"] = QString::fromUtf8(signature.toBase64());
}

QString C4LauncherGroup::FilePath() const
{
	return Config.AtSystemDataPath(name.toUtf8().data());
}

void C4LauncherGroup::ReadFileSignature()
{
	QFile file(FilePath());
	if (!file.open(QIODevice::ReadOnly))
	{
		// file does not exist or is not readable.
		signature.clear();
		return;
	}
	// The last 8 byte of a gzip stream are the CRC and the original file size.
	const qint64 GZIP_TRAILER = 8;
	file.seek(file.size() - GZIP_TRAILER);
	signature = file.read(GZIP_TRAILER);
}

bool C4LauncherGroup::NeedsUpdate() const
{
	return !failed && (hash != prevHash || !signature.size() || signature != prevSignature);
}

void C4LauncherGroup::Download(QNetworkAccessManager& qnam)
{
	// Open separate target file.
	QString fileName = FilePath() + "." + hash;
	auto file = std::make_unique<QFile>(fileName);
	if (!file->open(QIODevice::WriteOnly)) {
		QMessageBox::information(statusLabel, tr("Error"),
				tr("Unable to save the file %1: %2.")
				.arg(fileName, file->errorString()));
		return;
	}
	downloadFile = std::move(file);

	// Start the network request.
	auto urlStr = QString(Config.Launcher.Git2GroupURL.c_str()) + "/pack/" C4REVISION_RAW "/planet/" + name;
	QUrl url(urlStr);

	reply = qnam.get(QNetworkRequest(url));
	connect(reply, &QIODevice::readyRead, this, &C4LauncherGroup::downloadReadyRead);
	connect(reply, &QNetworkReply::finished, this, &C4LauncherGroup::downloadFinished);
	UpdateStatusLabel();
}

void C4LauncherGroup::downloadReadyRead()
{
	if (downloadFile)
		downloadFile->write(reply->readAll());
}

void C4LauncherGroup::downloadFinished()
{
	if (reply->error())
	{
		statusLabel->setText(tr("Download failed: %1").arg(reply->errorString()));
		LogF("Launcher: %s", tr("Download failed: %1").arg(reply->errorString()).toUtf8().data());
		failed = true;
		reply->deleteLater(); reply = nullptr;
		return;
	}
	reply->deleteLater(); reply = nullptr;

	// The Qt API doesn't allow atomically replacing a file, so we just hope
	// for the best here and ignore any errors.
	auto targetFile = FilePath();
	if (QFile::exists(targetFile))
		QFile::remove(targetFile);
	downloadFile->rename(targetFile);
	downloadFile = nullptr;

	// Finally, update the signature.
	ReadFileSignature();
	if (signature.size())
		prevSignature = signature;
	UpdateStatusLabel();

	emit DownloadFinished();
}

void C4LauncherGroup::UpdateStatusLabel()
{
	QString status;
	if (reply)
		status = tr("Downloading...");
	else if (NeedsUpdate())
		status = tr("Update required");
	else
		status = tr("Ok");
	statusLabel->setText(status);
}

C4LauncherWindow::C4LauncherWindow(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	RequestGroupsList();
}

void C4LauncherWindow::RequestGroupsList()
{
	auto urlStr = QString(Config.Launcher.Git2GroupURL.c_str()) + "/list/" C4REVISION_RAW "/planet";
	QUrl url(urlStr);

	ui.groupsStatusLabel->setText(tr("Updating list..."));
	reply = qnam.get(QNetworkRequest(url));
	connect(reply, &QNetworkReply::finished, this, &C4LauncherWindow::GroupsListFinished);
}

void C4LauncherWindow::GroupsListFinished()
{
	if (reply->error())
	{
		ui.groupsStatusLabel->setText(tr("Update failed: %1").arg(reply->errorString()));
		reply->deleteLater(); reply = nullptr;
		return;
	}

	QJsonParseError error;
	auto jsonGroups = QJsonDocument::fromJson(reply->readAll(), &error);

	reply->deleteLater(); reply = nullptr;

	if (error.error != QJsonParseError::NoError)
	{
		ui.groupsStatusLabel->setText(tr("Parse error: %1").arg(error.errorString()));
		return;
	}
	ui.groupsStatusLabel->setText("");

	int idx = 0;
	for (const auto group : jsonGroups.array())
	{
		auto groupobj = group.toObject();
		auto statusLabel = new QLabel(ui.groupsContainer);
		ui.groupsContainerLayout->addWidget(statusLabel, idx, 0);
		auto nameLabel = new QLabel(ui.groupsContainer);
		ui.groupsContainerLayout->addWidget(nameLabel, idx++, 1);
		auto name = groupobj[QString("Name")].toString();
		nameLabel->setText(name);
		groups.append(new C4LauncherGroup(this, statusLabel, name, groupobj[QString("Hash")].toString()));
	}
	ReadGroupsStatus();
	UpdateGroups();
}

// UpdateGroups sequentially updates all groups, downloading only one group at
// once.
void C4LauncherWindow::UpdateGroups()
{
	for (auto& group : groups)
	{
		if (group->NeedsUpdate())
		{
			group->Download(qnam);
			connect(group, &C4LauncherGroup::DownloadFinished, this, &C4LauncherWindow::GroupUpdateFinished);
			break;
		}
	}
}

void C4LauncherWindow::GroupUpdateFinished()
{
	WriteGroupsStatus();
	UpdateGroups();
}

QString C4LauncherWindow::GroupsStatusFilePath() const
{
	return Config.AtSystemDataPath("groups.json");
}

void C4LauncherWindow::ReadGroupsStatus()
{
	QFile file(GroupsStatusFilePath());
	if (!file.open(QIODevice::ReadOnly))
	{
		Log("Launcher: No groups.json found");
		return;
	}
	auto doc = QJsonDocument::fromJson(file.readAll());
	auto obj = doc.object();
	for (auto& group : groups)
	{
		group->ReadFileSignature();
		group->Read(obj[group->GetName()].toObject());
	}
}

void C4LauncherWindow::WriteGroupsStatus() const
{
	QFile file(GroupsStatusFilePath());
	if (!file.open(QIODevice::WriteOnly))
	{
		Log("Launcher: Couldn't save groups.json");
		return;
	}
	QJsonObject obj;
	for (auto& group : groups)
	{
		QJsonObject gobj;
		group->Write(gobj);
		obj[group->GetName()] = gobj;
	}
	QJsonDocument doc(obj);
	file.write(doc.toJson());
}
