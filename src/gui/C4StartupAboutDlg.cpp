/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2001-2009, RedWolf Design GmbH, http://www.clonk.de/
 * Copyright (c) 2010-2016, The OpenClonk Team and contributors
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
// About/credits screen

#include "C4Include.h"
#include "gui/C4StartupAboutDlg.h"

#include "C4Version.h"
#include "graphics/C4GraphicsResource.h"
#include "gui/C4UpdateDlg.h"

struct PersonList
{
	virtual void WriteTo(C4GUI::TextWindow *textbox, CStdFont &font) = 0;
	virtual ~PersonList() { }
};

static struct DeveloperList : public PersonList
{
	struct Entry 
	{
		const char *name, *nick;
	};
	std::vector<Entry> developers;

	DeveloperList(std::initializer_list<Entry> l) : developers(l) { }

	void WriteTo(C4GUI::TextWindow *textbox, CStdFont &font)
	{
		for (auto& p : developers)
		{
			textbox->AddTextLine(FormatString("%s <c f7f76f>(%s)</c>", p.name, p.nick).getData(), &font, C4GUI_MessageFontClr, false, true);
		}
	}
}
engineAndTools =
{
	{"Sven Eberhardt", "Sven2"},
	{"Günther Brammer", "Günther"},
	{"Nicolas Hake", "Isilkor"},
	{"Armin Burgmeier", "Clonk-Karl"},
	{"Lukas Werling", "Luchs"},
	{"Julius Michaelis", "JCaesar"},
	{"Peter Wortmann", "PeterW"},
},
scriptingAndContent =
{
	{"Maikel de Vries", "Maikel"},
	{"David Dormagen", "Zapper"},
	{"Mark Haßelbusch", "Marky"},
	{"Felix Wagner", "Clonkonaut"},
	{"Bernhard Bonigl", "Boni"},
},
administration =
{
	{"Tobias Zwick", "Newton"},
},
artAndContent =
{
	{"Charles Spurrill", "Ringwaul"},
	{"Richard Gerum", "Randrian"},
	{"Timo Stabbert", "Mimmo"},
	{"Matthias Rottländer", "Matthi"},
},
musicAndSound =
{
	{"David Oerther", "ala"},
	{"Martin Strohmeier", "K-Pone"},
};

static struct ContributorList : public PersonList
{
	struct Entry 
	{
		const char *name, *nick;
	};
	static const std::vector<Entry> contributorsThisRelease, contributors, packageMaintainers;

	StdStrBuf ConcatNames(const std::vector<Entry>& names)
	{
		StdStrBuf result;
		for (auto& p : names)
		{
			if (result.getLength()) result.Append(", ");
			if (p.nick)
				result.AppendFormat("%s <c f7f76f>(%s)</c>", p.name, p.nick);
			else 
				result.Append(p.name);
		}
		return result;
	}

	void WriteTo(C4GUI::TextWindow *textbox, CStdFont &font)
	{
		StdStrBuf text;
		text = "Contributors for OpenClonk 8.0: ";
		text.Append(ConcatNames(contributorsThisRelease));
		textbox->AddTextLine(text.getData(), &font, C4GUI_MessageFontClr, false, true);

		text = "Previous contributors: ";
		text.Append(ConcatNames(contributors));
		textbox->AddTextLine(text.getData(), &font, C4GUI_MessageFontClr, false, true);

		text = "Also thanks to our Linux package maintainers ";
		text.Append(ConcatNames(packageMaintainers));
		textbox->AddTextLine(text.getData(), &font, C4GUI_MessageFontClr, false, true);

		text = "Finally, a big thanks to Matthes Bender and all those who contributed to previous Clonk titles for the passion they put into the game and for agreeing to make Clonk open source.";
		textbox->AddTextLine(text.getData(), &font, C4GUI_MessageFontClr, false, true);
	}
} contributors;

const std::vector<ContributorList::Entry> ContributorList::contributorsThisRelease = {
	{"Fulgen", nullptr},
	{"Linus Heckemann", "sphalerite"},
	{"Dominik Bayerl", "Kanibal"},
	{"Armin Schäfer", nullptr},
	{"Tushar Maheshwari", nullptr},
	{"jok", nullptr},
	{"Philip Kern", "pkern"},
	{"Matthias Mailänder", nullptr},
};

const std::vector<ContributorList::Entry> ContributorList::contributors = {
	{"Martin Adam", "Win"},
	{"Florian Graier", "Nachtfalter"},
	{"Merten Ehmig", "pluto"},
	{"Benjamin Herr", "Loriel"},
	{"Pyrit", nullptr},
	{"Philip Holzmann", "Batman"},
	{"Alexander Semeniuk", "AlteredARMOR"},
	{"Andriel", nullptr},
	{"Peewee", nullptr},
	{"Oliver Schneider", "ker"},
	{"Fabian Pietsch", nullptr},
	{"Manuel Rieke", "MrBeast"},
	{"Felix Riese", "Fungiform"},
	{"Carl-Philip Hänsch", "Carli"},
	{"Sebastian Rühl", nullptr},
	{"Gurkenglas", nullptr},
	{"Asmageddon", nullptr},
	{"mizipzor", nullptr},
	{"Tim Blume", nullptr},
	{"Apfelclonk", nullptr},
	{"Sven-Hendrik Haase", nullptr},
	{"Lauri Niskanen", "Ape"},
	{"Daniel Theuke", "ST-DDT"},
	{"Russell", nullptr},
	{"Stan", nullptr},
	{"TomyLobo", nullptr},
	{"Clonkine", nullptr},
	{"Koronis", nullptr},
	{"Johannes Nixdorf", "mixi"},
	{"grgecko", nullptr},
	{"Misty de Meo", nullptr},
	{"Lorenz Schwittmann", nullptr},
	{"hasufell", nullptr},
	{"Jan Heberer", nullptr},
	{"dylanstrategie", nullptr},
	{"Checkmaty", nullptr},
	{"Faby", nullptr},
};

const std::vector<ContributorList::Entry> ContributorList::packageMaintainers = {
	{"Benedict Etzel", "B_E"},
	{"Philip Kern", "pkern"},
	{"Kevin Zeng", nullptr},
};

// ------------------------------------------------
// --- C4StartupAboutDlg

C4StartupAboutDlg::C4StartupAboutDlg() : C4StartupDlg(LoadResStr("IDS_DLG_ABOUT"))
{
	// ctor
	UpdateSize();

	CStdFont &rUseFont = ::GraphicsResource.TextFont;
	C4Rect rcClient = GetContainedClientRect();

	// bottom line buttons and copyright messages
	C4GUI::ComponentAligner caMain(rcClient, 0,0, true);
	C4GUI::ComponentAligner caButtons(caMain.GetFromBottom(caMain.GetHeight()*1/8), 0,0, false);
	C4GUI::CallbackButton<C4StartupAboutDlg> *btn;
	int32_t iButtonWidth = caButtons.GetInnerWidth() / 4;
	AddElement(btn = new C4GUI::CallbackButton<C4StartupAboutDlg>(LoadResStr("IDS_BTN_BACK"), caButtons.GetGridCell(0,3,0,1,iButtonWidth,C4GUI_ButtonHgt,true), &C4StartupAboutDlg::OnBackBtn));
	btn->SetToolTip(LoadResStr("IDS_DLGTIP_BACKMAIN"));
#ifdef WITH_AUTOMATIC_UPDATE
	AddElement(btn = new C4GUI::CallbackButton<C4StartupAboutDlg>(LoadResStr("IDS_BTN_CHECKFORUPDATES"), caButtons.GetGridCell(2,3,0,1,iButtonWidth,C4GUI_ButtonHgt,true), &C4StartupAboutDlg::OnUpdateBtn));
	btn->SetToolTip(LoadResStr("IDS_DESC_CHECKONLINEFORNEWVERSIONS"));
#endif

	AddElement(new C4GUI::Label("'Clonk' is a registered trademark of Matthes Bender.",
		caButtons.GetFromBottom(rUseFont.GetLineHeight())));

	C4GUI::ComponentAligner caDevelopers(caMain.GetFromTop(caMain.GetHeight() * 1/2), 0,0, false);
	C4GUI::ComponentAligner caContributors(caMain.GetFromTop(caMain.GetHeight()), 0,0, false);
	DrawPersonList(C4StartupAboutEngineAndTools, engineAndTools, caDevelopers.GetFromLeft(caMain.GetWidth()*1/3));
	C4GUI::ComponentAligner caDevelopersCol2(caDevelopers.GetFromLeft(caMain.GetWidth()*1/3), 0,0, false);
	DrawPersonList(C4StartupAboutScriptingAndContent, scriptingAndContent, caDevelopersCol2.GetFromTop(caDevelopers.GetHeight()*2/3));
	DrawPersonList(C4StartupAboutAdministration, administration, caDevelopersCol2.GetFromTop(caDevelopers.GetHeight()*1/3));
	C4GUI::ComponentAligner caDevelopersCol3(caDevelopers.GetFromLeft(caMain.GetWidth()*1/3), 0,0, false);
	DrawPersonList(C4StartupAboutArtAndContent, artAndContent, caDevelopersCol3.GetFromTop(caDevelopers.GetHeight()*2/3));
	DrawPersonList(C4StartupAboutMusicAndSound, musicAndSound, caDevelopersCol3.GetFromTop(caDevelopers.GetHeight()*1/3));

	DrawPersonList(C4StartupAboutContributors, contributors, caContributors.GetFromTop(caContributors.GetHeight()));

}

C4StartupAboutDlg::~C4StartupAboutDlg() = default;


void C4StartupAboutDlg::DrawPersonList(int title, PersonList& persons, C4Rect& rect)
{
	CStdFont &rUseFont = ::GraphicsResource.TextFont;
	auto image = C4Startup::Get()->Graphics.fctAboutTitles.GetPhase(0, title);
	int height = 2*rUseFont.GetFontHeight();
	auto textbox = new C4GUI::TextWindow(rect, image.GetWidthByHeight(height), height, 0, 100, 4096, "", true, &image, 0, true);
	AddElement(textbox);
	textbox->SetDecoration(false, false, nullptr, true);
	persons.WriteTo(textbox, rUseFont);
	textbox->UpdateHeight();
}

void C4StartupAboutDlg::DoBack()
{
	C4Startup::Get()->SwitchDialog(C4Startup::SDID_Main);
}

void C4StartupAboutDlg::DrawElement(C4TargetFacet &cgo)
{
}

#ifdef WITH_AUTOMATIC_UPDATE
void C4StartupAboutDlg::OnUpdateBtn(C4GUI::Control *btn)
{
	C4UpdateDlg::CheckForUpdates(GetScreen());
}
#endif
