/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2012  Armin Burgmeier
 *
 * Portions might be copyrighted by other authors who have contributed
 * to OpenClonk.
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * See isc_license.txt for full license and disclaimer.
 *
 * "Clonk" is a registered trademark of Matthes Bender.
 * See clonk_trademark_license.txt for full license.
 */

#include "C4Include.h"
#include "object/C4Rope.h"
#include "script/C4AulDefFunc.h"

static void FnRemove(C4Rope* Rope)
{
	Game.Ropes.RemoveRope(Rope);
}

static C4Object* FnGetFront(C4Rope* Rope)
{
	return Rope->GetFront()->GetObject();
}

static C4Object* FnGetBack(C4Rope* Rope)
{
	return Rope->GetBack()->GetObject();
}

static void FnSetFront(C4Rope* Rope, C4Object* obj, Nillable<int> x, Nillable<int> y)
{
	Rope->SetFront(obj, x.IsNil() ? Fix0 : itofix(x), y.IsNil() ? Fix0 : itofix(y));
}

static void FnSetBack(C4Rope* Rope, C4Object* obj, Nillable<int> x, Nillable<int> y)
{
	Rope->SetBack(obj, x.IsNil() ? Fix0 : itofix(x), y.IsNil() ? Fix0 : itofix(y));
}

static void FnSetFrontAutoSegmentation(C4Rope* Rope, int max)
{
	Rope->SetFrontAutoSegmentation(itofix(max));
}

static void FnSetBackAutoSegmentation(C4Rope* Rope, int max)
{
	Rope->SetBackAutoSegmentation(itofix(max));
}

static void FnSetFrontFixed(C4Rope* Rope, bool fixed)
{
	Rope->SetFrontFixed(fixed);
}

static void FnSetBackFixed(C4Rope* Rope, bool fixed)
{
	Rope->SetBackFixed(fixed);
}

static void FnPullFront(C4Rope* Rope, int force)
{
	Rope->PullFront(itofix(force));
}

static void FnPullBack(C4Rope* Rope, int force)
{
	Rope->PullBack(itofix(force));
}

C4RopeAul::C4RopeAul() : C4PropListStaticMember(nullptr, nullptr, ::Strings.RegString(NAME)) {}

void C4RopeAul::InitFunctionMap(C4AulScriptEngine* engine)
{
	engine->RegisterGlobalConstant(NAME, C4VPropList(this));

	::AddFunc(this, "Remove", FnRemove);
	::AddFunc(this, "GetFront", FnGetFront);
	::AddFunc(this, "GetBack", FnGetBack);
	::AddFunc(this, "SetFront", FnSetFront);
	::AddFunc(this, "SetBack", FnSetBack);
	::AddFunc(this, "SetFrontAutoSegmentation", FnSetFrontAutoSegmentation);
	::AddFunc(this, "SetBackAutoSegmentation", FnSetBackAutoSegmentation);
	::AddFunc(this, "SetFrontFixed", FnSetFrontFixed);
	::AddFunc(this, "SetBackFixed", FnSetBackFixed);
	::AddFunc(this, "PullFront", FnPullFront);
	::AddFunc(this, "PullBack", FnPullBack);

	Freeze();
}
