/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2011-2016, The OpenClonk Team and contributors
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

#include "../../include/c4script/c4script.h"

#include "C4Include.h"

#include "c4group/C4Group.h"
#include "lib/C4Random.h"
#include "object/C4DefList.h"
#include "script/C4Aul.h"
#include "script/C4ScriptHost.h"

/* StandaloneStubs.cpp is shared with mape, which has a real implementation of these */
C4Def* C4DefList::GetByName(const StdStrBuf &) {return nullptr;}
C4Def * C4DefList::GetDef(int) {return nullptr;}
int C4DefList::GetDefCount() {return 0;}
void C4DefList::SortByPriority() {}
void C4DefList::CallEveryDefinition() {}
void C4DefList::ResetIncludeDependencies() {}

// Script error handler that forwards errors and warnings to C functions.
struct ForwardToCErrorHandler : public C4AulErrorHandler
{
	c4s_errorhandlers *handlers;

	ForwardToCErrorHandler(c4s_errorhandlers *handlers)
		: handlers(handlers)
	{
		assert(handlers != nullptr);
		::ScriptEngine.RegisterErrorHandler(this);
	}

	virtual ~ForwardToCErrorHandler() { }

	void OnError(const char *msg)
	{
		if (handlers->errors)
			handlers->errors(handlers->ctx, msg);
		++::ScriptEngine.errCnt;
	}

	void OnWarning(const char *msg)
	{
		if (handlers->warnings)
			handlers->warnings(handlers->ctx, msg);
		++::ScriptEngine.warnCnt;
	}
};

static void InitializeC4Script()
{
	InitCoreFunctionMap(&ScriptEngine);

	// Seed PRNG
	FixedRandom(time(nullptr));
}

static void ClearC4Script()
{
	GameScript.Clear();
	ScriptEngine.Clear();
}

static C4Value RunLoadedC4Script()
{
	// Link script engine (resolve includes/appends, generate code)
	ScriptEngine.Link(nullptr);

	C4Value result = GameScript.Call("Main");
	return result;
}

static int RunFile(const char * filename, c4s_errorhandlers *handlers, bool checkOnly)
{
	C4Group File;
	if (!File.Open(GetWorkingDirectory()))
	{
		fprintf(stderr, "Open failed: %s\n", File.GetError());
		return 1;
	}

	// get scripts
	StdStrBuf fn;
	File.ResetSearch();
	if (!File.FindNextEntry(filename, &fn))
	{
		fprintf(stderr, "FindNextEntry failed: %s\n", File.GetError());
		return 1;
	}

	InitializeC4Script();
	{
		std::unique_ptr<ForwardToCErrorHandler> errorhandler;
		if (handlers)
			errorhandler = std::make_unique<ForwardToCErrorHandler>(handlers);
		GameScript.Load(File, fn.getData(), nullptr, nullptr);
		if (!checkOnly)
			RunLoadedC4Script();
	}
	ClearC4Script();
	return ScriptEngine.errCnt;
}

static int RunString(const char *script, c4s_errorhandlers *handlers, bool checkOnly)
{
	InitializeC4Script();
	{
		std::unique_ptr<ForwardToCErrorHandler> errorhandler;
		if (handlers)
			errorhandler = std::make_unique<ForwardToCErrorHandler>(handlers);
		GameScript.LoadData("<memory>", script, nullptr);
		if (!checkOnly)
			RunLoadedC4Script();
	}
	ClearC4Script();
	return ScriptEngine.errCnt;
}


int c4s_runfile(const char *filename, c4s_errorhandlers *handlers) { return RunFile(filename, handlers, false); }
int c4s_runstring(const char *script, c4s_errorhandlers *handlers) { return RunString(script, handlers, false); }

int c4s_checkfile(const char *filename, c4s_errorhandlers *handlers) { return RunFile(filename, handlers, true); }
int c4s_checkstring(const char *script, c4s_errorhandlers *handlers) { return RunString(script, handlers, true); }
