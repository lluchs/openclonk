/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2001-2009, RedWolf Design GmbH, http://www.clonk.de/
 * Copyright (c) 2013-2016, The OpenClonk Team and contributors
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

/* Core component of a folder */

#ifndef INC_C4Folder
#define INC_C4Folder

class C4FolderHead
{
public:
	int32_t Index;                      // Folder index in scenario selection dialog
public:
	void Default();
	void CompileFunc(StdCompiler *pComp);
};

class C4Folder
{
public:
	C4Folder();
public:
	C4FolderHead Head;
public:
	void Default();
	bool Load(C4Group &hGroup);
	void CompileFunc(StdCompiler *pComp);
protected:
	bool Compile(const char *szSource);
};

#endif // INC_C4Folder
