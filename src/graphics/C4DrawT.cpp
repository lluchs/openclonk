/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 2001-2009, RedWolf Design GmbH, http://www.clonk.de/
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
#include "C4Include.h"
#include "graphics/C4DrawT.h"
#include "lib/StdMeshMaterial.h"

CStdNoGfx::CStdNoGfx()
{
	Default();
}

bool CStdNoGfx::RestoreDeviceObjects()
{
	Log("Graphics disabled.");
	MaxTexSize = 2147483647;
	return true;
}

bool CStdNoGfx::PrepareMaterial(StdMeshMatManager& mat_manager, StdMeshMaterialLoader& loader, StdMeshMaterial& mat)
{
	mat.BestTechniqueIndex=0; return true;
}
