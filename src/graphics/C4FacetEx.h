/*
 * OpenClonk, http://www.openclonk.org
 *
 * Copyright (c) 1998-2000, Matthes Bender
 * Copyright (c) 2001-2009, RedWolf Design GmbH, http://www.clonk.de/
 * Copyright (c) 2009-2016, The OpenClonk Team and contributors
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

/* A facet that can hold its own surface and also target coordinates */

#ifndef INC_C4FacetEx
#define INC_C4FacetEx

#include "graphics/C4Facet.h"
#include "graphics/C4Surface.h"

const int C4FCT_Full   = -1,
          C4FCT_Height = -2,
          C4FCT_Width  = -3;

class C4TargetFacet: public C4Facet
{
public:
	C4TargetFacet() { Default(); }
	~C4TargetFacet() { }
public:
	float TargetX,TargetY,Zoom;

	// Reference values for parallax computations. This is similar to
	// a scrolling position. In most cases these are the same as TargetX
	// and TargetY, however for full map screenshots, which are composed
	// of several individual screenshots, these are kept fixed while
	// TargetX/TargetY are varied to cover the full map. This prevents
	// duplicate parallax objects in fullscreen map screenshots. If
	// TargetX/TargetY are different from ParRefX/ParRefY it can be thought
	// of as drawing only a part of a window/viewport at a given fixed
	// scroll position.
	// See bug #1042.
	float ParRefX, ParRefY;
public:
	void Default() { TargetX=TargetY=0; Zoom=1; ParRefX=ParRefY=0; C4Facet::Default(); }
	void Clear() { Surface=NULL; }

	void Set(const C4Facet &cpy) { TargetX=TargetY=0; Zoom=1; ParRefX=ParRefY=0; C4Facet::Set(cpy); }
	void Set(const C4TargetFacet &cpy) { *this = cpy; }
	void Set(class C4Surface *nsfc, float nx, float ny, float nwdt, float nhgt, float ntx=0, float nty=0, float Zoom=1);
	void Set(class C4Surface *nsfc, const C4Rect & r, float ntx=0, float nty=0, float Zoom=1);
	void Set(class C4Surface *nsfc, float nx, float ny, float nwdt, float nhgt, float ntx, float nty, float Zoom, float prx, float pry);

public:
	C4TargetFacet &operator = (const C4Facet& rhs)
	{
		Set(rhs.Surface,rhs.X,rhs.Y,rhs.Wdt,rhs.Hgt);
		return *this;
	}
	void SetRect(C4TargetRect &rSrc);
};

// facet that can hold its own surface
class C4FacetSurface : public C4Facet
{
private:
	C4Surface Face;

private:
	C4FacetSurface(const C4FacetSurface &rCpy); // NO copy!
	C4FacetSurface &operator = (const C4FacetSurface &rCpy); // NO copy assignment!
public:
	C4FacetSurface() { Default(); }
	~C4FacetSurface() { Clear(); }

	void Default() { Face.Default(); C4Facet::Default(); }
	void Clear() { Face.Clear(); }

	void Set(const C4Facet &cpy) { Clear(); C4Facet::Set(cpy); }
	void Set(C4Surface * nsfc, int nx, int ny, int nwdt, int nhgt)
	{ C4Facet::Set(nsfc, nx,ny,nwdt,nhgt); }

	void Grayscale(int32_t iOffset = 0);
	bool Create(int iWdt, int iHgt, int iWdt2=C4FCT_Full, int iHgt2=C4FCT_Full);
	C4Surface &GetFace() { return Face; } // get internal face
	bool CreateClrByOwner(C4Surface *pBySurface);
	bool EnsureOwnSurface();
	bool Load(C4Group &hGroup, const char *szName, int iWdt, int iHgt, bool fNoErrIfNotFound, int iFlags);
	bool Save(C4Group &hGroup, const char *szName);
	void GrabFrom(C4FacetSurface &rSource)
	{
		Clear(); Default();
		Face.MoveFrom(&rSource.Face);
		Set(rSource.Surface == &rSource.Face ? &Face : rSource.Surface, rSource.X, rSource.Y, rSource.Wdt, rSource.Hgt);
		rSource.Default();
	}
	bool CopyFromSfcMaxSize(C4Surface &srcSfc, int32_t iMaxSize, PlayerColor Color={0,0,0});
};

// facet with source group ID; used to avoid doubled loading from same group
class C4FacetID : public C4FacetSurface
{
public:
	int32_t idSourceGroup;

	C4FacetID() : C4FacetSurface(), idSourceGroup(0) { } // ctor

	void Default() { C4FacetSurface::Default(); idSourceGroup = 0; } // default to std values
	void Clear() { C4FacetSurface::Clear(); idSourceGroup = 0; } // clear all data in class
};

#endif
