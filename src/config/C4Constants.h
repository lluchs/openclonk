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

/* Lots of constants */

#ifndef INC_C4Constants
#define INC_C4Constants

//============================= Main =====================================================

const size_t C4MaxTitle = 512;
const int
	C4MaxDefString = 100,
	C4MaxMessage = 256,
	C4RetireDelay =  60,
	C4MaxKey = 12,
	C4MaxKeyboardSet = 4,
	C4MaxControlSet = C4MaxKeyboardSet+4, // keyboard sets+gamepads
	C4MaxControlRate = 20,
	C4MaxGammaUserRamps = 8,
	C4MaxGammaRamps = C4MaxGammaUserRamps+1;

// gamma ramp indices
#define   C4GRI_SCENARIO  0
#define   C4GRI_SEASON    1
#define   C4GRI_RESERVED1 2
#define   C4GRI_DAYTIME   3
#define   C4GRI_RESERVED2 4
#define   C4GRI_LIGHTNING 5
#define   C4GRI_MAGIC     6
#define   C4GRI_RESERVED3 7
#define   C4GRI_USER      8

const int 
	C4M_MaxName = 15,
	C4M_MaxDefName = 2*C4M_MaxName+1,
	C4M_MaxTexIndex = 255; // last texture map index is reserved for diff

const int C4S_MaxPlayer = 4;

const int C4D_MaxVertex = 30;

const int 
	C4SymbolSize = 35,
	C4UpperBoardHeight = 50,
	C4PictureSize = 64,
	C4MaxBigIconSize = 64;

const int C4P_MaxPosition = 4;

const int C4ViewportScrollBorder = 40; // scrolling past landscape allowed at range of this border

//============================= Engine Return Values ======================================

const int 
	C4XRV_Completed = 0,
	C4XRV_Failure = 1,
	C4XRV_Aborted = 2;

//============================= Object Character Flags ====================================

const uint32_t	
	OCF_None = 0,
	OCF_All = ~OCF_None,
	OCF_Normal = 1,
	OCF_Construct = 1<<1,
	OCF_Grab = 1<<2,
	OCF_Carryable = 1<<3,
	OCF_OnFire = 1<<4,
	OCF_HitSpeed1 = 1<<5,
	OCF_FullCon = 1<<6,
	OCF_Inflammable = 1<<7,

	OCF_Rotate = 1<<9,
	OCF_Exclusive = 1<<10,
	OCF_Entrance = 1<<11,
	OCF_HitSpeed2 = 1<<12,
	OCF_HitSpeed3 = 1<<13,     
	OCF_Collection = 1<<14,

	OCF_HitSpeed4 = 1<<16,
	OCF_NotContained = 1<<18,
	OCF_CrewMember = 1<<19,
	OCF_InLiquid = 1<<20,
	OCF_InSolid = 1<<21,
	OCF_InFree = 1<<22,
	OCF_Available = 1<<23,
	OCF_Container = 1<<24,
	OCF_Alive = 1<<25;

//================================== Contact / Attachment ==============================================

const BYTE // Directional
	CNAT_None = 0,
	CNAT_Left = 1,
	CNAT_Right = 2,
	CNAT_Top = 4,
	CNAT_Bottom = 8,
	CNAT_Center = 16,
	// Additional flags
	CNAT_MultiAttach = 32, // new attachment behaviour; see C4Shape::Attach
	CNAT_NoCollision = 64, // turn off collision for this vertex
	CNAT_PhaseHalfVehicle = 128;

const BYTE CNAT_Flags = CNAT_MultiAttach | CNAT_NoCollision | CNAT_PhaseHalfVehicle; // all attchment flags that can be combined with regular attachment

//=================================== Control Commands ======================================================

const BYTE 
	COM_MenuEnter = 38,
	COM_MenuEnterAll = 39,
	COM_MenuClose = 40,
	COM_MenuShowText = 42,
	COM_MenuLeft = 52,
	COM_MenuRight = 53,
	COM_MenuUp = 54,
	COM_MenuDown = 55,
	COM_MenuSelect = 60;

//=================================== Owners ==============================================

const int 
	NO_OWNER = -1,
	ANY_OWNER = -2;

//=================================== League (escape those damn circular includes =========

enum C4LeagueDisconnectReason
{
	C4LDR_Unknown,
	C4LDR_ConnectionFailed,
	C4LDR_Desync
};

//=================================== Player (included by C4PlayerInfo and C4Player)

enum C4PlayerType
{
	C4PT_None = 0,
	C4PT_User = 1,     // Normal player
	C4PT_Script = 2    // AI players, etc.
};

//=================================== AllowPictureStack (DefCore value)

enum C4AllowPictureStack
{
	APS_Color = 1<<0,
	APS_Graphics = 1<<1,
	APS_Name = 1<<2,
	APS_Overlay = 1<<3
};

// Material constants
// Material Density Levels
const int32_t
	C4M_Vehicle = 100,
	C4M_Solid = 50,
	C4M_SemiSolid = 25,
	C4M_Liquid = 25,
	C4M_Background = 0;

const int32_t MNone = -1;


// Object size
const int32_t FullCon = 100000;

#endif // INC_C4Constants
