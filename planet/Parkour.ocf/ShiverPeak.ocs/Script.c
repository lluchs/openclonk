/*-- 
	Shiver Peak
	Authors: Ringwaul, Asmageddon
	
	Climb to the top of the peak.
--*/


protected func Initialize()
{
	// Parkour goal: from bottom to top.
	var goal = CreateObject(Goal_Parkour, 0, 0, NO_OWNER);
	// Start at bottom of the map.
	var sx = LandscapeWidth() / 2, sy = LandscapeHeight() - 120;
	goal->SetStartpoint(sx, sy);
	// Finish exactly at the peak of the mountain, so find it.
	var fx = LandscapeWidth() / 2, fy = 0;
	while (PathFree(0, fy + 20, LandscapeWidth(), fy + 20) && fy < LandscapeHeight())
		fy += 10;
	goal->SetFinishpoint(fx, fy, true);
	// All checkpoints are ordered and provide respawn.
	// Checkpoints form a more or less straight line from start to finish.
	var cp_mode = PARKOUR_CP_Check | PARKOUR_CP_Respawn | PARKOUR_CP_Ordered | PARKOUR_CP_Team;
	var cp_count = 6;
	var dx = (fx - sx) / (cp_count + 1);
	var dy = (fy - sy) / (cp_count + 1);
	var x, y;
	// Create cp_count checkpoints.
	for (var cp_num = 1; cp_num <= cp_count; cp_num++)
	{
		x = sx + cp_num * dx + Random(200) - 100;
		y = sy + cp_num * dy;
		// Move around a little to find a decent location (150) tries.
		var move = 10;
		for (var i = 0; i < 150; i++)
		{
			x += Random(6 * move) - 3 * move;
			y += Random(2 * move) - move;
			if (!GBackSolid(x, y))
				break;
		}
		var cp = goal->AddCheckpoint(x, y, cp_mode);
		cp->ClearCPBack();
	}
	
	MapBottomFix();
	
	/* --Environmental Effects-- */

	// Time
	var time = CreateObject(Time);
	time->SetCycleSpeed(0);
	time->SetTime(900);

	// Snow
	AddEffect("Snowfall", nil, 1, 2);
	//Wind
	Sound("Environment::WindLoop", true, 40, nil, +1);

	// Spawn some chests with items around the map.
	for (var i = 0; i < 10; i++)
		PlaceChest();

	return;
}

private func PlaceChest()
{
	var pos = FindLocation(Loc_Material("Tunnel"), Loc_Wall(CNAT_Bottom));
	var chest = CreateObjectAbove(Chest, pos.x, pos.y);

	var spawnlist = [PowderKeg, DynamiteBox, Dynamite, Loam, Pickaxe, Ropeladder, Torch];
	for (var i = 0; i < 5; i++)
		chest->CreateContents(spawnlist[Random(GetLength(spawnlist))]);
	return;
}

private func MapBottomFix()
{
	for (var i = 1; i < LandscapeWidth(); i++)
	{
		var sway = Sin(i, 10);
		if (GetMaterial(i, LandscapeHeight() - 1) == Material("Tunnel"))
			DrawMaterialQuad("Granite", i - 1, LandscapeHeight() - 13 + sway, i + 1, LandscapeHeight() - 13 + sway, i + 1, LandscapeHeight(), i - 1, LandscapeHeight(), true);
	}
	return;
}

protected func InitializePlayer(int player)
{
	SetPlayerTeam(player, 1);
	return;
}

// Callback from parkour goal: give the player useful tools on respawn.
protected func OnPlayerRespawn(int plr, object cp)
{
	var clonk = GetCrew(plr);
	clonk->CreateContents(Shovel);
	clonk->CreateContents(Loam);
	clonk->CreateContents(Dynamite);
	clonk->CreateContents(Torch);
	return;
}

global func FxSnowfallTimer(object target, effect, int timer)
{
	CastPXS("Snow", 5, 1, RandomX(0, LandscapeWidth()), 1);
	return 1;
}

// This scenario has day and night but no celestial objects.
public func HasNoCelestials() { return true; }

