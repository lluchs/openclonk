/*--
	Plane part: Propeller
	Author: Sven2

	Used to construct the plane
--*/

#include Library_PlanePart

private func Hit()
{
	Sound("Hits::Materials::Wood::WoodHit*");
}

public func Definition(proplist def)
{
}

public func IsPlanePart() { return true; }

local Collectible = true;
local Name = "$Name$";
local Description = "$Description$";
local HitPoints = 20;
