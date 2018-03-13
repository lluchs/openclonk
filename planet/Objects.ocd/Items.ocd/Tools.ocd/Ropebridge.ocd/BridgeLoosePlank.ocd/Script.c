/**
	Ropebridge Plank
	
	@author Randrian
*/

protected func Hit()
{
	Sound("Hits::Materials::Wood::WoodHit?");
	return;
}

public func Incineration()
{
	SetClrModulation(RGB(48, 32, 32));
}

public func IsFuel() { return true; }
public func GetFuelAmount(int requested_amount) { return 30; }     // disregard the parameter, because only a complete chunk should be removed 

// Main bridge object is saved.
func SaveScenarioObject() { return false; }


/*-- Properties --*/

local Collectible = 0;
local Name = "$Name$";
local Description = "$Description$";
local BlastIncinerate = 5;
local ContactIncinerate = 1;