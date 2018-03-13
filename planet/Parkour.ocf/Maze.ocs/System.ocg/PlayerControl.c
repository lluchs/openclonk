/*--
		Override screenshot functionality
--*/

global func PlayerControl(int plr, int ctrl, ...)
{
	if (ctrl == CON_TryScreenshot)
	{
		CustomMessage(Format("$MsgCheater$", GetTaggedPlayerName(plr)));
		Sound("UI::Error", true);
		//var crew = GetCursor(plr); - used for cheating
		//if (crew) crew->Punch(crew, 50);
		return true;
	}
	return _inherited(plr, ctrl, ...);
}