package redbacks.input;

import arachne.lib.hardware.ButtonPanel;

public class OperatorPanel extends ButtonPanel
{
	public enum Button {
		ZERO_HEADING(1),
		DISABLE_INDEXER(2),
		RAISE_HOOKS(6),
		RELEASE_PLATFORM(8),
		RAISE_ROBOT(4);

		private final int buttonID;

		private Button(int buttonID) {
			this.buttonID = buttonID;
		}
	}

	public OperatorPanel(int port) {
		super(port);
	}

	public boolean getButton(Button button) {
		return getButton(button.buttonID);
	}
}
