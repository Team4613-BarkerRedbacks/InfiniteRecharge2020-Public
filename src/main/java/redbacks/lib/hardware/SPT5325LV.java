package redbacks.lib.hardware;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class SPT5325LV extends PWMSpeedController {
	public SPT5325LV(final int channel) {
		super(channel);

	    setBounds(2.000, 1.550, 1.500, 1.450, 1.000);
	    setPeriodMultiplier(PeriodMultiplier.k4X);
	    setSpeed(0.0);
	    setZeroLatch();

	    SendableRegistry.setName(this, "SPT5325LV", getChannel());
	}
}
