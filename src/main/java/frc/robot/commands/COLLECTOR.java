package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Collector.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class COLLECTOR extends SequentialCommandGroup {
	/** Creates a new COLLECTOR. */

	public COLLECTOR(Collector collector, Hopper hopper, Shooter shooter, ArmState armState) {
		// Add your commands in the addCommands() call, e.g.
		addCommands(new COLLECT(collector, hopper),
				new HopperRun(hopper, collector),
				new COLLECTORSTOWSTOP(collector));
	}
}
