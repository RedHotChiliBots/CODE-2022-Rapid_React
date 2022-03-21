package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Collector.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class COLLECTOR extends SequentialCommandGroup {
	/** Creates a new COLLECTOR. */

	public COLLECTOR(Collector collector, ArmState armState) {
		// Add your commands in the addCommands() call, e.g.
		addCommands(new CollectorArm(collector, ArmState.DEPLOY),
				new CollectorCollect(collector),
				// Hopper
				new CollectorStop(collector),
				new CollectorArm(collector, ArmState.STOW));
	}
}
