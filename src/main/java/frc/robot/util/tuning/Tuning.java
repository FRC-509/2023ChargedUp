package frc.robot.util.tuning;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Tuning extends CommandBase {
	TuningCommandV[] tuners;

	public Tuning(Swerve swerve) {
		this.tuners = new TuningCommandV[] {
				new TuningCommandV(swerve, 0),
				new TuningCommandV(swerve, 1),
				new TuningCommandV(swerve, 2),
				new TuningCommandV(swerve, 3),
		};

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		for (TuningCommandV tuner : tuners) {
			tuner.execute();
		}
	}
}
