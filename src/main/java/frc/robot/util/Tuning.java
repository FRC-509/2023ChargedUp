package frc.robot.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Tuning extends CommandBase {
  TuningCommand[] tuners;

  public Tuning(Swerve swerve) {
    this.tuners = new TuningCommand[] {
      new TuningCommand(swerve, 0),
      new TuningCommand(swerve, 1),
      new TuningCommand(swerve,2),
      new TuningCommand(swerve, 3),
    };

    addRequirements(swerve);
  }

  @Override
  public void execute() {
    for (TuningCommand tuner : tuners) {
      tuner.execute();
    }
  }
}
