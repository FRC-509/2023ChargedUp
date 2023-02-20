
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  private final Intake intakeSubsystem;
  private final DoubleSupplier armAngle;
  private final DoubleSupplier speedSupplier;
  private final boolean spinIntake;
  public IntakeCommand(Intake intakeSubsystem, DoubleSupplier armAngle, DoubleSupplier speedSupplier, boolean spinIntake) {
    this.intakeSubsystem = intakeSubsystem;
    this.armAngle = armAngle;
    this.speedSupplier = speedSupplier;
    this.spinIntake = spinIntake;
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void initialize() {
    this.intakeSubsystem.drop();
  }

  @Override
  public void execute() {
    if (spinIntake){
      this.intakeSubsystem.spin(this.speedSupplier.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.spin(0);
    this.intakeSubsystem.retract();
  }
}