/*
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  private final Intake intakeSubsystem;
  private final DoubleSupplier speedSupplier;

  public IntakeCommand(Intake intakeSubsystem, DoubleSupplier speedSupplier) {
    this.intakeSubsystem = intakeSubsystem;
    this.speedSupplier = speedSupplier;
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void initialize() {
    this.intakeSubsystem.drop();
  }

  @Override
  public void execute() {
    this.intakeSubsystem.spin(this.speedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.spin(0);
    this.intakeSubsystem.retract();
  }
}
*/
