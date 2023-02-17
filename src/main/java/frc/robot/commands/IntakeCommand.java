
package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
  private final Intake intakeSubsystem;
  private final DoubleSupplier speedSupplier;
  private final boolean boolS;
  public IntakeCommand(Intake intakeSubsystem, DoubleSupplier speedSupplier, boolean boolS) {
    this.intakeSubsystem = intakeSubsystem;
    this.speedSupplier = speedSupplier;
    this.boolS = boolS;
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void initialize() {
    this.intakeSubsystem.drop();
  }

  @Override
  public void execute() {
    if (boolS){
    this.intakeSubsystem.spin(this.speedSupplier.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.intakeSubsystem.spin(0);
    this.intakeSubsystem.retract();
  }
}