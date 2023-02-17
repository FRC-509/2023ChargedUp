package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawCommand extends CommandBase{
  private final Claw clawSubsystem;
  private BooleanSupplier toggle;

  public ClawCommand(Claw claw, BooleanSupplier toggle){
    addRequirements(claw);
    clawSubsystem = claw;
    this.toggle = toggle;
  }

  @Override
  public void execute() {
    if (toggle.getAsBoolean()) {
      clawSubsystem.openClose();
    }
  }
}