package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawIntakeCommand extends CommandBase {
	private final Claw clawSubsystem;
	private BooleanSupplier toggleClose;
	private BooleanSupplier intake;
	private BooleanSupplier outake;

	public ClawIntakeCommand(Claw claw, BooleanSupplier toggleClose,
			BooleanSupplier intake, BooleanSupplier outake) {
		addRequirements(claw);
		clawSubsystem = claw;
		this.intake = intake;
		this.outake = outake;
		this.toggleClose = toggleClose;
	}

	@Override
	public void execute() {
		if (intake.getAsBoolean()) {
			clawSubsystem.spinIntake(true);
		} else if (outake.getAsBoolean()) {
			clawSubsystem.spinIntake(false);
		} else {
			clawSubsystem.stopIntake();
		}

		if (toggleClose.getAsBoolean()) {
			clawSubsystem.toggleClaw();
		}
	}
}