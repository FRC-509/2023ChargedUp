package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase {
	private final Arm s_Arm;
	private final DoubleSupplier rotationSup;
	private final DoubleSupplier extensionSup;
	private final BooleanSupplier resetExtension;

	public ArmCommand(
			Arm s_Arm,
			DoubleSupplier rotationSup,
			DoubleSupplier extensionSup,
			BooleanSupplier resetExtension) {
		this.s_Arm = s_Arm;
		addRequirements(s_Arm);

		this.rotationSup = rotationSup;
		this.extensionSup = extensionSup;
		this.resetExtension = resetExtension;
	}

	@Override
	public void execute() {

		/*
		 * if (resetExtension.getAsBoolean()) {
		 * s_Arm.resetExtensionSensorPosition();
		 * }
		 */

		s_Arm.setPivotOutput(rotationSup.getAsDouble());

		if (Constants.isExtensionClosedLoop) {
			s_Arm.setExtensionPosition(extensionSup.getAsDouble());
		} else {
			s_Arm.setExtensionRaw(extensionSup.getAsDouble());
		}
	}
}
