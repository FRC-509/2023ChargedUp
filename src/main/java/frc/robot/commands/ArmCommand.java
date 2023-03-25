package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.util.Debug;

public class ArmCommand extends CommandBase {
	private final Arm s_Arm;
	private final DoubleSupplier rotationSup;
	private final DoubleSupplier extensionSup;
	private BooleanSupplier rawOutputSup;
	private boolean inRawOutput;

	public ArmCommand(
			Arm s_Arm,
			DoubleSupplier rotationSup,
			DoubleSupplier extensionSup,
			BooleanSupplier rawOUtputSup) {
		this.s_Arm = s_Arm;
		addRequirements(s_Arm);

		this.rotationSup = rotationSup;
		this.extensionSup = extensionSup;
		this.rawOutputSup = rawOUtputSup;
	}

	double pivot = 0.0d;
	double extension = 0.0d;

	@Override
	public void execute() {

		boolean isValid = s_Arm.isValidState(s_Arm.getPivotDegrees(), s_Arm.getArmLength());
		SmartDashboard.putNumber("bias height:", s_Arm.getBiasedArmHeight());
		SmartDashboard.putNumber("bias height from ground:", s_Arm.getBiasedHeightFromGround());
		SmartDashboard.putNumber("bias base:", s_Arm.getBiasedArmBase());
		SmartDashboard.putBoolean("is valid state:", isValid);
		SmartDashboard.putNumber("height: ", s_Arm.getHeight());
		SmartDashboard.putNumber("height from ground: ", s_Arm.getHeightFromGround());
		SmartDashboard.putNumber("arm length: ", s_Arm.getArmLength());
		SmartDashboard.putNumber("height limit: ", s_Arm.getHeightLimit());
		SmartDashboard.putNumber("extension position: ", s_Arm.getExtensionPosition());

		s_Arm.setPivotOutput(rotationSup.getAsDouble());

		if (rawOutputSup.getAsBoolean()) {
			inRawOutput = true;
			s_Arm.setExtensionRawOutput(extensionSup.getAsDouble());
		} else {
			if (inRawOutput) {
				s_Arm.resetExtensionPosition();
			}

			inRawOutput = false;
			s_Arm.setExtensionOutput(extensionSup.getAsDouble());
		}

	}
}