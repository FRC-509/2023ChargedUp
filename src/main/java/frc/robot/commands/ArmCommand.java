package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmCommand extends CommandBase {
	private final Arm s_Arm;
	private final DoubleSupplier rotationSup;
	private final DoubleSupplier extensionSup;
	private final BooleanSupplier disableSoftStops;
	private double maintainPosition;
	private boolean toggler;

	private boolean hasInput = false;

	public ArmCommand(
			Arm s_Arm,
			DoubleSupplier rotationSup,
			DoubleSupplier extensionSup,
			BooleanSupplier disableSoftStops) {
		this.s_Arm = s_Arm;
		addRequirements(s_Arm);

		this.rotationSup = rotationSup;
		this.extensionSup = extensionSup;
		this.disableSoftStops = disableSoftStops;
	}

	@Override
	public void execute() {
		s_Arm.setPivotOutput(rotationSup.getAsDouble());

		if (extensionSup.getAsDouble() == 0) {
			if (hasInput) {
				maintainPosition = s_Arm.getExtensionPosition();
				s_Arm.setExtensionPosition(maintainPosition);
			}

			hasInput = false;
		} else {
			hasInput = true;
			s_Arm.setExtensionOutput(extensionSup.getAsDouble());
		}
	}
}
