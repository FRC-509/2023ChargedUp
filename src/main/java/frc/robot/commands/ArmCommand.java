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
	private BooleanSupplier lbTriggerSup;
	private BooleanSupplier rbTriggerSup;

	public ArmCommand(
			Arm s_Arm,
			DoubleSupplier rotationSup,
			DoubleSupplier extensionSup,
			BooleanSupplier rawOUtputSup,
			BooleanSupplier lbTriggerSup,
			BooleanSupplier rbTriggerSup) {
		this.s_Arm = s_Arm;
		addRequirements(s_Arm);

		this.rotationSup = rotationSup;
		this.extensionSup = extensionSup;
		this.rawOutputSup = rawOUtputSup;
		this.lbTriggerSup = lbTriggerSup;
		this.rbTriggerSup = rbTriggerSup;
	}

	double pivot = 0.0d;
	double extension = 0.0d;

	@Override
	public void execute() {
		boolean rb = rbTriggerSup.getAsBoolean();
		boolean lb = lbTriggerSup.getAsBoolean();
		SmartDashboard.putNumber("Extension POs", s_Arm.getExtensionPosition());
		if (lb && rb) {
			(new OneConeTeleopHigh(s_Arm, false)).schedule();
			// (new PositionArm(s_Arm, 45, 0.0)).schedule();

		} else if (rb) {
			(new OneConeTeleopMid(s_Arm, true)).schedule();
		} else if (lb) {
			(new OneConeTeleopHigh(s_Arm, true)).schedule();
		}

		if (rawOutputSup.getAsBoolean()) {
			inRawOutput = true;
			s_Arm.setPivotRawOutput(rotationSup.getAsDouble());
			s_Arm.setExtensionRawOutput(extensionSup.getAsDouble());
		} else {
			if (inRawOutput) {
				s_Arm.resetExtensionPosition();
			}

			inRawOutput = false;
			s_Arm.setPivotOutput(rotationSup.getAsDouble());
			s_Arm.setExtensionOutput(extensionSup.getAsDouble());
		}
	}
}