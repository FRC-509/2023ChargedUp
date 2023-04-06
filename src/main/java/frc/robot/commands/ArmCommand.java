package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.PositionArmTeleop.ArmState;
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
	private BooleanSupplier homeSup;
	private BooleanSupplier groundSup;

	public ArmCommand(
			Arm s_Arm,
			DoubleSupplier rotationSup,
			DoubleSupplier extensionSup,
			BooleanSupplier rawOUtputSup,
			BooleanSupplier lbTriggerSup,
			BooleanSupplier rbTriggerSup,
			BooleanSupplier homeSup,
			BooleanSupplier groundSup) {
		this.s_Arm = s_Arm;
		addRequirements(s_Arm);

		this.rotationSup = rotationSup;
		this.extensionSup = extensionSup;
		this.rawOutputSup = rawOUtputSup;
		this.lbTriggerSup = lbTriggerSup;
		this.rbTriggerSup = rbTriggerSup;
		this.homeSup = homeSup;
		this.groundSup = groundSup;
	}

	double pivot = 0.0d;
	double extension = 0.0d;

	@Override
	public void execute() {
		boolean rb = rbTriggerSup.getAsBoolean();
		boolean lb = lbTriggerSup.getAsBoolean();

		if (homeSup.getAsBoolean()) {
			(new PositionArmTeleop(s_Arm, ArmState.Home)).schedule();
		} else if (groundSup.getAsBoolean()) {
			(new PositionArmTeleop(s_Arm, ArmState.GroudPickup)).schedule();
		} else if (lb && rb) {
			(new PositionArmTeleop(s_Arm, ArmState.Substation)).schedule();
		} else if (rb) {
			(new PositionArmTeleop(s_Arm, ArmState.ConeMid)).schedule();
		} else if (lb) {
			(new PositionArmTeleop(s_Arm, ArmState.ConeHigh)).schedule();
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