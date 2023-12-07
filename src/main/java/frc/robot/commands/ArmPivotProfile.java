package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

public class ArmPivotProfile extends TrapezoidProfileCommand {
	public ArmPivotProfile(double targetPivot, Arm arm) {
		super(new TrapezoidProfile(
				new TrapezoidProfile.Constraints(Constants.Arm.maxPivotSpeed, 15),
				new TrapezoidProfile.State(targetPivot, 0),
				new TrapezoidProfile.State(arm.getPivotDegrees(), arm.getPivotVelocityDegreesPerSecond())),
				arm::setState,
				arm);
	}
}