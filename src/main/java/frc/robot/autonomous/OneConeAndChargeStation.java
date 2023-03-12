package frc.robot.autonomous;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChargeStation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeAndChargeStation extends SequentialCommandGroup {
	public OneConeAndChargeStation(Arm arm, Claw claw, Swerve swerve, Pigeon2 gyro) {
		addCommands(
				new InstantCommand(() -> claw.toggleClaw(), claw),
				new FunctionalCommand(() -> {
				}, () -> arm.setPivotDegrees(100),
						(end) -> {
							arm.setPivotOutput(0);
						},
						() -> false, arm).withTimeout(2),
				new FunctionalCommand(() -> {
				}, () -> arm.setExtensionPosition(505),
						(end) -> {
							arm.stopExtensionMotor();
						},
						() -> false, arm).withTimeout(4),
				new InstantCommand(() -> claw.toggleClaw(), claw).withTimeout(1.5),
				new FunctionalCommand(() -> {
				}, () -> arm.setExtensionPosition(-455),
						(end) -> {
							arm.stopExtensionMotor();
						},
						() -> false, arm).withTimeout(4),
				new FunctionalCommand(() -> {
				}, () -> arm.setPivotDegrees(0),
						(end) -> {
							arm.setPivotOutput(0);
						},
						() -> false, arm).withTimeout(2),
				new ChargeStation(swerve, gyro, -1.0));
		// new DriveCommand(swerve, -0.5, 0, 0, false).withTimeout(2));
	}
}
