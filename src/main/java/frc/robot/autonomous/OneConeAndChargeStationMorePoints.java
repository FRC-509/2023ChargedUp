package frc.robot.autonomous;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ChargeStation;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeAndChargeStationMorePoints extends SequentialCommandGroup {
	public OneConeAndChargeStationMorePoints(Arm arm, Claw claw, Swerve swerve, Pigeon2 gyro) {
		addCommands(
				new InstantCommand(() -> claw.toggleClaw(), claw),
				new FunctionalCommand(() -> {
				}, () -> arm.setPivotDegrees(100),
						(end) -> {
							arm.setPivotOutput(0);
						},
						() -> false, arm).withTimeout(1.1),
				new FunctionalCommand(() -> {
				}, () -> arm.setExtensionPosition(505),
						(end) -> {
							arm.stopExtensionMotor();
						},
						() -> false, arm).withTimeout(3),
				new InstantCommand(() -> claw.toggleClaw(), claw),
				new WaitCommand(0.5),
				new FunctionalCommand(() -> {
				}, () -> arm.setExtensionPosition(-455),
						(end) -> {
							arm.stopExtensionMotor();
						},
						() -> false, arm).withTimeout(3),
				new FunctionalCommand(() -> {
				}, () -> arm.setPivotDegrees(0),
						(end) -> {
							arm.setPivotOutput(0);
						},
						() -> false, arm).withTimeout(1.1),
				new DriveCommand(swerve, -0.5, 0, 0, false).withTimeout(2),
				new ChargeStation(swerve, gyro, 1.0));
	}
}