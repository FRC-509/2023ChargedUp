package frc.robot.autonomous;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChargeStation;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeAndChargeStationMorePoints extends SequentialCommandGroup {
	public OneConeAndChargeStationMorePoints(Arm arm, Claw claw, Swerve swerve, Pigeon2 gyro) {
		addCommands(
				new OneCone(arm, claw, swerve),
				new DriveCommand(swerve, -0.5, 0, 0, false).withTimeout(2),
				new ChargeStation(swerve, gyro, 1.0));
	}
}