package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ChargeStation;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import frc.robot.util.drivers.PigeonWrapper;

public class OneConeAndChargeStationMorePoints extends SequentialCommandGroup {
	public OneConeAndChargeStationMorePoints(Arm arm, Claw claw, Swerve swerve, PigeonWrapper gyro) {
		addCommands(
				new OneCone(arm, claw, swerve),
				new DriveCommand(swerve, 0.4, 0, 0, false).withTimeout(3.3),
				new DriveCommand(swerve, -0.4, 0, 0, false).withTimeout(0.25),
				new ChargeStation(swerve, gyro, 1.0));
	}
}