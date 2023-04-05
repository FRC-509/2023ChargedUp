package frc.robot.commands;

import java.util.concurrent.ExecutionException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;

public class PlaceCube extends SequentialCommandGroup {
	public PlaceCube(Arm arm) {
		addCommands(
				// new ExtendArm(arm, 0),
				new RotateArm(arm, 95),
				new ExtendArm(arm, 75));
	}
}
