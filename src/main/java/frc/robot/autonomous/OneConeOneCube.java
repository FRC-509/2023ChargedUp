package frc.robot.autonomous;

import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;

public class OneConeOneCube extends SequentialCommandGroup {
	public OneConeOneCube(Arm arm, Claw claw, Swerve swerve) {
		PathPlannerTrajectory trajectory = PathPlanner.loadPath("line",
				new PathConstraints(Constants.maxSpeed / 3, 3.2 / 2));
		SwerveAutoBuilder builder = new SwerveAutoBuilder(
				swerve::getRawOdometeryPose,
				swerve::resetOdometry,
				Constants.swerveKinematics,
				new PIDConstants(3.0, 0, 0),
				new PIDConstants(1.3, 0.3, 0.1),
				swerve::setModuleStates,
				Map.of(),
				swerve);
		SequentialCommandGroup drive = new SequentialCommandGroup(
				new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialHolonomicPose()),
						swerve),
				builder.followPath(trajectory),
				new InstantCommand(() -> swerve.drive(new Translation2d(), 0, false, true), swerve));
		// new InstantCommand(() -> swerveSubsystem.setTargetHeading(0),
		// swerveSubsystem),
		// new PickUpCubeFromGround(armSubsystem, clawSubsystem));
		addCommands(
				new OneCone(arm, claw, swerve));
	}
}
