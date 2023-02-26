package frc.robot.util;

import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TrajectoryBuilderWrapper {
  private static final PathConstraints pathConstraints = new PathConstraints(
	  Constants.maxSpeed, 3.0d);
  private PathPlannerTrajectory trajectory;

  public TrajectoryBuilderWrapper(String pathName) {
	this.trajectory = PathPlanner.loadPath(pathName, pathConstraints);
  }

  public CommandBase getSwerveControllerCommand(Swerve swerve) {
	SwerveAutoBuilder builder = new SwerveAutoBuilder(
		swerve::getPose, // Pose2d supplier
		swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
		Constants.swerveKinematics, // SwerveDriveKinematics
		new PIDConstants(0.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
										 // PID controllers)
		new PIDConstants(0.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation
		// controller)
		swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
		Map.of(),
		true, // Should the path be automatically mirrored depending on alliance color.
			  // Optional, defaults to true
		swerve // The drive subsystem. Used to properly set the requirements of path following
			   // commands
	);

	return builder.followPath(this.trajectory);
  }

  public Trajectory getTrajectory() {
	return this.trajectory;
  }

  public Command getPathFollowingCommand(Swerve swerve) {
	CommandBase pathFollowingCommand = getSwerveControllerCommand(swerve);
	if (pathFollowingCommand == null) {
	  DriverStation.reportError("no autonomous trajectory path", true);
	}
	return pathFollowingCommand;
  }
}