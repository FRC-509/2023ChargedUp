package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.vision.Odometry;

public class OdometryCommand extends CommandBase {

  private SwerveDriveOdometry swerveDriveOdometry;
  private Odometry odometry;

  public OdometryCommand(Odometry odometry, SwerveDriveOdometry swerveOdometry) {
    this.odometry = odometry;
    this.swerveDriveOdometry = swerveOdometry;

    addRequirements(odometry);
  }

  @Override
  public void execute() {
    odometry.updatePose(this.swerveDriveOdometry);
  }

}
