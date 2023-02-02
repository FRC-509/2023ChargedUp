
package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightWrapper {

  // Limelight name
  private String limelightName;

  // Camera pose
  public Pose3d cameraPose;

  // Constructor
  public LimelightWrapper(String name) {
    limelightName = name;
  }

  // LED funcs
  public void turnLEDOff() {
    NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ledMode").setNumber(1);
  }

  public void turnLEDOn() {
    NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ledMode").setNumber(0);
  }

  public void setLED(double state) {
    NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ledMode").setNumber(state);
    SmartDashboard.putNumber("LimeLight State", state);
  }

  // Get offset funcs
  public double getXOffset() {
    return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tx").getDouble(0);
  }

  public double getYOffset() {
    return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("ty").getDouble(0);
  }

  public double getZOffset() {
    return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tz").getDouble(0);
  }

  // Targeting funcs
  public boolean hasTarget() {
    return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tv").getDouble(0) != 0;
  }

  public double[] getCameraTransform() {
    return NetworkTableInstance.getDefault().getTable(limelightName).getEntry("camtran")
        .getDoubleArray((double[]) null);
  }

  // Gets best april tag ID
  public int getBestAprilTagID() {
    Double rawTagID = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("tid").getDouble(0);
    return rawTagID.intValue();
  }

  /// NOTE - CREATE FUNCTION TO GET POSITION OF APRIL TAG USIN FIELD APRIL TAGS
  /// AND TAG ID

  // Gets location for target
  // bpublic Optional<Transform3d> getLineupLocation() {

  // Get target (make sure to configure settings in localhost limelight control
  // panel)
  // Get the height of that target in pixels
  // Compare height of target with a controlled height to distance comparison to
  // get how far from camera the best target is
  // end up with transform of location to line up with poles

  /// NOTE - CALLED OUTSIDE OF THE FUNCTION ITSELF
  // send move/rotate directions to make lineupLocation == robot pose
  // wait till robot is lined up
  // call getTargetPosition()
  // }

  // Get final best target when in linup position
  // public Transform3d getBestTarget() {

  // Get height of target and see if tall or short target
  // Get position of target relative to camera
  // X and Y to center of camera crosshairs
  // Distance from target to camera
  // Return target and what type (Position.TargetType)

  // }

  // Returns a Pose3d containing the position of the robot on the field, polled
  // from the Limelight.
  public Optional<Pose3d> botPose() {
    if (!hasTarget()) {
      return Optional.empty();
    }
    double[] poseData = null;
    if (DriverStation.getAlliance().compareTo(Alliance.Blue) == 0) {
      poseData = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("botpose_wpiblue")
          .getDoubleArray(new double[6]);
    } else {
      poseData = NetworkTableInstance.getDefault().getTable(limelightName).getEntry("botpose_wpired")
          .getDoubleArray(new double[6]);
    }
    Translation3d botTranslation = new Translation3d(poseData[0], poseData[1], poseData[2]);
    Rotation3d botRotation = new Rotation3d(poseData[3], poseData[4], poseData[5]);
    Pose3d outputPose = new Pose3d(botTranslation, botRotation);
    return Optional.of(outputPose);
  }
}