
package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.vision.VisionTypes.GamePiece;
import frc.robot.vision.VisionTypes.PipelineState;

// Limelight Wrapper
public class LimelightWrapper {

	// Properties
	private String limelightName;
	private PipelineState currentState;
	public Pose3d cameraPose;

	// Constructor
	public LimelightWrapper(String name) {
		this.limelightName = name;
		this.currentState = PipelineState.values()[NetworkTableInstance
				.getDefault()
				.getTable(limelightName)
				.getEntry("pipeline")
				.getNumber(0).intValue()];
	}

	public void setPipeline(PipelineState pipeline) {
		if (currentState == pipeline) {
			return;
		}
		currentState = pipeline;
		NetworkTableInstance
				.getDefault()
				.getTable(limelightName)
				.getEntry("pipeline")
				.setNumber(pipeline.getValue());
	}

	// Set LED state to on (true) or off (false)
	public void setLEDState(boolean state) {
		NetworkTableInstance
				.getDefault()
				.getTable(this.limelightName)
				.getEntry("ledMode")
				.setNumber(state ? 0 : 1);
	}

	public boolean getLEDState() {
		return NetworkTableInstance
				.getDefault()
				.getTable(this.limelightName)
				.getEntry("ledMode")
				.getNumber(0).intValue() == 0;
	}

	public PipelineState getPipeline() {
		return currentState;
	}

	// Get offset funcs
	// public double getCameraFeed( }
	public double getXOffset() {
		return NetworkTableInstance
				.getDefault()
				.getTable(this.limelightName)
				.getEntry("tx")
				.getDouble(0);
	}

	public double getYOffset() {
		return NetworkTableInstance
				.getDefault()
				.getTable(this.limelightName)
				.getEntry("ty")
				.getDouble(0);
	}

	public double getZOffset() {
		return NetworkTableInstance
				.getDefault()
				.getTable(this.limelightName)
				.getEntry("tz")
				.getDouble(0);
	}

	// Checks if camera has target
	public boolean hasTarget() {
		return NetworkTableInstance
				.getDefault()
				.getTable(this.limelightName)
				.getEntry("tv")
				.getDouble(0) != 0;
	}

	// Gets best april tag ID
	public int getBestAprilTagID() {
		double rawTagID = NetworkTableInstance
				.getDefault()
				.getTable(this.limelightName)
				.getEntry("tid")
				.getDouble(0);

		return (int) rawTagID;
	}

	public GamePiece getBestGamePiece() {
		PipelineState oldState = getPipeline();
		setPipeline(PipelineState.MLGamePieces);
		double tclass = NetworkTableInstance
				.getDefault()
				.getTable(this.limelightName)
				.getEntry("tclass")
				.getDouble(0);
		setPipeline(oldState);
		return GamePiece.values()[(int) tclass];
	}

	public Optional<Translation2d> getRobotTransform() {

		// Make sure has targets
		if (!hasTarget()) {
			return Optional.empty();
		}

		// Get raw robot position
		double[] poseData;
		if (DriverStation.getAlliance() == Alliance.Blue) {
			poseData = NetworkTableInstance
					.getDefault()
					.getTable(this.limelightName)
					.getEntry("botpose_wpiblue")
					.getDoubleArray(new double[6]);
		} else {
			poseData = NetworkTableInstance
					.getDefault()
					.getTable(this.limelightName)
					.getEntry("botpose_wpired")
					.getDoubleArray(new double[6]);
		}

		// Get and return final robot position
		Translation2d translation = new Translation2d(poseData[0], poseData[1]);
		return Optional.of(translation);
	}

	// Gets robot position on field using april tags
	public Optional<Pose3d> getRobotPose() {

		// Make sure has targets
		if (!hasTarget()) {
			return Optional.empty();
		}

		// Get raw robot position
		double[] poseData;
		if (DriverStation.getAlliance() == Alliance.Blue) {
			poseData = NetworkTableInstance
					.getDefault()
					.getTable(this.limelightName)
					.getEntry("botpose_wpiblue")
					.getDoubleArray(new double[6]);
		} else {
			poseData = NetworkTableInstance
					.getDefault()
					.getTable(this.limelightName)
					.getEntry("botpose_wpired")
					.getDoubleArray(new double[6]);
		}

		// Get and return final robot position
		Translation3d translation = new Translation3d(poseData[0], poseData[1], poseData[2]);
		Rotation3d rotation = new Rotation3d(poseData[3], poseData[4], poseData[5]);
		Pose3d robotPose = new Pose3d(translation, rotation);
		return Optional.of(robotPose);
	}
}