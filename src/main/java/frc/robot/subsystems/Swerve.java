package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.util.Utils;
import frc.robot.vision.LimelightWrapper;

public class Swerve extends SubsystemBase {

	public SwerveModule[] swerveModules;
	public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
	private Field2d field2d;
	private LimelightWrapper limelight;
	private Pigeon2 pigeon;
	private double kNorm = 1.5d;

	// the angle the robot SHOULD face
	private double targetHeading;
	private double timeStamp;
	private PIDController rotationPID = new PIDController(8, 0.5, 0.1);

	private void serializeRotationPID() {
		double kP = Utils.serializeNumber("rot P", 0.0);
		double kI = Utils.serializeNumber("rot I", 0.0);
		double kD = Utils.serializeNumber("rot D", 0.0);
		kNorm = Utils.serializeNumber("kNorm", 1.0);

		rotationPID.setP(kP);
		rotationPID.setI(kI);
		rotationPID.setD(kD);
	}

	public Swerve(Pigeon2 pigeon, LimelightWrapper limelight) {
		this.pigeon = pigeon;
		this.limelight = limelight;

		swerveModules = new SwerveModule[] {
				new SwerveModule(Constants.s_frontLeft),
				new SwerveModule(Constants.s_backLeft),
				new SwerveModule(Constants.s_backRight),
				new SwerveModule(Constants.s_frontRight),
		};

		targetHeading = 0.0;
		timeStamp = Timer.getFPGATimestamp();

		// Pause initialization for the pheonix server to start to prevent dropping CAN
		// frames on init
		Timer.delay(1.0);

		resetIntegratedToAbsolute();
		field2d = new Field2d();

		swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
				Constants.swerveKinematics,
				getYaw(),
				getModulePositions(),
				limelight.getRobotPose().orElse(new Pose3d()).toPose2d());

		Shuffleboard.getTab("Robot Field Position").add(field2d);
	}

	public void drive(Translation2d translationMetersPerSecond, double rotationRadiansPerSecond,
			boolean fieldRelative) {

		serializeRotationPID();

		// amount heading changes in degrees
		double delta = Units.radiansToDegrees(rotationRadiansPerSecond) * (Timer.getFPGATimestamp() - timeStamp);
		timeStamp = Timer.getFPGATimestamp();

		// update headings
		targetHeading += delta / kNorm;

		// compute rotation output in radians
		double pidfOutput = rotationPID.calculate(pigeon.getYaw(), targetHeading);
		double rotationInput = Units.degreesToRadians(pidfOutput);

		SwerveModuleState[] moduleStates;

		if (fieldRelative) {
			moduleStates = Constants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
					translationMetersPerSecond.getX(),
					translationMetersPerSecond.getY(),
					rotationInput,
					getYaw()));
		} else {
			moduleStates = Constants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(
					translationMetersPerSecond.getX(),
					translationMetersPerSecond.getY(),
					rotationInput));
		}

		// normalize wheel speeds
		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.maxSpeed);

		for (SwerveModule mod : swerveModules) {
			mod.setDesiredState(moduleStates[mod.moduleNumber]);
		}
	}

	public void enterXStance() {
		for (SwerveModule module : swerveModules) {
			module.setDesiredState(new SwerveModuleState(
					0.0d,
					Rotation2d.fromDegrees(45.0d)));
		}
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxSpeed);

		for (SwerveModule mod : this.swerveModules) {
			mod.setDesiredState(desiredStates[mod.moduleNumber]);
		}
	}

	public Pose2d getPose() {
		return this.swerveDrivePoseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {
		System.out.println("[Swerve::resetOdometry] Passed pose: " + pose.getX() + ", " + pose.getY());
		this.swerveDrivePoseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
	}

	public SwerveModule[] getModules() {
		return this.swerveModules;
	}

	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : this.swerveModules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : this.swerveModules) {
			positions[mod.moduleNumber] = mod.getPosition();
		}
		return positions;
	}

	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(this.pigeon.getYaw());
	}

	public void resetIntegratedToAbsolute() {
		for (SwerveModule mod : this.swerveModules) {
			mod.resetAngleToAbsolute();
		}
	}

	@Override
	public void periodic() {
		Optional<Pose3d> llPose = limelight.getRobotPose();
		if (llPose.isPresent()) {
			swerveDrivePoseEstimator.addVisionMeasurement(llPose.get().toPose2d(), Timer.getFPGATimestamp());
		}
		swerveDrivePoseEstimator.update(getYaw(), getModulePositions());

		field2d.setRobotPose(getPose());

		SmartDashboard.putNumber("theta", getYaw().getDegrees());
		SmartDashboard.putNumber("odometry-x", this.swerveDrivePoseEstimator.getEstimatedPosition().getX());
		SmartDashboard.putNumber("odometry-y", this.swerveDrivePoseEstimator.getEstimatedPosition().getY());

		for (SwerveModule module : this.swerveModules) {
			module.debug();
		}
	}

	public void zeroHeading() {
		targetHeading = 0.0d;
	}
}
