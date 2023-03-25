package frc.robot.subsystems;

import java.util.Optional;

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
import frc.robot.util.Debug;
import frc.robot.util.drivers.PigeonWrapper;
import frc.robot.util.math.Conversions;
import frc.robot.util.math.Interpolator;
import frc.robot.util.math.Utils;
import frc.robot.vision.LimelightWrapper;

public class Swerve extends SubsystemBase {

	public SwerveModule[] swerveModules;
	public SwerveDrivePoseEstimator swerveDrivePoseEstimator;
	private Field2d field2d;
	private LimelightWrapper limelight;
	private PigeonWrapper pigeon;

	private TimeStamp timeStamp;
	private Interpolator rotationInterplator;

	// the angle the robot SHOULD face
	private double targetHeading;
	private PIDController rotationPassivePID = new PIDController(1.1, 0.8, 0.05);
	private PIDController rotationAggressivePID = new PIDController(1.2, 0.3, 0.1);
	private double rotationTimeout = 0.5;
	private Timer timer;

	public Swerve(TimeStamp stamp, PigeonWrapper pigeon, LimelightWrapper limelight) {
		this.timer = new Timer();
		timer.reset();
		timer.start();

		this.pigeon = pigeon;
		this.limelight = limelight;
		this.timeStamp = stamp;
		this.rotationInterplator = new Interpolator(timeStamp, Constants.maxAngularVelocity);
		this.targetHeading = pigeon.getAbsoluteZero();

		swerveModules = new SwerveModule[] {
				new SwerveModule(Constants.s_frontLeft),
				new SwerveModule(Constants.s_backLeft),
				new SwerveModule(Constants.s_backRight),
				new SwerveModule(Constants.s_frontRight),
		};

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

	private void serializeRotationPID() {
		SmartDashboard.putNumber("Relative Yaw: ", pigeon.getRelativeYaw());
		SmartDashboard.putNumber("Absolute Yaw: ", pigeon.getAbsoluteYaw());
		SmartDashboard.putNumber("Target Yaw: ", targetHeading);
		SmartDashboard.putNumber("interpolation: ", rotationInterplator.getPosition());
		SmartDashboard.putNumber("timer: ", timer.get());

		double kP = Debug.debugNumber("rot P", 0.0);
		double kI = Debug.debugNumber("rot I", 0.0);
		double kD = Debug.debugNumber("rot D", 0.0);

		rotationAggressivePID.setP(kP);
		rotationAggressivePID.setI(kI);
		rotationAggressivePID.setD(kD);
	}

	public void drive(Translation2d translationMetersPerSecond, double rotationRadiansPerSecond,
			boolean fieldRelative, boolean omitRotationCorrection) {

		rotationInterplator.setPoint(rotationRadiansPerSecond);

		double rotationOutput;
		double interpolatedRotation = rotationInterplator.update();
		boolean hasRotationInput = !Utils.withinDeadband(rotationRadiansPerSecond, 0, 0.01);

		if (hasRotationInput) {
			timer.reset();
		}

		double speed = Math.hypot(translationMetersPerSecond.getX(), translationMetersPerSecond.getY());
		SmartDashboard.putBoolean("correcting Heading", speed < Constants.minHeadingCorrectionSpeed);
		SmartDashboard.putNumber("spped", speed);

		if ((speed != 0 && speed < Constants.minHeadingCorrectionSpeed) || omitRotationCorrection || hasRotationInput
				|| timer.get() < rotationTimeout) {
			setTargetHeading(pigeon.getRelativeYaw());
			rotationOutput = interpolatedRotation;
		} else {
			double delta = pigeon.getRelativeYaw() - targetHeading;
			if (delta > 180.0d) {
				delta -= 360.0d;
			}
			if (delta < -180.0d) {
				delta += 360.0d;
			}

			double outputDegrees = Math.abs(delta) > 5.0d
					? Constants.Voltage * rotationAggressivePID.calculate(delta)
					: Constants.Voltage * rotationPassivePID.calculate(delta);

			rotationOutput = Units.degreesToRadians(outputDegrees);
		}

		SwerveModuleState[] moduleStates;

		if (fieldRelative) {
			moduleStates = Constants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
					translationMetersPerSecond.getX(),
					translationMetersPerSecond.getY(),
					rotationOutput,
					getYaw()));
		} else {
			moduleStates = Constants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(
					translationMetersPerSecond.getX(),
					translationMetersPerSecond.getY(),
					rotationOutput));
		}

		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
				Constants.maxSpeed);

		for (SwerveModule mod : swerveModules) {
			mod.setDesiredState(moduleStates[mod.moduleNumber]);
		}
	}

	public void enterXStance() {
		swerveModules[0].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(45.0d)));
		swerveModules[1].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(135.0d)));
		swerveModules[2].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(45.0d)));
		swerveModules[3].setDesiredState(new SwerveModuleState(0.0d, Rotation2d.fromDegrees(135.0d)));
	}

	public void setTargetHeading(double heading) {
		targetHeading = heading % 360.0d;
	}

	public void setGyroHeading(double heading) {
		pigeon.setRelativeYaw(heading);
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.maxSpeed);
		ChassisSpeeds speeds = Constants.swerveKinematics.toChassisSpeeds(desiredStates);
		drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond,
				false, false);

		// for (SwerveModule mod : this.swerveModules) {
		// mod.setDesiredState(desiredStates[mod.moduleNumber]);
		// }
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
		return Rotation2d.fromDegrees(pigeon.getRelativeYaw());
	}

	public void resetIntegratedToAbsolute() {
		for (SwerveModule mod : this.swerveModules) {
			mod.resetAngleToAbsolute();
		}
	}

	public void supplyVelocity(double velocityMps) {
		for (SwerveModule mod : this.swerveModules) {
			mod.supplyVelocity(velocityMps);
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

		SmartDashboard.putNumber("yaw", getYaw().getDegrees());
		SmartDashboard.putNumber("pitch", pigeon.getPitch());
		SmartDashboard.putNumber("roll", pigeon.getRoll());
		SmartDashboard.putNumber("odometry-x", this.swerveDrivePoseEstimator.getEstimatedPosition().getX());
		SmartDashboard.putNumber("odometry-y", this.swerveDrivePoseEstimator.getEstimatedPosition().getY());

		// pointing up is -negative Pitch
		// down is +positive Pitch
		// for (SwerveModule module : this.swerveModules) {
		// module.debug();
		// }

		timeStamp.update();
	}

	public void zeroHeading() {
		targetHeading = pigeon.getAbsoluteZero();
	}

	public void setHeadingToGyro() {
		targetHeading = pigeon.getRelativeYaw();
	}
}
