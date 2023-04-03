package frc.robot;

import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.TimeStamp;
import frc.robot.util.Device;
import frc.robot.util.controllers.JoystickController;
import frc.robot.util.controllers.LogitechController;
import frc.robot.util.controllers.JoystickController.StickButton;
import frc.robot.util.controllers.LogitechController.LogiButton;
import frc.robot.util.drivers.PigeonWrapper;
import frc.robot.util.math.Utils;
import frc.robot.vision.*;
import frc.robot.vision.VisionTypes.TargetType;

import java.io.IOException;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
	public final JoystickController leftStick = new JoystickController(1);
	public final JoystickController rightStick = new JoystickController(0);
	public final LogitechController controller = new LogitechController(2);

	public static TimeStamp timeStamp = new TimeStamp();

	public final LimelightWrapper limelight = new LimelightWrapper(Device.limelightName);
	public final PigeonWrapper pigeon = new PigeonWrapper(Device.pigeonId, Device.CanBus, 180.0d);
	public AprilTagFieldLayout fieldLayout;

	public final Swerve swerveSubsystem;
	public final Arm armSubsystem;
	public final Claw clawSubsystem;
	public final UsbCamera usbCamera = new UsbCamera(Device.webcamName, 1);
	private final SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		if (RobotBase.isReal()) {
			UsbCamera cam = CameraServer.startAutomaticCapture();
			cam.setFPS(15);
			cam.setResolution(10, 10);
		}

		// Initialize and configure the gyroscope.
		this.pigeon.configFactoryDefault();
		// Initialize subsystems.
		this.swerveSubsystem = new Swerve(timeStamp, pigeon, limelight);
		this.armSubsystem = new Arm();
		this.clawSubsystem = new Claw();

		// Configure button bindings
		this.configureButtonBindings();
		this.addAutonomousRoutines();

		this.zeroGyro();

		// Initialize the AprilTagFieldLayout
		try {
			fieldLayout = new AprilTagFieldLayout(
					Filesystem
							.getDeployDirectory()
							.toPath()
							.resolve("2023-chargedup.json"));
		} catch (IOException e) {
			DriverStation.reportWarning("failed to load april tag field layout json", true);
			fieldLayout = null;
		}
	}

	public double deltaTime() {
		return timeStamp.deltaTime();
	}

	public void configureButtonBindings() {
		timeStamp.setDefaultCommand(new InstantCommand(
				() -> timeStamp.update(),
				timeStamp));

		if (RobotBase.isReal()) {
			// The thrustmaster joysticks on the Driver Station.
			swerveSubsystem.setDefaultCommand(new DriveCommand(
					swerveSubsystem,
					() -> Utils.pow(-leftStick.getY(), 1.0d),
					() -> Utils.pow(-leftStick.getX(), 1.0d),
					() -> -rightStick.getX(),
					() -> false, // leftStick.isDown(StickButton.Bottom),
					() -> rightStick.isDown(StickButton.Bottom),
					() -> rightStick.isDown(StickButton.Trigger),
					() -> leftStick.isDown(StickButton.Trigger)));
		} else {
			swerveSubsystem.setDefaultCommand(new DriveCommand(
					swerveSubsystem,
					() -> -controller.getLeftStickY(),
					() -> -controller.getLeftStickX(),
					() -> -controller.getRightStickX(),
					() -> controller.isDown(LogiButton.Y),
					() -> controller.isDown(LogiButton.X),
					() -> controller.isDown(LogiButton.RTrigger),
					() -> controller.isDown(LogiButton.LTrigger)));
		}

		rightStick.isDownBind(StickButton.Left,
				new AlignWithTarget(
						swerveSubsystem,
						limelight,
						() -> Utils.pow(-leftStick.getY(), 2),
						TargetType.ConeNode));

		leftStick.isDownBind(StickButton.Bottom, new InstantCommand(() -> zeroGyro(), swerveSubsystem));

		clawSubsystem.setDefaultCommand(new ClawIntakeCommand(
				clawSubsystem,
				() -> controller.isPressed(LogiButton.A),
				() -> controller.isDown(LogiButton.RTrigger),
				() -> controller.isDown(LogiButton.LTrigger)));

		armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem,
				() -> MathUtil.applyDeadband(controller.getLeftStickY(),
						Constants.stickDeadband)
						* -Constants.armPivotOperatorCoefficient,
				() -> MathUtil.applyDeadband(controller.getRightStickY(),
						Constants.stickDeadband)
						* -Constants.armExtensionOperatorCoefficient,
				() -> controller.isDown(LogiButton.B),
				() -> controller.isPressed(LogiButton.LBTrigger),
				() -> controller.isPressed(LogiButton.RBTrigger)));

		controller.isPressedBind(LogiButton.X,
				new InstantCommand(() -> Led.setMode(Led.BlinkinLedMode.SOLID_VIOLET)));
		controller.isPressedBind(LogiButton.Y,
				new InstantCommand(() -> Led.setMode(Led.BlinkinLedMode.SOLID_ORANGE)));

	}

	private void addAutonomousRoutines() {

		chooser.addOption("One Cone and Taxi (Stable)",
				new OneConeAndTaxiStable(armSubsystem, clawSubsystem, swerveSubsystem));
		chooser.addOption("One Cone and Charge Station",
				new OneConeAndChargeStation(armSubsystem, clawSubsystem, swerveSubsystem,
						pigeon));
		chooser.addOption("One Cone",
				new OneCone(armSubsystem, clawSubsystem, swerveSubsystem));
		chooser.setDefaultOption("One Cone + Taxi Charge",
				new OneConeAndChargeStationMorePoints(armSubsystem, clawSubsystem,
						swerveSubsystem, pigeon));
		chooser.addOption("One Cone + Taxi + Pick Up Cube + Charge",
				new OneConeAndChargeStationMorePoints(armSubsystem, clawSubsystem,
						swerveSubsystem, pigeon));
		chooser.addOption("Charge Station",
				new ChargeStation(swerveSubsystem, pigeon, -1));
		chooser.addOption("None", null);
		chooser.addOption("PathPlanner testing (DO NOT USE!)", followPath("inTheShop", swerveSubsystem));

		SmartDashboard.putData("Auto Chooser", chooser);
	}

	public static SequentialCommandGroup followPath(String path, Swerve swerve) {
		PathPlannerTrajectory trajectory = PathPlanner.loadPath(path,
				new PathConstraints(Constants.maxSpeed, 3.0));
		SwerveAutoBuilder builder = new SwerveAutoBuilder(swerve::getRawOdometeryPose,
				swerve::resetOdometry,
				Constants.swerveKinematics,
				new PIDConstants(3.5, 0, 0),
				new PIDConstants(1, 0.0, 0.0),
				swerve::setModuleStates,
				Map.of(),
				true,
				swerve);
		return new SequentialCommandGroup(
				new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialHolonomicPose()), swerve),
				builder.followPath(trajectory),
				new InstantCommand(() -> swerve.stopModules(), swerve));
	}

	public void zeroGyro() {
		pigeon.setYaw(0);
		pigeon.zeroGyroBiasNow();
		swerveSubsystem.zeroHeading();
	}

	public void setGyroHeading(double heading) {
		pigeon.setYaw(heading);
		swerveSubsystem.setTargetHeading(heading);
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}
