package frc.robot;

import frc.robot.autonomous.OneCone;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ClawIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
//import frc.robot.subsystems.Led;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.cameraserver.*;
import frc.robot.subsystems.TimeStamp;
//import frc.robot.subsystems.Led.PatternID;
import frc.robot.util.controllers.JoystickController;
import frc.robot.util.controllers.LogitechController;
import frc.robot.util.controllers.JoystickController.StickButton;
import frc.robot.util.controllers.LogitechController.LogiButton;
import frc.robot.vision.*;
import frc.robot.subsystems.Led;
import java.io.IOException;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.cameraserver.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	public final JoystickController leftStick = new JoystickController(1);
	public final JoystickController rightStick = new JoystickController(0);
	public final LogitechController controller = new LogitechController(2);

	public static TimeStamp timeStamp = new TimeStamp();

	public final LimelightWrapper limelight = new LimelightWrapper(Constants.limelightName);
	public final Pigeon2 pigeon2 = new Pigeon2(30, Constants.CANIvore);
	public AprilTagFieldLayout fieldLayout;

	public final Swerve swerveSubsystem;
	public final Arm armSubsystem;
	public final Claw clawSubsystem;
	public final UsbCamera usbCamera = new UsbCamera("509cam", 1);

	// public final Spark led;
	private final SendableChooser<Command> chooser = new SendableChooser<Command>();
	private final SendableChooser<String> loopTypeForExtension = new SendableChooser<String>();

	public RobotContainer() {
		UsbCamera cam = CameraServer.startAutomaticCapture();
		cam.setFPS(5);
		cam.setResolution(10, 10);
		// CameraServer.startAutomaticCapture(null, 0)
		// usbCamera.setFPS(10);s
		// usbCamera.setResolution(720, 480);
		// CameraServer.startAutomaticCapture();
		// led = new Spark(5);
		// 0)
		// CameraServer.addServer("camera");

		// SmartDashboard.putData(usbCamera.get);
		// Initialize and configure the gyroscope.
		this.pigeon2.configFactoryDefault();

		// Initialize subsystems.
		this.swerveSubsystem = new Swerve(timeStamp, pigeon2, limelight);
		this.armSubsystem = new Arm();
		this.clawSubsystem = new Claw();

		// Configure button bindings
		this.configureButtonBindings();
		this.addAutonomousRoutines();

		this.loopTypeForExtension.addOption("Closed Loop", "CL");
		this.loopTypeForExtension.addOption("Open Loop", "OL");
		SmartDashboard.putData(loopTypeForExtension);

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
				() -> {
					timeStamp.update();
				},
				timeStamp));

		swerveSubsystem.setDefaultCommand(new DriveCommand(
				swerveSubsystem,
				() -> -leftStick.getY(),
				() -> -leftStick.getX(),
				() -> -rightStick.getX(),
				() -> leftStick.getRawButton(2)));

		clawSubsystem.setDefaultCommand(new ClawIntakeCommand(
				clawSubsystem,
				() -> controller.isPressed(LogiButton.A),
				() -> controller.isDown(LogiButton.RTrigger),
				() -> controller.isDown(LogiButton.LTrigger)));

		armSubsystem.setDefaultCommand(new ArmCommand(armSubsystem,
				() -> MathUtil.applyDeadband(controller.getLeftStickY(), Constants.stickDeadband)
						* -Constants.armPivotOperatorCoefficient,
				() -> MathUtil.applyDeadband(controller.getRightStickY(), Constants.stickDeadband)
						* -Constants.armExtensionOperatorCoefficient,
				() -> controller.isPressed(LogiButton.B)));

		leftStick.isDownBind(StickButton.Bottom, new InstantCommand(() -> zeroGyro(), swerveSubsystem));
		controller.isPressedBind(LogiButton.X,
				new InstantCommand(() -> Led.setMode(Led.BlinkinLedMode.SOLID_VIOLET)));
		controller.isPressedBind(LogiButton.Y,
				new InstantCommand(() -> Led.setMode(Led.BlinkinLedMode.SOLID_ORANGE)));
		controller.isPressedBind(LogiButton.B,
				new InstantCommand(() -> Led.setMode((Led.BlinkinLedMode.SOLID_DARK_RED))));

		/*
		 * controller.isPressedBind(LogiButton.X, new InstantCommand(() ->
		 * Led.set(PatternID.OFF)));
		 * controller.isPressedBind(LogiButton.Y, new InstantCommand(() ->
		 * Led.set(PatternID.YELLOW)));
		 * controller.isPressedBind(LogiButton.B, new InstantCommand(() ->
		 * Led.set(PatternID.VIOLET)));
		 */
	}

	private void addAutonomousRoutines() {
		chooser.setDefaultOption("WeekZeroTaxi", new DriveCommand(
				swerveSubsystem,
				1.0,
				0,
				0, true).withTimeout(0.7));
		chooser.addOption("one cone", new OneCone(armSubsystem, clawSubsystem, swerveSubsystem));
		chooser.addOption("None", null);

		SmartDashboard.putData("Auto Chooser", chooser);
	}

	public void zeroGyro() {
		pigeon2.setYaw(0);
		pigeon2.zeroGyroBiasNow();
		swerveSubsystem.zeroHeading();
	}

	public Pose2d getEstimatedPose() {
		return swerveSubsystem.getPose();
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}

	public void onTeleopInit() {
		Constants.isExtensionClosedLoop = loopTypeForExtension.getSelected() == "CL";
	}
}
