package frc.robot;

import frc.robot.autonomous.OneConeAndTaxiStable;
import frc.robot.autonomous.OneCone;
import frc.robot.autonomous.OneConeAndChargeStation;
import frc.robot.autonomous.OneConeAndChargeStationMorePoints;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ChargeStation;
import frc.robot.commands.ClawIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AlignWithTarget;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.cameraserver.*;
import frc.robot.subsystems.TimeStamp;
import frc.robot.subsystems.Led.BlinkinLedMode;
import frc.robot.util.Device;
import frc.robot.util.controllers.JoystickController;
import frc.robot.util.controllers.LogitechController;
import frc.robot.util.controllers.JoystickController.StickButton;
import frc.robot.util.controllers.LogitechController.LogiButton;
import frc.robot.util.drivers.PigeonWrapper;
import frc.robot.util.math.Scaling;
import frc.robot.vision.*;
import frc.robot.vision.VisionTypes.TargetType;
import frc.robot.subsystems.Led;
import java.io.IOException;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
	public final JoystickController leftStick = new JoystickController(1);
	public final JoystickController rightStick = new JoystickController(0);
	public final LogitechController controller = new LogitechController(2);

	public static TimeStamp timeStamp = new TimeStamp();

	public static boolean isRedAlliance = true;

	public final LimelightWrapper limelight = new LimelightWrapper(Device.limelightName);
	public final PigeonWrapper pigeon = new PigeonWrapper(30, Device.CanBus, 180.0d);
	public AprilTagFieldLayout fieldLayout;

	public final Swerve swerveSubsystem;
	public final Arm armSubsystem;
	public final Claw clawSubsystem;
	public final UsbCamera usbCamera = new UsbCamera("509cam", 01);
	private final SendableChooser<Command> chooser = new SendableChooser<Command>();

	public RobotContainer() {
		if (RobotBase.isReal()) {
			UsbCamera cam = CameraServer.startAutomaticCapture();
			cam.setFPS(15);
			cam.setResolution(10, 10);
		}

		Led.setMode(BlinkinLedMode.FIXED_BREATH_RED);

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
					() -> Scaling.pow(-leftStick.getY(), 1.0d),
					() -> Scaling.pow(-leftStick.getX(), 1.0d),
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
				new AlignWithTarget(swerveSubsystem, limelight, TargetType.Substation));
		rightStick.isDownBind(StickButton.Right, new AlignWithTarget(swerveSubsystem, limelight, TargetType.ConeNode));
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
		chooser.addOption("PATHPLANNER??!?!", null);

		SmartDashboard.putData("Auto Chooser", chooser);
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

	public Pose2d getEstimatedPose() {
		return swerveSubsystem.getPose();
	}

	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}
