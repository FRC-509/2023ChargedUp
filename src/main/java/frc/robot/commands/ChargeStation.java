
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Debug;
import frc.robot.util.PIDWrapper;
import frc.robot.util.drivers.PigeonWrapper;
import frc.robot.util.math.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargeStation extends CommandBase {
	private double driveUpSpeed = 2.0d;
	private double timeForBalance = 0.5d;
	private double beginBalancingPitch = 10.0d;
	private PIDWrapper balancePID = new PIDWrapper(0.03, 0.0d, 0.0d, 0.0d);

	private boolean beganBalancing = false;
	private Timer balanceTimer;

	private Swerve swerve;
	private PigeonWrapper gyro;
	private boolean invert;

	// Constructor
	public ChargeStation(Swerve swerve, PigeonWrapper gyro, boolean invert) {
		this.swerve = swerve;
		this.gyro = gyro;
		this.balanceTimer = new Timer();
		// invert determines what direction the robot will face during the command.
		// Pass -1.0 if the robot should approach the charge station while facing away
		// from it, and 1.0 if the robot should approach the charge station while facing
		// it.
		this.invert = invert;
		addRequirements(this.swerve);
	}

	@Override
	public void initialize() {
		// Set the target setpoint to zero degrees.
		// balancePID.setSetpoint(0);
		// // Drive forward for two seconds at 40% of the maximum speed, then stop
		// // (gets us onto the edge of the charge station, starting from the edge of
		// the
		// // community)
		// Translation2d driveTranslation = new Translation2d(invert * 0.3,
		// 0).times(Constants.maxSpeed);
		// swerve.drive(driveTranslation, 0, false, true);
		// Timer.delay(1.5);
		// swerve.drive(new Translation2d(), 0, false, true);

		balanceTimer.reset();
	}

	@Override
	public void execute() {
		double translation = 0.0d;

		if (!Utils.withinDeadband(gyro.getPitch(), 0.0d, 0.1d)) {
			balanceTimer.reset();
		}

		if (Math.abs(gyro.getPitch()) >= beginBalancingPitch) {
			beganBalancing = true;
		}

		if (!beganBalancing) {
			// drive up to charge station
			translation = driveUpSpeed;
		} else {
			translation = balancePID.calculate(gyro.getPitch());
		}

		translation *= invert ? -1.0d : 1.0d;

		Translation2d driveTranslation = new Translation2d(translation, 0.0d);
		swerve.drive(driveTranslation, 0.0d, true, false);

		balancePID.debug("balance");
		driveUpSpeed = Debug.debugNumber("drive up speed", driveUpSpeed);
		timeForBalance = Debug.debugNumber("time for balance", timeForBalance);
		beginBalancingPitch = Debug.debugNumber("begin balancing pitch", beginBalancingPitch);

		// // Get the gyro pitch, pass it to the PID controller and clamp it to 40%
		// speed
		// // (prevents us from flying off the charge station at first)
		// double increment = invert *
		// MathUtil.clamp(balancePID.calculate(gyro.getPitch()), -0.5, 0.5);
		// // Drive forward by the calculated increment
		// Translation2d driveTranslation = new Translation2d(increment, 0);
		// System.out.println("Driving at speed: " + increment);
		// swerve.drive(driveTranslation, 0, false, true);
	}

	@Override
	public boolean isFinished() {
		// Ensure that the command is always running
		return balanceTimer.get() > timeForBalance && beganBalancing;
	}

	@Override
	public void end(boolean wasInterrupted) {
		swerve.enterXStance();
	}
}
