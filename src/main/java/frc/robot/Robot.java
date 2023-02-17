// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.DriveCommand;
import frc.robot.Constants.Chassis;

public class Robot extends TimedRobot {
  // Joysticks
  private Joystick leftJoystick = new Joystick(0);
  private Joystick rightJoystick = new Joystick(1);
  private GenericHID operatorController = new GenericHID(2);

  // Subsystems
  private Swerve drivetrainSubsystem;
  private Arm armSubsystem;
  private Intake intakeSubsystem;

  // Gyroscope
  private Pigeon2 gyro = new Pigeon2(Constants.DeviceId.gyro);

  //
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    armSubsystem = new Arm();
    intakeSubsystem = new Intake();
    drivetrainSubsystem = new Swerve(gyro);

    // drivetrainSubsystem.setDefaultCommand(new DriveCommand(
    // drivetrainSubsystem,
    // () -> leftJoystick.getY(),
    // () -> leftJoystick.getX(),
    // () -> rightJoystick.getX(),
    // () -> false));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double translationVal = MathUtil.applyDeadband(leftJoystick.getY(), 0.1);
    double strafeVal = MathUtil.applyDeadband(leftJoystick.getX(), 0.1);
    double rotationVal = MathUtil.applyDeadband(rightJoystick.getX(), 0.1);

    drivetrainSubsystem.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConfig.maxSpeed),
        rotationVal * Constants.SwerveConfig.maxAngularSpeed,
        false);

    // armSubsystem.setPivotOutput(operatorController.getRawAxis(0) / 2.0d);
    // armSubsystem.setExtensionOutput(operatorController.getRawAxis(1));

    // if (rightJoystick.getRawButtonPressed(1)) {
    // intakeSubsystem.toggleIntake();
    // }
  }

  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /* /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
