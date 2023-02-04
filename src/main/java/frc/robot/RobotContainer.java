package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.Odometery;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  public Swerve swerveSubsystem;
  public final Odometery odometerySubsystem;
  public static final Joystick leftStick = new Joystick(1);
  public static final Joystick rightStick = new Joystick(0);
  public static Pigeon2 pigeon;
  

  private final GenericHID genericHid = new GenericHID(2);
  private JoystickButton rightTrigger = new JoystickButton(rightStick, 1);
  private JoystickButton leftStickButtonTwo = new JoystickButton(leftStick, 2);

  public RobotContainer() {
    pigeon = new Pigeon2(Constants.pigeonId);
    swerveSubsystem = new Swerve(pigeon);
    odometerySubsystem = new Odometery(pigeon);
  
    // Configure the trigger bindings
    swerveSubsystem.setDefaultCommand(new DriveCommand(
        swerveSubsystem,
        () -> leftStick.getY(),
        () -> leftStick.getX(),
        () -> rightStick.getX(),
        () -> leftStick.getRawButton(2)));

    leftStickButtonTwo.whileTrue(
        new InstantCommand(() -> swerveSubsystem.zeroGyro(),
            swerveSubsystem));
  }

  public void initializeDriveTrain() {
    swerveSubsystem.resetIntegratedToAbsolute();
    PathPlannerServer.startServer(5811);
  }

  public Command getAutonomousCommand() {
    return new TrajectoryBuilderWrapper("New Path").getPathFollowingCommand(swerveSubsystem);
  }
}
