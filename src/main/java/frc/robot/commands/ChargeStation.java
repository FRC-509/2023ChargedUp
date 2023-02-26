
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Charge Station Command
public class ChargeStation extends CommandBase {

  // Finished flag
  private boolean finished;

  // Pitch buffer
  private double pitchBuffer;

  // Move increment
  private double moveIncrement;

  // Swerve drive reference
  private Swerve swerve;

  // Gyro reference
  private Pigeon2 gyro;

  // Odometry reference
  // private OdometryDeprecated odometry;


  // Constructor
  public ChargeStation(double pitchBuffer_, double moveIncrement_, Swerve swerve_, Pigeon2 gyro_) {

	finished = false;
	this.pitchBuffer = pitchBuffer_;
	this.moveIncrement = moveIncrement_;

	// Get reference to swerve
	this.swerve = swerve_;
	addRequirements(this.swerve);

	// Get reference to gyro
	this.gyro = gyro_;
  }

  // Initialize
  @Override
  public void initialize() {
	
	// Makes robot parallel to charge station before driving
	double robotRotationDegrees = swerve.getPose().getRotation().getDegrees();
	double difference = 90 - robotRotationDegrees;
	swerve.drive(new Translation2d(), difference, true);
  
	// Start driving onto charge station
	double distance = moveIncrement / 2;
	Translation2d driveTranslation = new Translation2d(distance, 0).times(Constants.maxSpeed);
	swerve.drive(driveTranslation, 0, true);
  }

  // Execute
  @Override
  public void execute() {

	// Get gyro pitch change from last frame
	double pitch = gyro.getPitch();

	// Move on charge station
	if (pitch <= pitchBuffer || pitch >= -pitchBuffer) {

	  // Count as stable if within pitch buffer
	  finished = true;

	} else {

	  // Move robot to center of charge station
	  // Use move increment and pitch to find correct distance
	  double distance = moveIncrement * pitch;
	  Translation2d driveTranslation = new Translation2d(distance, 0).times(Constants.maxSpeed);
	  swerve.drive(driveTranslation, 0, true);
	}
  }

  // Is Finished
  @Override
  public boolean isFinished() {

	// Return finished
	return finished;
  }

  @Override
  public void end(boolean wasInterrupted) {
	// Enter the drivetrain's X-Stance to lock our position on the station.
	swerve.enterXStance();
  }
}