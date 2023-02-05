package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public final class Tuning extends CommandBase {
  static final double step = 0.2;
  static final double testTime = 10.0;
  final SwerveModule module;

  ArrayList<Double> velPoints;
  ArrayList<Double> vltPoints;
  Swerve swerve;
  double curOutput;
  double curVelSum;
  int curSampleCount;

  double curPose;
  double prePose;

  double curTime;
  double preTime;
  double begTime;

  public Tuning(Swerve swerve, int moduleId) {
    addRequirements(swerve);

    velPoints = new ArrayList<>();
    vltPoints = new ArrayList<>();
    this.swerve = swerve;
    begTime = 0.0;
    curSampleCount = 0;
    curOutput = 0.0;
    curVelSum = 0.0;

    module = swerve.getModules()[moduleId];
    prePose = module.getPosition().distanceMeters;

    curTime = begTime;
    preTime = begTime;
  }

  public double deriveData() {
    if (begTime == -1) {
      begTime = Timer.getFPGATimestamp();
    }

    if (curOutput > 1) {
      return curVelSum / curSampleCount;
    }

    module.supplyVoltage(curOutput);

    curTime = Timer.getFPGATimestamp();
    curPose = module.getPosition().distanceMeters;

    if (curTime - begTime > testTime) {
      // add (Volt, vel) data npoint
      vltPoints.add(curOutput);
      velPoints.add(curVelSum / curSampleCount);

      // reset data measurements
      begTime = Timer.getFPGATimestamp();
      curSampleCount = 0;
      curVelSum = 0.0;

      // step voltage
      curOutput += step;
    }

    double vel = getVelocity();

    curVelSum += vel;
    curSampleCount++;

    preTime = curTime;
    prePose = curPose;

    return curVelSum / curSampleCount;
  }

  public double getVelocity() {
    double deltaTime = curTime - preTime;
    double deltaPose = curPose - prePose;

    return deltaPose / deltaTime;
  }

  @Override
  public void initialize() {
    begTime = Timer.getFPGATimestamp();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber(module.moduleNumber + " output", curOutput);
    for (int i = 0; i < velPoints.size(); i++) {
      SmartDashboard.putNumber(module.moduleNumber + " vel" + i, velPoints.get(i));
    }
    SmartDashboard.putNumber(module.moduleNumber + " vel", deriveData());
  }
}
