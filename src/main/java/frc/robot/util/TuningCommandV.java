package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public final class TuningCommandV {
  static final double step = 0.2;
  static final double testTime = 3.0;
  final SwerveModule module;
  final int moduleNumber;

  ArrayList<Double> encoderVelPoints;
  ArrayList<Double> suppliedPercentVelPoints;
  ArrayList<Double> busVltPoints;
  Swerve swerve;
  double curSuppliedPercentVelocity;
  double curVelSum;
  int curSampleCount;

  double curPose;
  double prePose;

  double curTime;
  double preTime;
  double begTime;

  public TuningCommandV(Swerve swerve, int moduleId) {
    encoderVelPoints = new ArrayList<>();
    suppliedPercentVelPoints = new ArrayList<>();
    busVltPoints = new ArrayList<>();
    this.swerve = swerve;
    this.moduleNumber = moduleId;
    begTime = 0.0;
    curSampleCount = 0;
    curSuppliedPercentVelocity = 0.0;
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

    if (curSuppliedPercentVelocity > 1) {
      return curVelSum / curSampleCount;
    }

    if (moduleNumber == 0 || moduleNumber == 1) {
      module.supplyVelocity(curSuppliedPercentVelocity);
    } else {
      module.supplyVelocity(-curSuppliedPercentVelocity);
    }

    curTime = Timer.getFPGATimestamp();
    curPose = module.getPosition().distanceMeters;

    if (curTime - begTime > testTime) {
      // add (% Vel, vel) data npoint
      suppliedPercentVelPoints.add(curSuppliedPercentVelocity);
      encoderVelPoints.add(curVelSum / curSampleCount);
      busVltPoints.add(module.getBusVoltage());

      // reset data measurements
      begTime = Timer.getFPGATimestamp();
      curSampleCount = 0;
      curVelSum = 0.0;

      // step voltage
      curSuppliedPercentVelocity += step;
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

  public void initialize() {
    begTime = Timer.getFPGATimestamp();
  }

  public boolean isFinished() {
    return false;
  }

  public void execute() {
    double suppliedVel = Utils.MPSToFalcon(curSuppliedPercentVelocity * Constants.maxSpeed, Constants.wheelCircumference, Constants.driveGearRatio);
    SmartDashboard.putNumber(module.moduleNumber + " supvel", suppliedVel);
    for (int i = 0; i < encoderVelPoints.size(); i++) {
      SmartDashboard.putNumber(module.moduleNumber + " encvel" + i, encoderVelPoints.get(i));
      SmartDashboard.putNumber(module.moduleNumber + " bus vlt" + i, busVltPoints.get(i));
    }
    SmartDashboard.putNumber(module.moduleNumber + " vel", deriveData());
  }
}
