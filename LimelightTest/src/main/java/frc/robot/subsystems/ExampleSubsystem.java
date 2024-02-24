// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // CONSTANTS
  final double limelightMountHeight = 1270;
  final double armBaseHeight = 1270;
  final double armLength = 582;

  final double limelightMountAngleDeg = 90 - Math.toDegrees(Math.atan(37/54));
  final double shooterAngleToArmDeg = 110;

  public NetworkTable table;
  public ExampleSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    
    double tagHeight = 1367;

    double heightFromLimelight = tagHeight-limelightMountHeight;
    double heightFromArmBase = tagHeight - armBaseHeight;

    double angleFromLimelight = limelightMountAngleDeg + y;
    
    double horizontalDistance = heightFromLimelight / Math.tan(Math.toRadians(angleFromLimelight));

    double armBaseTargetDistance = Math.sqrt(heightFromArmBase*heightFromArmBase + horizontalDistance*horizontalDistance);
    double angleFromArmBase = Math.toDegrees(Math.atan(heightFromArmBase/horizontalDistance));

    // double limelightTargetDistance = heightFromLimelight / Math.sin(Math.toRadians(angleFromLimelight));

    double theta = Math.toDegrees(Math.asin((armLength*Math.sin(Math.toRadians(180-shooterAngleToArmDeg)))/armBaseTargetDistance));

    double armAngle = 180 - angleFromArmBase + theta - shooterAngleToArmDeg;

//post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("Arm Angle", armAngle);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}




