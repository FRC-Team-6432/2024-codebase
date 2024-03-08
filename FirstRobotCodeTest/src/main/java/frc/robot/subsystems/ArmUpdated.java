// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ArmUpdated extends SubsystemBase {
  /** Creates a new Arm (Elliot's Version). */
  final int LEFT_MOTOR_ID = 0;
  final int RIGHT_MOTOR_ID = 1;
  final int ENCODER_CHANNEL = 0;

  final double LIMELIGHT_MOUNT_HEIGHT = 1270;
  final double ARM_BASE_HEIGHT = 1270;
  final double ARM_LENGTH = 582;

  final double LIMELIGHT_MOUNT_ANGLE = 90 - Math.toDegrees(Math.atan(37/54));
  final double SHOOTER_ARM_ANGLE = 110;

  final double INTAKE_ENCODER_VALUE = 0.143; // Encoder value when arm is at 0 deg to horizontal ie intake position
  final double MAX_ENCODER_VALUE = INTAKE_ENCODER_VALUE+0.3; // Max arm is allowed to go back

  WPI_TalonSRX motorLeft = new WPI_TalonSRX(LEFT_MOTOR_ID);
  WPI_TalonSRX motorRight = new WPI_TalonSRX(RIGHT_MOTOR_ID);
  DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ENCODER_CHANNEL);

  double kp = 8;
  double ki = 0;
  double kd = 0;

  PIDController pidController = new PIDController(kp, ki, kd);

  public NetworkTable table;

  public ArmUpdated() {
    motorRight.setInverted(true);
    motorLeft.follow(motorRight);
    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public double getAngleToGoal() {
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    
    double tagHeight = 1367;

    double heightFromLimelight = tagHeight-LIMELIGHT_MOUNT_HEIGHT;
    double heightFromArmBase = tagHeight - ARM_BASE_HEIGHT;

    double angleFromLimelight = LIMELIGHT_MOUNT_ANGLE + y;
    
    double horizontalDistance = heightFromLimelight / Math.tan(Math.toRadians(angleFromLimelight));

    double armBaseTargetDistance = Math.sqrt(heightFromArmBase*heightFromArmBase + horizontalDistance*horizontalDistance);
    double angleFromArmBase = Math.toDegrees(Math.atan(heightFromArmBase/horizontalDistance));

    // double limelightTargetDistance = heightFromLimelight / Math.sin(Math.toRadians(angleFromLimelight));

    double theta = Math.toDegrees(Math.asin((ARM_LENGTH*Math.sin(Math.toRadians(180-SHOOTER_ARM_ANGLE)))/armBaseTargetDistance));

    double armAngle = 180 - angleFromArmBase + theta - SHOOTER_ARM_ANGLE;
    return armAngle;
  }

  public void setArmToAngle(double angleDeg) {
    double position = angleDeg/360 + INTAKE_ENCODER_VALUE;
    motorLeft.set(ControlMode.Position, pidController.calculate(boreEncoder.getDistance(), position));  
  }

  public void climb(XboxController controller){
      motorLeft.set(ControlMode.Velocity, controller.getLeftTriggerAxis());
      motorLeft.set(ControlMode.Velocity, -controller.getRightTriggerAxis());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
