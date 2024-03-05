// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmUpdated extends SubsystemBase {
  /** Creates a new Arm (Elliot's Version). */
  final int LEFT_MOTOR_ID = 0;
  final int RIGHT_MOTOR_ID = 1;
  final int ENCODER_CHANNEL = 0;

  final double OFFSET_ENCODER = 0; // Encoder value when arm is at 0 deg to horizontal

  CANSparkMax motorLeft = new CANSparkMax(LEFT_MOTOR_ID, MotorType.kBrushed);
  CANSparkMax motorRight = new CANSparkMax(RIGHT_MOTOR_ID, MotorType.kBrushed);

  DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ENCODER_CHANNEL);

  double kp = 1;
  double ki = 0;
  double kd = 0;

  PIDController pidController = new PIDController(kp, ki, kd);

  public ArmUpdated() {
    motorRight.follow(motorLeft);
  }

  public void setArmToAngle(double angleDeg) {
    double position = angleDeg/360 + OFFSET_ENCODER;
    motorLeft.set(pidController.calculate(boreEncoder.getDistance(), position));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
