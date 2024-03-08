// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< HEAD
=======
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
>>>>>>> 9dff2868b34f4e720c6e23a27bafb1635cd6da47

import edu.wpi.first.math.controller.PIDController;
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

  final double OFFSET_ENCODER = 0; // Encoder value when arm is at 0 deg to horizontal
  WPI_TalonSRX motorLeft = new WPI_TalonSRX(LEFT_MOTOR_ID);
  WPI_TalonSRX motorRight = new WPI_TalonSRX(RIGHT_MOTOR_ID);
  DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ENCODER_CHANNEL);

  double kp = 8;
  double ki = 0;
  double kd = 0;

  PIDController pidController = new PIDController(kp, ki, kd);

  public ArmUpdated() {
    motorRight.follow(motorLeft);
  }

  public void setArmToAngle(double angleDeg) {
    double position = angleDeg/360 + OFFSET_ENCODER;
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
