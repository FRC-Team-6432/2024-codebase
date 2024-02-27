// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.CANifier.PWMChannel;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class Arm extends SubsystemBase {
  CANSparkMax armMotor1 =new CANSparkMax(8, MotorType.kBrushed);
  AbsoluteEncoder armEncoder;
  
  /** Creates a new Arm. */
  public Arm() {
    armEncoder = armMotor1.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {    
  double AbsPos = armEncoder.getPosition(); 
  SmartDashboard.putNumber("encoder", AbsPos);
  //You might want the encoder's absolute position to be negative

    // This method will be called once per scheduler run
  }
  public void angleArmShooter(XboxController controller){
    XboxController contoller = controller;
    if (contoller.getYButton()){
      NetworkTableEntry ty = table.getEntry("ty"); 
      double targetOffsetAngle_Vertical = ty.getDouble(0.0);
      double limelightMountAngleDegrees = 25.0; 
      double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    }
  }
  public void angleArmAmp(XboxController controller){
    //IF
  }
  public void intakeNote(XboxController controller){
    //IF
  }
}
