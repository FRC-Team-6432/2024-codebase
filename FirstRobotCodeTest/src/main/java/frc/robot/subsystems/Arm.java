// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
  DutyCycleEncoder armEncoder = new DutyCycleEncoder(4);
  TalonFX armMotor = new TalonFX(8);
  NetworkTable table;
  Rotation2d angleChange = new Rotation2d(0);

  /** Creates a new Arm. */
  public Arm() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    double AbsPos = armEncoder.getAbsolutePosition() - angleChange.getRotations(); //You might want the encoder's absolute position to be negative
    armMotor.setPosition(AbsPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void angleArmShooter(XboxController controller){
    XboxController contoller = controller;
    if (contoller.getYButton()){
      NetworkTableEntry ty = table.getEntry("ty"); 
      double targetOffsetAngle_Vertical = ty.getDouble(0.0);
      double limelightMountAngleDegrees = 25.0; 
      double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
      armMotor.setPosition(angleToGoalDegrees);
    }
  }
  public void angleArmAmp(XboxController controller){
    //IF
    armMotor.setPosition(90);
  }
  public void intakeNote(XboxController controller){
    //IF
    armMotor.setPosition(0);
  }
}
