// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonSRX climberLeft = new WPI_TalonSRX(6);
  WPI_TalonSRX climberRight = new WPI_TalonSRX(7);
  public Climber() {
    climberRight.follow(climberLeft);
    climberRight.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(XboxController controller){
    climberLeft.set(controller.getRightTriggerAxis());
  }
  // public void unclimb(XboxController controller){
  //   climberLeft.set(-1*controller.getRightTriggerAxis());
  // }
}
