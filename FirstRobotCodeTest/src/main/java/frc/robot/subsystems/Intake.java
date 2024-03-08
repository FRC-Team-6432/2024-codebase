// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public DigitalInput sensor;
  private VictorSPX intakeMotor = new VictorSPX(5);
  private TalonFX shooterLeft = new TalonFX(4);
  private TalonFX shooterRight = new TalonFX(3);
  
  public Intake() {
    sensor = new DigitalInput(1);
    
  }

  public void yoink(XboxController controller){
    if (sensor.get()){
      intakeMotor.set(ControlMode.Velocity, 0);
    }
    else if(controller.getLeftBumper()){
      intakeMotor.set(ControlMode.Velocity, 1);
    }
    else{
      intakeMotor.set(ControlMode.Velocity, 0);
    }
  }

  public void yeet(XboxController controller){
    if (controller.getRightBumper()){
      shooterLeft.set(1);
      shooterRight.set(-1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
