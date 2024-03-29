// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public DigitalInput sensor;
  private WPI_VictorSPX intakeMotor = new WPI_VictorSPX(5);
  private TalonFX shooterLeft = new TalonFX(4);
  private TalonFX shooterRight = new TalonFX(3);
  private Timer timer = new Timer();
  
  public Intake() {
    sensor = new DigitalInput(0);
    
  }

  public void yoink(XboxController controller){
    SmartDashboard.putBoolean("intake", false);
    if(controller.getRightBumper()){
      if(!sensor.get()){
        intakeMotor.set(0);
      }
      else{
        SmartDashboard.putBoolean("intake", true);
        intakeMotor.set(0.4);
      }
    }
    else{
      intakeMotor.set(0);
    }
  }

  public boolean getNote(){
      return !sensor.get();
  }

  public void yeet(XboxController controller){
    if (controller.getLeftBumper()){
      shooterLeft.set(-0.8);
      shooterRight.set(0.8);
      intakeMotor.set(0.4);
      
    }
    else{
      shooterLeft.set(0);
      shooterRight.set(0);
    }
  }

  public void autoyoink(){
    intakeMotor.set(0.6);
  }

  public void autoyeet(){
    
    shooterLeft.set(-1);
    shooterRight.set(1);
    intakeMotor.set(0.2);
  }

  public void autoyeet2(){
    shooterLeft.set(-0.7);
    shooterRight.set(0.7);
    intakeMotor.set(0.3);
  }
  public void stop(){
    shooterLeft.set(0);
    shooterRight.set(0);
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("sensor", !sensor.get());
    
    // This method will be called once per scheduler run
  }
}
