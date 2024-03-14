// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.time.StopWatch;

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
    if(controller.getLeftBumper()){
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

  public void yeet(XboxController controller){
    if (controller.getRightBumper()){
      timer.reset();
      timer.start();
      shooterLeft.set(-1);
      shooterRight.set(1);
      intakeMotor.set(0);
      while(timer.get()<2){

      }
      intakeMotor.set(0.3);
      
    }
    else{
      shooterLeft.set(0);
      shooterRight.set(0);
    }
  }

  public void autoyeet(){
    
    shooterLeft.set(-1);
    shooterRight.set(1);
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
