// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public ColorSensorV3 sensor;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private VictorSPX intakeMotor = new VictorSPX(5);
  private TalonFX shooterLeft = new TalonFX(4);
  private TalonFX shooterRight = new TalonFX(3);
  
  public Intake() {
    sensor = new ColorSensorV3(i2cPort);
    
  }

  public void yoink(XboxController controller){
    if (sensor.getRed() > 700){
      intakeMotor.set(ControlMode.Velocity, 0);
    }
    else if(controller.getLeftBumper()){
      intakeMotor.set(ControlMode.Velocity, 1);
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
