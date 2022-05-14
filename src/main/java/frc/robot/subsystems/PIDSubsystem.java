// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PIDSubsystem extends SubsystemBase {
  /** Creates a new PIDSubsystem. */
  CANSparkMax testMotor;
  SparkMaxPIDController pid;
  RelativeEncoder sparkEncoder;


 

  public PIDSubsystem() 
  {
      testMotor = new CANSparkMax(1, MotorType.kBrushless);
      testMotor.setSmartCurrentLimit(Constants.current_limit);
      testMotor.setOpenLoopRampRate(Constants.ramp_rate);
      testMotor.enableVoltageCompensation(Constants.voltage_comp);

      pid = testMotor.getPIDController();
      sparkEncoder = testMotor.getEncoder();

      pid.setFF(Constants.DEFAULT_V);
      pid.setP(Constants.DEFAULT_P);
      pid.setI(Constants.DEFAULT_I);
      pid.setD(Constants.DEFAULT_D);


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  


  public void setPIDReference(double speed)
  {
    pid.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PID");
    builder.addDoubleProperty("Current velocity (RPM)", sparkEncoder::getVelocity, null);
  }
}
