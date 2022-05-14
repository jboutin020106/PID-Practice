// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDSubsystem;

public class SetPIDCommand extends CommandBase {
  /** Creates a new SetPIDCommand. */
  PIDSubsystem pid;
  Joystick joy;

  double y;
  int rpm = 500;

  public SetPIDCommand(PIDSubsystem pid1, Joystick j) {
    // Use addRequirements() here to declare subsystem dependencies.
    pid = pid1;
    joy = j;
    addRequirements(pid);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    y = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    y = MathUtil.applyDeadband(joy.getY(), 0.15);
    pid.setPIDReference(y * rpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    y = 0.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PID");
    builder.addDoubleProperty("Setpoint", () -> y * rpm, null);
  }
}
