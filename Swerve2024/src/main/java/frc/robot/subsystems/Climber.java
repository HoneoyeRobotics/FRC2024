// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private CANSparkMax LeftClimber;
  private CANSparkMax RightClimber;

  /** Creates a new Climber. */
  public Climber() {
    LeftClimber = new CANSparkMax(ClimberConstants.LeftClimber, MotorType.kBrushless);
    RightClimber = new CANSparkMax(ClimberConstants.RightClimber, MotorType.kBrushless);
    LeftClimber.setIdleMode(IdleMode.kBrake);
    RightClimber.setIdleMode(IdleMode.kBrake);
    LeftClimber.getEncoder().setPosition(0.0);
    RightClimber.getEncoder().setPosition(0.0);
  }

  public void resetLeftClimberEncoder(){
    LeftClimber.getEncoder().setPosition(0);
  }
  public void resetRightClimberEncoder(){
    RightClimber.getEncoder().setPosition(0);
  }
  public void setLeftClimber(double speed) {
    LeftClimber.set(speed);

    SmartDashboard.putNumber("LeftClimbSpeed", speed);
  }

  public void setRightClimber(double speed) {
    RightClimber.set(speed);

    SmartDashboard.putNumber("RightClimbSpeed", speed);
  }

  public double getRightClimberPosition() {
    SmartDashboard.putNumber("RightClimbPos", RightClimber.getEncoder().getPosition());
    return RightClimber.getEncoder().getPosition();

  }

  public double getLeftClimberPosition() {
    SmartDashboard.putNumber("LeftClimbPos", LeftClimber.getEncoder().getPosition());
    return LeftClimber.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("LeftClimbPos", LeftClimber.getEncoder().getPosition());
    // SmartDashboard.putNumber("RightClimbPos", RightClimber.getEncoder().getPosition()); 
  }
}
