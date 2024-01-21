// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotPrefs;

public class Arms extends SubsystemBase {
  /** Creates a new Arms. */

  private CANSparkMax lowerElbowMotor = new CANSparkMax(Constants.ArmConstants.LowerElbowCanID, MotorType.kBrushless);
  private CANSparkMax upperElbowMotor = new CANSparkMax(Constants.ArmConstants.UpperElbowCanID, MotorType.kBrushless);

  private boolean PIDEnabled = true;
  private PIDController lowerElbowPIDContrller;
  private PIDController upperElbowPIDContrller;

  private double lowerElbowMotorSetpoint = 0.0;
  private double upperElbowMotorSetpoint = 0.0;

  public Arms() {

    lowerElbowPIDContrller = new PIDController(RobotPrefs.getLowerElbowP(), RobotPrefs.getLowerElbowI(),
        RobotPrefs.getLowerElbowD());
    lowerElbowPIDContrller.setSetpoint(0.0);
    lowerElbowPIDContrller.setTolerance(RobotPrefs.getLowerElbowTolerance());

    upperElbowPIDContrller = new PIDController(RobotPrefs.getUpperElbowP(), RobotPrefs.getUpperElbowI(),
        RobotPrefs.getUpperElbowD());
    upperElbowPIDContrller.setSetpoint(0.0);
    upperElbowPIDContrller.setTolerance(RobotPrefs.getUpperElbowTolerance());

    lowerElbowMotor.setIdleMode(IdleMode.kBrake);
    upperElbowMotor.setIdleMode(IdleMode.kBrake);

    lowerElbowMotor.setSmartCurrentLimit(40);
    upperElbowMotor.setSmartCurrentLimit(40);
  }

  public void resetLowerElbowEncoder() {
    lowerElbowMotor.getEncoder().setPosition(0.0);
    lowerElbowMotorSetpoint = 0;
  }

  public void resetUpperElbowEncoder() {
    upperElbowMotor.getEncoder().setPosition(0.0);
    upperElbowMotorSetpoint = 0;
  }

  public double getUpperElbowPosition() {
    return upperElbowMotor.getEncoder().getPosition();
  }

  public double getLowerElbowPosition() {
    return lowerElbowMotor.getEncoder().getPosition();
  }

  public void moveLowerElbow(double position, boolean setPosition) {
    if (setPosition)
      lowerElbowMotorSetpoint = position;
    else
      lowerElbowMotorSetpoint += position;
  }

  public void moveUpperElbow(double position, boolean setPosition) {
    if (setPosition)
      upperElbowMotorSetpoint = position;
    else
      upperElbowMotorSetpoint += position;
  }

  public boolean isUpperElbowAtPosition() {
    return upperElbowPIDContrller.atSetpoint();
  }

  public boolean isLowerElbowAtPosition() {
    return lowerElbowPIDContrller.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (PIDEnabled) {
      double lowerRotateSpeed = lowerElbowPIDContrller.calculate(getLowerElbowPosition(), lowerElbowMotorSetpoint);
      lowerElbowMotor.set(lowerRotateSpeed);
      double upperRotateSpeed = upperElbowPIDContrller.calculate(getUpperElbowPosition(), upperElbowMotorSetpoint);
      upperElbowMotor.set(upperRotateSpeed);

      SmartDashboard.putNumber("UpperElbowSetpoint", upperElbowMotorSetpoint);
      SmartDashboard.putNumber("UpperElbowSpeed", upperRotateSpeed);

      SmartDashboard.putNumber("LowerElbowSetpoint", lowerElbowMotorSetpoint);
      SmartDashboard.putNumber("LowerElbowSpeed", lowerRotateSpeed);
    }

  }
}
