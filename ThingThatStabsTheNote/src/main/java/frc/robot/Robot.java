// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>
 * Joystick analog values range from -1 to 1 and motor controller inputs also
 * range from -1 to 1
 * making it easy to work together.
 *
 * <p>
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 13;
  private static final int kJoystickPort = 0;
  private static final int kEncoderPortA = 0;
  private static final int kEncoderPortB = 1;

  private VictorSPX m_motor;
  private CommandXboxController m_joystick;
  private Encoder m_encoder;
  private Relay relay = new Relay(0);

  @Override
  public void robotInit() {
  m_motor = new VictorSPX(kMotorPort);
    m_joystick = new CommandXboxController(kJoystickPort);
 

    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.

    //m_joystick.y().onTrue(relay.set(Value.kOn));

  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
    if (m_joystick.button(1).getAsBoolean())
     m_motor.set(VictorSPXControlMode.PercentOutput, 1);

     else if (m_joystick.button(2).getAsBoolean())
     m_motor.set(VictorSPXControlMode.PercentOutput, -1);

    else
      m_motor.set(VictorSPXControlMode.PercentOutput, 0);


    if(m_joystick.y().getAsBoolean()){
      relay.set(Value.kOn);
     }
     else{
      relay.set(Value.kOff);
    }


    
  }
}
