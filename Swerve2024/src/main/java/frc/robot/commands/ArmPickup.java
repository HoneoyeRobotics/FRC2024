// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arms;

import frc.robot.commands.ArmMovement.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmPickup extends InstantCommand {
  /** Creates a new ArmHome. */
  private Arms arms;

  public ArmPickup(Arms arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.arms = arms;
  }

  @Override
  public void initialize() {
    switch (arms.getArmPosition()) {
      case Home:
      new HomeToPickUp(arms).schedule();
        break;
      case Speaker:
        new SpeakerToPickup(arms).schedule();
        break;
      case Feeder:
        new FeederToHome(arms).schedule();
        break;
      case Pickup:
        
        break;
      case Amp:
        new AmpToPickUp(arms).schedule();
        break;
    }
  }
}