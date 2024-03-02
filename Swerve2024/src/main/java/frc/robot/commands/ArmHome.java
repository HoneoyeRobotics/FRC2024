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
public class ArmHome extends InstantCommand {
  /** Creates a new ArmHome. */
  private Arms arms;

  public ArmHome(Arms arms) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.arms = arms;
  }

  @Override
  public void initialize() {
    switch (arms.getArmPosition()) {
      case Home:
        break;
      case Speaker:
        new SpeakerToHome(arms).schedule();;
        break;
      case Feeder:
        new FeederToHome(arms).schedule();
        break;
      case Pickup:
        new PickUpToHome(arms).schedule();
        break;
      case Amp:
        new AmpToHome(arms).schedule();
        break;
        case Climber:
          new ClimberToHome(arms).schedule();
          break;
    }
  }
}
