// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

/** Add your docs here. */
public final class RobotPrefs {

    public static boolean getDebugMode() {
        if (!Preferences.containsKey("DebugMode")) {
            Preferences.setBoolean("DebugMode", false);
        }
        return Preferences.getBoolean("DebugMode", false);
    }

    public static double getElbowP() {
        if (!Preferences.containsKey("ElbowP")) {
            Preferences.setDouble("ElbowP", 0.045);
        }
        return Preferences.getDouble("ElbowP", 0.045);
    }

    public static double getElbowI() {
        if (!Preferences.containsKey("ElbowI")) {
            Preferences.setDouble("ElbowI", 0.02);
        }
        return Preferences.getDouble("ElbowI", 0.02);
    }

    public static double getElbowD() {
        if (!Preferences.containsKey("ElbowD")) {
            Preferences.setDouble("ElbowD", 0.0);
        }
        return Preferences.getDouble("ElbowD", 0.0);
    }

    public static double getElbowTolerance() {
        if (!Preferences.containsKey("ElbowTolerance")) {
            Preferences.setDouble("ElbowTolerance", 0.75);
        }
        return Preferences.getDouble("ElbowTolerance", 0.75);
    }

    public static double getElbowMovement() {
        if (!Preferences.containsKey("ElbowMovement")) {
            Preferences.setDouble("ElbowMovement", 1);
        }
        return Preferences.getDouble("ElbowMovement", 1);
    }

    public static double getShoulderP() {
        if (!Preferences.containsKey("ShoulderP")) {
            Preferences.setDouble("ShoulderP", 0.045);
        }
        return Preferences.getDouble("ShoulderP", 0.045);
    }

    public static double getShoulderI() {
        if (!Preferences.containsKey("ShoulderI")) {
            Preferences.setDouble("ShoulderI", 0.02);
        }
        return Preferences.getDouble("ShoulderI", 0.02);
    }

    public static double getShoulderD() {
        if (!Preferences.containsKey("ShoulderD")) {
            Preferences.setDouble("ShoulderD", 0.0);
        }
        return Preferences.getDouble("ShoulderD", 0.0);
    }

    public static double getShoulderTolerance() {
        if (!Preferences.containsKey("ShoulderTolerance")) {
            Preferences.setDouble("ShoulderTolerance", 0.75);
        }
        return Preferences.getDouble("ShoulderTolerance", 0.75);
    }

    public static double getShoulderMovement() {
        if (!Preferences.containsKey("ShoulderMovement")) {
            Preferences.setDouble("ShoulderMovement", 1);
        }
        return Preferences.getDouble("ShoulderMovement", 1);
    }

    public static double getShooterAmpSpeed() {
        if (!Preferences.containsKey("ShooterAmpSpeed")) {
            Preferences.setDouble("ShooterAmpSpeed", -0.25);
        }
        return Preferences.getDouble("ShooterAmpSpeed", 0.25);
    }

    public static double getPickupElbow() {
        if (!Preferences.containsKey("PickupElbow")) {
            Preferences.setDouble("PickupElbow", -4.6);
        }
        return Preferences.getDouble("PickupElbow", -4.6);
    }

    public static double getPickupShoulder() {
        if (!Preferences.containsKey("PickupShoulder")) {
            Preferences.setDouble("PickupShoulder", 20.7);
        }
        return Preferences.getDouble("PickupShoulder", 20.7);
    }




    public static double getAmpElbow() {
        if (!Preferences.containsKey("AmpElbow")) {
            Preferences.setDouble("AmpElbow", -18.97);
        }
        return Preferences.getDouble("AmpElbow", -18.97);
    }

    public static double getAmpShoulder() {
        if (!Preferences.containsKey("AmpShoulder")) {
            Preferences.setDouble("AmpShoulder", 13.9);
        }
        return Preferences.getDouble("AmpShoulder", 13.9);
    }


    public static double getAmpWait() {
        if (!Preferences.containsKey("AmpWait")) {
            Preferences.setDouble("AmpWait", 1);
        }
        return Preferences.getDouble("AmpWait", 1);
    }
    public static double getAmpTimeout() {
        if (!Preferences.containsKey("AmpTimeout")) {
            Preferences.setDouble("AmpTimeout", 3);
        }
        return Preferences.getDouble("AmpTimeout", 3);
    }



    public static double getSpeakerWait() {
        if (!Preferences.containsKey("SpeakerWait")) {
            Preferences.setDouble("SpeakerWait", 1);
        }
        return Preferences.getDouble("SpeakerWait", 1);
    }
    public static double getSpeakerTimeout() {
        if (!Preferences.containsKey("getSpeakerTimeout")) {
            Preferences.setDouble("getSpeakerTimeout", 3);
        }
        return Preferences.getDouble("getSpeakerTimeout", 3);
    }

      public static double getTheThingThatStabsTheNote() {
        if (!Preferences.containsKey("TheThingThatStabesTheNote")) {
            Preferences.setDouble("TheThingThatStabesTheNote", .25);
        }
        return Preferences.getDouble("TheThingThatStabesTheNote", .25);
    }

    public static int getTheThingThatStabsTheNoteRot() {
        if (!Preferences.containsKey("StabsNoteRot")) {
            Preferences.setInt("StabsNoteRot", 25);
        }
        return Preferences.getInt("StabsNoteRot", 25);
    }
 public static double getMiddleTolerance() {
        if (!Preferences.containsKey("MiddleTolerance")) {
            Preferences.setDouble("MiddleTolerance", 1);
        }
        return Preferences.getDouble("MiddleTolerance", 1);
    }

     public static double getEndTolerance() {
        // if (!Preferences.containsKey("EndTolerance")) {
        //     Preferences.setDouble("EndTolerance", 1);
        // }
        // return Preferences.getDouble("EndTolerance", 1);
        return 0.25;
    }

    
    public static double getStabP() {
        if (!Preferences.containsKey("StabP")) {
            Preferences.setDouble("StabP", 0.045);
        }
        return Preferences.getDouble("StabP", 0.045);
    }

    public static double getStabI() {
        if (!Preferences.containsKey("StabI")) {
            Preferences.setDouble("StabI", 0.02);
        }
        return Preferences.getDouble("StabI", 0.02);
    }

    public static double getStabD() {
        if (!Preferences.containsKey("StabD")) {
            Preferences.setDouble("StabD", 0.0);
        }
        return Preferences.getDouble("StabD", 0.0);
    }
    
}