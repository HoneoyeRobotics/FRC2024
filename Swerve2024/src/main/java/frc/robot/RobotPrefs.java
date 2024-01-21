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

    public static double getUpperElbowP() {
        if (!Preferences.containsKey("UpperElbowP")) {
            Preferences.setDouble("UpperElbowP", 0.045);
        }
        return Preferences.getDouble("UpperElbowP", 0.045);
    }

    public static double getUpperElbowI() {
        if (!Preferences.containsKey("UpperElbowI")) {
            Preferences.setDouble("UpperElbowI", 0.02);
        }
        return Preferences.getDouble("UpperElbowI", 0.02);
    }

    public static double getUpperElbowD() {
        if (!Preferences.containsKey("UpperElbowD")) {
            Preferences.setDouble("UpperElbowD", 0.0);
        }
        return Preferences.getDouble("UpperElbowD", 0.0);
    }

    public static double getUpperElbowTolerance() {
        if (!Preferences.containsKey("UpperElbowTolerance")) {
            Preferences.setDouble("UpperElbowTolerance", 0.75);
        }
        return Preferences.getDouble("UpperElbowTolerance", 0.75);
    }

    public static double getUpperElbowMovement() {
        if (!Preferences.containsKey("UpperElbowMovement")) {
            Preferences.setDouble("UpperElbowMovement", 1);
        }
        return Preferences.getDouble("UpperElbowMovement", 1);
    }

    public static double getLowerElbowP() {
        if (!Preferences.containsKey("LowerElbowP")) {
            Preferences.setDouble("LowerElbowP", 0.045);
        }
        return Preferences.getDouble("LowerElbowP", 0.045);
    }

    public static double getLowerElbowI() {
        if (!Preferences.containsKey("LowerElbowI")) {
            Preferences.setDouble("LowerElbowI", 0.02);
        }
        return Preferences.getDouble("LowerElbowI", 0.02);
    }

    public static double getLowerElbowD() {
        if (!Preferences.containsKey("LowerElbowD")) {
            Preferences.setDouble("LowerElbowD", 0.0);
        }
        return Preferences.getDouble("LowerElbowD", 0.0);
    }

     public static double getLowerElbowTolerance() {
        if (!Preferences.containsKey("LowerElbowTolerance")) {
            Preferences.setDouble("LowerElbowTolerance", 0.75);
        }
        return Preferences.getDouble("LowerElbowTolerance", 0.75);
    }

    
    public static double getLowerElbowMovement() {
        if (!Preferences.containsKey("LowerElbowMovement")) {
            Preferences.setDouble("LowerElbowMovement", 1);
        }
        return Preferences.getDouble("LowerElbowMovement", 1);
    }


}