// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Simple enum to store what type of game object we have */
public enum GameObject {
    CUBE,
    CONE;

    @Override
    public String toString() {
        if (this.equals(CONE)) {
            return "Cone";
        } else {
            return "Cube";
        }
    }
}
