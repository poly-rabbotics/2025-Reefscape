// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.Angle;

/*
 * Manages the robot's pigeon.
 */
public class Pigeon {
    private static final int PIGEON_CAN_ID = 20;
    private static final Pigeon2 pigeon = new Pigeon2(PIGEON_CAN_ID);

    /**
     * Sets "relative forward" to the current position. This will make getting
     * the relative rotation always return a rotation relative to the rotation
     * the robot is in at the point this method is called.
     */
    public static void setFeildZero() {
        pigeon.setYaw(0.0);
    }

    public static void setFeildOrientation(Angle value) {
        pigeon.setYaw(value.degrees());
    }

    /**
     * Gets Yaw.
     */
    public static Angle getYaw() {
        return new Angle().setDegrees(pigeon.getYaw().getValue().magnitude() % 360.0);
    }

    /**
     * Gets pitch.
     */
    public static Angle getPitch() {
        return new Angle().setDegrees(pigeon.getPitch().getValue().magnitude());
    }

    /**
     * Gets roll.
     */
    public static Angle getRoll() {
        return new Angle().setDegrees(pigeon.getRoll().getValue().magnitude());
    }

    /**
     * Gets the X-axis acceleration as a coefficiant to the acceleration of 
     * gravity.
     */
    public static double getAccelerationX() {
        return pigeon.getAccelerationX().getValueAsDouble();
    }
    
    /**
     * Gets the Y-axis acceleration as a coefficiant to the acceleration of 
     * gravity.
     */
    public static double getAccelerationY() {
        return pigeon.getAccelerationY().getValueAsDouble();
    }
    
    /**
     * Gets the Z-axis acceleration as a coefficiant to the acceleration of 
     * gravity.
     */
    public static double getAccelerationZ() {
        return pigeon.getAccelerationZ().getValueAsDouble();
    }

    public static void recordState() {
        Logger.recordOutput("Pigeon Yaw (Radians)", getYaw().radians());
        Logger.recordOutput("Pigeon Yaw (Degrees)", getYaw().degrees());
        Logger.recordOutput("Pigeon Roll (Radians)", getRoll().radians());
        Logger.recordOutput("Pigeon Roll (Degrees)", getRoll().degrees());
        Logger.recordOutput("Pigeon Pitch (Radians)", getPitch().radians());
        Logger.recordOutput("Pigeon Pitch (Degrees)", getPitch().degrees());

        Logger.recordOutput("Pigeon Acceleration X (g)", getAccelerationX());
        Logger.recordOutput("Pigeon Acceleration Y (g)", getAccelerationY());
        Logger.recordOutput("Pigeon Acceleration Z (g)", getAccelerationZ());
    }
}
