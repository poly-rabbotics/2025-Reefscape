package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import frc.robot.Robot;
import frc.robot.subsystems.Algae.*;

public class Algae {
    private static final int CLIMBER_PIVOT_MOTOR_ID = 1;
    private static final int CLIMBER_ACTUATOR_CHANNEL = 1;

    private static Climber climber = Robot.climber;

    public enum ClimberPose {
        ClimbHold, ClimbAproach, Stow
    }

    public static void run() {
        climber.run();
    }

    public static void setPose(ClimberPose pose) {
        climber.setPose(pose);
    }

    public static void _setPivotSpeed(double speed) {
        climber._setPivotSpeed(speed);
    }
    
    public static void recordState() {
        Logger.recordOutput("Climber Pivot Speed", climber.getPivotSpeed());
        //Logger.recordOutput("Climber Actuator Position", climber.getActuatorSet());
        Logger.recordOutput("Climber Pivot Velocity (Degrees per Second)", climber.getPivotVelocity());
        Logger.recordOutput("Climber Set Pose", climber.getSetPose());
        Logger.recordOutput("Climber Angle (Degrees)", climber.getAngle().degrees());
        Logger.recordOutput("Climber Set Angle (Degrees)", climber.getSetAngle().degrees());
    }
}
