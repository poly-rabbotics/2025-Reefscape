package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Algae.*;

public class Algae {
    private static final int ALGAE_PIVOT_MOTOR_ID = 1;

    private static AlgaeIntake algaeIntake
        = new AlgaeIntake(ALGAE_PIVOT_MOTOR_ID);

    public enum AlgaePose {
        Handoff, Processor, Intake, Stow
    }

    public static void run() {
        algaeIntake.run();
    }

    public static void setPose(AlgaePose pose) {
        algaeIntake.setPose(pose);
    }

    public static void _setPivotSpeed(double speed) {
        algaeIntake._setPivotSpeed(speed);
    }
    
    public static void recordState() {
        Logger.recordOutput("Algae Intake Pivot Speed", algaeIntake.getPivotSpeed());
        Logger.recordOutput(
            "Algae Intake Pivot Velocity (Degrees per Second)",
            algaeIntake.getPivotVelocity()
        );
        Logger.recordOutput("Algae Intake Set Pose", algaeIntake.getSetPose());
        Logger.recordOutput("Algae Intake Angle (Degrees)", algaeIntake.getAngle().degrees());
        Logger.recordOutput("Algae Intake Set Angle (Degrees)", algaeIntake.getSetAngle().degrees());
    }
}
