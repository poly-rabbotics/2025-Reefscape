package frc.robot.systems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class CoralIntake {
    private static final SparkMax rightMotor = new SparkMax( 13 , MotorType.kBrushless);
    private static final SparkMax leftMotor = new SparkMax( 14, MotorType.kBrushless);
    public enum IntakeState {
        INTAKING, OUTTAKING, IDLE
    }

    public static void setIntakeState(IntakeState state) {
        if (state == IntakeState.INTAKING) {
            leftMotor.set(.25);
            rightMotor.set(-.25);
        } else if (state == IntakeState.OUTTAKING) {
            leftMotor.set(-.25);
            rightMotor.set(.25);
        } else if(state == IntakeState.IDLE){
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }
}



