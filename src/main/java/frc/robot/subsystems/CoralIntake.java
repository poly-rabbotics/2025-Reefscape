package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralIntake {

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    public CoralIntake(int leftMotorID, int rightMotorID) {
        super();

        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);
    }

    public void run(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}
