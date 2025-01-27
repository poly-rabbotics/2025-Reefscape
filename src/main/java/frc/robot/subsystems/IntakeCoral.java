package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeCoral {

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private double speed;

    public IntakeCoral(int leftMotorID, int rightMotorID) {
        super();
        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

        speed = 0;
    }

    public void run() {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void set(double speed) {
        this.speed = speed;
    }
}
