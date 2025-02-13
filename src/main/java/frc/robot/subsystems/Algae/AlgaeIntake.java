package frc.robot.subsystems.Algae;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class AlgaeIntake {

    private final SparkMax intakeMotor;

    private double speed;

    public AlgaeIntake(int intakeMotorID) {
        
        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
    }

    public void run() {
        intakeMotor.set(speed);
    }

    public void set(double speed) {
        this.speed = speed;
    }
}

