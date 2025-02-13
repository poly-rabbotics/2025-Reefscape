package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class CoralIntake {

    private static SparkMax rightMotor = new SparkMax(7, MotorType.kBrushless);
    private static SparkMax leftMotor = new SparkMax(11, MotorType.kBrushless);

    private double speed;

    public CoralIntake(int rightMotorID, int leftMotorID) {
        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);
        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        leftMotorConfig.follow(rightMotor, true);
        
        leftMotor.configure(
            leftMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        rightMotor.configure(
            rightMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public void set(double speed) {
        this.speed = speed;
        rightMotor.set(this.speed);
    }

    public double get() {
        return speed;
    }
}

