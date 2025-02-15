package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralIntake {
    
    private SparkMax rightMotor;
    private SparkMax leftMotor;

    private double currentSpeed = 0;

    public CoralIntake(int rightMotorID, int leftMotorID) {
        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);
        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);

        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

        rightMotorConfig.follow(leftMotor, true);

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
        currentSpeed = speed;
    }

    public void run() {
        leftMotor.set(currentSpeed);
    }

    public double get() {
        return leftMotor.get();
    }

    public double getCurrent() {
        return leftMotor.getOutputCurrent();
    }

    public double getMotorTemp() {
        return leftMotor.getMotorTemperature();
    }
}
