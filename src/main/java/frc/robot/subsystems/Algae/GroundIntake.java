package frc.robot.subsystems.Algae;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.fasterxml.jackson.databind.cfg.SerializerFactoryConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Algae;
import frc.robot.systems.Algae.ClimberPose;
import frc.robot.systems.Coral.ScoringPoint;

public class GroundIntake {
    Joystick buttonPanelRight;
    Joystick buttonPanelLeft;
    //change these values
    private static final double MOTOR_GEAR_RATIO = 1.0 / 15.0;
    private static final double SPROCKET_RATIO = 15.0 / 18.0;
    private static final double WHEEL_CIRCUMFRANCE = 2.25 * Math.PI; 
    private static final double TRANSFER_DISTANCE = 500.0;
   
    private static final double INTAKE_SPEED = -1.0;
    private static final double OUTTAKE_SPEED = 1.0;

    private static final int GROUND_INTAKE_BUTTON = 8;
    private static final int GROUND_OUTAKE_BUTTON = 9;
    private static final int GROUND_TRANSFER_BUTTON = 9;
    //change pid values
    private static final double PID_P = 0.035;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    
    double encoderLeft, encoderRight;

    private TalonFX leftMotor;
    private TalonFX rightMotor;
    TalonFXConfiguration leftConfig;
    TalonFXConfiguration rightConfig;

    private IntakeState currentState = IntakeState.Idle;
    private PIDController controller = new PIDController(PID_P, PID_I, PID_D);

    
    public enum IntakeState {
       Intaking, Outtaking, Transferring, Idle
    }

    public GroundIntake(
        int leftMotorID,
        int rightMotorID
    ) {
        buttonPanelLeft = new Joystick(2);
        buttonPanelRight = new Joystick(3);
        leftMotor = new TalonFX(leftMotorID);
        rightMotor = new TalonFX(rightMotorID);

        encoderLeft = leftMotor.getPosition().getValueAsDouble();
        encoderRight = rightMotor.getPosition().getValueAsDouble();
        leftMotor.setPosition(rightMotorID);
        rightMotor.setPosition(leftMotorID);

        leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfig.Slot0.kP = 0.0;
        leftConfig.Slot0.kI = 0.0;
        leftConfig.Slot0.kD = 0.0;
        leftMotor.getConfigurator().apply(leftConfig);
                          


        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfig.Slot0.kP = 0.0;
        rightConfig.Slot0.kI = 0.0;
        rightConfig.Slot0.kD = 0.0;
        rightMotor.getConfigurator().apply(rightConfig);
                          
   
    }

    
public void run() {
if(buttonPanelRight.getRawButton(GROUND_INTAKE_BUTTON)){
leftMotor.set(INTAKE_SPEED);
rightMotor.set(-INTAKE_SPEED);
} else if(buttonPanelRight.getRawButton(GROUND_OUTAKE_BUTTON)){
    leftMotor.set(OUTTAKE_SPEED);
    rightMotor.set(-OUTTAKE_SPEED);
}
if((buttonPanelRight.getRawButtonReleased(GROUND_INTAKE_BUTTON))
||(buttonPanelRight.getRawButtonReleased(GROUND_OUTAKE_BUTTON))){
    leftMotor.set(0);
    rightMotor.set(0);
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
if(buttonPanelLeft.getRawButtonPressed(GROUND_TRANSFER_BUTTON)){
    leftMotor.setPosition(TRANSFER_DISTANCE);
    rightMotor.setPosition(TRANSFER_DISTANCE);
}

}
SmartDashboard.putNumber("Ground Intake left Motor Pos", leftMotor.getPosition().getValueAsDouble());
SmartDashboard.putNumber("Ground Intake right Motor Pos", rightMotor.getPosition().getValueAsDouble());
}

}
    