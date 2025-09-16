package frc.robot.subsystems.Algae;

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
import frc.robot.systems.Algae;

public class GroundIntake {
    Joystick buttonPanel;
    //change these values
    private static final double MOTOR_GEAR_RATIO = 1.0 / 15.0;
    private static final double SPROCKET_RATIO = 15.0 / 18.0;
    private static final double WHEEL_CIRCUMFRANCE = 2.25 * Math.PI; 
    private static final double TRANSFER_DISTANCE = 500.0;
   
    private static final double INTAKE_SPEED = -1.0;
    private static final double OUTTAKE_SPEED = 1.0;

    private static final int GROUND_INTAKE_BUTTON = 1;
    private static final int GROUND_OUTAKE_BUTTON = 1;
    private static final int GROUND_TRANSFER_BUTTON = 1;
    //change pid values
    private static final double PID_P = 0.035;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    
    RelativeEncoder encoderLeft, encoderRight;

    private SparkMax leftMotor;
    private SparkMax rightMotor;


    private IntakeState currentState = IntakeState.Idle;
    private PIDController controller = new PIDController(PID_P, PID_I, PID_D);

    
    public enum IntakeState {
       Intaking, Outtaking, Transferring, Idle
    }

    public GroundIntake(
        int leftMotorID,
        int rightMotorID
    ) {
        buttonPanel = new Joystick(3);
        leftMotor = new SparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorID, MotorType.kBrushless);

        encoderLeft = leftMotor.getEncoder();
        encoderRight = rightMotor.getEncoder();
        encoderLeft.setPosition(0);
        encoderRight.setPosition(0);

        leftMotor.configure(
            new SparkMaxConfig()
                .apply(new EncoderConfig()
                    .positionConversionFactor(MOTOR_GEAR_RATIO * SPROCKET_RATIO * WHEEL_CIRCUMFRANCE)
                    .velocityConversionFactor(MOTOR_GEAR_RATIO * SPROCKET_RATIO * WHEEL_CIRCUMFRANCE)
    
                )
                .inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        rightMotor.configure(
            new SparkMaxConfig()
                .follow(rightMotor, true)
                .apply(new EncoderConfig()
                    .positionConversionFactor(MOTOR_GEAR_RATIO * SPROCKET_RATIO * WHEEL_CIRCUMFRANCE)
                    .velocityConversionFactor(MOTOR_GEAR_RATIO * SPROCKET_RATIO * WHEEL_CIRCUMFRANCE)
                ),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

    }

    
public void run() {
    
if(buttonPanel.getRawButton(GROUND_INTAKE_BUTTON)){
leftMotor.set(INTAKE_SPEED);
rightMotor.set(-INTAKE_SPEED);
} else if(buttonPanel.getRawButton(GROUND_OUTAKE_BUTTON)){
    leftMotor.set(OUTTAKE_SPEED);
    rightMotor.set(-OUTTAKE_SPEED);
}
if((buttonPanel.getRawButtonReleased(GROUND_INTAKE_BUTTON))
||(buttonPanel.getRawButtonReleased(GROUND_OUTAKE_BUTTON))){
    leftMotor.set(0);
    rightMotor.set(0);
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
    leftMotor.getClosedLoopController().setReference(0, ControlType.kPosition);
    rightMotor.getClosedLoopController().setReference(0, ControlType.kPosition);
if(buttonPanel.getRawButtonPressed(GROUND_TRANSFER_BUTTON)){
    leftMotor.getClosedLoopController().setReference(TRANSFER_DISTANCE, ControlType.kPosition);
    rightMotor.getClosedLoopController().setReference(TRANSFER_DISTANCE, ControlType.kPosition);
}
}

}
}


    