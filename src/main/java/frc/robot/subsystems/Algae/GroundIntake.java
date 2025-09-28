package frc.robot.subsystems.Algae;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class GroundIntake {
    Joystick buttonPanelRight;
    Joystick buttonPanelLeft;
    //change these values

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

    
    public enum IntakeState {
       Intaking, Outtaking, Transferring, Idle
    }

    public GroundIntake(int leftMotorID,int rightMotorID) {
        buttonPanelLeft = new Joystick(2);
        buttonPanelRight = new Joystick(3);
        leftMotor = new TalonFX(leftMotorID);
        rightMotor = new TalonFX(rightMotorID);

        encoderLeft = leftMotor.getPosition().getValueAsDouble();
        encoderRight = rightMotor.getPosition().getValueAsDouble();

        leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfig.Slot0.kP = PID_P;
        leftConfig.Slot0.kI = PID_I;
        leftConfig.Slot0.kD = PID_D;
        leftMotor.getConfigurator().apply(leftConfig);                         


        rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfig.Slot0.kP = PID_P;
        rightConfig.Slot0.kI = PID_I;
        rightConfig.Slot0.kD = PID_D;
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
    leftMotor.setControl(new PositionDutyCycle(0));
    rightMotor.setControl(new PositionDutyCycle(0));
}
if(buttonPanelLeft.getRawButtonPressed(GROUND_TRANSFER_BUTTON)){
    leftMotor.setControl(new PositionDutyCycle(TRANSFER_DISTANCE));
    rightMotor.setControl(new PositionDutyCycle(TRANSFER_DISTANCE));
}

}

    public void print()
    {
        SmartDashboard.putNumber("Ground Intake left Motor Pos", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Ground Intake right Motor Pos", rightMotor.getPosition().getValueAsDouble());
    }

}
    