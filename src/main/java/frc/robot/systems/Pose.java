package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pose {
    public static String pose = "";
public static double elevatorPos = 0.0;
public static double pivotPos = 0.0;
public static double climberPos = 0.0;

public static double defaultElevatorPose = 0.0;
public static double defaultPivotPose = 18.0;
public static double defaultClimberPose = 140.0;

public Pose(){
    pose = "FldNav";
    elevatorPos = defaultElevatorPose;
    pivotPos = defaultPivotPose;
    climberPos = defaultClimberPose;

    }

public static void run(){
switch (pose){
    case "FldNav":
        elevatorPos = defaultElevatorPose;
            pivotPos = defaultPivotPose;
                climberPos = defaultClimberPose;
                break;
    case "StnInt":
        elevatorPos = 0.0;
            pivotPos = 5.0;
                climberPos = defaultClimberPose;
                break;
    case "L1":
        elevatorPos = 0.0;
            pivotPos = 60;
                climberPos = defaultClimberPose;
                break;
    case "L2":
        elevatorPos = 6.0;
            pivotPos = -20.0;
                climberPos = defaultClimberPose;
                break;
    case "L3":
        elevatorPos = 26.5;
            pivotPos = -30.0;
                climberPos = defaultClimberPose;
                break;
    case "L4":
        elevatorPos = 55.0;
            pivotPos = -50.0;
                climberPos = defaultClimberPose;
                break;
    case "Al2":
        elevatorPos = 6.0;
            pivotPos = -90.0;
                climberPos = defaultClimberPose;
                break;
    case "Al3":
        elevatorPos = 26.5;
            pivotPos = -90.0;
                climberPos = defaultClimberPose;
                break;
    case "Net":
        elevatorPos = 58.0;
            pivotPos = -40.0;
                climberPos = defaultClimberPose;
                break;
    case "GrndAl": 
        elevatorPos = defaultElevatorPose;
            pivotPos = defaultPivotPose;
                climberPos = 30.0;
                break;
    case "Processor": 
        elevatorPos = defaultElevatorPose;
            pivotPos = defaultPivotPose;
                climberPos = 55.0;
                break;
    case "AlXFRPrep": // All Positions TBD
        elevatorPos = 20.0;
            pivotPos = defaultPivotPose;
                climberPos = 100.0;
                        break;
    case "AlXFR": // All Positions TBD
        elevatorPos = 20.0;
            pivotPos = 105;
                climberPos = 150.0;
                break;
    case "Climb": 
        elevatorPos = defaultElevatorPose;
            pivotPos = -90.0;
                climberPos = 150.0;
                break;
    case "ClimbApproach": 
                elevatorPos = defaultElevatorPose;
                    pivotPos = -90.0;
                        climberPos = 10.0;
                        break;
        default: 
        break;

        }
        
    }
    public double getClimberPos(){


            return climberPos;}

    public double getElvatorPos(){

        
            return elevatorPos;}
        
    public double getpivotPos(){

        
            return pivotPos;
        }

    public static void print(){
        SmartDashboard.putString("Pose", pose);
        SmartDashboard.putNumber("Climber Setpoint", climberPos);
        SmartDashboard.putNumber("Elevator Setpoint", elevatorPos);
        SmartDashboard.putNumber("Coral Pivot Setpoint", pivotPos);
    }
    }
