package frc.robot.systems;

public class Pose {
    public static String pose = "";
public static double elevatorPos = 0.0;
public static double pivotPos = 0.0;
public static double climberPos = 0.0;

public Pose(){

    }

public static void run(){
switch (pose){
    case "FldNav":
        elevatorPos = 0.0;
            pivotPos = 25.0;
                climberPos = 0.0;
                break;
    case "StnInt":
        elevatorPos = 0.0;
            pivotPos = 5.0;
                climberPos = 0.0;
                break;
    case "L1":
        elevatorPos = 0.0;
            pivotPos = 0.0;
                climberPos = 0.0;
                break;
    case "L2":
        elevatorPos = 6.0;
            pivotPos = -20.0;
                climberPos = 0.0;
                break;
    case "L3":
        elevatorPos = 26.5;
            pivotPos = -30.0;
                climberPos = 0.0;
                break;
    case "L4":
        elevatorPos = 56.0;
            pivotPos = -50.0;
                climberPos = 0.0;
                break;
    case "AL2":
        elevatorPos = 6.0;
            pivotPos = -90.0;
                climberPos = 0.0;
                break;
    case "AL3":
        elevatorPos = 26.5;
            pivotPos = -90.0;
                climberPos = 0.0;
                break;
    case "Net":
        elevatorPos = 58.0;
            pivotPos = -40.0;
                climberPos = 0.0;
                break;
    case "GrndAl": // All Positions TBD
        elevatorPos = 0.0;
            pivotPos = 0.0;
                climberPos = 0.0;
                break;
    case "Processor": // All Positions TBD
        elevatorPos = 0.0;
            pivotPos = 0.0;
                climberPos = 0.0;
                break;
    case "AlXFR": // All Positions TBD
        elevatorPos = 0.0;
            pivotPos = 0.0;
                climberPos = 0.0;
                break;
    case "Climb": // All Positions TBD ( except climberPos )
        elevatorPos = 0.0;
            pivotPos = 0.0;
                climberPos = 140.0;
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
    }
