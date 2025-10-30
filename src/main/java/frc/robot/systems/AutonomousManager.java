package frc.robot.systems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
/*import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;*/

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.DrivePath;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.subsystems.Coral.CoralIntake.IntakeState;
import frc.robot.systems.Coral.ScoringPoint;

public class AutonomousManager {
    private static final int AUTO_SWITCH_COUNT = 4;
    private static AutonomousProcedure[] procedures = new AutonomousProcedure[1 << AUTO_SWITCH_COUNT];
    static Joystick controlPanel = new Joystick(3); 


    static {
        buildProcedures();
    }

    public static void reset() {
        buildProcedures();
    }

    /**
     * Gets an auto procedure by its index.
     */
    public static Optional<AutonomousProcedure> getAutoProcedure(int mode) {
        if (mode < 0 || mode > procedures.length) {
            return Optional.empty();
        }

        return Optional.of(procedures[mode]);
    }

    /**
     * Gets an auto mode by a set of binary switches, switches should be given
     * in order of least significant bit first.
     */
    public static Optional<AutonomousProcedure> getProcedureFromSwitches(boolean... switches) {
        return getAutoProcedure(getSwitchSelection());
    }

    public static int getSwitchSelection() {
        int mode = 0;

        if(controlPanel.getRawButton(1))
            mode+=1;
        if(controlPanel.getRawButton(2))
            mode+=2;
        if(controlPanel.getRawButton(3))
            mode +=4;
        if(controlPanel.getRawButton(4))
            mode +=8;

        return mode;
    }

    public static void print()
    {
        SmartDashboard.putNumber("Auto Selection:", getSwitchSelection());
    }

    private static Function<StepStatus, StepStatus> makeStep(Runnable fn) {
        return (prevState) -> {
            fn.run();
            
            if (prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    private static Function<StepStatus, StepStatus> makeTimedStep(double time) {
        Timer stepTimer = new Timer();

        return (prevState) -> {
            if (!stepTimer.isRunning()) {
                stepTimer.start();
            }

            if (stepTimer.get() >= time && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    private static Function<StepStatus, StepStatus> makeTimedStep(double time, Runnable fn) {
        Timer stepTimer = new Timer();

        return (prevState) -> {
            if (!stepTimer.isRunning()) {
                stepTimer.start();
            }

            if (stepTimer.get() >= time && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            fn.run();

            return StepStatus.Running;
        };
    }

    private static Function<StepStatus, StepStatus> makeTimedStep(double time, Function<StepStatus, StepStatus> fn) {
        Timer stepTimer = new Timer();

        return (prevState) -> {
            if (!stepTimer.isRunning()) {
                stepTimer.start();
            }

            if (stepTimer.get() >= time && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return fn.apply(prevState);
        };
    }

    /**
     * Makes a step for running the drive.
     */
    private static Function<StepStatus, StepStatus> makeDriveStep(Pose2d... poses) {
        ArrayList<Pose2d> listPoses = new ArrayList<Pose2d>();

        for (Pose2d pose : poses) {
            listPoses.add(pose);
        }
        
        DrivePath path = new DrivePath(listPoses, AutonomousManager::withinPositionTolerance);
        return makeDriveStep(path);
    }

    private static Function<StepStatus, StepStatus> makeDriveStep(List<Pose2d> poses) {
        ArrayList<Pose2d> listPoses = new ArrayList<Pose2d>(poses);

        for (Pose2d pose : poses) {
            listPoses.add(pose);
        }
        
        DrivePath path = new DrivePath(listPoses, AutonomousManager::withinPositionTolerance);
        return makeDriveStep(path);
    }

    /**
     * Makes a step for running the drive.
     */
    private static Function<StepStatus, StepStatus> makeDriveStep(DrivePath path) {
        return (prevState) -> {
            DriverStation.getAlliance().ifPresent((a) -> path.convertToAlliance(a));
            SwerveDrive.setPath(path);
            SwerveDrive.run();

            if (path.isCompleted() && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    /**
     * Returns true if the drive is within tolerance to its desired position. Intended for use with
     * a swerve mode which takes over driving.
     */
    public static boolean withinPositionTolerance(Pose2d setPathPosition) {
        final double TOLERANCE_X = 0.05;
        final double TOLERANCE_Y = 0.05;
        final double TOLERANCE_THETA = 0.1;

        // makes sure angle are within [0, tau)
        double measuredAngle =(PoseEstimator.estimate2d().getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;
        double setAngle = (setPathPosition.getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;

        return Math.abs(PoseEstimator.estimate2d().getX() - setPathPosition.getX()) < TOLERANCE_X
            && Math.abs(PoseEstimator.estimate2d().getY() - setPathPosition.getY()) < TOLERANCE_Y
            && Math.abs(measuredAngle - setAngle) < TOLERANCE_THETA;
    }

    public static Function<StepStatus, StepStatus> makeCoralStep(ScoringPoint pose) {
        return (prevState) -> {
            Coral.setScoringPoint(pose);

            if (Coral.atScoringPoint() && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    private static Function<StepStatus, StepStatus> scoreCoral() {
        final double outtakeTime = 1.5;
        Timer stepTimer = new Timer();

        return (prevState) -> {
            if (!stepTimer.isRunning()) {
                stepTimer.start();
            }

            if (stepTimer.get() >= outtakeTime && prevState == StepStatus.Done) {
                Coral.setIntakeState(IntakeState.Idle);
                return StepStatus.Done;
            }

            if (Coral.getIntakeState() != IntakeState.Outtaking) {
                Coral.setIntakeState(IntakeState.Outtaking);
            }

            return StepStatus.Running;
        };
    }

    private static Function<StepStatus, StepStatus> intakeCoral() {
        return (prevState) -> {
            switch (Coral.getIntakeState()) {
                case Intaking:
                    return StepStatus.Running;

                case Storing:
                    if (prevState == StepStatus.Done) {
                        return StepStatus.Done;
                    }

                    return StepStatus.Waiting;

                default:
                    Coral.setIntakeState(IntakeState.Intaking);
                    return StepStatus.Running;
            }
        };
    }

    private static void buildProcedures() {
        try {
           // PathPlannerPath path = PathPlannerPath.fromPathFile("New New Path.path");

            //path.getPathPoses();
        
        // Case 0: Moves out towards reef one meter then stalls.
        procedures[0] = new AutonomousProcedure()
            // Drive out straight 1m, then stall
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(6.576, 3.989, new Rotation2d(Math.PI)))));
            .andThen(makeTimedStep(3));
            
        }
        catch(Exception e){};

        //procedures[1] = new AutonomousProcedure()
        //    .andThen(makeStep(() -> Pose.pose = "L3"))

        //    .andThen(makeTimedStep(3.0))

            // Move to L2 point
        //    .andThen(makeCoralStep(ScoringPoint.ScoringL2))

            // Back left reef, center/right side (driver POV)
        //    .andThen(makeTimedStep(1.0, makeDriveStep(new Pose2d(5.4, 6.0, new Rotation2d(-2.1)))))
        //    .andThen(makeTimedStep(2.8, makeDriveStep(new Pose2d(5.188, 5.010, new Rotation2d(-2.1)))))

            // Score coral
        //    .andThen(scoreCoral())

            // Move out of the way.
        //    .andThen(makeDriveStep(new Pose2d(5.31, 7.0, new Rotation2d(-2.1))));
            

        // Start right side score L2 driver side reef
        // procedures[2] = new AutonomousProcedure()

            // Set preload position
        //    .andThen(makeStep(() -> Coral.setCoralPosition(2.6)))

            // Move to L2
        //    .andThen(makeCoralStep(ScoringPoint.ScoringL2))

            // Drive to driver side reef
        //    .andThen(makeTimedStep(2.0, makeDriveStep(new Pose2d(2.5957, 2.4092, new Rotation2d()))))
        //    .andThen(makeTimedStep(2.0, makeDriveStep(new Pose2d(3.2207, 3.8563, new Rotation2d()))))

            // Score on L2
        //    .andThen(scoreCoral())

            // Drive back and stow
        //    .andThen(makeTimedStep(2.0, makeDriveStep(new Pose2d(1.8364, 3.8931, new Rotation2d()))))
        //    .andThen(makeCoralStep(ScoringPoint.Stowed))

            // Set up for intake in Tele
        //    .andThen(makeDriveStep(new Pose2d(1.8595, 0.7987, new Rotation2d(-2.2377))))
            ;

        // Start center, get leave points
        //procedures[3] = new AutonomousProcedure()
        //  .andThen(makeTimedStep(3.0))

        //    .andThen(makeTimedStep(2.0, makeDriveStep(new Pose2d(6.4540, 3.8510, new Rotation2d(Math.PI)))))

            // .andThen(makeCoralStep(ScoringPoint.ScoringL3))

            // .andThen(makeTimedStep(2.0, makeDriveStep(new Pose2d(5.8252, 3.8510, new Rotation2d(Math.PI)))))

            // .andThen(scoreCoral())
            
            // .andThen(makeTimedStep(2.0, makeDriveStep(new Pose2d(6.4540, 3.8510, new Rotation2d(Math.PI)))))

            // .andThen(makeCoralStep(ScoringPoint.Stowed))
            ;

        // Start left most edge, angled 135 degrees from forward clockwise.
        // Pushes friend backward,
        // Scores l2
        //procedures[9] = new AutonomousProcedure()
            // Push our friend a bit
        //    .andThen(
        //        makeTimedStep(2.0,makeDriveStep(new Pose2d(8.0, 6.67, new Rotation2d(-135.0 * Math.PI / 180.0)))
                

        //    .andThen(makeStep(() -> Coral.setCoralPosition(2.6)))

            // Move to L2 point
        //    .andThen(makeCoralStep(ScoringPoint.ScoringL2))

            // Back left reef, center/right side (driver POV)
        //     .andThen(makeTimedStep(1.0, makeDriveStep(new Pose2d(5.4, 6.0, new Rotation2d(-2.1)))))
        //    .andThen(makeTimedStep(2.8, makeDriveStep(new Pose2d(5.188, 5.010, new Rotation2d(-2.1)))))

            // Score coral
        //    .andThen(scoreCoral())

            // Move out of the way.
        //    .andThen(makeDriveStep(new Pose2d(5.31, 7.0, new Rotation2d(-2.1))))
            ;
        
        // CASE 5: 2 Coral Auto that starts from the center, and scores all coral on the right side in relation to the Driver POV
        procedures[1] = new AutonomousProcedure()
            // Drive to back center right reef closest to start point, then moves in and scores on center right reef (from Driver POV)
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(5.392, 2.143, new Rotation2d(2.0944)))))
            .andThen(makeStep(() -> Pose.pose = "L4"))
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(5.080, 2.660, new Rotation2d(2.0944)))))
            .andThen(scoreCoral())
        
            // Moves back out and switches into Field Nav Position
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.422, 1.363, new Rotation2d(-2.181)))))
            .andThen(makeStep(() -> Pose.pose = "FldNav"))
            
            // Drives to right side human player station (Driver POV)
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(1.355, 1.061, new Rotation2d(-2.181)))))
            .andThen(intakeCoral())

            // Scores coral again at center back right reef (Driver POV)
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(5.626, 1.802, new Rotation2d(2.0944)))))
            .andThen(makeStep(() -> Pose.pose = "L4"))
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(5.392, 1.093, new Rotation2d(2.0944)))))
            .andThen(scoreCoral())

            // Drives out and then switches into Field Navigation Position
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(5.626, 1.957, new Rotation2d(Math.PI)))))
            .andThen(makeStep(() -> Pose.pose = "FldNav"));

        // Case 2: Starts right edge (Driver POV), scores both coral at the center right edge of the reef, and then goes into Field Navigation Position
        procedures[2] = new AutonomousProcedure()
            // Scores initially center right reef, then scores at right human player station (from Driver POV)
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.428, 1.945 , new Rotation2d(1.0472)))))
            .andThen(makeStep(() -> Pose.pose = "L4"))
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.620, 2.880, new Rotation2d(1.0472)))))
            .andThen(scoreCoral())
            
            // Drives away from the reef, switches into Field Nav
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.428, 1.945 , new Rotation2d(-2.181)))))
            .andThen(makeStep(() -> Pose.pose = "FldNav"))

            // Drives to right side human player station and then intakes one coral
            .andThen(makeTimedStep(1,makeDriveStep(new Pose2d(1.211, 1.154, new Rotation2d(-2.181)))))
            .andThen(intakeCoral())

            // Drives to center right side reef and scores coral again, then shifts away from the reef and returns to field Nav Position
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.428, 1.945, new Rotation2d(1.0472)))))
            .andThen(makeStep(() -> Pose.pose = "L4"))
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.956,2.700, new Rotation2d(1.0472)))))
            .andThen(scoreCoral())

            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.956, 1.945, new Rotation2d(Math.PI)))))
            .andThen(makeStep(() -> Pose.pose = "FldNav"));
        // Case 1: Starts left edge (Driver POV), scores both coral on front left center reef and then moves to Field Nav on the left side of the reef.
        procedures[3] = new AutonomousProcedure()
            // Scores coral on the center left edge (from driver POV)
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.908, 5.973, new Rotation2d(-1.0472)))))
            .andThen(makeStep(() -> Pose.pose = "L4"))
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.908, 5.362, new Rotation2d(-1.0472)))))
            .andThen(scoreCoral())


            // Drives to the left side human player station (driver POV)
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.908, 5.973, new Rotation2d(2.23402)))))
            .andThen(makeStep(() -> Pose.pose = "FldNav"))
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(1.283, 6.956, new Rotation2d(2.23402)))))
            .andThen(intakeCoral())

            // Intakes one coral then scores on center left edge reef (from driver POV)
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.908, 5.973, new Rotation2d(-1.0472)))))
            .andThen(makeStep(() -> Pose.pose = "L4"))
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(3.632, 5.362, new Rotation2d(-1.0472)))))
            .andThen(scoreCoral())

            // Moves away from the reef and into Field Navigation Position
            .andThen(makeTimedStep(1, makeDriveStep(new Pose2d(4.771, 7.256, new Rotation2d(Math.PI) ))))
            .andThen(makeStep(() -> Pose.pose = "L4" ));

        // Starts center position, drives out to reef and scores on center back reef
        procedures [4] = new AutonomousProcedure()
            // Switches into L3 Position and then moves forwards and scores
            .andThen(makeStep(() -> Pose.pose = "L3"))
            .andThen(makeTimedStep(3.0))
            .andThen(makeTimedStep(2.0, makeDriveStep(new Pose2d(5.910,3.815, new Rotation2d(Math.PI)))))
            .andThen(scoreCoral())

            // Waits for a second then switches into Field Nav Position
            .andThen(makeTimedStep(1.0))
            .andThen(makeTimedStep(2.0, makeDriveStep(new Pose2d(5.910, 3.024, new Rotation2d(Math.PI)))))
            .andThen(makeStep(() -> Pose.pose = "FldNav"));














            
            

           
    
        // Starts center then scores coral far reef right side (driver POV) L3, intakes and moves
        // counter clockwise to score as many L3 as possible. Blue Side.
        // procedures[2] = new AutonomousProcedure()
        //     .andThen(makeStep(() -> Coral.setCoralPosition(4.8)))

        //     // Move to right side of far reef (driver POV)
        //     .andThen(makeDriveStep(new Pose2d(5.55, 4.35, new Rotation2d(Math.PI))))
            
        //     // Score coral L4
        //     .andThen(makeCoralStep(ScoringPoint.ScoringL4))
        //     .andThen(scoreCoral())

        //     .andThen(makeDriveStep(new Pose2d(5.75, 4.05, new Rotation2d(Math.PI))))

        //     .andThen(makeDriveStep(new Pose2d(5.55, 4.05, new Rotation2d(Math.PI))))

        //     .andThen(makeCoralStep(ScoringPoint.Stowed))

        //     .andThen(makeDriveStep(new Pose2d(5.75, 4.55, new Rotation2d(Math.PI))))

        //     .andThen(makeCoralStep(ScoringPoint.DealgaeficateL2))
        //     .andThen(makeStep(() -> Coral.setDealgaeficatorSpeed(-1.0)))

        //     .andThen(makeDriveStep(new Pose2d(5.55, 4.55, new Rotation2d(Math.PI))))
        //     // .andThen(makeDriveStep(new Pose2d(5.75, 4.55, new Rotation2d(Math.PI))))
        //     ;

            // // Move to avoid reef and go to the left far intake station (driver POV), then intake a
            // // coral
            // .andThen(makeDriveStep(
            //     new Pose2d(5.75, 5.0, new Rotation2d(Math.PI)),
            //     new Pose2d(1.647, 7.47, new Rotation2d(2.2))
            // ))
            // .andThen(intakeCoral())

            // // Move around reef then move to left side of far reef (driver POV)
            // .andThen(makeDriveStep(
            //     new Pose2d(5.75, 5.0, new Rotation2d(Math.PI)),
            //     new Pose2d(5.75, 4.18, new Rotation2d(Math.PI))
            // ))

            // // Score coral L4
            // .andThen(makeCoralStep(ScoringPoint.ScoringL3))
            // .andThen(scoreCoral())
            
            // // Move to avoid reef and go to the left far intake station (driver POV), then intake a
            // // coral
            // .andThen(makeDriveStep(
            //     new Pose2d(5.75, 5.0, new Rotation2d(Math.PI)),
            //     new Pose2d(1.647, 7.47, new Rotation2d(2.2))
            // ))
            // .andThen(intakeCoral())

            // // Move to back left reef right/center side
            // .andThen(makeDriveStep(new Pose2d(5.26, 5.04, new Rotation2d(-2.1))))

            // // Score coral L3
            // .andThen(makeCoralStep(ScoringPoint.ScoringL4))
            // .andThen(scoreCoral())
            
            // // Move to avoid reef and go to the left far intake station (driver POV), then intake a
            // // coral
            // .andThen(makeDriveStep(
            //     new Pose2d(1.647, 7.47, new Rotation2d(2.2))
            // ))
            // .andThen(intakeCoral())

            // // Move to back left reef left/away from center side
            // .andThen(makeDriveStep(new Pose2d(4.97, 5.22, new Rotation2d(-2.1))))

            // // Score coral L4
            // .andThen(makeCoralStep(ScoringPoint.ScoringL4))
            // .andThen(scoreCoral())
            // ;
    }
}
