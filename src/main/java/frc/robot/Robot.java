// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.*;
import frc.robot.systems.*;
import frc.robot.systems.Algae.AlgaePose;
import frc.robot.systems.Coral.IntakeState;
import frc.robot.systems.Coral.ScoringPoint;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private static final int CONTROLLER_PORT_DRIVER = 0;
    private static final int CONTROLLER_PORT_OPERATOR = 1;
    private static final int CONTROL_PANEL_PORT_CORAL = 2; // Elevator
    private static final int CONTROL_PANEL_PORT_ALGAE = 3; // Algae and switches

    private static Compressor comp = new Compressor(0, PneumaticsModuleType.CTREPCM);

    private static final XboxController controllerDriver
        = new XboxController(CONTROLLER_PORT_DRIVER);
    private static final XboxController controllerOperator
        = new XboxController(CONTROLLER_PORT_OPERATOR);
    private static final Joystick controlPanelCoral
        = new Joystick(CONTROL_PANEL_PORT_CORAL);
    private static final Joystick controlPanelAlgae
        = new Joystick(CONTROL_PANEL_PORT_ALGAE);

    @SuppressWarnings("unused")
    private static PowerDistribution powerDistributionHub
        = new PowerDistribution(1, ModuleType.kRev);

    private static Optional<AutonomousProcedure> autonomousProcedure;
    
    private static boolean invertedTurn = true;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", "7042 Theseus");

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            try {
                // Try to replay from a log file.
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                setUseTiming(false);

                System.out.println("Replaying from '" + logPath + "' in simulation ...");
            } catch (Exception e) {
                // Simulate if no log file exists.
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());

                System.out.println("Could not find log to replay from ...");
                System.out.println("Starting non-replay simulation ...");
            }
        }

        Logger.start();

        // Log driver station data
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        Pigeon.setYaw(new Angle().setDegrees(0.0));

        // Compressor stuffs
        comp.enableDigital();
    }

    @Override
    public void robotPeriodic() {
        PoseEstimator.update();

        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_DRIVER)) {
            if (controllerDriver.getBackButtonReleased()) {
                Pose3d pose = new Pose3d();

                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                    pose = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, Math.PI));
                }

                PoseEstimator.reset(pose);

                // Prevents fast rotation when setting heading.
                if (SwerveDrive.getMode() == SwerveMode.SetAngle) {
                    SwerveDrive.setMode(SwerveMode.Headless);
                }
            }

            if (controllerDriver.getStartButtonReleased()) {
                Angle yaw;

                switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
                    case Red:
                        yaw = new Angle().setDegrees(180.0);
                        break;
                    default:
                    case Blue:
                        yaw = new Angle().setDegrees(0.0);
                        break;
                }
                
                Pigeon.setYaw(yaw);

                // Prevents fast rotation when setting heading.
                if (SwerveDrive.getMode() == SwerveMode.SetAngle) {
                    SwerveDrive.setMode(SwerveMode.Headless);
                }
            }
        }

        recordLogFrame();

        Logger.recordOutput("Alliance", DriverStation.getAlliance().orElse(Alliance.Blue));
        Logger.recordOutput("Selected Auto Number", AutonomousManager.getSwitchSelection(
            controlPanelAlgae.getRawButton(1),
            controlPanelAlgae.getRawButton(2),
            controlPanelAlgae.getRawButton(3),
            controlPanelAlgae.getRawButton(4)
        ));
    }

    @Override
    public void autonomousInit() {
        AutonomousManager.reset();
        SwerveDrive.setMode(SwerveMode.FollowPath);

        autonomousProcedure = AutonomousManager.getProcedureFromSwitches(
            controlPanelAlgae.getRawButton(1),
            controlPanelAlgae.getRawButton(2),
            controlPanelAlgae.getRawButton(3),
            controlPanelAlgae.getRawButton(4)
        );
    }

    @Override
    public void autonomousPeriodic() {
        SwerveDrive.run();
        Coral.run();
        Algae.run();

        autonomousProcedure.ifPresent(proc -> {
            proc.run();
            proc.recordState();
        });
    }

    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.Headless);
        SwerveDrive.setRotationCurve(Controls::turnCurveRohan);
        SwerveDrive.setTranslationCurve(Controls::quinticCurveTwoDimensional);

        if (invertedTurn) {
            SwerveDrive.setRotationCurve((x) -> Controls.turnCurveRohan(-x));
        }
    }

    @Override
    public void teleopPeriodic() {
        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_DRIVER)) {
            driverTeleop();
        }

        if (DriverStation.isJoystickConnected(CONTROLLER_PORT_OPERATOR)
            || DriverStation.isJoystickConnected(CONTROL_PANEL_PORT_CORAL)
            || DriverStation.isJoystickConnected(CONTROL_PANEL_PORT_ALGAE)) {
            operatorTeleop();
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    private void recordLogFrame() {
        SwerveDrive.recordState();
        Pigeon.recordState();
        PoseEstimator.recordState();
        
        Coral.recordState();
        Algae.recordState();
    }

    /**
     * To be called periodically during teleop, this method handles all behavior that requires input
     * from the driver's controller.
     */
    private void driverTeleop() {
        // Reset field oriented heading, based on alliance.
        if (controllerDriver.getStartButtonReleased()) {
            Pigeon.setYaw(new Angle().setDegrees(0.0));
        }

        boolean isFieldOriented
            = SwerveDrive.getMode() == SwerveMode.Headless
            || SwerveDrive.getMode() == SwerveMode.SetAngle;

        // Left stick changes between headless and relative control modes.
        if (controllerDriver.getLeftStickButtonReleased()) {
            SwerveDrive.setMode(
                isFieldOriented ? SwerveMode.Relative : SwerveMode.Headless
            );
        }

        // Invert turn on right stick.
        if (controllerDriver.getRightStickButtonReleased()) {
            if (!invertedTurn) {
                SwerveDrive.setRotationCurve(x -> Controls.turnCurveRohan(-x));
                invertedTurn = true;
            } else {
                SwerveDrive.setRotationCurve(Controls::turnCurveRohan);
                invertedTurn = false;
            }
        }

        double pov = controllerDriver.getPOV();

        // If a D-Pad button is pressed, snap to that angle.
        if (pov != -1 && isFieldOriented) {
            Angle angle = new Angle().setDegrees(-(double) pov);

            SwerveDrive.setMode(SwerveMode.SetAngle);
            SwerveDrive.setTargetAngle(angle);
        }

        // If the driver is not actively turning the robot, enable automated turn control to
        // compensate for drift.
        if (
            SwerveDrive.getMode() == SwerveMode.Headless &&
            Math.abs(controllerDriver.getRightX()) < 0.15
        ) {
            SwerveDrive.setTargetAngle(Pigeon.getYaw());
            SwerveDrive.setMode(SwerveMode.SetAngle);
        }

        // If the driver tries to trun the robot, drop out of automated turn control.
        if (
            SwerveDrive.getMode() == SwerveMode.SetAngle &&
            Math.abs(controllerDriver.getRightX()) > 0.15
        ) {
            SwerveDrive.setMode(SwerveMode.Headless);
        }

        SwerveDrive.conditionalTempTranslationCurve(
            Controls.cardinalLock(Controls::defaultCurveTwoDimensional),
            controllerDriver.getXButton()
        ); // Lock to cardinal directions.
        SwerveDrive.conditionalTempTranslationCurve(
            (x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 3.0,
            controllerDriver.getRightBumperButton()
        ); // Lessen translation speed.
        SwerveDrive.conditionalTempTranslationCurve(
            Controls.cardinalLock((x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 3.0),
            controllerDriver.getRightBumperButton() &&
            controllerDriver.getXButton()
        ); // Lessen translation speed and lock cardinal directions.

        SwerveDrive.conditionalTempMode(SwerveMode.Rock, controllerDriver.getBButton());
        SwerveDrive.conditionalTempMode(SwerveMode.Relative, controllerDriver.getYButton());

        double speedX = -controllerDriver.getLeftY();
        double speedY = -controllerDriver.getLeftX();

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            speedX *= -1.0;
            speedY *= -1.0;
        }

        SwerveDrive.run(speedX, speedY, -controllerDriver.getRightX());
    }

    /**
     * To be called periodically during teleop, this method handles all behavior that requires input
     * from the operator's controller.
     */
    public void operatorTeleop() {
        double dealgaeSpeed = 0.0;
        double groundAlgaeSpeed = 0.0;
        ScoringPoint scoringPoint = ScoringPoint.Stowed;
        IntakeState coralIntakeState = IntakeState.Idle;
        AlgaePose algaePose = AlgaePose.Stow;

        if (controlPanelAlgae.getRawButton(7)) {
            dealgaeSpeed = -1;
        } else if (controlPanelAlgae.getRawButton(6)){
            dealgaeSpeed = 1;
        }

        if (controlPanelAlgae.getRawButton(9)) {
            groundAlgaeSpeed = 1.0;
        } else if (controlPanelAlgae.getRawButton(8)) {
            groundAlgaeSpeed = -1.0;
        }

        if (controlPanelCoral.getRawButton(1)) {
            scoringPoint = ScoringPoint.ScoringL1;
        } else if (controlPanelCoral.getRawButton(2)) {
            scoringPoint = ScoringPoint.ScoringL2;
        } else if (controlPanelCoral.getRawButton(3)) {
            scoringPoint = ScoringPoint.ScoringL3;
        } else if (controlPanelCoral.getRawButton(4)) {
            scoringPoint = ScoringPoint.ScoringL4;
        } else if (controlPanelCoral.getRawButton(6)) {
            scoringPoint = ScoringPoint.DealgaeficateL2;
        } else if (controlPanelCoral.getRawButton(5)) {
            scoringPoint = ScoringPoint.DealgaeficateL3;
        }

        if (controlPanelAlgae.getRawButton(10)) {
            coralIntakeState = IntakeState.Intaking;
        } else if (controlPanelAlgae.getRawButton(11)) {
            coralIntakeState = IntakeState.Outtaking;
        }

        if (controlPanelCoral.getRawButton(8)) {
            algaePose = AlgaePose.Processor;
        } else if (controlPanelCoral.getRawButton(9)) {
            algaePose = AlgaePose.Intake;
        }

        if (controlPanelAlgae.getRawButton(5)) {
            algaePose = AlgaePose.Handoff;
            scoringPoint = ScoringPoint.Handoff;
        }

        Coral.setDealgaeficatorSpeed(dealgaeSpeed);
        Coral.setScoringPoint(scoringPoint);
        Coral.setIntakeState(coralIntakeState);

        Algae.setIntakeSpeed(groundAlgaeSpeed);
        Algae.setPose(algaePose);

        Algae.run();
        Coral.run();
        
        double mainOverrideSpeed = Controls.defaultCurve(-controllerOperator.getLeftY());
        double secondaryOverrideSpeed = Controls.defaultCurve(-controllerOperator.getRightY());
        double intakeOverrideSpeed
            = Controls.defaultCurve(controllerOperator.getRightTriggerAxis())
            - Controls.defaultCurve(controllerOperator.getLeftTriggerAxis());

        if (controllerOperator.getRightBumperButton()) {
            Coral._setElevatorSpeed(mainOverrideSpeed);
            Coral._setPivotSpeed(secondaryOverrideSpeed / 3.0);
            Coral._setIntakeSpeed(intakeOverrideSpeed);
        }

        if (controllerOperator.getXButton()) {
            Algae._setPivotSpeed(mainOverrideSpeed);
            Algae.setIntakeSpeed(intakeOverrideSpeed);
        }

        if (controllerOperator.getStartButtonReleased()) {
            Coral._setElevatorPose(0.0);
        }

        if (controlPanelCoral.getRawButton(7)) {
            Coral._setElevatorSpeed(-0.75);
        }
    }

    XboxController simController = new XboxController(5);
    Pose2d simPose = new Pose2d();
    double simFrameTime = DriverStation.getMatchTime();
    double simDeltaTime = 0;

    @Override
    public void simulationPeriodic() {
        double rotationSpeed = simController.getRightX();

        if (Math.abs(rotationSpeed) < 0.1) {
            rotationSpeed = 0;
        }

        double translationSpeedX = simController.getLeftX();

        if (Math.abs(translationSpeedX) < 0.1) {
            translationSpeedX = 0;
        }

        double translationSpeedY = simController.getLeftY();

        if (Math.abs(translationSpeedY) < 0.1) {
            translationSpeedY = 0;
        }

        double dx = Math.pow(translationSpeedX, 3) * 0.1;
        double dy = Math.pow(-translationSpeedY, 3) * 0.1;
        double dTheta = Math.pow(-rotationSpeed, 5) * 0.1;

        simPose = new Pose2d(
            simPose.getX() + dx,
            simPose.getY() + dy,
            simPose.getRotation().plus(new Rotation2d(dTheta))
        );

        Logger.recordOutput("Simulation Pose", simPose);
        Logger.recordOutput("Simulation Delta Time", simDeltaTime);
    }
}
