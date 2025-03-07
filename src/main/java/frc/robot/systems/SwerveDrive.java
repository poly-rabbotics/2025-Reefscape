// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.Angle;
import frc.robot.subsystems.DrivePath;
import frc.robot.subsystems.SwerveMode;
import frc.robot.subsystems.SwerveModule;

/*
 * Manages the swerve drive train.
 */
public class SwerveDrive {
    // CAN IDs of drive motors and encoders.
    private static final int[] MODULE_MOVEMENT_CAN_IDS = { 1,   2,   3,   4  };
    private static final int[] MODULE_ROTATION_CAN_IDS = { 5,   6,   7,   8  };
    private static final int[] MODULE_CANCODER_CAN_IDS = { 9,   10,  11,  12 };
    
    // Side length of the drive. Assumes a square drive.
    private static final double CHASSIS_SIDE_LENGTH = 0.6604; // ~26"

    // Offsets of each absolute encoder (in this case CANCoders) from zero, where we take an angle
    // of zero to be forward. By adding an offset to the corosponding encoder's reported angle, we
    // should be given the actual angle of the swerve module's wheel.
    private static final Angle[] MODULE_CANCODER_OFFSETS = {
        new Angle().setDegrees(-71.89),
        new Angle().setDegrees(-38.94 - 180.0),
        new Angle().setDegrees(-64.86 - 180.0 - 10.0),
        new Angle().setDegrees(-389.16 + 90.0)
    };

    // Positions for each wheel in "Rock Mode".
    private static final Angle[] MODULE_ROCK_MODE_POSITIONS = { 
        new Angle().setRadians( -Angle.TAU / 8  ), 
        new Angle().setRadians(  Angle.TAU / 8  ), 
        new Angle().setRadians( -Angle.TAU / 8  ), 
        new Angle().setRadians(  Angle.TAU / 8  ) 
    };

    // The physical positions of the drive's modules from the drive's center, in meters. The drive's
    // center need not be the robot's center.
    private static final Translation2d[] MODULE_PHYSICAL_POSITIONS = {
        new Translation2d(  CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2),
        new Translation2d( -CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2),
        new Translation2d( -CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
        new Translation2d(  CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
    };

    // Human readable names for module positions, useful for logging and more easily understanding
    // the drives state quickly.
    private static final String[] MODULE_PHYSICAL_POSITION_NAMES = {
        "FR", "BR", "BL", "FL"
    };

    // PID parameter constants for following autonomous trajectories. These are used to follow a
    // setpoint given as a two dimensional point and a rotation.
    private static final double TRAJECTORY_STRAFE_X_PID_P = 0.75;
    private static final double TRAJECTORY_STRAFE_X_PID_I = 0.0;
    private static final double TRAJECTORY_STRAFE_X_PID_D = 0.0;

    private static final double TRAJECTORY_STRAFE_Y_PID_P = 0.75;
    private static final double TRAJECTORY_STRAFE_Y_PID_I = 0.0;
    private static final double TRAJECTORY_STRAFE_Y_PID_D = 0.0;

    private static final double TRAJECTORY_ROTATE_PID_P = 0.9;
    private static final double TRAJECTORY_ROTATE_PID_I = 0.0;
    private static final double TRAJECTORY_ROTATE_PID_D = 0.015;

    // Set angle constants. Use in teleop for "snap angles" available to the driver, or any other
    // controls function in teleop in which the driver may determine a desired angle, rather than
    // controlling the speed ot rotation itself.
    private static final double SET_ANGLE_PID_P = 0.9;
    private static final double SET_ANGLE_PID_I = 0.0;
    private static final double SET_ANGLE_PID_D = 0.015;

    // Module objects. Stores the module objects themselves which allow us to interface with them.
    private static final SwerveModule modules[] = new SwerveModule[MODULE_MOVEMENT_CAN_IDS.length];

    // Positions of the modules, are used for things like odometry.
    private static final SwerveModulePosition positions[]
        = new SwerveModulePosition[MODULE_MOVEMENT_CAN_IDS.length];

    // Kinematics takes our desired drive state and produces states for each module.
    private static final SwerveDriveKinematics kinematics;

    // PID controllers for autonomous trajectory following. Despite being an autonomous ability,
    // these controllers could be used in teleop, as the drive mode that uses them is always
    // available. Keep this in mind if auto-alignment to a target is desired and we have an accurate
    // source of odometry data.
    private static final PIDController trajectoryStrafeXController
        = new PIDController(
            TRAJECTORY_STRAFE_X_PID_P,
            TRAJECTORY_STRAFE_X_PID_I,
            TRAJECTORY_STRAFE_X_PID_D
        );
    private static final PIDController trajectoryStrafeYController
        = new PIDController(
            TRAJECTORY_STRAFE_Y_PID_P,
            TRAJECTORY_STRAFE_Y_PID_I,
            TRAJECTORY_STRAFE_Y_PID_D
        );
    private static final PIDController trajectoryRotateController
        = new PIDController(
            TRAJECTORY_ROTATE_PID_P,
            TRAJECTORY_ROTATE_PID_I,
            TRAJECTORY_ROTATE_PID_D
        );


    // Controller for set angles, may be used on command by the driver while retaining full 
    // translation control. This controller is still available to the whole class, and so may be 
    // employed elsewhere.
    private static final PIDController setAngleController
        = new PIDController(SET_ANGLE_PID_P, SET_ANGLE_PID_I, SET_ANGLE_PID_D);

    // Control curves for teleop run methods. The translation curve is two dimensional since both 
    // axes of the controller should be taken in context, rotation is simply one dimensional. The
    // intactive curves of either are for tracking what curve to return to when setting temporary
    // curves, which are used for hold-button functions rather than toggle-button ones. This allows
    // a curve to only be active when holdling a button, for example, a button to reduce speed may
    // be a hold-button.
    private static BiFunction<Double, Double, Double> translationCurve
        = Controls::defaultCurveTwoDimensional;
    private static BiFunction<Double, Double, Double> inactiveTransationCurve = null;
    private static Function<Double, Double> rotationCurve = Controls::defaultCurve;
    private static Function<Double, Double> inactiveRotationCurve = null;

    // Drive mode fields. Inactive mode works the same as an inactive curve, and tracks the drive
    // mode to return to when setting temporary modes. The display mode is used only for display,
    // and is useful when displaying aspects of the drive on something like LED lights.
    private static SwerveMode mode = SwerveMode.Headless;
    private static SwerveMode inactiveMode = null;
    private static SwerveMode displayMode = SwerveMode.Headless;

    // Drive state objects. Module states represent the state of each module as independent items,
    // they are used as input to each modules and allow us to control them within code.
    private static SwerveModuleState[] moduleStates = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    // A path for the `FOLLOW_PATH` drive mode.
    private static DrivePath path = null;

    // Target angle of the drive. This can be set from elsewhere in code via a setter method and can
    // allow the driver to control the angle of the drive, rather than rotation speed.
    // NOTE: See set angle controller and relevant constants above.
    private static Angle setAngle = new Angle().setRadians(0.0);

    // Translation speeds along the X and Y axes as well as rotation speed. These are set by drive
    // modes and can be accessed elsewhere in code. They are used to form ChassisSpeeds objects.
    private static double translationSpeedX = 0.0;
    private static double translationSpeedY = 0.0;
    private static double rotationSpeed = 0.0;

    static {
        // Create swerve modules using device CAN IDs, encoder offsets, and physical positions.
        for (int i = 0; i < MODULE_MOVEMENT_CAN_IDS.length; i++) {
            modules[i] = new SwerveModule(
                MODULE_MOVEMENT_CAN_IDS[i], 
                MODULE_ROTATION_CAN_IDS[i], 
                MODULE_CANCODER_CAN_IDS[i], 
                MODULE_CANCODER_OFFSETS[i]
            );
        }

        // Populate the modules positions for later.
        updateModulePositions();

        // Create the kinematics object. From now on all interfacin with kinematics must be done
        // with modules in the same order. We've already set up all our module related objects in
        // arrays of length four in the same order, which goes front right, back right, back left,
        // front left.
        kinematics = new SwerveDriveKinematics(
            MODULE_PHYSICAL_POSITIONS[0],
            MODULE_PHYSICAL_POSITIONS[1],
            MODULE_PHYSICAL_POSITIONS[2],
            MODULE_PHYSICAL_POSITIONS[3]
        );

        trajectoryStrafeXController.setTolerance(0.1);
        trajectoryStrafeYController.setTolerance(0.1);

        // Enable continuous input and set tolerance on the rotation controllers. Continuous input
        // between zero and tau will make the controller recognize that it works on a circle, and
        // that a position at zero or any multiple of tau are the same position.
        trajectoryRotateController.enableContinuousInput(0.0, Angle.TAU);
        trajectoryRotateController.setTolerance(0.1);
        setAngleController.enableContinuousInput(0.0, Angle.TAU);
        setAngleController.setTolerance(0.1);
    }

    /**
     * Sets the current mode of the swerve drive. This may change the way run methods work.
     * @param mode The mode in which to operate.
     */
    public static void setMode(SwerveMode mode) {
        SwerveDrive.mode = mode;
    }

    /**
     * Get the current non-temporary swerve mode.
     */
    public static SwerveMode getMode() {
        return mode;
    }

    /**
     * Gets the current display mode of the drive. The display mode is the
     * either the current set mode or a temporary set mode but is not cleared
     * at the end of a run loop, rather it is cleared once the next loop 
     * finishes, making it ideal for display purposes (e.i. LED lights).
     */
    public static SwerveMode getDisplayMode() {
        return displayMode;
    }

    /**
     * May be called before running the swerve drive to temporarily set a mode
     * for that call of a run method.
     * 
     * Each time a run method completes it will change the mode back to what it
     * was when this method was called.
     * 
     * If this method is called more than once before running then the latest
     * given temporary mode will be used, the inactive mode to be reset to after
     * running will not be effected
     */
    public static void tempMode(SwerveMode mode) {
        if (inactiveMode != null) {
            SwerveDrive.mode = mode;
            return;
        }

        inactiveMode = SwerveDrive.mode;
        SwerveDrive.mode = mode;
    }

    /**
     * Exactly like `tempMode` but predicates the temporary mode on the given 
     * boolean condition.
     */
    public static void conditionalTempMode(SwerveMode mode, boolean condition) {
        if (!condition) {
            return;
        }

        tempMode(mode);
    }

    /**
     * Sets the curve function for directional inputs (translations).
     * @param curve The BiFunction to use for proccessing the curve, the first 
     * argument is what should be curved, the second is used for context. Return
     * the curved direction.
     */
    public static void setTranslationCurve(BiFunction<Double, Double, Double> curve) {
        translationCurve = curve;
    }

    /**
     * Gets the current curve used for directional inputs.
     */
    public static BiFunction<Double, Double, Double> getTranslationCurve() {
        return translationCurve;
    }

    /**
     * Temporarily sets the curve function for directional inputs (translations).
     * This action will atomatically be undone after calling a run method.
     * @param curve The BiFunction to use for proccessing the curve, the first 
     * argument is what should be curved, the second is used for context. Return
     * the curved direction.
     */
    public static void tempTranslationCurve(BiFunction<Double, Double, Double> curve) {
        if (inactiveTransationCurve != null) {
            translationCurve = curve;
            return;
        }

        inactiveTransationCurve = translationCurve;
        translationCurve = curve;
    }

    /**
     * Exactly like `tempDirectionalCurve` but predicated on a boolean condition.
     */
    public static void conditionalTempTranslationCurve(
        BiFunction<Double, Double, Double> curve, 
        boolean condition
    ) {
        if (!condition) {
            return;
        }

        tempTranslationCurve(curve);
    }

    /**
     * Sets the curve function for turn inputs.
     * @param curve The Function to use for proccessing the curve.
     */
    public static void setRotationCurve(Function<Double, Double> curve) {
        rotationCurve = curve;
    }

    /**
     * Gets the Function currently used for turning.
     */
    public static Function<Double, Double> getRotationCurve() {
        return rotationCurve;
    }

    /**
     * Temporarily sets the curve function for turn inputs. Undone after running.
     * @param curve The Function to use for proccessing the curve.
     */
    public static void tempRotationCurve(Function<Double, Double> curve) {
        if (inactiveRotationCurve != null) {
            rotationCurve = curve;
            return;
        }

        inactiveRotationCurve = rotationCurve;
        rotationCurve = curve;
    }

    /**
     * Exactly like `tempTurnCurve` but predicated on a boolean condition.
     */
    public static void conditionalTempRotationCurve(
        Function<Double, Double> curve, 
        boolean condition
    ) {
        if (!condition) {
            return;
        }

        tempRotationCurve(curve);
    }

    /**
     * Runs with a static speed of zero for all inputs, meant to be used for
     * trajectory following in autonomous.
     */
    public static void run() {
        runUncurved(0.0, 0.0, 0.0);
    }

    /**
     * Runs swerve, behavior changes based on the drive's mode. This will reset
     * temporary modes on completion.
     * @param translationX The X axis of the directional control, between 1 and -1
     * @param translationY The Y axis of the directional control, between 1 and -1.
     * @param speed The speed scalar for the drive.
     * @param rotation A value between 1 and -1 that determines the turning speed.
     * @param lowSense The angle to move in low sensitivity in degrees, -1 for no movement.
     */
    public static void run(
        double translationX,
        double translationY,
        double speed,
        double rotation
    ) {
        speed = Math.abs(translationX) <= 0.05 && Math.abs(translationY) <= 0.05 ? 0.0 : speed;

        // angle is in radians as per Java's trig methods.
        var angle = Math.atan2(translationY, translationX);
        translationX = Math.cos(angle) * speed;

        translationY = Math.sin(angle) * speed;
        run(translationX, translationY, rotation);
    }

    /**
     * Runs swerve, behavior changes based on the drive's mode. Derives speed
     * from directional inputs. This will reset temporary modes on completion.
     * @param translationX The X axis of the directional control, between 1 and -1
     * @param translationY The Y axis of the directional control, between 1 and -1.
     * @param rotation A value between 1 and -1 that determines the turning speed.
     * @param lowSense The angle to move in low sensitivity in degrees, -1 for no movement.
     */
    public static void run(double translationX, double translationY, double rotation) {
        var x = translationCurve.apply(translationX, translationY);
        var y = translationCurve.apply(translationY, translationX);
        rotation = rotationCurve.apply(rotation);
        runUncurved(x, y, rotation);
    }
    
    /**
     * Run the swerve drive exactly by the arguments passed, no curving will 
     * occure on the given inputs, nor will they be deadbanded. This will reset
     * temporary modes on completion.
     * @param translationX Speed along the x axis. -1.0 - 1.0
     * @param translationY Speed along the y axis. -1.0 - 1.0
     * @param rotation Rate of turn. -1.0 - 1.0
     */
    public static void runUncurved(double translationX, double translationY, double rotation) {
        translationSpeedX = translationX;
        translationSpeedY = translationY;
        rotationSpeed = rotation;

        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        boolean holdPos = false;

        switch (mode) {
            case SetAngle: {
                double gyroAngle = Pigeon.getYaw().radians();
                rotationSpeed = setAngleController.calculate(gyroAngle, setAngle.radians());
                // Fall-through to `Headless`
            }

            case Headless: {
                moduleStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        rotationSpeed, 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
                break;
            }

            case Relative: {
                moduleStates = kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        rotationSpeed
                    )
                ); 
                break;
            }

            case Rock: {
                assert moduleStates.length == modules.length;
                holdPos = true;

                for (int i = 0; i < modules.length; i++) {
                    moduleStates[i] = new SwerveModuleState(
                        0.0,
                        new Rotation2d(
                            MODULE_ROCK_MODE_POSITIONS[i].radians()
                        )
                    );
                }

                break;
            }

            case FollowPath: {
                if (path == null || path.isCompleted()) {
                    // Break early if we have no path.
                    translationX = 0.0;
                    translationY = 0.0;
                    rotationSpeed = 0.0;

                    moduleStates = kinematics.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            translationX,
                            translationY,
                            rotationSpeed, 
                            new Rotation2d(Pigeon.getYaw().radians())
                        )
                    );

                    break;
                }

                // `path.currentTarget()` should never be empty here, as we have already determined
                // that the path is not empty above.
                Pose2d setPosition = path.currentTarget().get();
                Pose2d position = PoseEstimator.estimate2d();
                SwerveDrive.setAngle = Pigeon.getYaw();

                translationSpeedX
                    = trajectoryStrafeXController.calculate(position.getX(), setPosition.getX());
                translationSpeedY
                    = trajectoryStrafeYController.calculate(position.getY(), setPosition.getY());
                rotationSpeed = trajectoryRotateController.calculate(
                    position.getRotation().getRadians(),
                    setPosition.getRotation().getRadians()
                );

                moduleStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationSpeedX,
                        translationSpeedY,
                        rotationSpeed, 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );

                break;
            }

            // This branch should never be reached as the enum used should never
            // have more than the above possible values.
            default: assert false;
        }

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
            modules[i].setRockMode(holdPos);
            modules[i].run();
        }

        // Record module states

        SwerveDrive.moduleStates = moduleStates;
        updateModulePositions();

        // Reset temporary states
        
        displayMode = mode;
        
        if (inactiveMode != null) {
            mode = inactiveMode;
            inactiveMode = null;
        }
        
        if (inactiveTransationCurve != null) {
            translationCurve = inactiveTransationCurve;
            inactiveTransationCurve = null;
        }

        if (inactiveRotationCurve != null) {
            rotationCurve = inactiveRotationCurve;
            inactiveRotationCurve = null;
        }
    }

    /**
     * Updates the current positions of all modules.
     */
    private static void updateModulePositions() {
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
    }

    /**
     * Gets the average tempurature of all motors on the drive in celsius.
     */
    public static double getAverageMotorTemp() {
        double tempSum = 0.0;

        for (SwerveModule module : modules) {
            tempSum += module.getRotationMotorTemp();
            tempSum += module.getMovementMotorTemp();
        }

        return tempSum / (modules.length * 2.0);
    }

    /**
     * Sets the set angle for `SET_ANGLE` drive mode.
     */
    public static void setTargetAngle(Angle angle) {
        setAngle = angle;
    }

    /**
     * Sets the drive path for the `FOLLOW_PATH` mode.
     */
    public static void setPath(DrivePath path) {
        SwerveDrive.path = path;
    }

    /**
     * Create a new pose estimator for this drive.
     */
    public static SwerveDrivePoseEstimator3d createSwervePoseEstimator(Pose3d startPose) {
        return new SwerveDrivePoseEstimator3d(
            kinematics,
            new Rotation3d(
                Pigeon.getYaw().radians(),
                Pigeon.getPitch().radians(),
                Pigeon.getRoll().radians()
            ),
            positions,
            startPose
        );
    }

    /**
     * Sets the current position of the given estimator.
     */
    public static void setPoseOf(SwerveDrivePoseEstimator3d estimator, Pose3d pose) {
        estimator.resetPosition(
            new Rotation3d(
                Pigeon.getRoll().radians(),
                Pigeon.getPitch().radians(),
                Pigeon.getYaw().radians()
            ),
            positions,
            new Pose3d()
        );
    }

    /**
     * Updates the given pose estimator using measurements from swerve modules and the gyro.
     */
    public static void updatePoseEstimator(SwerveDrivePoseEstimator3d poseEstimator) {
        poseEstimator.update(
            new Rotation3d(
                Pigeon.getRoll().radians(),
                Pigeon.getPitch().radians(),
                Pigeon.getYaw().radians()
            ),
            positions
        );
    }

    /**
     * Record states for logging and displays like AdvantageKit.
     */
    public static void recordState() {
        SwerveModuleState[] measuredState = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            measuredState[i] = modules[i].getActualState();

            Logger.recordOutput(
                "Swerve Module " + MODULE_PHYSICAL_POSITION_NAMES[i] + " Movement Current",
                modules[i].getMovementTorqueCurrent()
            );
            Logger.recordOutput(
                "Swerve Module " + MODULE_PHYSICAL_POSITION_NAMES[i] + " Rotation Current",
                modules[i].getRotationTorqueCurrent()
            );
            Logger.recordOutput(
                "Swerve Module " + MODULE_PHYSICAL_POSITION_NAMES[i] + " Movement Voltage",
                modules[i].getMovementVoltage()
            );
            Logger.recordOutput(
                "Swerve Module " + MODULE_PHYSICAL_POSITION_NAMES[i] + " Rotation Voltage",
                modules[i].getRotationVoltage()
            );
            Logger.recordOutput(
                "Swerve Module " + MODULE_PHYSICAL_POSITION_NAMES[i] + " Movement Temp (Celsius)",
                modules[i].getMovementMotorTemp()
            );
            Logger.recordOutput(
                "Swerve Module " + MODULE_PHYSICAL_POSITION_NAMES[i] + " Rotation Temp (Celsius)",
                modules[i].getRotationMotorTemp()
            );
        }

        Logger.recordOutput("Swerve Module States (Set)", moduleStates);
        Logger.recordOutput("Swerve Module States (Measured)", measuredState);

        Logger.recordOutput("Swerve Average Motor Tempurature (Celsius)", getAverageMotorTemp());

        Logger.recordOutput("Swerve Speed Rotation", rotationSpeed);
        Logger.recordOutput("Swerve Speed Translation X", translationSpeedX);
        Logger.recordOutput("Swerve Speed Translation Y", translationSpeedY);

        Logger.recordOutput("Swerve Drive Mode", getMode());
        Logger.recordOutput("Swerve Display Mode", getDisplayMode());

        Logger.recordOutput("Swerve Set Angle (Radians)", setAngle.radians());
        Logger.recordOutput("Swerve Set Angle (Degrees)", setAngle.degrees());

        Logger.recordOutput(
            "Swerve Drive Set Pose", 
            Optional
                .ofNullable(path)
                .flatMap((path) -> path.currentTarget())
                .orElse(new Pose2d())
        );
    }
}
