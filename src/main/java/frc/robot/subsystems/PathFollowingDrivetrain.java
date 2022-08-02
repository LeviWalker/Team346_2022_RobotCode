package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotContainer;

/**
 * An extension of the regular drivetrain that houses
 * the needed methods for path following.
 */
public class PathFollowingDrivetrain extends Drivetrain {

    /**
     * Keeps track of where the robot is on the field
     */
    private DifferentialDriveOdometry odometry;

    private static final double TRACKWIDTH_FEET = 1;
    private static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACKWIDTH_FEET);
    private static final double DRIVE_KS = 0;
    private static final double DRIVE_KV = 0;
    private static final double AUTO_KP = 0;
    private static final double AUTO_KI = 0;
    private static final double AUTO_KD = 0;
    private static final double MAX_AUTO_SPEED_FPS = 0;
    private static final double MAX_AUTO_ACCELERATION_FPS2 = 0;

    public PathFollowingDrivetrain() {
        super();

        odometry = new DifferentialDriveOdometry(new Rotation2d(0));
    }

    /**
     * Drives the robot using volts directly to avoid
     * 
     * @param leftV  volts for the left side
     * @param rightV volts for the right side
     */
    public void tankDriveVolts(double leftV, double rightV) {
        leftDrive.setVoltage(leftV);
        rightDrive.setVoltage(rightV);
        drive.feed();
    }

    /**
     * Returns the differential drive speeds (inches per second)
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityFeetPerSecond(), getRightVelocityFeetPerSecond());
    }

    /**
     * Returns the position of the left side of the drivetrain.
     * 
     * @return position in feet
     */
    public double getLeftPositionFeet() {
        return leftEncoder.getPosition() * revPerInch / 12;
    }

    /**
     * Returns the position of the right side of the drivetrain.
     * 
     * @return position in feet
     */
    public double getRightPositionFeet() {
        return rightEncoder.getPosition() * revPerInch / 12;
    }

    /**
     * Returns the velocity of the left side of the drivetrain.
     * 
     * @return velocity in inches per second
     */
    public double getLeftVelocityFeetPerSecond() {
        // revPerInch actually has units inches/rev
        return leftEncoder.getVelocity() * revPerInch / (60 * 12);
    }

    /**
     * Returns the velocity of the right side of the drivetrain.
     * 
     * @return velocity in inches per second
     */
    public double getRightVelocityFeetPerSecond() {
        return rightEncoder.getVelocity() * revPerInch / (60 * 12);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return getContinuousAngleDegrees(RobotContainer.climber.gyro.getGyroAngleZ());
    }

    /**
     * Returns the heading of the robot as a Rotation2d.
     *
     * @return Rotation2D with the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Converts an accumulated angle in degrees to one that is
     * continuous from -180 degrees to 180 degrees.
     */
    private double getContinuousAngleDegrees(double angleDegrees) {
        return MathUtil.inputModulus(angleDegrees, -180, 180);
    }

    /**
     * Resets the odometry to a given position
     * 
     * @param pose current position
     */
    public void resetOdometry(Pose2d pose) {
        resetDrivetrainEncoders();
        odometry.resetPosition(pose, getHeadingRotation2d());
    }

    /**
     * Updates the odometry with the current heading and encoder positions
     */
    public void updateOdometry() {
        odometry.update(getHeadingRotation2d(), getLeftPositionFeet(), getRightPositionFeet());
    }

    /**
     * Returns the position of the drivetrain
     * 
     * @return position in feet, angle measured in degrees
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters(); // says meters but actually feet
    }

    @Override
    public void periodic() {
        super.periodic(); // just to make sure this is compatable with base drivetrain

        updateOdometry();

        // // says meters but it's actually feet
        // SmartDashboard.putNumber("drive x feet", getPose().getX());
        // SmartDashboard.putNumber("drive y feet", getPose().getY());
        // SmartDashboard.putNumber("drive theta degrees",
        // getPose().getRotation().getDegrees());
    }

    /**
     * Makes the trajectory following command
     * 
     * @param trajectory trajectory to follow
     * @return
     */
    public RamseteCommand makeFollowingCommand(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,
                this::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV),
                KINEMATICS,
                this::getWheelSpeeds,
                new PIDController(AUTO_KP, AUTO_KI, AUTO_KD),
                new PIDController(AUTO_KP, AUTO_KI, AUTO_KD),
                this::tankDriveVolts,
                this);
    }

    /**
     * Creates a configuration that places constraints on the drive motion
     * for a forward moving trajectory.
     * @return the configuration
     */
    public TrajectoryConfig getTrajectoryConfig() {
        TrajectoryConfig config = new TrajectoryConfig(
                MAX_AUTO_SPEED_FPS,
                MAX_AUTO_ACCELERATION_FPS2);
        config.addConstraint(
                new DifferentialDriveKinematicsConstraint(KINEMATICS, 5));
        return config;
    }

    /**
     * Creates a configuration that places constraints on the drive motion
     * for a backwards moving trajectory.
     * @return the configuration
     */
    public TrajectoryConfig getReverseTrajectoryConfig() {
        return getTrajectoryConfig().setReversed(true);
    }
}