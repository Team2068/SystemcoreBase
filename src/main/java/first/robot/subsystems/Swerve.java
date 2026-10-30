package first.robot.subsystems;

import static org.wpilib.units.Units.Degree;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import first.robot.swerve.Swerve.Constants;
import first.robot.utility.Util;
import first.robot.swerve.Module;

public class Swerve extends SubsystemBase {

    public boolean field_oritented = true;
    private final SwerveDriveKinematics kinematics;

    public final Pigeon2 pigeon2 = new Pigeon2(Constants.PIGEON_ID, first.robot.swerve.Swerve.getBus());

    StructArrayPublisher<SwerveModuleVelocity> current_states = Util.table
    .getStructArrayTopic("Current Module States",
    SwerveModuleVelocity.struct).publish();
    StructArrayPublisher<SwerveModuleVelocity> target_states = Util.table
    .getStructArrayTopic("Target Module States",
    SwerveModuleVelocity.struct).publish();
    StructPublisher<Pose2d> posePublisher =
    NetworkTableInstance.getDefault().getTable("Debug")
    .getStructTopic("Current Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> estimatedPublisher =
    NetworkTableInstance.getDefault().getTable("Debug")
    .getStructTopic("Estimated Pose", Pose2d.struct).publish();
    StructPublisher<Pose2d> estimatedPublisherWithYaw =
    NetworkTableInstance.getDefault().getTable("Debug")
    .getStructTopic("Estimated Pose with Yaw", Pose2d.struct).publish();

    SwerveDrivePoseEstimator estimator;
    SwerveDrivePoseEstimator estimatorWithYaw;

    final SwerveDriveOdometry odometry;
    final Module[] modules = new Module[4];
    ChassisVelocities speeds = new ChassisVelocities();
    public final Constants constants = new Constants();
    Timer timer = new Timer();

    Limelight limelight = new Limelight("limelight-main");

    public boolean active = true;

    public Swerve() {
        kinematics = new SwerveDriveKinematics(
                new Translation2d(constants.TRACKWIDTH / 2.0, constants.WHEELBASE / 2.0),
                new Translation2d(constants.TRACKWIDTH / 2.0, -constants.WHEELBASE / 2.0),
                new Translation2d(-constants.TRACKWIDTH / 2.0, constants.WHEELBASE / 2.0),
                new Translation2d(-constants.TRACKWIDTH / 2.0, -constants.WHEELBASE / 2.0));

        // ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(
                    Constants.DRIVE_ID[i],
                    Constants.STEER_ID[i],
                    Constants.ENCODER_ID[i]);
        }

        odometry = new SwerveDriveOdometry(kinematics, rotation(), modulePositions(),
                new Pose2d(0, 0, new Rotation2d()));

        estimator = new SwerveDrivePoseEstimator(kinematics, rotation(), modulePositions(), odometry.getPose());
        estimatorWithYaw = new SwerveDrivePoseEstimator(kinematics, rotation(), modulePositions(), odometry.getPose());

        AutoBuilder.configure(
                this::pose,
                this::resetOdometry,
                this::getSpeeds,
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(2.5, 0.0, 0.0),
                        new PIDConstants(0.0, 0.0, 0.0)
                ),
                constants.autoConfig,
                () -> {
                    var alliance = MatchState.getAlliance();
                    return alliance.isPresent() && alliance.get() == Alliance.RED;
                },
                this);

        timer.start();
    }

    public void resetAngle() {
        pigeon2.setYaw(0);
    }

    public Rotation2d rotation() {
        double rotation = (pigeon2.getYaw().getValueAsDouble() + 90) % 360;
        rotation += (rotation < 0) ? 360 : 0;
        return new Rotation2d(Degree.of(rotation - 180));
    }

    public void adjustRotation() {
        pigeon2.setYaw((rotation().getDegrees() + 90) % 360);
    }

    public void drive(ChassisVelocities speeds) {
        this.speeds = speeds;
    }

    public void stop() {
        speeds = new ChassisVelocities();
    }

    public double distance(Pose2d reference_point) {
        return pose().getTranslation().getDistance(reference_point.getTranslation());
    }

    public SwerveModulePosition[] modulePositions() {
        SwerveModulePosition[] pos = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++)
            pos[i] = new SwerveModulePosition(modules[i].drivePosition(), Rotation2d.fromRadians(modules[i].angle()));
        return pos;
    }

    public double getYaw() {
        return (pigeon2.getYaw().getValueAsDouble() % 360 + 360) % 360;
    }

    public Pose2d pose() {
        return odometry.getPose();
    }

    public void resetOdometry() {
        resetOdometry(new Pose2d(new Translation2d(), new Rotation2d(0)));
    }

    public void resetOdometry(Pose2d pose) {
        pigeon2.setYaw(pose.getRotation().getDegrees());
        resetModulePositions();

        odometry.resetPosition(rotation(), modulePositions(), pose);
    }

    public void resetModulePositions() {
        for (Module mod : modules)
            mod.resetDrivePosition();
    }

    public void syncEncoders() {
        for (Module mod : modules)
            mod.syncEncoders();
    }

    public void zeroAbsolute() {
        for (Module mod : modules)
            mod.zeroAbsolute();
    }

    public ChassisVelocities getSpeeds() {
        return kinematics.toChassisVelocities(moduleStates(modules));
    }

    public void setModuleStates(SwerveModuleVelocity[] states) {
        SwerveDriveKinematics.desaturateWheelVelocities(states, Constants.MAX_VELOCITY);
        for (int i = 0; i < modules.length; i++) {
            states[i].optimize(new Rotation2d(modules[i].angle()));
            modules[i].set((states[i].velocity / Constants.MAX_VELOCITY) * .8,
                    states[i].angle.getRadians());
        }
    }

    public SwerveModuleVelocity[] moduleStates(Module[] modules) {
        SwerveModuleVelocity[] state = new SwerveModuleVelocity[4];
        for (int i = 0; i < modules.length; i++)
            state[i] = new SwerveModuleVelocity(modules[i].velocity(), new Rotation2d(modules[i].angle()));
        return state;
    }

    public double getRoll() {
        return pigeon2.getRoll().getValueAsDouble();
    }

    public void toggle() {
        active = !active;

        for (Module mod : modules)
            mod.stop();
    }

    public void estimatePose(Limelight limelight) {
        estimator.update(rotation(), modulePositions());
        estimatorWithYaw.update(rotation(), modulePositions());
        limelight.setRobotOrientation(pigeon2.getYaw().getValueAsDouble(), 0, pigeon2.getPitch().getValueAsDouble(), 0, pigeon2.getRoll().getValueAsDouble(), 0, false);
        PoseEstimate mt1 = limelight.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);

        boolean doRejectUpdate = false;

        if (mt1.reportedTagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .30) {
                doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].getDistanceToCamera() > 3) {
                doRejectUpdate = true;
            }
        }
        if (mt1.reportedTagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.05, .07, 9999999));
            estimatorWithYaw.setVisionMeasurementStdDevs(VecBuilder.fill(.05, .07, 0.5));
            estimator.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
            estimatedPublisher.set(estimator.getEstimatedPosition());

            estimatorWithYaw.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
            estimatedPublisherWithYaw.set(estimatorWithYaw.getEstimatedPosition());
        }
    }

    public Pose2d getEstimatedPose() {
        return estimator.getEstimatedPosition();
    }

    public Pose2d getEstimatedPoseWithYaw() {
        return estimatorWithYaw.getEstimatedPosition();
    }

    public double getEstimatedRotation() {
        return getEstimatedPoseWithYaw().getRotation().getDegrees();
    }

    public void periodic() {
        SwerveModuleVelocity[] states = kinematics.toSwerveModuleVelocities(speeds);
        if (active && speeds != new ChassisVelocities())
            setModuleStates(states);

        current_states.set(moduleStates(modules));
        target_states.set(states);

        Pose2d pose = odometry.update(rotation(), modulePositions());
        estimatePose(limelight);
        posePublisher.set(pose);
        estimatedPublisher.set(estimator.getEstimatedPosition());
        estimatedPublisherWithYaw.set(estimatorWithYaw.getEstimatedPosition());

        Telemetry.log("Timer", timer.get());

        if ((int) (timer.get()) >= 5) {
            syncEncoders();

            timer.restart();
        }

        Telemetry.log("Limelight Estimated Rotation", getEstimatedRotation());
        Telemetry.log("Odometry rotation", rotation().getDegrees());
        Telemetry.log("Pigeon Yaw", getYaw());
        Telemetry.log("Pigeon Pitch",
                pigeon2.getPitch().getValueAsDouble());
        Telemetry.log("Pigeon Roll",
                pigeon2.getRoll().getValueAsDouble());

        Telemetry.log("Drive Mode", (field_oritented) ? "Field-Oriented" : "Robot-Oriented");
    }
}