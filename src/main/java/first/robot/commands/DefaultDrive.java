package first.robot.commands;

import first.robot.subsystems.Swerve.swerveState;
import first.robot.swerve.Swerve;
import first.robot.swerve.Swerve.Constants;
import first.robot.utility.IO;

import java.util.function.DoubleSupplier;

import org.wpilib.math.kinematics.ChassisVelocities;

public class DefaultDrive extends Command {

    private final IO io;
    private CommandXboxController controller;
    private final DoubleSupplier x_supplier;
    private final DoubleSupplier y_supplier;
    private final DoubleSupplier rotation_supplier;

    public DefaultDrive(IO io, ChassisSpeeds chassisSpeeds) {
        this(io, () -> chassisSpeeds.vxMetersPerSecond, () -> chassisSpeeds.vyMetersPerSecond,
                () -> chassisSpeeds.omegaRadiansPerSecond);
    }

    public DefaultDrive(IO io, CommandXboxController controller) {
        this(io, () -> -modifyAxis(controller.getLeftY()) * Swerve.Constants.MAX_VELOCITY,
                () -> -modifyAxis(controller.getLeftX()) * Swerve.Constants.MAX_VELOCITY,
                () -> -modifyAxis(controller.getRightX()) * Swerve.Constants.MAX_ANGULAR_VELOCITY);
        this.controller = controller;
    }

    public DefaultDrive(IO io,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {

        this.io = io;
        this.x_supplier = translationXSupplier;
        this.y_supplier = translationYSupplier;
        this.rotation_supplier = rotationSupplier;

        addRequirements(io.chassis);
    }

    @Override
    public void execute() {
        double down_scale = 1.25 - modifyAxis(controller.getLeftTriggerAxis());
        double up_scale = (Swerve.Constants.MAX_VELOCITY * .2) * modifyAxis(controller.getRightTriggerAxis());
        double shooting = (io.chassis.currentState == swerveState.SCORING) ? 2 : 1;

        double scale = (Constants.transFactor * down_scale + up_scale) / shooting;
        double rot_scale = Constants.rotFactor * down_scale;

        double xSpeed = x_supplier.getAsDouble() * scale;
        double ySpeed = y_supplier.getAsDouble() * scale;

        double rotationSpeed;

        if (io.chassis.targetRotation != null) {
            rotationSpeed = io.chassis.targetRotation;
        } else {
            rotationSpeed = rotation_supplier.getAsDouble() * down_scale * rot_scale
            ;
        }

        ChassisVelocities output = new ChassisVelocities(xSpeed, ySpeed, rotationSpeed);

        if (io.chassis.field_oritented)
            output = ChassisVelocities.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, io.chassis.rotation());

        io.chassis.drive(output);
    }

    @Override
    public void end(boolean interrupted) {
        io.chassis.stop();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) <= deadband)
            return 0.0;
        deadband *= (value > 0.0) ? 1 : -1;
        return (value + deadband) / (1.0 + deadband);
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.1);
        value = Math.copySign(value * value, value);
        return value;
    }
}