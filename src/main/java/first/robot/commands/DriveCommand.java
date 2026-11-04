package first.robot.commands;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.math.kinematics.ChassisVelocities;

import first.robot.swerve.Swerve;
import first.robot.swerve.Swerve.Constants;
import first.robot.utility.SubsystemManager;

public class DriveCommand extends Command {
  private final SubsystemManager subsystemManager;
  private CommandXboxController controller;
  private final DoubleSupplier x_supplier;
  private final DoubleSupplier y_supplier;
  private final DoubleSupplier rotation_supplier;

  public DriveCommand(SubsystemManager subsystemManager, DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.subsystemManager = subsystemManager;
    this.x_supplier = translationXSupplier;
    this.y_supplier = translationYSupplier;
    this.rotation_supplier = rotationSupplier;

    addRequirements(subsystemManager.chassis);
  }

  public DriveCommand(SubsystemManager subsystemManager, CommandXboxController controller) {
        this(subsystemManager, () -> -modifyAxis(controller.getLeftY()) * Swerve.Constants.MAX_VELOCITY,
                () -> -modifyAxis(controller.getLeftX()) * Swerve.Constants.MAX_VELOCITY,
                () -> -modifyAxis(controller.getRightX()) * Swerve.Constants.MAX_ANGULAR_VELOCITY);
        this.controller = controller;
  }

  @Override
  public void execute() {
    double down_scale = 1.25 - modifyAxis(controller.getLeftTrigger());
    double up_scale = (Swerve.Constants.MAX_VELOCITY * .2) * modifyAxis(controller.getRightTrigger());

    double scale = (Constants.transFactor * down_scale + up_scale);
    double rot_scale = Constants.rotFactor * down_scale;

    double xSpeed = x_supplier.getAsDouble() * scale;
    double ySpeed = y_supplier.getAsDouble() * scale;

    double rotationSpeed;

    rotationSpeed = rotation_supplier.getAsDouble() * down_scale * rot_scale;

    ChassisVelocities output = new ChassisVelocities(xSpeed, ySpeed, rotationSpeed);

    if (subsystemManager.chassis.field_oritented)
      output = output.toRobotRelative(subsystemManager.chassis.rotation());

    subsystemManager.chassis.drive(output);
  }

  @Override
  public void end(boolean interrupted) {
    subsystemManager.chassis.stop();
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
