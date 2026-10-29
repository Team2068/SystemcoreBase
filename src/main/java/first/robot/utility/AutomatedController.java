package first.robot.utility;

import java.util.function.BooleanSupplier;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.event.EventLoop;
import org.wpilib.tunable.Selectable;

import first.robot.commands.DefaultDrive;

public class AutomatedController {
    public final CommandXboxController controller;
    public final Selectable<Runnable> selector = new Selectable<Runnable>();
    public int mode = 0;
    public EventLoop eventLoop = new EventLoop();

    IO io;

    public AutomatedController(int port, IO io) {
        this.io = io;

        selector.addDefault("Automated", () -> {mode = 0;});
        selector.addDefault("Manual", () -> {mode = 1;});
        selector.add("Characterise", () -> {mode = 2;});
        selector.addDefault("Debug", () -> {mode = 3;});

        selector.onChange((x) -> {
            x.run();
        });

        controller = new Gamepad(port);

        controller.rightStick()
                .onTrue(new InstantCommand(() -> io.chassis.field_oritented = !io.chassis.field_oritented));
        controller.leftStick().debounce(2).onTrue(new InstantCommand(io.chassis::resetAngle));
        configure();

    }

    public BooleanSupplier mode(int targetMode) {
        return () -> {
            return mode == targetMode;
        };
    }

    public BooleanSupplier automated() {
        return mode(0);
    }

    public BooleanSupplier manual() {
        return mode(1);
    }

    public BooleanSupplier characterise() {
        return mode(2);
    }

    public BooleanSupplier debug() {
        return mode(3);
    }

    public void switchMode() {
        mode = (mode + 1) % 3;
    }

    public void configure() {
        controller.start(eventLoop).and(controller.getHID()::getBackButtonPressed).onTrue(Util.Do(this::switchMode));
        configureAutomated();
        configureManual();
        configureCharacterisaton();
        configureDebug();

    }

    void configureAutomated() {
        // CHASSIS CONTROL
        controller.back().onTrue(Util.Do(io.chassis::resetAngle, io.chassis));
        controller.dpadDown().and(automated())
                .onTrue(Util.Do(() -> io.chassis.field_oritented = !io.chassis.field_oritented));
    }

    public void configureManual() {
        controller.back().and(manual()).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));
    }

    void configureCharacterisaton() {
        controller.x().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kForward));
        controller.a().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.quasistatic(Direction.kReverse));
        controller.y().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kForward));
        controller.b().and(characterise()).toggleOnTrue(io.chassis.driveRoutine.dynamic(Direction.kReverse));

        controller.dpadUp().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.quasistatic(Direction.kForward));
        controller.dpadDown().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.quasistatic(Direction.kReverse));
        controller.dpadRight().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kForward));
        controller.dpadLeft().and(characterise()).toggleOnTrue(io.chassis.steerRoutine.dynamic(Direction.kReverse));
    }

    void configureDebug() {
        controller.back().and(debug()).onTrue(Util.Do(io.chassis::resetOdometry, io.chassis));

        controller.povDown().and(debug()).onTrue(Util.Do(io.chassis::toggle));
        controller.povLeft().and(debug()).onTrue(Util.Do(io.chassis::syncEncoders));
        controller.povRight().and(debug()).and(() -> {
            return !io.chassis.active;
        }).onTrue(new InstantCommand(io.chassis::zeroAbsolute));

        controller.a().and(debug()).onTrue(Util.Do(() -> io.flywheel.hoodAngle(6.5)));
        controller.y().and(debug()).onTrue(Util.Do(io.flywheel::resetEncoder));
        // controller.x().and( debug()).onTrue(Util.Do(()-> io.flywheel.RPM(4500)));
        controller.x().and(debug()).onTrue(new DistanceShoot(io, new DistanceShootUtil(28, 5900)));

        controller.leftBumper().and(debug()).onTrue(Util.Do(() -> io.flywheel.hoodSpeed(0.1)))
                .onFalse(Util.Do(io.flywheel::stopHood));
        controller.rightBumper().and(debug()).onTrue(Util.Do(() -> io.flywheel.hoodSpeed(-0.1)))
                .onFalse(Util.Do(io.flywheel::stopHood));
        controller.rightTrigger().and(debug()).onTrue(Util.Do(() -> io.flywheel.flywheelSpeed(1)))
                .onFalse(Util.Do(io.flywheel::stopFlywheel));
        controller.leftTrigger().and(debug()).onTrue(Util.Do(() -> io.feeder.speed(.75)))
                .onFalse(Util.Do(io.feeder::stop));
        controller.povUp().and(debug()).onTrue(Util.Do(io.feeder::stop));
    }

}