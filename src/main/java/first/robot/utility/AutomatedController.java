package first.robot.utility;

import java.util.function.BooleanSupplier;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.event.EventLoop;
import org.wpilib.tunable.Selectable;

public class AutomatedController {
    public final CommandXboxController controller;
    public final Selectable<Runnable> selector = new Selectable<Runnable>();
    public int mode = 0;
    public EventLoop eventLoop = new EventLoop();

    SubsystemManager subsystemManager;

    public AutomatedController(int port, SubsystemManager subsystemManager) {
        this.subsystemManager = subsystemManager;

        selector.addDefault("Automated", () -> {mode = 0;});
        selector.addDefault("Manual", () -> {mode = 1;});
        selector.add("Characterise", () -> {mode = 2;});
        selector.addDefault("Debug", () -> {mode = 3;});

        selector.onChange((x) -> {
            x.run();
        });

        controller = new CommandXboxController(port);

        controller.rightStick()
                .onTrue(new InstantCommand(() -> subsystemManager.chassis.field_oritented = !subsystemManager.chassis.field_oritented));
        controller.leftStick().debounce(2).onTrue(new InstantCommand(subsystemManager.chassis::resetAngle));
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

    public BooleanSupplier debug() {
        return mode(2);
    }

    public void switchMode() {
        mode = (mode + 1) % 3;
    }

    public void configure() {
        controller.menu().and(controller.view()).onTrue(Util.Do(this::switchMode));
        configureAutomated();
        configureManual();
        configureDebug();
    }

    void configureAutomated() {
        controller.view().onTrue(Util.Do(subsystemManager.chassis::resetAngle, subsystemManager.chassis));
        controller.dpadDown().and(automated())
                .onTrue(Util.Do(() -> subsystemManager.chassis.field_oritented = !subsystemManager.chassis.field_oritented));
    }

    public void configureManual() {
        controller.view().and(manual()).onTrue(Util.Do(subsystemManager.chassis::resetOdometry, subsystemManager.chassis));
    }

    void configureDebug() {
        controller.view().and(debug()).onTrue(Util.Do(subsystemManager.chassis::resetOdometry, subsystemManager.chassis));
    }

}