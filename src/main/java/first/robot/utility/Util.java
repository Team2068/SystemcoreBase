package first.robot.utility;

import java.util.function.BooleanSupplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ConditionalCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.Subsystem;

public class Util {
    public static InstantCommand Do(Runnable action, Subsystem... systems) {
        return new InstantCommand(action, systems);
    }

    public static Command Do(Runnable action, Runnable end, Subsystem... systems) {
        return Commands.runEnd(action, end, systems);
    }

    public static InstantCommand blank = new InstantCommand();

    public static Command DoCheck(Command cmd, BooleanSupplier condition, Subsystem... systems){
        return new ConditionalCommand(cmd, blank, condition);
    }

    public static Command DoUntil(Runnable action, Runnable end, BooleanSupplier Condition, Subsystem... systems) {
        return Commands.runEnd(action, end, systems).until(Condition);
    }

    public static ConditionalCommand Do(Command onTrue, Command onFalse, BooleanSupplier condition){
        return new ConditionalCommand(onTrue, onFalse, condition);
    }
}