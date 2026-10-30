package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.PrintCommand;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.Selectable;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;

import first.robot.commands.DriveCommand;
import first.robot.utility.AutomatedController;
import first.robot.utility.SubsystemManager;
import first.robot.utility.Util;

public class RobotContainer {
  public SubsystemManager subsystemManager = new SubsystemManager();
  public final AutomatedController main;

  private Selectable<Command> auto_selector;
  Command current_auto = new PrintCommand("");
  final Selectable<Integer> driver_selector = new Selectable<Integer>();

  public RobotContainer() {
    CanandEventLoop.getInstance();
    main = new AutomatedController(0, subsystemManager);

    auto_selector = AutoBuilder.buildAutoChooser();
    auto_selector.onChange((command) -> {
      current_auto = command;
    });

    Telemetry.log("Autos", auto_selector);
    Telemetry.log("Main-Controller Mode", main.selector);

    Telemetry.log("Driver", driver_selector);
    subsystemManager.chassis.setDefaultCommand(new DriveCommand(subsystemManager, main.controller));
  }

  public void configureAuton() {
    NamedCommands.registerCommand("SyncEncoders", Util.Do(subsystemManager.chassis::syncEncoders));
  }

  public Command getAutonomousCommand() {
    return auto_selector.getSelected();
  }
}
