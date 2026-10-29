// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

import com.reduxrobotics.canand.CanandEventLoop;

public class RobotContainer {
  public RobotContainer() {
    CanandEventLoop.getInstance();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
