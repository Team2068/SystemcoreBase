package first.robot.utility;

import org.wpilib.command2.SubsystemBase;
import first.robot.subsystems.Swerve;

public class IO extends SubsystemBase {
        public final Swerve chassis = new Swerve();
        
        public IO() {}

        @Override
        public void periodic() {}
}