package first.robot.utility;

import org.wpilib.command2.SubsystemBase;
import first.robot.subsystems.Swerve;

public class SubsystemManager extends SubsystemBase {
        public final Swerve chassis = new Swerve();
        
        public SubsystemManager() {}

        @Override
        public void periodic() {}
}