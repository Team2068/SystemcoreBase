package first.robot.swerve;

import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.robot.swerve.Swerve.Constants;


public class Module {
    public final TalonFX drive;
    public final TalonFX steer;
    public final Swerve.Encoder encoder;

    double desiredAngle;
    PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEER_REDUCTION = 26.09;

    public Module(int driveID, int steerID, int encoderID) {
        drive = new TalonFX(driveID, Constants.getSwerveBus());
        steer = new TalonFX(steerID, Constants.getSwerveBus());
        encoder = new Swerve.Canand(encoderID);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.CurrentLimits.StatorCurrentLimit = 80;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        steerConfig.Slot0.kP = 10;
        steerConfig.Slot0.kI = 0.0;
        steerConfig.Slot0.kD = 0.0;
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        steerConfig.Feedback.SensorToMechanismRatio = STEER_REDUCTION;
        
        steer.getConfigurator().apply(steerConfig);
        steer.setPosition(angle() / Constants.PI2);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.CurrentLimits.StatorCurrentLimit = 80;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfig.CurrentLimits.SupplyCurrentLimit = 20;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        drive.getConfigurator().apply(driveConfig);


        // TODO Find alternative to Shuffleboard tab to log module data
    }

    public void resetDrivePosition() {
        drive.setPosition(0.0);
    }

    public void syncEncoders() {
        steer.setPosition(encoder.angle() / Constants.PI2);
    }

    public void zeroAbsolute() {
        encoder.zero();
    }

    public double drivePosition() {
        return drive.getPosition().getValueAsDouble() * .500 * WHEEL_DIAMETER;
    }

    public LinearVelocity velocity() {
        return MetersPerSecond.of(drive.getVelocity().getValueAsDouble() * Constants.PI2 * .500 * WHEEL_DIAMETER);
    }

    public Voltage voltage(){
        return drive.getMotorVoltage().getValue();
    }

    public double angle() {
        return encoder.angle();
    }

    public AngularVelocity steerVelocity(){
        return encoder.velocity();
    }

    public Voltage steerVoltage(){
        return steer.getSupplyVoltage().getValue();
    }

    public void stop() {
        drive.stopMotor();
        steer.stopMotor();
    }

    public void set(double driveVolts, double targetAngle) {
        double normalized = MathUtil.inputModulus(targetAngle, 0, Constants.PI2);
        drive.setThrottle(driveVolts);
        steer.setControl(positionRequest.withPosition(normalized / Constants.PI2));
    }

    public void setSteer(double steerVolts){
        drive.setThrottle(0);
        steer.setThrottle(steerVolts);
    }
}