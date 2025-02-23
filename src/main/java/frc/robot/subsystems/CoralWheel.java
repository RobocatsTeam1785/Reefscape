package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.CoralWheelConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class CoralWheel extends SubsystemBase {
    // hardware
    protected SparkMax motor;
    @Logged protected RelativeEncoder encoder;

    // loop control
    @Logged protected PIDController pid;
    protected SimpleMotorFeedforward ff;

    // logging
    protected Voltage sysIdVoltage;

    public CoralWheel() {
        initMotor();
        initControl();
    }

    // initialization
    protected void initMotor() {
        // motor config
        SparkMaxConfig config = new SparkMaxConfig();

        // see logic in SwerveModule#initMotors for an explanation involving units
        double rotationsToMeters = CoralWheelConstants.WHEEL_CF;
        double rpmToMetersPerSecond = CoralWheelConstants.WHEEL_CF / 60.0;

        // TODO evaluate whether these current limit and idle mode parameters are actually suitable
        // set motor parameters
        config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        // set encoder parameters
        config.encoder
            .positionConversionFactor(rotationsToMeters)
            .velocityConversionFactor(rpmToMetersPerSecond);

        // initialize motor and configure it
        // TODO check whether we're still using brushless motors
        motor = new SparkMax(CoralWheelConstants.MOTOR_ID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoder - an initial value of zero is fine, as we only care about velocity
        encoder = motor.getEncoder();
    }

    protected void initControl() {
        // configure PID and FF using constants
        pid = new PIDController(CoralWheelConstants.KP, CoralWheelConstants.KI, CoralWheelConstants.KD);
        ff = new SimpleMotorFeedforward(CoralWheelConstants.KS, CoralWheelConstants.KV, CoralWheelConstants.KA);
    }

    // system identification
     public void sysIdDrive(Voltage voltage) {
        // set value for use in SysIdRoutine logging
        sysIdVoltage = voltage;

        motor.setVoltage(voltage);
    }

    public void sysIdLog(SysIdRoutineLog log) {
        // TODO update these units if the conversion factor units change
        log.motor("wheel")
            .voltage(sysIdVoltage)
            .linearPosition(Meters.of(encoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(encoder.getVelocity()));
    }

    // drive
    /** updates the wheel velocity to the desired velocity setpoint */
    public void updateSetpoint(LinearVelocity velocity) {
        final double output = pid.calculate(encoder.getVelocity(), velocity.in(MetersPerSecond));
        final double feed = ff.calculate(velocity.in(MetersPerSecond));

        motor.setVoltage(output + feed);
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        pid.setPID(CoralWheelConstants.KP, CoralWheelConstants.KI, CoralWheelConstants.KD);
        ff = new SimpleMotorFeedforward(CoralWheelConstants.KS, CoralWheelConstants.KV, CoralWheelConstants.KA);
    }
}
