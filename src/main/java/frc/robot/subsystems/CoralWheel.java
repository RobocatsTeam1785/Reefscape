package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.CoralWheelConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class CoralWheel extends SubsystemBase {
    // hardware
    public SparkMax motor;
    @Logged public RelativeEncoder encoder;

    // control/filtering
    @Logged public PIDController pid;
    public SimpleMotorFeedforward ff;

    public SlewRateLimiter rateLimiter;

    // logging
    public Voltage sysIdVoltage;
    @Logged public double lastVelocityMetersPerSecond;
    @Logged public double lastVoltageVolts;

    public CoralWheel() {
        initMotor();
        initControl();
    }

    // initialization
    public void initMotor() {
        // motor config
        // ! this resets the spark max configuration every time the robot code runs, so be careful
        SparkMaxConfig config = new SparkMaxConfig();

        // see logic in SwerveModule#initMotors for an explanation involving units
        double rotationsToMeters = CoralWheelConstants.WHEEL_CF;
        double rpmToMetersPerSecond = CoralWheelConstants.WHEEL_CF / 60.0;

        // TODO evaluate whether these current limit and idle mode parameters are actually suitable
        // set motor parameters
        config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

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

    public void initControl() {
        // configure PID and FF using constants
        pid = new PIDController(CoralWheelConstants.KP, CoralWheelConstants.KI, CoralWheelConstants.KD);
        ff = new SimpleMotorFeedforward(CoralWheelConstants.KS, CoralWheelConstants.KV, CoralWheelConstants.KA);

        // filtering
        // set the maximum acceleration of provided setpoints
        rateLimiter = new SlewRateLimiter(CoralWheelConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond));
    }

    // state
    public Voltage lastVoltage() {
        return Volts.of(lastVoltageVolts);
    }

    public LinearVelocity linearVelocity() {
        return MetersPerSecond.of(encoder.getVelocity());
    }

    // system identification
     public void sysIdDrive(Voltage voltage) {
        // set value for use in SysIdRoutine logging
        sysIdVoltage = voltage;
        lastVoltageVolts = voltage.in(Volts);

        motor.setVoltage(voltage);
    }

    public void sysIdLog(SysIdRoutineLog log) {
        // TODO update these units if the conversion factor units change
        log.motor("coral_wheel")
            .voltage(sysIdVoltage)
            .linearPosition(Meters.of(encoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(encoder.getVelocity()));
    }

    // drive
    /** updates the wheel velocity to the desired velocity setpoint, within the maximum speed and acceleration */
    public void updateSetpoint(LinearVelocity velocity) {
        // clamp to maximum speed
        if (velocity.in(MetersPerSecond) > CoralWheelConstants.MAX_SPEED.in(MetersPerSecond)) {
            velocity = CoralWheelConstants.MAX_SPEED;
        }

        // clamp to maximum acceleration
        velocity = MetersPerSecond.of(rateLimiter.calculate(velocity.in(MetersPerSecond)));

        final double output = pid.calculate(encoder.getVelocity(), velocity.in(MetersPerSecond));
        final double feed = ff.calculate(velocity.in(MetersPerSecond));

        // update logged values
        lastVelocityMetersPerSecond = velocity.in(MetersPerSecond);
        lastVoltageVolts = output + feed;

        // set voltage
        motor.setVoltage(output + feed);
    }

    /** directly applies the specified voltage to the motor */
    public void updateVoltage(Voltage voltage) {
        lastVoltageVolts = voltage.in(Volts);

        motor.setVoltage(voltage);
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        pid.setPID(CoralWheelConstants.KP, CoralWheelConstants.KI, CoralWheelConstants.KD);
        ff = new SimpleMotorFeedforward(CoralWheelConstants.KS, CoralWheelConstants.KV, CoralWheelConstants.KA);
    }
}
