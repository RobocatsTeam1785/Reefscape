package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

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
import frc.lib.constants.AlgaeWheelConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class AlgaeWheel extends SubsystemBase {
    // hardware
    protected SparkMax leftMotor, rightMotor;
    @Logged protected RelativeEncoder leftEncoder, rightEncoder;

    // control/filtering
    @Logged protected PIDController leftPID, rightPID;

    // only one feedforward instance is necessary as feedforward is state-independent
    protected SimpleMotorFeedforward ff;

    protected SlewRateLimiter rateLimiter;

    // logging
    protected Voltage sysIdVoltage;

    public AlgaeWheel() {
        initMotors();
        initControl();
    }

    // initialization
    protected void initMotors() {
        // motor config
        // TODO include warning for every subsystem that uses sparks that doing this resets the existing config
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();

        // see logic in SwerveModule#initMotors for an explanation involving units
        double rotationsToMeters = AlgaeWheelConstants.WHEEL_CF;
        double rpmToMetersPerSecond = AlgaeWheelConstants.WHEEL_CF / 60.0;

        // TODO evaluate whether these current limit and idle mode parameters are actually suitable
        // set motor parameters
        leftConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        
        // TODO determine whether the left or the right motor needs to be inverted for positive rotation to be intuitive
        rightConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        // set encoder parameters
        leftConfig.encoder
            .positionConversionFactor(rotationsToMeters)
            .velocityConversionFactor(rpmToMetersPerSecond);

        rightConfig.encoder
            .positionConversionFactor(rotationsToMeters)
            .velocityConversionFactor(rpmToMetersPerSecond);

        // initialize motor and configure it
        // TODO check whether we're still using brushless motors
        leftMotor = new SparkMax(AlgaeWheelConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(AlgaeWheelConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoders - an initial value of zero is fine, as we only care about velocity
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
    }

    protected void initControl() {
        // configure PID and FF using constants
        leftPID = new PIDController(AlgaeWheelConstants.KP, AlgaeWheelConstants.KI, AlgaeWheelConstants.KD);
        rightPID = new PIDController(AlgaeWheelConstants.KP, AlgaeWheelConstants.KI, AlgaeWheelConstants.KD);

        ff = new SimpleMotorFeedforward(AlgaeWheelConstants.KS, AlgaeWheelConstants.KV, AlgaeWheelConstants.KA);

        // filtering
        // set the maximum acceleration of provided setpoints
        rateLimiter = new SlewRateLimiter(AlgaeWheelConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond));
    }

    // system identification
     public void sysIdDrive(Voltage voltage) {
        // set value for use in SysIdRoutine logging
        sysIdVoltage = voltage;

        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    public void sysIdLog(SysIdRoutineLog log) {
        // TODO update these units if the conversion factor units change
        log.motor("algae_wheel_left")
            .voltage(sysIdVoltage)
            .linearPosition(Meters.of(leftEncoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(leftEncoder.getVelocity()));
        
        log.motor("algae_wheel_right")
            .voltage(sysIdVoltage)
            .linearPosition(Meters.of(rightEncoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(rightEncoder.getVelocity()));
    }

    // drive
    // TODO include warning for elevator, algae wheel, and coral wheel that the updateSetpoint method has its own speed and acceleration clamping
    /** updates the wheel velocity to the desired velocity setpoint */
    public void updateSetpoint(LinearVelocity velocity) {
        // clamp to maximum speed
        if (velocity.in(MetersPerSecond) > AlgaeWheelConstants.MAX_SPEED.in(MetersPerSecond)) {
            velocity = AlgaeWheelConstants.MAX_SPEED;
        }

        // clamp to maximum acceleration
        velocity = MetersPerSecond.of(rateLimiter.calculate(velocity.in(MetersPerSecond)));

        final double leftOutput = leftPID.calculate(leftEncoder.getVelocity(), velocity.in(MetersPerSecond));
        final double rightOutput = rightPID.calculate(rightEncoder.getVelocity(), velocity.in(MetersPerSecond));

        final double feed = ff.calculate(velocity.in(MetersPerSecond));

        leftMotor.setVoltage(leftOutput + feed);
        rightMotor.setVoltage(rightOutput + feed);
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        leftPID.setPID(AlgaeWheelConstants.KP, AlgaeWheelConstants.KI, AlgaeWheelConstants.KD);
        rightPID.setPID(AlgaeWheelConstants.KP, AlgaeWheelConstants.KI, AlgaeWheelConstants.KD);

        ff = new SimpleMotorFeedforward(AlgaeWheelConstants.KS, AlgaeWheelConstants.KV, AlgaeWheelConstants.KA);
    }
}
