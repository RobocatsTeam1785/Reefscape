package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.CoralArmConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class CoralArm extends SubsystemBase {
    // hardware
    protected SparkMax motor;
    @Logged protected RelativeEncoder relativeEncoder;
    @Logged protected DutyCycleEncoder hexEncoder;

    // loop control
    @Logged protected ProfiledPIDController pid;
    protected ArmFeedforward ff;

    // logging
    protected Voltage sysIdVoltage;
    @Logged protected double lastVelocityRadiansPerSecond;
    @Logged protected double lastVoltageVolts;

    public CoralArm() {
        initMotor();
        initHexEncoder();
        initControl();
    }

    // initialization
    protected void initMotor() {
        // motor config
        // ! this resets the spark max configuration every time the robot code runs, so be careful
        SparkMaxConfig config = new SparkMaxConfig();

        // see logic in SwerveModule#initMotors for an explanation involving units
        double rotationsToRadians = CoralArmConstants.ARM_CF;
        double rpmToRadiansPerSecond = CoralArmConstants.ARM_CF / 60.0;

        // TODO evaluate whether these current limit and idle mode parameters are actually suitable
        // set motor parameters
        config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        // set encoder parameters
        config.encoder
            .positionConversionFactor(rotationsToRadians)
            .velocityConversionFactor(rpmToRadiansPerSecond);

        // initialize motor and configure it
        // TODO check whether we're still using brushless motors
        motor = new SparkMax(CoralArmConstants.MOTOR_ID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoder
        relativeEncoder = motor.getEncoder();

        // the coral arm should be facing straight down when the robot initializes, so the initial encoder value should be a quarter rotation down
        relativeEncoder.setPosition(CoralArmConstants.MIN_ANGLE.in(Radians));
    }

    protected void initHexEncoder() {
        // TODO adjust expected zero if necessary
        hexEncoder = new DutyCycleEncoder(CoralArmConstants.HEX_ENCODER_CHANNEL, 1.0, 0.0);
    }

    protected void initControl() {
        // set the maximum speed and acceleration of the generated setpoints to the constant values
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            CoralArmConstants.MAX_SPEED.in(RadiansPerSecond),
            CoralArmConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
        );

        // configure PID and FF using constants and constraints
        pid = new ProfiledPIDController(CoralArmConstants.KP, CoralArmConstants.KI, CoralArmConstants.KD, constraints);
        ff = new ArmFeedforward(CoralArmConstants.KS, CoralArmConstants.KG, CoralArmConstants.KV, CoralArmConstants.KA);
    }

    // state
    public Angle hexPosition() {
        // the hex encoder is positioned after the gearbox, so 1 hex encoder rotation = 1 mechanism rotation
        return Rotations.of(hexEncoder.get());
    }

    // system identification
    /** sets arm motor voltage for system identification; because applying voltage outside the acceptable range of motion risks damage to the robot,
     * be <b>extremely careful</b> when using this method, and ensure some mechanism exists to avoid damaging the robot */
    public void sysIdDrive(Voltage voltage) {
        // set value for use in SysIdRoutine logging
        sysIdVoltage = voltage;
        lastVoltageVolts = voltage.in(Volts);

        // ! applying voltage outside the acceptable range of motion risks damage to the robot - be very careful when using this method
        motor.setVoltage(voltage);
    }

    public void sysIdLog(SysIdRoutineLog log) {
        // TODO update these units if the conversion factor units change
        log.motor("coral_arm")
            .voltage(sysIdVoltage)
            // TODO tweak the hexEncoder value until it achieves parity with the armEncoder value, and then replace the armEncoder value with it
            .angularPosition(Radians.of(relativeEncoder.getPosition()))
            .angularVelocity(RadiansPerSecond.of(relativeEncoder.getVelocity()));
    }

    // drive
    /** updates the arm motor position setpoint to the provided angle from horizontal, where positive rotation is upwards rotation */
    public void updateSetpoint(Angle angle) {
        // ! applying voltage outside the acceptable range of motion risks damage to the robot - be very careful when using this method
        // TODO tweak the hexEncoder value until it achieves parity with the armEncoder value, and then replace the armEncoder value with it
        final double output = pid.calculate(relativeEncoder.getPosition(), angle.in(Radians));
        final double feed = ff.calculate(angle.in(Radians), pid.getSetpoint().velocity);

        // update logged values
        lastVelocityRadiansPerSecond = pid.getSetpoint().velocity;
        lastVoltageVolts = output + feed;

        // set voltage
        motor.setVoltage(output + feed);
    }

    /** updates the arm motor velocity setpoint to the provided angular velocity, where positive rotational velocity is upwards rotation */
    public void updateSetpoint(AngularVelocity velocity) {
        // ! applying voltage outside the acceptable range of motion risks damage to the robot - be very careful when using this method
        // TODO tweak the hexEncoder value until it achieves parity with the armEncoder value, and then replace the armEncoder value with it
        final double feed = ff.calculate(relativeEncoder.getPosition(), velocity.in(RadiansPerSecond));

        // update logged values
        lastVelocityRadiansPerSecond = velocity.in(RadiansPerSecond);
        lastVoltageVolts = feed;

        // set voltage
        motor.setVoltage(feed);
    }

    /** <b>BE EXTREMELY CAREFUL WITH THIS METHOD - IF YOU ACCELERATE THE ARM TOO QUICKLY INTO THE ROBOT, YOU RISK DAMAGING IMPORTANT COMPONENTS</b>
     * <p>
     * directly applies the specified voltage to the motor */
    public void updateVoltage(Voltage voltage) {
        // ! BE EXTREMELY CAREFUL WITH THIS METHOD - IF YOU ACCELERATE THE ARM TOO QUICKLY INTO THE ROBOT, YOU RISK DAMAGING IMPORTANT COMPONENTS
        motor.setVoltage(voltage);
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        pid.setPID(CoralArmConstants.KP, CoralArmConstants.KI, CoralArmConstants.KD);
        ff = new ArmFeedforward(CoralArmConstants.KS, CoralArmConstants.KG, CoralArmConstants.KV, CoralArmConstants.KA);
    }
}
