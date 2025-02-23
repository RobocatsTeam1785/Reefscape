package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.CoralArmConstants;
import frc.lib.sensor.CumulativeDutyCycleEncoder;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class CoralArm extends SubsystemBase {
    // hardware
    protected SparkMax armMotor;
    @Logged protected RelativeEncoder armEncoder;
    @Logged protected CumulativeDutyCycleEncoder hexEncoder;

    // loop control
    @Logged protected ProfiledPIDController armPID;
    protected ArmFeedforward armFF;

    // logging
    protected Voltage sysIdVoltage;

    public CoralArm() {
        initMotor();
        initHexEncoder();
        initControl();
    }

    // initialization
    protected void initMotor() {
        // motor config
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
        armMotor = new SparkMax(CoralArmConstants.MOTOR_ID, MotorType.kBrushless);
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoder
        armEncoder = armMotor.getEncoder();

        // the coral arm should be facing straight down when the robot initializes, so the initial encoder value should be a quarter rotation down
        armEncoder.setPosition(CoralArmConstants.MIN_ANGLE.in(Radians));
    }

    protected void initHexEncoder() {
        // the [0, 1] restricted encoder
        // TODO adjust expected zero if necessary
        DutyCycleEncoder restrictedEncoder = new DutyCycleEncoder(CoralArmConstants.HEX_ENCODER_PORT, 1.0, 0.0);

        // the cumulative encoder - due to restrictions imposed by rollover detection, the mechanism cannot move faster than (or equal in speed to) 450°/s
        // without introducing inaccuracies in the revolution count; fortunately, however, it's unlikely that it will, considering its purpose

        // the 450°/s comes from (360°)/2 being the maximum change value in motor rotation degrees, divided by the gear ratio, 20, to get
        // mechanism rotation degrees, finally divided by the roboRIO period time, 20ms, to get the maximum speed

        // in summary, (360°)/2 = 180° / 20 = 9° / 20ms = 450°/s
        hexEncoder = new CumulativeDutyCycleEncoder(restrictedEncoder, 1.0, 0.499);

        // convert from motor rotations to mechanism radians
        hexEncoder.setPositionConversionFactor(CoralArmConstants.ARM_CF);
    }

    protected void initControl() {
        // set the maximum speed and acceleration of the generated setpoints to the constant values
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            CoralArmConstants.MAX_SPEED.in(RadiansPerSecond),
            CoralArmConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
        );

        // configure PID and FF using constants and constraints
        armPID = new ProfiledPIDController(CoralArmConstants.KP, CoralArmConstants.KI, CoralArmConstants.KD, constraints);
        armFF = new ArmFeedforward(CoralArmConstants.KS, CoralArmConstants.KG, CoralArmConstants.KV, CoralArmConstants.KA);
    }

    // system identification
    /** sets arm motor voltage for system identification; because applying voltage outside the acceptable range of motion risks damage to the robot,
     * be <b>extremely careful</b> when using this method, and ensure some mechanism exists to avoid damaging the robot */
    public void sysIdDrive(Voltage voltage) {
        // set value for use in SysIdRoutine logging
        sysIdVoltage = voltage;

        // ! applying voltage outside the acceptable range of motion risks damage to the robot - be very careful when using this method
        armMotor.setVoltage(voltage);
    }

    public void sysIdLog(SysIdRoutineLog log) {
        // TODO update these units if the conversion factor units change
        log.motor("arm")
            .voltage(sysIdVoltage)
            // TODO tweak the hexEncoder value until it achieves parity with the armEncoder value, and then replace the armEncoder value with it
            .angularPosition(Radians.of(armEncoder.getPosition()))
            .angularVelocity(RadiansPerSecond.of(armEncoder.getVelocity()));
    }

    // drive
    /** updates the arm motor position setpoint to the provided angle from horizontal, where positive rotation is upwards rotation */
    public void updateSetpoint(Angle angle) {
        // ! applying voltage outside the acceptable range of motion risks damage to the robot - be very careful when using this method
        // TODO tweak the hexEncoder value until it achieves parity with the armEncoder value, and then replace the armEncoder value with it
        final double output = armPID.calculate(armEncoder.getPosition(), angle.in(Radians));
        final double feed = armFF.calculate(angle.in(Radians), armPID.getSetpoint().velocity);

        armMotor.setVoltage(output + feed);
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        armPID.setPID(CoralArmConstants.KP, CoralArmConstants.KI, CoralArmConstants.KD);
        armFF = new ArmFeedforward(CoralArmConstants.KS, CoralArmConstants.KG, CoralArmConstants.KV, CoralArmConstants.KA);
    }
}
