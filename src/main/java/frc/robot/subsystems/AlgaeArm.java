package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.AlgaeArmConstants;
import frc.lib.constants.CoralArmConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class AlgaeArm extends SubsystemBase {
    // hardware
    public SparkMax motor;
    @Logged public RelativeEncoder relativeEncoder;
    @Logged public DutyCycleEncoder hexEncoder;

    // loop control
    @Logged public ProfiledPIDController pid;
    public ArmFeedforward ff;

    // logging
    public Voltage sysIdVoltage;
    @Logged public double lastVelocityRadiansPerSecond;
    @Logged public double lastVoltageVolts;

    // shuffleboard
    public GenericEntry angleEntry;

    public AlgaeArm() {
        initMotor();
        initHexEncoder();
        initControl();

        ShuffleboardTab tab = Shuffleboard.getTab("LiveWindow");

        SimpleWidget heightWidget = tab.add("Algae Arm Angle", hexPosition().in(Degrees))
            .withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("Min", CoralArmConstants.MIN_ANGLE.in(Degrees), "Max", CoralArmConstants.MAX_ANGLE.in(Degrees)));
        
        angleEntry = heightWidget.getEntry();
    }

    // initialization
    public void initMotor() {
        // motor config
        // ! this resets the spark max configuration every time the robot code runs, so be careful
        SparkMaxConfig config = new SparkMaxConfig();

        // see logic in SwerveModule#initMotors for an explanation involving units
        double rotationsToRadians = AlgaeArmConstants.ARM_CF;
        double rpmToRadiansPerSecond = AlgaeArmConstants.ARM_CF / 60.0;

        // TODO evaluate whether these current limit and idle mode parameters are actually suitable
        // set motor parameters
        config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        // set encoder parameters
        config.encoder
            .positionConversionFactor(rotationsToRadians)
            .velocityConversionFactor(rpmToRadiansPerSecond);

        // initialize motor and configure it
        // TODO check whether we're still using brushless motors
        motor = new SparkMax(AlgaeArmConstants.MOTOR_ID, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoder
        relativeEncoder = motor.getEncoder();

        // the algae arm should be facing 20° down when the robot initializes, so the initial encoder value should reflect that
        relativeEncoder.setPosition(AlgaeArmConstants.MIN_ANGLE.in(Radians));
    }

    public void initHexEncoder() {
        // TODO adjust expected zero if necessary
        hexEncoder = new DutyCycleEncoder(AlgaeArmConstants.HEX_ENCODER_CHANNEL, 1.0, 0.0);
    }

    public void initControl() {
        // set the maximum speed and acceleration of the generated setpoints to the constant values
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            AlgaeArmConstants.MAX_SPEED.in(RadiansPerSecond),
            AlgaeArmConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
        );

        // configure PID and FF using constants and constraints
        pid = new ProfiledPIDController(AlgaeArmConstants.KP, AlgaeArmConstants.KI, AlgaeArmConstants.KD, constraints);
        ff = new ArmFeedforward(AlgaeArmConstants.KS, AlgaeArmConstants.KG, AlgaeArmConstants.KV, AlgaeArmConstants.KA);

        // pid.enableContinuousInput(-Math.PI / 2.0, Math.PI / 2.0);
    }

    // state
    public Angle hexPosition() {
        // by default, 0.805 rotations at horizontal
        double rotations = hexEncoder.get();

        // the physical mechanism range is about 0.75 to 1.1, but 1.1 is reset to 0.1 because the value is constrained between 0 and 1
        // so, we just change the area below 0.25, which it can essentially not reach, to be above 1, so we see continuous input
        if (rotations < 0.25) rotations += 1.0;

        // now that we have a continuous range, we can properly define zero at horizontal
        rotations -= 0.805;

        // finally, invert the sign to make upwards rotation positive instead of negative
        // rotations *= -1;

        // the hex encoder is positioned after the gearbox, so 1 hex encoder rotation = 1 mechanism rotation
        return Rotations.of(rotations);
    }

    @Logged public double hexPositionRadians() { return hexPosition().in(Radians); }
    @Logged public double hexPositionRotations() { return hexPosition().in(Rotations); }

    public Voltage lastVoltage() {
        return Volts.of(lastVoltageVolts);
    }

    public AngularVelocity angularVelocity() {
        return RadiansPerSecond.of(relativeEncoder.getVelocity());
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
        log.motor("algae_arm")
            .voltage(sysIdVoltage)
            // TODO tweak the hexEncoder value until it achieves parity with the armEncoder value, and then replace the armEncoder value with it
            .angularPosition(hexPosition())
            .angularVelocity(RadiansPerSecond.of(relativeEncoder.getVelocity()));
    }

    // drive
    /** updates the arm motor position setpoint to the provided angle from horizontal, where positive rotation is upwards rotation */
    public void updateSetpoint(Angle angle) {
        // ! applying voltage outside the acceptable range of motion risks damage to the robot - be very careful when using this method
        // TODO tweak the hexEncoder value until it achieves parity with the armEncoder value, and then replace the armEncoder value with it
        final double output = pid.calculate(hexPositionRadians(), angle.in(Radians));
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
        final double feed = ff.calculate(hexPositionRadians(), velocity.in(RadiansPerSecond));

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
        lastVoltageVolts = voltage.in(Volts);

        motor.setVoltage(voltage);
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        pid.setPID(AlgaeArmConstants.KP, AlgaeArmConstants.KI, AlgaeArmConstants.KD);
        ff = new ArmFeedforward(AlgaeArmConstants.KS, AlgaeArmConstants.KG, AlgaeArmConstants.KV, AlgaeArmConstants.KA);
    }

    // periodic
    public void periodic() {
        angleEntry.setDouble(hexPosition().in(Degrees));
    }
}
