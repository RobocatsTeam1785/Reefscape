package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.ElevatorConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Elevator extends SubsystemBase {
    // hardware
    public SparkMax leftMotor, rightMotor;
    @Logged public RelativeEncoder leftEncoder, rightEncoder;

    // control/filtering
    @Logged public ProfiledPIDController leftPID, rightPID;

    public ElevatorFeedforward ff;

    public SlewRateLimiter rateLimiter;

    // logging
    public Voltage sysIdVoltage;
    @Logged public double lastHeight;
    @Logged public double lastLeftVelocityMetersPerSecond, lastRightVelocityMetersPerSecond;
    @Logged public double lastLeftVoltageVolts, lastRightVoltageVolts;

    public Elevator() {
        initMotors();
        initControl();
    }

    // initialization
    public void initMotors() {
        // motor config
        // ! this resets the spark max configuration every time the robot code runs, so be careful
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        SparkMaxConfig rightConfig = new SparkMaxConfig();

        double rotationsToMeters = ElevatorConstants.ELEVATOR_CF;
        double rpmToMetersPerSecond = rotationsToMeters / 60.0;

        // TODO evaluate whether these current limit and idle mode parameters are actually suitable
        // set motor parameters
        leftConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        rightConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        
        // set encoder parameters
        leftConfig.encoder
            .positionConversionFactor(rotationsToMeters)
            .velocityConversionFactor(rpmToMetersPerSecond);

        rightConfig.encoder
            .positionConversionFactor(rotationsToMeters)
            .velocityConversionFactor(rpmToMetersPerSecond);
        
        // initialize motors and configure them
        leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoders
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
    }

    public void initControl() {
        // loop control
        // set the maximum speed and acceleration of the generated setpoints to the constant values
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.MAX_SPEED.in(MetersPerSecond),
            ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond
        ));

        // configure PID and FF using constants and constraints
        leftPID = new ProfiledPIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD, constraints);
        rightPID = new ProfiledPIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD, constraints);

        ff = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);

        // filtering
        // set the maximum acceleration of provided setpoints
        rateLimiter = new SlewRateLimiter(ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond));
    }

    // state
    // - position
    public Distance leftHeight() {
        return Meters.of(leftEncoder.getPosition());
    }

    public Distance rightHeight() {
        return Meters.of(rightEncoder.getPosition());
    }

    // - velocity
    public LinearVelocity leftVelocity() {
        return MetersPerSecond.of(leftEncoder.getVelocity());
    }

    public LinearVelocity rightVelocity() {
        return MetersPerSecond.of(rightEncoder.getVelocity());
    }

    // - voltage
    public Voltage lastLeftVoltage() {
        return Volts.of(lastLeftVoltageVolts);
    }

    public Voltage lastRightVoltage() {
        return Volts.of(lastRightVoltageVolts);
    }

    // system identification
    public void sysIdDrive(Voltage voltage) {
        // set value for use in SysIdRoutine logging
        sysIdVoltage = voltage;
        lastLeftVoltageVolts = voltage.in(Volts);
        lastRightVoltageVolts = voltage.in(Volts);

        // ! applying voltage outside the acceptable range of motion risks damage to the robot
        // TODO implement a safety mechanism that disables movement outside of safe ranges - currently safe due to low max speed and acceleration
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    public void sysIdLog(SysIdRoutineLog log) {
        // TODO update these units if the conversion factor units change
        log.motor("elevator_left")
            .voltage(sysIdVoltage)
            .linearPosition(Meters.of(leftEncoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(leftEncoder.getVelocity()));
        
        log.motor("elevator_right")
            .voltage(sysIdVoltage)
            .linearPosition(Meters.of(rightEncoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(rightEncoder.getVelocity()));
    }

    // drive
    /** applies feedforward-only control to apply the provided speed to the elevator motors, within the maximum speed and acceleration */
    public void updateSetpoint(LinearVelocity speed) {
        // ! applying voltage outside the acceptable range of motion risks damage to the robot
        // TODO implement a safety mechanism that disables movement outside of safe ranges - currently safe due to low max speed and acceleration
        // TODO update units if necessary
        // clamp to maximum speed
        if (speed.in(MetersPerSecond) > ElevatorConstants.MAX_SPEED.in(MetersPerSecond)) {
            speed = ElevatorConstants.MAX_SPEED;
        }

        // clamp to maximum acceleration
        speed = MetersPerSecond.of(rateLimiter.calculate(speed.in(MetersPerSecond)));

        // use the provided setpoint
        final double feed = ff.calculate(speed.in(MetersPerSecond));

        // update logged values
        lastLeftVelocityMetersPerSecond = speed.in(MetersPerSecond);
        lastRightVelocityMetersPerSecond = speed.in(MetersPerSecond);

        lastLeftVoltageVolts = feed;
        lastRightVoltageVolts = feed;

        // set voltage
        leftMotor.setVoltage(feed);
        rightMotor.setVoltage(feed);
    }

    /** applies feedforward and PID control to reach the desired height, within the maximum speed and acceleration */
    public void updateSetpoint(Distance height) {
        // ! applying voltage outside the acceptable range of motion risks damage to the robot
        // TODO implement a safety mechanism that disables movement outside of safe ranges - currently safe due to low max speed and acceleration
        // TODO update units if necessary
        final double leftOutput = leftPID.calculate(leftEncoder.getPosition(), height.in(Meters));
        final double rightOutput = rightPID.calculate(rightEncoder.getPosition(), height.in(Meters));

        // FF requires a velocity argument which is not directly provided by a height, so we use the motion profile in the ProfiledPIDControllers
        // to smoothly generate setpoints that the FF can use, hence, the .getSetpoint().velocity
        final double leftFeed = ff.calculate(leftPID.getSetpoint().velocity);
        final double rightFeed = ff.calculate(rightPID.getSetpoint().velocity);

        // update logged values
        lastHeight = height.in(Meters);

        lastLeftVelocityMetersPerSecond = leftPID.getSetpoint().velocity;
        lastRightVelocityMetersPerSecond = rightPID.getSetpoint().velocity;

        lastLeftVoltageVolts = leftOutput + leftFeed;
        lastRightVoltageVolts = rightOutput + rightFeed;

        // set voltage
        leftMotor.setVoltage(leftOutput + leftFeed);
        rightMotor.setVoltage(rightOutput + rightFeed);
    }

    /** <b>BE EXTREMELY CAREFUL WITH THIS METHOD - IF YOU ACCELERATE THE ARM TOO QUICKLY INTO THE ROBOT, YOU RISK DAMAGING IMPORTANT COMPONENTS</b>
     * <p>
     * directly applies the specified voltage to the motor */
    public void updateVoltage(Voltage voltage) {
        // ! BE EXTREMELY CAREFUL WITH THIS METHOD - IF YOU ACCELERATE THE ARM TOO QUICKLY INTO THE ROBOT, YOU RISK DAMAGING IMPORTANT COMPONENTS
        lastLeftVoltageVolts = voltage.in(Volts);
        lastRightVoltageVolts = voltage.in(Volts);

        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        leftPID.setPID(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
        rightPID.setPID(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);

        ff = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);
    }
}
