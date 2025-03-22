package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
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
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

    // state
    /** whether the elevator has gone below the negative threshold, e.g., 0.1, and thus was initialized when not fully down,
     * risking breaking the elevator if the robot tries to automatically reach a setpoint with the misconfigured zero point */
    public boolean initializedAboveZero = false;

    // logging
    public Voltage sysIdVoltage;
    @Logged public double lastHeight;
    @Logged public double lastLeftVelocityMetersPerSecond, lastRightVelocityMetersPerSecond;
    @Logged public double lastLeftVoltageVolts, lastRightVoltageVolts;

    // shuffleboard
    public GenericEntry heightEntry;

    public final SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // (dont) Log state with SignalLogger class
            null
        ),
        new SysIdRoutine.Mechanism(
            output -> updateVoltage(output),
            log -> {
                log.motor("left-elevator-motor")
                    .voltage(Volts.of(lastLeftVoltageVolts))
                    .linearPosition(leftHeight())
                    .linearVelocity(leftVelocity());
                
                log.motor("right-elevator-motor")
                    .voltage(Volts.of(lastRightVoltageVolts))
                    .linearPosition(rightHeight())
                    .linearVelocity(rightVelocity());
            },
            this
        )
    );

    public Elevator() {
        initMotors();
        initControl();

        ShuffleboardTab tab = Shuffleboard.getTab("LiveWindow");

        SimpleWidget heightWidget = tab.add("Elevator Height", leftEncoder.getPosition())
            .withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("Min", 0.0, "Max", ElevatorConstants.MAX_HEIGHT.in(Meters) * 100));
        
        heightEntry = heightWidget.getEntry();
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
            .idleMode(IdleMode.kBrake);

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
        
        // initialize motors and configure them
        leftMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoders
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        // the robot should be at base position when initialized, so it should be zero
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
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

    @Logged public double leftHeightMeters() { return leftHeight().in(Meters); }
    @Logged public double rightHeightMeters() { return rightHeight().in(Meters); }

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

        // if (leftHeight().gt(ElevatorConstants.HEIGHT_SOFT_LIMIT) || rightHeight().gt(ElevatorConstants.HEIGHT_SOFT_LIMIT)) {
        //     leftMotor.setVoltage(0.0);
        //     rightMotor.setVoltage(0.0);
        // }

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
        if (leftHeight().gt(ElevatorConstants.HEIGHT_SOFT_LIMIT) || rightHeight().gt(ElevatorConstants.HEIGHT_SOFT_LIMIT)) {
            if (height.in(Meters) > lastHeight) {
                height = Meters.of(lastHeight);
            }
        }
        
        // ! applying voltage outside the acceptable range of motion risks damage to the robot
        // TODO implement a safety mechanism that disables movement outside of safe ranges - currently safe due to low max speed and acceleration
        // TODO update units if necessary
        double leftOutput = leftPID.calculate(leftEncoder.getPosition(), height.in(Meters));
        double rightOutput = rightPID.calculate(rightEncoder.getPosition(), height.in(Meters));

        // FF requires a velocity argument which is not directly provided by a height, so we use the motion profile in the ProfiledPIDControllers
        // to smoothly generate setpoints that the FF can use, hence, the .getSetpoint().velocity
        double leftFeed = ff.calculate(leftPID.getSetpoint().velocity);
        double rightFeed = ff.calculate(rightPID.getSetpoint().velocity);

        if (Math.abs(leftFeed - ElevatorConstants.KG) < 0.01 && leftOutput > 0.0) {
            leftOutput += ElevatorConstants.KS * Math.signum(leftOutput);
        }

        if (Math.abs(rightFeed - ElevatorConstants.KG) < 0.01 && rightOutput > 0.0) {
            rightOutput += ElevatorConstants.KS * Math.signum(rightOutput);
        }

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
     * 
     * directly applies the specified voltage to the motor */
    public void updateVoltage(Voltage voltage) {
        if (leftHeight().gt(ElevatorConstants.HEIGHT_SOFT_LIMIT) || rightHeight().gt(ElevatorConstants.HEIGHT_SOFT_LIMIT)) {
            leftMotor.setVoltage(0.0);
            rightMotor.setVoltage(0.0);
        }

        // ! BE EXTREMELY CAREFUL WITH THIS METHOD - IF YOU ACCELERATE THE ARM TOO QUICKLY INTO THE ROBOT, YOU RISK DAMAGING IMPORTANT COMPONENTS
        lastLeftVoltageVolts = voltage.in(Volts);
        lastRightVoltageVolts = voltage.in(Volts);

        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    public void graduallyReachHeight(Distance height) {
        // if we initialized above zero we're interpreting zero as above where it actually should be,
        // and so trying to reach the setpoint could shoot the elevator too high, breaking it
        if (initializedAboveZero) return;

        
        // sign of the elevator movement - positive if upwards, negative if downwards
        double movementSign = Math.signum(height.in(Meters) - leftEncoder.getPosition());

        // voltage required to keep the elevator at a certain height
        double gravityVoltage = ElevatorConstants.KG;

        // voltage required to make the elevator start moving
        double staticVoltage = 0.0;
        
        // we only add static voltage if we're moving upwards, as simply subtracting voltage from the gravity voltage is sufficient for moving downwards,
        // as gravity does much of the work for us
        if (movementSign > 0.0) {
            staticVoltage += ElevatorConstants.KS;
        }

        double normalizedHeight = leftEncoder.getPosition() / ElevatorConstants.MAX_HEIGHT.in(Meters);

        // move to the domain to an inversed [0, 10] to make the voltage roughly constant for longer
        // and apply a horizontal flip to apply roughly constant voltage first and then decay near the end
        double normalizedVoltage = Math.sqrt(-10.0 * (normalizedHeight - 1.0));

        // if normalizedVoltage is NaN, then its input argument was negative, meaning we exceeded the maximum height
        // that shouldn't happen, but if it did, we should immediately reset voltage to zero to lower the elevator
        if (Double.isNaN(normalizedVoltage)) {
            updateVoltage(Volts.of(0.0));
            return;
        }
        
        // TODO tune this value to find a speed that's not too fast and not too slow
        double movementVoltage = normalizedVoltage * 1.0;

        double totalVoltage = gravityVoltage + staticVoltage + movementVoltage;
        updateVoltage(Volts.of(totalVoltage));
    }

    // periodic
    public void periodic() {
        heightEntry.setDouble(leftEncoder.getPosition() * 100.0);

        // TODO tweak negative threshold value to ensure it's never reached in normal conditions, and only when the elevator is initialized when not fully down
        if (leftEncoder.getPosition() < -0.1) {
            initializedAboveZero = true;
        }
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        leftPID.setPID(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
        rightPID.setPID(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);

        ff = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);
    }
}
