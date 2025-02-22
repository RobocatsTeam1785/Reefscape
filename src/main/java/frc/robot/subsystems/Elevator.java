package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

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

@Logged
public class Elevator extends SubsystemBase {
    // hardware
    protected SparkMax leftMotor, rightMotor;
    protected RelativeEncoder leftEncoder, rightEncoder;

    // control/filtering
    protected ProfiledPIDController leftPID, rightPID;
    protected ElevatorFeedforward leftFF, rightFF;

    protected SlewRateLimiter rateLimiter;

    // logging
    protected Voltage sysIdVoltage;

    public Elevator(int leftId, int rightId) {
        initMotors(leftId, rightId);
        initControl();
    }

    // initialization
    protected void initMotors(int leftId, int rightId) {
        // motor config
        // ! this resets the spark max configuration every time the robot code runs, so be careful
        SparkMaxConfig config = new SparkMaxConfig();

        double rotationsToMeters = ElevatorConstants.ELEVATOR_CF;
        double rpmToMetersPerSecond = rotationsToMeters / 60.0;

        // TODO evaluate whether these current limit and idle mode parameters are actually suitable
        // set motor parameters
        config
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        
        // set encoder parameters
        config.encoder
            .positionConversionFactor(rotationsToMeters)
            .velocityConversionFactor(rpmToMetersPerSecond);
        
        // initialize motors and configure them
        leftMotor = new SparkMax(leftId, MotorType.kBrushless);
        rightMotor = new SparkMax(rightId, MotorType.kBrushless);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoders
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();
    }

    protected void initControl() {
        // loop control
        // set the maximum speed and acceleration of the generated setpoints to the constant values
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.MAX_SPEED.in(MetersPerSecond),
            ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond
        ));

        // configure PID and FF using constants and constraints
        leftPID = new ProfiledPIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD, constraints);
        rightPID = new ProfiledPIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD, constraints);

        leftFF = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);
        rightFF = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);

        // filtering
        // set the maximum acceleration of provided setpoints
        rateLimiter = new SlewRateLimiter(ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond));
    }

    // system identification
    public void sysIdDrive(Voltage voltage) {
        // set value for use in SysIdRoutine logging
        sysIdVoltage = voltage;

        // ! applying voltage outside the acceptable range of motion risks damage to the robot
        // TODO implement a safety mechanism that disables movement outside of safe ranges - currently safe due to low max speed and acceleration
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    public void sysIdLog(SysIdRoutineLog log) {
        // TODO update these units if the conversion factor units change
        log.motor("left")
            .voltage(sysIdVoltage)
            .linearPosition(Meters.of(leftEncoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(leftEncoder.getVelocity()));
        
        log.motor("right")
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
        final double leftFeed = leftFF.calculate(speed.in(MetersPerSecond));
        final double rightFeed = rightFF.calculate(speed.in(MetersPerSecond));

        leftMotor.setVoltage(leftFeed);
        rightMotor.setVoltage(rightFeed);
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
        final double leftFeed = leftFF.calculate(leftPID.getSetpoint().velocity);
        final double rightFeed = rightFF.calculate(rightPID.getSetpoint().velocity);

        leftMotor.setVoltage(leftOutput + leftFeed);
        rightMotor.setVoltage(rightOutput + rightFeed);
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        leftPID.setPID(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
        rightPID.setPID(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);

        leftFF = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);
        rightFF = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV, ElevatorConstants.KA);
    }
}
