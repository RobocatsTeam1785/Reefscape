package frc.lib.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.lib.constants.SwerveConstants;
import frc.lib.constants.RobotConstants;

@Logged
public class TalonSwerveModule {
    // hardware
    private TalonFX driveMotor, turnMotor;

    // we use an additional encoder so that rotational information persists between reboots
    private CANcoder absEncoder;

    // loop control
    private PIDController drivePID;
    private ProfiledPIDController turnPID;

    @NotLogged private SimpleMotorFeedforward driveFF, turnFF;

    // logging
    @NotLogged private Voltage sysIdDriveVoltage = Volts.of(0.0),
                               sysIdTurnVoltage = Volts.of(0.0);
    
    // used for logging via epilogue, so being unused is irrelevant
    @SuppressWarnings("unused")
    private double lastDriveOutput, lastDriveFeed, lastDriveVoltage, lastTurnOutput, lastTurnFeed, lastTurnVoltage;

    @SuppressWarnings("unused")
    private SwerveModuleState lastDriveState, lastTurnState, lastState;

    @SuppressWarnings("unused")
    private double lastDriveSetpointVelocity, lastAbsDriveSetpointVelocity;

    // properties
    /** name of this swerve module - used for labelling during system identification */
    public final String name;

    public TalonSwerveModule(String name, int driveId, int turnId, int absEncoderId) {
        this.name = name;

        initAbsEncoder(absEncoderId);
        initMotors(driveId, turnId);
        initControl();
    }

    // initialization
    public void initMotors(int driveId, int turnId) {
        // initialize motors
        driveMotor = new TalonFX(driveId);
        turnMotor = new TalonFX(turnId);

        // configure motors
            // initialize configuration to value stored in motor
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            driveMotor.getConfigurator().refresh(driveConfig);

            TalonFXConfiguration turnConfig = new TalonFXConfiguration();
            turnMotor.getConfigurator().refresh(turnConfig);

            // customize configuration
            driveConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.CURRENT_LIMIT.in(Amps);
            driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            // TODO see if this needs to be inverted
            driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            turnConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.CURRENT_LIMIT.in(Amps);
            turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            // TODO see if this needs to be inverted
            driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            // apply the configuration
            driveMotor.getConfigurator().apply(driveConfig);
            turnMotor.getConfigurator().apply(turnConfig);

        // encoders
        // recover the module angle from the absolute encoder, as the relative encoders do not preserve data between power cycles
        // TODO check if this works as it's supposed to, i.e., the CANcoder and talonFX encoder * the CF change at the same rate
        turnMotor.setPosition(cancoderAbsolutePosition().in(Radians) / SwerveConstants.TURN_CF);
    }

    public void initAbsEncoder(int id) {
        absEncoder = new CANcoder(id);

        // get the stored config
        CANcoderConfiguration config = new CANcoderConfiguration();
        absEncoder.getConfigurator().refresh(config);

        // set the range for the absolute position to [0, 1]
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;

        // make CCW rotation positive
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // apply the original config with the modifications
        absEncoder.getConfigurator().apply(config);
    }

    public void initControl() {
        // TODO tune these velocity and acceleration parameters - Andrew simply used very high values to essentially remove limits, but it might
        // TODO (cont.) be better to use specific constraint values, instead - would need testing to determine if that has any merit, though
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2 * Math.PI * 5, 2 * Math.PI * 5);

        drivePID = new PIDController(
            SwerveConstants.TRANSLATIONAL_KP,
            SwerveConstants.TRANSLATIONAL_KI,
            SwerveConstants.TRANSLATIONAL_KD
        );

        // use a profiled controller to generate setpoints to smoothly interpolate to the actual setpoint
        turnPID = new ProfiledPIDController(
            SwerveConstants.ROTATIONAL_KP,
            SwerveConstants.ROTATIONAL_KI,
            SwerveConstants.ROTATIONAL_KD,
            constraints
        );

        driveFF = new SimpleMotorFeedforward(
            SwerveConstants.TRANSLATIONAL_KS,
            SwerveConstants.TRANSLATIONAL_KV,
            SwerveConstants.TRANSLATIONAL_KA
        );

        turnFF = new SimpleMotorFeedforward(
            SwerveConstants.ROTATIONAL_KS,
            SwerveConstants.ROTATIONAL_KV,
            SwerveConstants.ROTATIONAL_KA
        );

        // enable continuous input to make error calculation more accurate
        // e.g., error is calculated as the distance between the output and setpoint, but on a circle, two
        // kinds of distance exist between angles: the major arc, and the minor arc. for our purposes, the minor
        // arc is more useful, as it prevents sudden spikes in distance as either point crosses the 2pi/0 boundary
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // logging
    public SwerveModuleState getState() {
        LinearVelocity driveVelocity = driveVelocity();
        Angle turnPosition = turnPosition();

        double speedMetersPerSecond = driveVelocity.in(MetersPerSecond);
        Rotation2d rotation = new Rotation2d(turnPosition);

        return new SwerveModuleState(speedMetersPerSecond, rotation);
    }

    public SwerveModulePosition getPosition() {
        Distance drivePosition = drivePosition();
        Angle turnPosition = turnPosition();

        double distanceMeters = drivePosition.in(Meters);
        Rotation2d rotation = new Rotation2d(turnPosition);

        return new SwerveModulePosition(distanceMeters, rotation);
    }

    // state
    // - talon encoder values
    public Distance drivePosition() {
        double rotations = driveMotor.getPosition().getValue().in(Rotations);
        double meters = rotations * SwerveConstants.DRIVE_CF;

        return Meters.of(meters);
    }

    public LinearVelocity driveVelocity() {
        double rotationsPerSecond = driveMotor.getVelocity().getValue().in(RotationsPerSecond);
        double metersPerSecond = rotationsPerSecond * SwerveConstants.DRIVE_CF;

        return MetersPerSecond.of(metersPerSecond);
    }

    public Angle turnPosition() {
        double rotations = turnMotor.getPosition().getValue().in(Rotations);
        double radians = rotations * SwerveConstants.TURN_CF;

        return Radians.of(radians);
    }

    public AngularVelocity turnVelocity() {
        double rotationsPerSecond = turnMotor.getVelocity().getValue().in(RotationsPerSecond);
        double radiansPerSecond = rotationsPerSecond * SwerveConstants.TURN_CF;

        return RadiansPerSecond.of(radiansPerSecond);
    }

    // - CANcoder values
    public Angle cancoderPosition() {
        StatusSignal<Angle> signal = absEncoder.getPosition();
        return signal.getValue();
    }

    public AngularVelocity cancoderVelocity() {
        StatusSignal<AngularVelocity> signal = absEncoder.getVelocity();
        return signal.getValue();
    }

    public Angle cancoderAbsolutePosition() {
        StatusSignal<Angle> signal = absEncoder.getAbsolutePosition();
        return signal.getValue();
    }

    // - PID values
    /** returns the turn position error (i.e., the difference between the angular setpoint and the current angular position) */
    public double turnErrorRadians() {
        return turnPID.getPositionError();
    }

    // state modification
    public void drivePosition(Distance position) {
        double meters = position.in(Meters);
        double rotations = meters / SwerveConstants.DRIVE_CF;

        driveMotor.setPosition(Rotations.of(rotations));
    }

    public void turnPosition(Angle position) {
        double radians = position.in(Radians);
        double rotations = radians / SwerveConstants.TURN_CF;

        turnMotor.setPosition(Rotations.of(rotations));
    }

    /** zeroes the relative drive encoder */
    public void zeroDrivePosition() {
        drivePosition(Meters.of(0.0));
    }

    /** zeroes the relative turn encoder */
    public void zeroTurnPosition() {
        turnPosition(Radians.of(0.0));
    }

    /** recovers the angle from the absolute encoder */
    public void recoverCancoderPosition() {
        // TODO check if this works as it's supposed to, i.e., cancoderAbsolutePosition() and turnPosition() change at the same rate
        turnPosition(cancoderAbsolutePosition());
    }

    private void invertMotor(TalonFX controller) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        controller.getConfigurator().refresh(config);

        if (config.MotorOutput.Inverted == InvertedValue.Clockwise_Positive) {
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }

        controller.getConfigurator().apply(config);
    }

    public void invertDriveMotor() {
        invertMotor(driveMotor);
    }

    public void invertTurnMotor() {
        invertMotor(turnMotor);
    }

    // system identification
    /** sets the drive motor voltage for system identification */
    public void sysIdDrive(Voltage voltage) {
        double volts = voltage.in(Volts);

        // update the last recorded sysid voltage for logging
        lastDriveVoltage = volts;
        sysIdDriveVoltage = voltage;

        // set the motor voltage
        driveMotor.setVoltage(volts);
    }

    /** sets the turn motor voltage for system identification */
    public void sysIdTurn(Voltage voltage) {
        double volts = voltage.in(Volts);

        // update the last recorded sysid voltage for logging
        lastTurnVoltage = volts;
        sysIdTurnVoltage = voltage;

        // set the motor voltage
        turnMotor.setVoltage(volts);
    }

    /** logs the current drive motor state for system identification */
    public void sysIdDriveLog(SysIdRoutineLog log) {
        log.motor(name)
            .voltage(sysIdDriveVoltage)
            .linearPosition(drivePosition())
            .linearVelocity(driveVelocity());
    }

    /** logs the current turn motor state for system identification */
    public void sysIdTurnLog(SysIdRoutineLog log) {
        log.motor(name)
            .voltage(sysIdTurnVoltage)
            .angularPosition(turnPosition())
            .angularVelocity(turnVelocity());
    }

    // drive
    // zeroing
    public void zeroVoltage() {
        driveMotor.setVoltage(0);
        turnMotor.setVoltage(0);
    }

    public void zeroDriveVoltage() {
        driveMotor.setVoltage(0);
    }

    public void zeroTurnVoltage() {
        turnMotor.setVoltage(0);
    }

    // utility
    // for understanding this code, see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
    public SwerveModuleState optimizeState(SwerveModuleState state) {
        Rotation2d relativeAngle = new Rotation2d(turnPosition());

        // minimize rotational movement required by taking the shorter path to the setpoint, if possible
        state.optimize(relativeAngle);

        return state;
    }

    // setpoints
    public void updateDriveSetpoint(SwerveModuleState state) {
        // current values
        Rotation2d relativeAngle = new Rotation2d(turnPosition());
        double currentSpeedMetersPerSecond = driveVelocity().in(MetersPerSecond);
        
        // apply cosine compensation - if perfectly aligned with the setpoint, use normal speed, but if perpendicular, apply zero speed, with the area
        // between following the cosine curve while state.angle - relativeAngle technically produces somewhat unintuitive values (negative for CCW,
        // positive for CW), the graph of cosine between -90 degrees and 90 degrees is symmetrical, making their order irrelevant
        state.speedMetersPerSecond *= state.angle.minus(relativeAngle).getCos();

        // calculate voltage
        final double driveOutput = drivePID.calculate(currentSpeedMetersPerSecond, state.speedMetersPerSecond);
        final double driveFeed = driveFF.calculate(state.speedMetersPerSecond);

        // apply voltage
        driveMotor.setVoltage(driveOutput + driveFeed);

        // update logging values
        lastDriveOutput = driveOutput;
        lastDriveFeed = driveFeed;
        lastDriveVoltage = driveOutput + driveFeed;

        lastDriveState = state;

        lastDriveSetpointVelocity = state.speedMetersPerSecond;
        lastAbsDriveSetpointVelocity = Math.abs(lastDriveSetpointVelocity);
    }

    public void updateTurnSetpoint(SwerveModuleState state) {
        // calculate voltage
        // - we're using position PID control, not velocity PID control, as we care more about turn position
        final double turnOutput = turnPID.calculate(turnPosition().in(Radians), state.angle.getRadians());

        // - we use turnPID.getSetpoint().velocity instead of state.angle.getRadians(), because simple feedforward
        // lacks a position term, and thus requires velocity instead, and because we're using a motion profile
        // to generate that velocity
        final double turnFeed = turnFF.calculate(turnPID.getSetpoint().velocity);

        // set voltage
        turnMotor.setVoltage(turnOutput + turnFeed);

        // update logging values
        lastTurnOutput = turnOutput;
        lastTurnFeed = turnFeed;
        lastTurnVoltage = turnOutput + turnFeed;

        lastTurnState = state;
    }

    public void updateSetpoint(SwerveModuleState state) {
        // update PID values
        if (RobotConstants.TUNING)
            tune();

        if (SwerveConstants.OPTIMIZE_STATES)
            state = optimizeState(state);
        
        updateDriveSetpoint(state);
        updateTurnSetpoint(state);

        lastState = state;
    }

    // tuning
    /** updates PID and feedforward constants - used for hot reloading constant changes when tuning */
    public void tune() {
        drivePID.setPID(SwerveConstants.TRANSLATIONAL_KP, SwerveConstants.TRANSLATIONAL_KI, SwerveConstants.TRANSLATIONAL_KD);
        turnPID.setPID(SwerveConstants.ROTATIONAL_KP, SwerveConstants.ROTATIONAL_KI, SwerveConstants.ROTATIONAL_KD);

        driveFF = new SimpleMotorFeedforward(
            SwerveConstants.TRANSLATIONAL_KS,
            SwerveConstants.TRANSLATIONAL_KV,
            SwerveConstants.TRANSLATIONAL_KA
        );

        turnFF = new SimpleMotorFeedforward(
            SwerveConstants.ROTATIONAL_KS,
            SwerveConstants.ROTATIONAL_KV,
            SwerveConstants.ROTATIONAL_KA
        );
    }
}
