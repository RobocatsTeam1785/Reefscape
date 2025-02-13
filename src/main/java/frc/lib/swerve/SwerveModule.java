package frc.lib.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
    private SparkMax driveMotor, turnMotor;
    private RelativeEncoder driveEncoder, turnEncoder;

    // we use an additional encoder so that rotational information persists between reboots
    private CANcoder absEncoder;

    private PIDController drivePID;
    private ProfiledPIDController turnPID;

    private SimpleMotorFeedforward driveFF, turnFF;

    // values used to log voltage for system identification
    private Voltage sysIdDriveVoltage = Volts.of(0.0),
                    sysIdTurnVoltage = Volts.of(0.0);

    /** name of this swerve module - used for labelling during system identification */
    public final String name;

    public SwerveModule(String name, int driveId, int turnId, int absEncoderId) {
        this.name = name;

        initAbsEncoder(absEncoderId);
        initMotors(driveId, turnId);
        initControl();
    }

    // initialization
    public void initMotors(int driveId, int turnId) {
        // motor config
            // TODO see if inverting the drive and turn motors is necessary - andrew did it, but it appears that
            // TODO (cont.) he did it for every single motor, so it seems redundant

            // TODO figure out why we use a current limit of 45A
            // TODO test a coast idle mode to see how it feels
            SparkMaxConfig driveConfig = new SparkMaxConfig();
            driveConfig
                .smartCurrentLimit(45)
                .idleMode(IdleMode.kBrake);
            
            SparkMaxConfig turnConfig = new SparkMaxConfig();
            turnConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);
        
        // encoder config
            // the gear ratio is the reciprocal of the rotation ratio, so we divide by it to get the number of driven rotations
            // then, we multiply by the wheel's circumference to get the number of meters the wheel has travelled

            // in summary, if B/A is the gear ratio and C is the wheel circumference:
            //             N drive rotations * (A driven rotations/B drive rotations) * (C meters/1 driven rotation) = N / (B/A) * C meters
            double rotationsToMeters = 1 / Constants.Drive.DRIVE_GEAR_RATIO * Constants.Drive.WHEEL_CIRCUMFERENCE.in(Meters);

            // to convert rpm to m/s, we perform the same logic to convert rotations to meters, but we divide by 60 to convert minutes to seconds
            // i.e., N RPM * (X meters/rotation) * (1 minute/60 seconds) = N * X / 60 m/s
            double rpmToMetersPerSecond = rotationsToMeters / 60.0;

            driveConfig.encoder
                .positionConversionFactor(rotationsToMeters)
                .velocityConversionFactor(rpmToMetersPerSecond);

            // we apply the turn gear ratio in a similar manner, and then multiply by 2pi to convert to radians, as well as dividing by 60 to convert RPM to radians/s

            // i.e., if B/A is the turn gear ratio:
            //       N drive rotations * (A driven rotations/B drive rotations) * (2pi radians/1 driven rotation) = N / (B/A) * 2pi
            double rotationsToRadians = 1 / Constants.Drive.TURN_GEAR_RATIO * (2 * Math.PI);

            // and we use the same method to convert rpm to radians/s
            // i.e., N RPM * (X radians/rotation) * (1 minute/60 seconds) = N * X / 60 radians/s
            double rpmToRadiansPerSecond = rotationsToRadians / 60.0;

            turnConfig.encoder
                .positionConversionFactor(rotationsToRadians)
                .velocityConversionFactor(rpmToRadiansPerSecond);

        // motors
            // we use NEO brushless motors (REV-21-1650, or NEO Brushless Motor V1.1)
            driveMotor = new SparkMax(driveId, MotorType.kBrushless);
            turnMotor = new SparkMax(turnId, MotorType.kBrushless);

            // apply the configured parameters
            driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            turnMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // encoders
            driveEncoder = driveMotor.getEncoder();
            turnEncoder = turnMotor.getEncoder();

            Angle absoluteAngle = absoluteAngle();

            // recover the module angle from the absolute encoder, as the relative encoders do not preserve data between power cycles
            // we use radians because we defined the position in terms of radians using positionConversionFactor()
            turnEncoder.setPosition(absoluteAngle.in(Radians));
    }

    public void initAbsEncoder(int id) {
        CANcoderConfiguration config = new CANcoderConfiguration();

        // set the range for the absolute position to [0, 1]
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0;

        // make CCW rotation positive
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absEncoder = new CANcoder(id);
        absEncoder.getConfigurator().apply(config);
    }

    public void initControl() {
        // TODO tune these velocity and acceleration parameters - Andrew simply used very high values to essentially remove limits, but it might
        // TODO (cont.) be better to use specific constraint values, instead - would need testing to determine if that has any merit, though
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(2 * Math.PI * 5, 2 * Math.PI * 5);

        drivePID = new PIDController(
            Constants.Drive.TRANSLATIONAL_KP,
            Constants.Drive.TRANSLATIONAL_KI,
            Constants.Drive.TRANSLATIONAL_KD
        );

        // use a profiled controller to generate setpoints to smoothly interpolate to the actual setpoint
        turnPID = new ProfiledPIDController(
            Constants.Drive.ROTATIONAL_KP,
            Constants.Drive.ROTATIONAL_KI,
            Constants.Drive.ROTATIONAL_KD,
            constraints
        );

        driveFF = new SimpleMotorFeedforward(
            Constants.Drive.TRANSLATIONAL_KS,
            Constants.Drive.TRANSLATIONAL_KV,
            Constants.Drive.TRANSLATIONAL_KA
        );

        turnFF = new SimpleMotorFeedforward(
            Constants.Drive.ROTATIONAL_KS,
            Constants.Drive.ROTATIONAL_KV,
            Constants.Drive.ROTATIONAL_KA
        );

        // enable continuous input to make error calculation more accurate
        // e.g., error is calculated as the distance between the output and setpoint, but on a circle, two
        // kinds of distance exist between angles: the major arc, and the minor arc. for our purposes, the minor
        // arc is more useful, as it prevents sudden spikes in distance as either point crosses the 2pi/0 boundary
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // state
    /** returns the number of absolute rotations that have occurred from the CAN encoder; persists between power cycles */
    private Angle absoluteAngle() {
        StatusSignal<Angle> signal = absEncoder.getAbsolutePosition();
        return signal.getValue();
    }

    /** returns the turn position error (i.e., the difference between the angular setpoint and the current angular position) */
    public double turnErrorRadians() {
        return turnPID.getPositionError();
    }

    // system identification
    /** sets the drive motor voltage for system identification */
    public void sysIdDrive(Voltage voltage) {
        // update the last recorded sysid voltage for logging
        sysIdDriveVoltage = voltage;

        // set the motor voltage
        driveMotor.setVoltage(voltage);
    }

    /** sets the turn motor voltage for system identification */
    public void sysIdTurn(Voltage voltage) {
        // update the last recorded sysid voltage for logging
        sysIdTurnVoltage = voltage;

        // set the motor voltage
        turnMotor.setVoltage(voltage);
    }

    /** logs the current drive motor state for system identification */
    public void sysIdDriveLog(SysIdRoutineLog log) {
        log.motor(name)
            .voltage(sysIdDriveVoltage)
            .linearPosition(Meters.of(driveEncoder.getPosition()))
            .linearVelocity(MetersPerSecond.of(driveEncoder.getVelocity()));
    }

    /** logs the current turn motor state for system identification */
    public void sysIdTurnLog(SysIdRoutineLog log) {
        log.motor(name)
            .voltage(sysIdTurnVoltage)
            .angularPosition(Radians.of(turnEncoder.getPosition()))
            .angularVelocity(RadiansPerSecond.of(turnEncoder.getVelocity()));
    }

    public void updateSetpoint(SwerveModuleState state) {
        // for understanding this code, see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html

        // optimize and apply cosine compensation to the state
            // the value of turnEncoder.getPosition() is in radians, as we set the CF to use radians instead of rotations using positionConversionFactor()
            Rotation2d relativeAngle = Rotation2d.fromRadians(turnEncoder.getPosition());

            // minimize rotational movement required by taking the shorter path to the setpoint, if possible
            state.optimize(relativeAngle);

            // if perfectly aligned with the setpoint, use normal speed, but if perpendicular, apply zero speed, with the area between following the cosine curve
            // while state.angle - relativeAngle technically produces somewhat unintuitive values (negative for CCW, positive for CW), the graph of cosine
            // between -90 degrees and 90 degrees is symmetrical, making their order irrelevant
            state.speedMetersPerSecond *= state.angle.minus(relativeAngle).getCos();
        
        // calculate FF and PID values
        final double driveOutput = drivePID.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);
        final double turnOutput = turnPID.calculate(turnEncoder.getPosition(), state.angle.getRadians());

        final double driveFeed = driveFF.calculate(state.speedMetersPerSecond);

        // we use turnPID.getSetpoint().velocity instead of state.angle.getRadians() because turnPID, using a trapezoid motion profile,
        // uses the goal we provide it with (as the second parameter to calculate()) to generate smoother intermediary setpoints, which
        // we use in the feedforward here
        final double turnFeed = turnFF.calculate(turnPID.getSetpoint().velocity);
        
        // set voltages
        driveMotor.setVoltage(driveOutput + driveFeed);
        turnMotor.setVoltage(turnOutput + turnFeed);

        // update PID values
        if (Constants.TUNING) {
            drivePID.setPID(Constants.Drive.TRANSLATIONAL_KP, Constants.Drive.TRANSLATIONAL_KI, Constants.Drive.TRANSLATIONAL_KD);
            turnPID.setPID(Constants.Drive.ROTATIONAL_KP, Constants.Drive.ROTATIONAL_KI, Constants.Drive.ROTATIONAL_KD);

            driveFF = new SimpleMotorFeedforward(
                Constants.Drive.TRANSLATIONAL_KS,
                Constants.Drive.TRANSLATIONAL_KV,
                Constants.Drive.TRANSLATIONAL_KA
            );

            turnFF = new SimpleMotorFeedforward(
                Constants.Drive.ROTATIONAL_KS,
                Constants.Drive.ROTATIONAL_KV,
                Constants.Drive.ROTATIONAL_KA
            );
        }

        // set logging values
        if (Constants.DEBUGGING) {
            // meters
            // SmartDashboard.putNumber(name + " drive position", driveEncoder.getPosition());

            // // m/s
            // SmartDashboard.putNumber(name + " drive velocity", driveEncoder.getVelocity());

            // // m/s
            // SmartDashboard.putNumber(name + "drive setpoint", state.speedMetersPerSecond);

            // // volts
            // SmartDashboard.putNumber(name + " drive output", driveOutput);
            // SmartDashboard.putNumber(name + " drive feed", driveFeed);

            // radians
            SmartDashboard.putNumber(name + " turn position", turnEncoder.getPosition());

            // radians/s
            SmartDashboard.putNumber(name + " turn velocity", turnEncoder.getVelocity());

            // radians
            // SmartDashboard.putNumber(name + "turn setpoint", state.angle.getRadians());

            // volts
            SmartDashboard.putNumber(name + " turn output", turnOutput);
            SmartDashboard.putNumber(name + " turn feed", turnFeed);
        }
    }
}
