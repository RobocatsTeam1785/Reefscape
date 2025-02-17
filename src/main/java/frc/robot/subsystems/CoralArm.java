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
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.CoralArmConstants;

@Logged
public class CoralArm extends SubsystemBase {
    // hardware
    protected SparkMax armMotor;
    protected RelativeEncoder armEncoder;

    // loop control
    protected ProfiledPIDController armPID;
    protected ArmFeedforward armFF;

    // logging
    protected Voltage sysIdVoltage;

    public CoralArm(int motorId) {
        initMotor(motorId);
        initControl();
    }

    // initialization
    protected void initMotor(int motorId) {
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
        armMotor = new SparkMax(motorId, MotorType.kBrushless);
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoder
        armEncoder = armMotor.getEncoder();
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
    public void sysIdDrive(Voltage voltage) {
        // set value for use in SysIdRoutine logging
        sysIdVoltage = voltage;

        // ! applying voltage outside the acceptable range of motion risks damage to the robot
        // TODO implement a safety mechanism that disables movement outside of safe ranges before uncommenting
        // armMotor.setVoltage(voltage);
    }

    public void sysIdLog(SysIdRoutineLog log) {
        // TODO update these units if the conversion factor units change
        log.motor("arm")
            .voltage(sysIdVoltage)
            .angularPosition(Radians.of(armEncoder.getPosition()))
            .angularVelocity(RadiansPerSecond.of(armEncoder.getVelocity()));
    }

    // drive
    public void updateSetpoint(Angle angle) {
        // ! applying voltage outside the acceptable range of motion risks damage to the robot
        // TODO implement a safety mechanism that disables movement outside of safe ranges before uncommenting
        // final double output = armPID.calculate(armEncoder.getPosition(), angle.in(Radians));
        // final double feed = armFF.calculate(armPID.getSetpoint().velocity);

        // armMotor.setVoltage(output + feed);
    }

    // tuning
    /** update constants; intended for use when hand-tuning constants using a debugger and hot code replace */
    public void tune() {
        armPID.setPID(CoralArmConstants.KP, CoralArmConstants.KI, CoralArmConstants.KD);
        armFF = new ArmFeedforward(CoralArmConstants.KS, CoralArmConstants.KG, CoralArmConstants.KV, CoralArmConstants.KA);
    }
}
