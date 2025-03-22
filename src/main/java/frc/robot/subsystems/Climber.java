package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.constants.ClimberConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {
    // hardware
    public SparkMax motor;
    public RelativeEncoder encoder;

    public Climber() {
        // motor config
        // ! this resets the spark max configuration every time the robot code runs, so be careful
        SparkMaxConfig config = new SparkMaxConfig();

        config
            .smartCurrentLimit((int)ClimberConstants.SMART_CURRENT_LIMIT.in(Amps))
            .idleMode(IdleMode.kBrake);
        
        // initialize motors and configure them
        motor = new SparkMax(ClimberConstants.MOTOR_ID, MotorType.kBrushless);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // initialize encoders
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);
    }

    /**
     * ! be VERY careful when using this - the climber lifts the entire robot up, so mishandling this has a very real risk of damaging the entire robot to a severe degree
     */
    public void updateVoltage(double volts) {
        motor.setVoltage(volts);
    }
}
