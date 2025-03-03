package frc.robot.input.shuffleboard;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.input.shuffleboard.ResponsiveToggleableSlider;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeWheel;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralWheel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class ShuffleboardInputProcessor {
    // properties
    private final ShuffleboardTab tab;

    // initialization
    public ShuffleboardInputProcessor(
        String tabTitle,

        final Swerve swerve,
        final Elevator elevator,

        final CoralArm coralArm,
        final CoralWheel coralWheel,

        final AlgaeArm algaeArm,
        final AlgaeWheel algaeWheel
    ) {
        this.tab = Shuffleboard.getTab(tabTitle);

        // SbBox box = new SbBox(tab, 0, 0, 80, 40);

        ShuffleboardLayout layout = tab.getLayout("Control", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 9, "Number of rows", 4))
            .withPosition(0, 0)
            .withSize(80, 40);

        // swerve FL module
        ResponsiveToggleableSlider.voltSlider(
            layout, "Swerve FL Drive Voltage", 0, 0, 
            swerve.flModule::sysIdDrive, swerve.flModule::lastDriveVoltage
        );

        ResponsiveToggleableSlider.meterPerSecondSlider(
            layout, "Swerve FL Drive Velocity", 0, 1, 10.0,
            swerve.flModule::updateDriveSetpoint, swerve.flModule::driveVelocity
        );
        
        ResponsiveToggleableSlider.voltSlider(
            layout, "Swerve FL Turn Voltage", 0, 2,
            swerve.flModule::sysIdTurn, swerve.flModule::lastTurnVoltage
        );

        ResponsiveToggleableSlider.radianSlider(
            layout, "Swerve FL Turn Position", 0, 3,
            swerve.flModule::updateTurnSetpoint, swerve.flModule::turnPosition
        );

        // swerve FR module
        ResponsiveToggleableSlider.voltSlider(
            layout, "Swerve FR Drive Voltage", 1, 0,
            swerve.frModule::sysIdDrive, swerve.frModule::lastDriveVoltage
        );

        ResponsiveToggleableSlider.meterPerSecondSlider(
            layout, "Swerve FR Drive Velocity", 1, 1, 10.0,
            swerve.frModule::updateDriveSetpoint, swerve.frModule::driveVelocity
        );
        
        ResponsiveToggleableSlider.voltSlider(
            layout, "Swerve FR Turn Voltage", 1, 2,
            swerve.frModule::sysIdTurn, swerve.frModule::lastTurnVoltage
        );

        ResponsiveToggleableSlider.radianSlider(
            layout, "Swerve FR Turn Position", 1, 3,
            swerve.frModule::updateTurnSetpoint, swerve.frModule::turnPosition
        );

        // swerve BL module
        ResponsiveToggleableSlider.voltSlider(
            layout, "Swerve BL Drive Voltage", 2, 0, 
            swerve.blModule::sysIdDrive, swerve.blModule::lastDriveVoltage
        );

        ResponsiveToggleableSlider.meterPerSecondSlider(
            layout, "Swerve BL Drive Velocity", 2, 1, 10.0,
            swerve.blModule::updateDriveSetpoint, swerve.blModule::driveVelocity
        );
        
        ResponsiveToggleableSlider.voltSlider(
            layout, "Swerve BL Turn Voltage", 2, 2,
            swerve.blModule::sysIdTurn, swerve.blModule::lastTurnVoltage
        );

        ResponsiveToggleableSlider.radianSlider(
            layout, "Swerve BL Turn Position", 2, 3,
            swerve.blModule::updateTurnSetpoint, swerve.blModule::turnPosition
        );

        // swerve BR module
        ResponsiveToggleableSlider.voltSlider(
            layout, "Swerve BR Drive Voltage", 3, 0, 
            swerve.brModule::sysIdDrive, swerve.brModule::lastDriveVoltage
        );

        ResponsiveToggleableSlider.meterPerSecondSlider(
            layout, "Swerve BR Drive Velocity", 3, 1, 10.0,
            swerve.brModule::updateDriveSetpoint, swerve.brModule::driveVelocity
        );
        
        ResponsiveToggleableSlider.voltSlider(
            layout, "Swerve BR Turn Voltage", 3, 2,
            swerve.brModule::sysIdTurn, swerve.brModule::lastTurnVoltage
        );

        ResponsiveToggleableSlider.radianSlider(
            layout, "Swerve BR Turn Position", 3, 3,
            swerve.brModule::updateTurnSetpoint, swerve.brModule::turnPosition
        );

        // elevator
        ResponsiveToggleableSlider.voltSlider(
            layout, "Elevator Voltage", 4, 0, 
            elevator::updateVoltage, elevator::lastLeftVoltage, elevator::lastRightVoltage
        );

        // TODO measure the max height
        ResponsiveToggleableSlider.meterSlider(
            layout, "Elevator Position", 4, 1, 0.0, 100.0,
            elevator::updateSetpoint, elevator::leftHeight, elevator::rightHeight
        );

        // TODO measure a reasonable max velocity
        ResponsiveToggleableSlider.meterPerSecondSlider(
            layout, "Elevator Velocity", 4, 2, 1.0,
            elevator::updateSetpoint, elevator::leftVelocity, elevator::rightVelocity
        );

        // coral arm
        ResponsiveToggleableSlider.voltSlider(
            layout, "Coral Arm Voltage", 5, 0,
            coralArm::updateVoltage, coralArm::lastVoltage
        );

        ResponsiveToggleableSlider.radianSlider(
            layout, "Coral Arm Rotation", 5, 1,
            coralArm::updateSetpoint, coralArm::hexPosition
        );

        ResponsiveToggleableSlider.radianPerSecondSlider(
            layout, "Coral Arm Angular Velocity", 5, 2, 2.0,
            coralArm::updateSetpoint, coralArm::angularVelocity
        );

        // coral wheel
        ResponsiveToggleableSlider.voltSlider(
            layout, "Coral Wheel Voltage", 6, 0, 
            elevator::updateVoltage, elevator::lastLeftVoltage, elevator::lastRightVoltage
        );

        // TODO measure a reasonable max velocity
        ResponsiveToggleableSlider.meterPerSecondSlider(
            layout, "Coral Wheel Velocity", 6, 1, 10.0,
            coralWheel::updateSetpoint, coralWheel::linearVelocity
        );

        // algae arm
        ResponsiveToggleableSlider.voltSlider(
            layout, "Algae Arm Voltage", 7, 0,
            algaeArm::updateVoltage, algaeArm::lastVoltage
        );

        ResponsiveToggleableSlider.radianSlider(
            layout, "Algae Arm Rotation", 7, 1,
            algaeArm::updateSetpoint, algaeArm::hexPosition
        );

        ResponsiveToggleableSlider.radianPerSecondSlider(
            layout, "Algae Arm Angular Velocity", 7, 2, 2.0,
            algaeArm::updateSetpoint, algaeArm::angularVelocity
        );

        // algae wheel
        ResponsiveToggleableSlider.voltSlider(
            layout, "Algae Wheel Voltage", 8, 0, 
            algaeWheel::updateVoltage, algaeWheel::lastLeftVoltage, algaeWheel::lastRightVoltage
        );

        // TODO measure a reasonable max velocity
        ResponsiveToggleableSlider.meterPerSecondSlider(
            layout, "Algae Wheel Velocity", 8, 1, 10.0,
            algaeWheel::updateSetpoint, algaeWheel::lastLeftVelocity, algaeWheel::lastRightVelocity
        );
    }

    // periodic
    public void periodic() {

    }
}
