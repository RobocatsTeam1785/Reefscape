package frc.robot.input.shuffleboard;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.constants.AlgaeArmConstants;
import frc.lib.constants.CoralArmConstants;
import frc.lib.constants.ElevatorConstants;
import frc.lib.input.shuffleboard.ResponsiveToggleableSlider;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeWheel;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralWheel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class ShuffleboardInputProcessor {
    // properties
    String tabTitle;

    final Swerve swerve;
    final Elevator elevator;

    final CoralArm coralArm;
    final CoralWheel coralWheel;

    final AlgaeArm algaeArm;
    final AlgaeWheel algaeWheel;

    private ShuffleboardTab tab;

    private ResponsiveToggleableSlider
        // flDriveVoltage, flDriveVelocity, flTurnVoltage, flTurnPosition,
        // frDriveVoltage, frDriveVelocity, frTurnVoltage, frTurnPosition,
        // blDriveVoltage, blDriveVelocity, blTurnVoltage, blTurnPosition,
        // brDriveVoltage, brDriveVelocity, brTurnVoltage, brTurnPosition,

        elevatorVoltage, elevatorPosition, elevatorVelocity,
        coralArmVoltage, coralArmRotation, coralArmVelocity,
        coralWheelVoltage, coralWheelVelocity,
        algaeArmVoltage, algaeArmRotation, algaeArmVelocity,
        algaeWheelVoltage, algaeWheelVelocity;

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
        this.tabTitle = tabTitle;

        this.swerve = swerve;
        this.elevator = elevator;

        this.coralArm = coralArm;
        this.coralWheel = coralWheel;
        
        this.algaeArm = algaeArm;
        this.algaeWheel = algaeWheel;

        initialize(tabTitle, swerve, elevator, coralArm, coralWheel, algaeArm, algaeWheel);
    }

    public void initialize(
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
            .withSize(40, 19);

        // // swerve FL module
        // flDriveVoltage = ResponsiveToggleableSlider.voltSlider(
        //     layout, "Swerve FL Drive Voltage", 0, 0, 
        //     swerve.flModule::sysIdDrive, swerve.flModule::lastDriveVoltage
        // );

        // flDriveVelocity = ResponsiveToggleableSlider.meterPerSecondSlider(
        //     layout, "Swerve FL Drive Velocity", 0, 1, 10.0,
        //     swerve.flModule::updateDriveSetpoint, swerve.flModule::driveVelocity
        // );
        
        // flTurnVoltage = ResponsiveToggleableSlider.voltSlider(
        //     layout, "Swerve FL Turn Voltage", 0, 2,
        //     swerve.flModule::sysIdTurn, swerve.flModule::lastTurnVoltage
        // );

        // flTurnPosition = ResponsiveToggleableSlider.radianSlider(
        //     layout, "Swerve FL Turn Position", 0, 3,
        //     swerve.flModule::updateTurnSetpoint, swerve.flModule::turnPosition
        // );

        // // swerve FR module
        // frDriveVoltage = ResponsiveToggleableSlider.voltSlider(
        //     layout, "Swerve FR Drive Voltage", 1, 0,
        //     swerve.frModule::sysIdDrive, swerve.frModule::lastDriveVoltage
        // );

        // frDriveVelocity = ResponsiveToggleableSlider.meterPerSecondSlider(
        //     layout, "Swerve FR Drive Velocity", 1, 1, 10.0,
        //     swerve.frModule::updateDriveSetpoint, swerve.frModule::driveVelocity
        // );
        
        // frTurnVoltage = ResponsiveToggleableSlider.voltSlider(
        //     layout, "Swerve FR Turn Voltage", 1, 2,
        //     swerve.frModule::sysIdTurn, swerve.frModule::lastTurnVoltage
        // );

        // frTurnPosition = ResponsiveToggleableSlider.radianSlider(
        //     layout, "Swerve FR Turn Position", 1, 3,
        //     swerve.frModule::updateTurnSetpoint, swerve.frModule::turnPosition
        // );

        // // swerve BL module
        // blDriveVoltage = ResponsiveToggleableSlider.voltSlider(
        //     layout, "Swerve BL Drive Voltage", 2, 0, 
        //     swerve.blModule::sysIdDrive, swerve.blModule::lastDriveVoltage
        // );

        // blDriveVelocity = ResponsiveToggleableSlider.meterPerSecondSlider(
        //     layout, "Swerve BL Drive Velocity", 2, 1, 10.0,
        //     swerve.blModule::updateDriveSetpoint, swerve.blModule::driveVelocity
        // );
        
        // blTurnVoltage = ResponsiveToggleableSlider.voltSlider(
        //     layout, "Swerve BL Turn Voltage", 2, 2,
        //     swerve.blModule::sysIdTurn, swerve.blModule::lastTurnVoltage
        // );

        // blTurnPosition = ResponsiveToggleableSlider.radianSlider(
        //     layout, "Swerve BL Turn Position", 2, 3,
        //     swerve.blModule::updateTurnSetpoint, swerve.blModule::turnPosition
        // );

        // // swerve BR module
        // brDriveVoltage = ResponsiveToggleableSlider.voltSlider(
        //     layout, "Swerve BR Drive Voltage", 3, 0, 
        //     swerve.brModule::sysIdDrive, swerve.brModule::lastDriveVoltage
        // );

        // brDriveVelocity = ResponsiveToggleableSlider.meterPerSecondSlider(
        //     layout, "Swerve BR Drive Velocity", 3, 1, 10.0,
        //     swerve.brModule::updateDriveSetpoint, swerve.brModule::driveVelocity
        // );
        
        // brTurnVoltage = ResponsiveToggleableSlider.voltSlider(
        //     layout, "Swerve BR Turn Voltage", 3, 2,
        //     swerve.brModule::sysIdTurn, swerve.brModule::lastTurnVoltage
        // );

        // brTurnPosition = ResponsiveToggleableSlider.radianSlider(
        //     layout, "Swerve BR Turn Position", 3, 3,
        //     swerve.brModule::updateTurnSetpoint, swerve.brModule::turnPosition
        // );

        // elevator
        elevatorVoltage = ResponsiveToggleableSlider.voltSlider(
            layout, "Elevator Voltage", 4, 0, 
            elevator::updateVoltage, elevator::lastLeftVoltage, elevator::lastRightVoltage
        );

        // TODO measure the max height
        elevatorPosition = ResponsiveToggleableSlider.meterSlider(
            layout, "Elevator Position", 4, 1, 0.0, ElevatorConstants.MAX_HEIGHT.in(Meters),
            elevator::updateSetpoint, elevator::leftHeight, elevator::rightHeight
        );

        // TODO measure a reasonable max velocity
        elevatorVelocity = ResponsiveToggleableSlider.meterPerSecondSlider(
            layout, "Elevator Velocity", 4, 2, 1.0,
            elevator::updateSetpoint, elevator::leftVelocity, elevator::rightVelocity
        );

        // coral arm
        coralArmVoltage = ResponsiveToggleableSlider.voltSlider(
            layout, "Coral Arm Voltage", 5, 0,
            coralArm::updateVoltage, coralArm::lastVoltage
        );

        double minAngle = CoralArmConstants.MIN_ANGLE.in(Degrees);
        double maxAngle = CoralArmConstants.MAX_ANGLE.in(Degrees);
        double mediumAngle = (maxAngle - minAngle) / 2.0;

        coralArmRotation = ResponsiveToggleableSlider.degreeSlider(
            layout, "Coral Arm Rotation", 5, 1,
            minAngle, minAngle, mediumAngle, minAngle, maxAngle,
            coralArm::updateSetpoint, coralArm::hexPosition
        );

        coralArmVelocity = ResponsiveToggleableSlider.degreePerSecondSlider(
            layout, "Coral Arm Angular Velocity", 5, 2, 90.0,
            coralArm::updateSetpoint, coralArm::angularVelocity
        );

        // coral wheel
        coralWheelVoltage = ResponsiveToggleableSlider.voltSlider(
            layout, "Coral Wheel Voltage", 6, 0, 
            coralWheel::updateVoltage, coralWheel::lastVoltage
        );

        // TODO measure a reasonable max velocity
        coralWheelVelocity = ResponsiveToggleableSlider.meterPerSecondSlider(
            layout, "Coral Wheel Velocity", 6, 1, 10.0,
            coralWheel::updateSetpoint, coralWheel::linearVelocity
        );

        // algae arm
        algaeArmVoltage = ResponsiveToggleableSlider.voltSlider(
            layout, "Algae Arm Voltage", 7, 0,
            algaeArm::updateVoltage, algaeArm::lastVoltage
        );

        minAngle = AlgaeArmConstants.MIN_ANGLE.in(Degrees);
        maxAngle = AlgaeArmConstants.MAX_ANGLE.in(Degrees);
        mediumAngle = (maxAngle - minAngle) / 2.0;

        algaeArmRotation = ResponsiveToggleableSlider.degreeSlider(
            layout, "Algae Arm Rotation", 7, 1,
            minAngle, minAngle, mediumAngle, minAngle, maxAngle,
            algaeArm::updateSetpoint, algaeArm::hexPosition
        );

        algaeArmVelocity = ResponsiveToggleableSlider.degreePerSecondSlider(
            layout, "Algae Arm Angular Velocity", 7, 2, 90.0,
            algaeArm::updateSetpoint, algaeArm::angularVelocity
        );

        // algae wheel
        // algaeWheelVoltage = ResponsiveToggleableSlider.voltSlider(
        //     layout, "Algae Wheel Voltage", 8, 0, 
        //     algaeWheel::updateVoltage, algaeWheel::lastLeftVoltage, algaeWheel::lastRightVoltage
        // );

        // // TODO measure a reasonable max velocity
        // algaeWheelVelocity = ResponsiveToggleableSlider.meterPerSecondSlider(
        //     layout, "Algae Wheel Velocity", 8, 1, 10.0,
        //     algaeWheel::updateSetpoint, algaeWheel::lastLeftVelocity, algaeWheel::lastRightVelocity
        // );
    }

    // periodic
    public void periodic() {
        ResponsiveToggleableSlider[] sliders = {
            // flDriveVoltage, flDriveVelocity, flTurnVoltage, flTurnPosition,
            // frDriveVoltage, frDriveVelocity, frTurnVoltage, frTurnPosition,
            // blDriveVoltage, blDriveVelocity, blTurnVoltage, blTurnPosition,
            // brDriveVoltage, brDriveVelocity, brTurnVoltage, brTurnPosition,

            elevatorVoltage, elevatorPosition, elevatorVelocity,
            coralArmVoltage, coralArmRotation, coralArmVelocity,
            // coralWheelVoltage, coralWheelVelocity,
            algaeArmVoltage, algaeArmRotation, algaeArmVelocity,
            // algaeWheelVoltage, algaeWheelVelocity
        };

        for (ResponsiveToggleableSlider slider : sliders) {
            slider.periodic();
        }
    }
}
