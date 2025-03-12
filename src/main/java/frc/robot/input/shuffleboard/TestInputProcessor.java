package frc.robot.input.shuffleboard;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.input.MasterInputProcessor;
import frc.robot.commands.swerve.AlignWheelsCommand;
import frc.robot.commands.swerve.RotateCommand;
import frc.robot.subsystems.Swerve;

public class TestInputProcessor extends MasterInputProcessor {
    public final CommandXboxController c;

    public final Swerve swerve;

    public TestInputProcessor(int controllerPort, Swerve swerve) {
        c = new CommandXboxController(controllerPort);

        this.swerve = swerve;
    }

    @Override
    public void configure() {
        // when A is pressed, rotate to the angle specified by the left joystick
        c.a().onTrue(new InstantCommand(() -> {
            // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC

            // fully right means 1, which is intuitive in ENU
            double x = c.getLeftX();

            // fully up means -1, which is unintuitive in ENU, so it requires inversion
            double y = -c.getLeftY();

            double angleRadians = Math.atan2(y, x);
            Angle rotateTo = Radians.of(angleRadians);

            RotateCommand rotate = new RotateCommand(rotateTo, swerve);
            rotate.afterAlignment().schedule();
        }, swerve));

        // when B is pressed, rotate to the angle specified by the left joystick
        c.b().onTrue(new InstantCommand(() -> {
            // convert X and Y rotation from NED CCC to X and Y coordinates in ENU CCC

            // fully right means 1, which is intuitive in ENU
            double x = c.getLeftX();

            // fully up means -1, which is unintuitive in ENU, so it requires inversion
            double y = -c.getLeftY();

            double angleRadians = Math.atan2(y, x);
            Angle rotateTo = Radians.of(angleRadians);

            AlignWheelsCommand align = new AlignWheelsCommand(rotateTo, swerve);
            align.schedule();
        }, swerve));
    }
}
