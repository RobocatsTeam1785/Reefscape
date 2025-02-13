package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.robot.subsystems.SysIdDrive;
import frc.robot.subsystems.SysIdTurn;

public class SysIdBot {
    private final SysIdTurn drive;
    private final CommandXboxController driver = new CommandXboxController(0);

    public SysIdBot(double period) {
        drive = new SysIdTurn(period);

        configureCommands();
    }

    private void configureCommands() {
        // button + left bumper triggers the forward test, and button + right bumper triggers the reverse test
        // while held, a triggers dynamic tests, and b triggers quasistatic tests
        driver.a().and(driver.leftBumper()).whileTrue(drive.dynamic(Direction.kForward));
        driver.a().and(driver.rightBumper()).whileTrue(drive.dynamic(Direction.kReverse));

        driver.b().and(driver.leftBumper()).whileTrue(drive.quasistatic(Direction.kForward));
        driver.b().and(driver.rightBumper()).whileTrue(drive.quasistatic(Direction.kReverse));
    }
}
