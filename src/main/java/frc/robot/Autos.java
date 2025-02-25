package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.constants.AutoConstants;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Autos {
    private final Map<String, PathPlannerAuto> autos = new HashMap<>();
    private final SendableChooser<Command> chooser;

    public Autos() {
        // initialize autos on startup to avoid problematic delays mid-game from lazy loading
        for (String auto : AutoConstants.ALL_AUTOS) {
            autos.put(auto, new PathPlannerAuto(auto));
        }

        // build and register auto chooser
        chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);
    }

    @Logged
    public Command autonomousCommand() {
        return chooser.getSelected();
    }
}
