// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;

import com.studica.frc.AHRS;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
@Logged
public class Robot extends TimedRobot {
  // pass the period so the drive subsystem can discretize the chassis speeds with it
  public RobotContainer container;
  private Swerve swerve;
  private LinearVelocity xSpeed, ySpeed;
  private AngularVelocity angVel;
  // public SysIdBot bot;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    
    container = new RobotContainer(getPeriod());
    //robot doesn't initialize the pose this early on and since pose is just needed i put this as a null value
    swerve = new Swerve(kDefaultPeriod,null);


    
    


    
    // bot = new SysIdBot(getPeriod());

    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    container.periodic();
  }

  @Override
  public void autonomousInit() {
    //x is set instead of y axis because of robots pose 
    xSpeed = MetersPerSecond.of(-10);
    ySpeed = MetersPerSecond.of(0);
    angVel = RadiansPerSecond.of(0);
    swerve.driveRobotRelative(xSpeed,ySpeed,angVel);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
