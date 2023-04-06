package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SmartDashboardHandler;

public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();
    PathPlannerServer.startServer(5811);
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  @Override
  public void disabledInit() {
  
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    setlights();
    // schedule the autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  
  @Override
  public void autonomousPeriodic() {
    if (m_autonomousCommand == null) {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousExit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      CommandScheduler.getInstance().cancelAll();
    }
    m_autonomousCommand = null;
  }
  //PWM ourpwm = new PWM(10);
  @Override
  public void teleopInit() {        
    //solenioidCube.set(false);
    setlights();
    //ourpwm.setRaw(255);
    //ourpwm.setPeriodMultiplier(PeriodMultiplier.k4X);
    //ourpwm.setSpeed(1.0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public void setlights(){
   String ConeOrCube = SmartDashboard.getString(SmartDashboardHandler.kConeCubeModeName, SmartDashboardHandler.kConeCubeModeConeMode);

    if(ConeOrCube.equals(SmartDashboardHandler.kConeCubeModeConeMode))
    {
      m_robotContainer.solenioidCone.set(true);
      m_robotContainer.solenioidCube.set(false);
    }
    else
    {
      m_robotContainer.solenioidCone.set(false);
      m_robotContainer.solenioidCube.set(true);
    }
  }
  @Override
  public void teleopPeriodic() {
    //ourpwm.setSpeed(1.0);
    //m_robotContainer.ourRelay.setDirection(Direction.kReverse);
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
