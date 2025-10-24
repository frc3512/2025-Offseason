package org.frc3512.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {

  public Robot() {}

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void robotInit(){
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Start AdvantageKit logger
    Logger.start();
  }
}
