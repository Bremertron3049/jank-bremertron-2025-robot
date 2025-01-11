// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.TrackingConstants;
import frc.robot.commands.ApriltagTracker;
import frc.robot.commands.SwerveController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveDrive = new SwerveSubsystem();
  private final ApriltagTracker apriltagTracker = new ApriltagTracker(TrackingConstants.kCameraName);

  private final XboxController driverController = new XboxController(0);

  public RobotContainer() {
    swerveDrive.setDefaultCommand(new SwerveController(
      swerveDrive, 
      () -> driverController.getLeftY(),
      () -> driverController.getLeftX(),
      () -> driverController.getRightX(),
      () -> (false))  
    );

    apriltagTracker.schedule();

    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
