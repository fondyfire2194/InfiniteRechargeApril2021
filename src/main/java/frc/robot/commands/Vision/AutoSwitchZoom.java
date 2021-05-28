/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;

public class AutoSwitchZoom extends CommandBase {
  /**
   * Creates a new AutoSwitchZoom.
   */

  private final LimeLight m_limelight;

  private final double minUseableTargetHeight = 40;
  private final double maxUseableTargetHeight = 120;
  private boolean changeLocked;
  private int changeToZoom2Counter;
  private int changeToZoom1Counter;
  private int changeLockedCounter;
  private double switchTime = 1.2;
  private int counterLimit = (int) switchTime * 50;

  public AutoSwitchZoom(LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limelight = limelight;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  /**
   * need to change between no and 2 x zoom as target gets too small or large
   * 
   * The tilt and turret lock loops should be stopped during the changeover.
   * 
   * 
   * 
   * 
   */
  public void execute() {

    // check if too far away for pipeline 1 - no zoom

    if (m_limelight.getIsTargetFound()) {

      if (m_limelight.getPipeline() == 1 && m_limelight.getBoundingBoxHeight() < minUseableTargetHeight
          && !changeLocked) {

        changeToZoom2Counter++;

      } else

      {
        changeToZoom2Counter = 0;
      }

      if (changeToZoom2Counter >= counterLimit) {

        m_limelight.setPipeline(2);

        changeLocked = true;

        changeLockedCounter = 10;

        changeToZoom2Counter = 0;
      }

      // check if too close for 2 x zoom
      if (m_limelight.getPipeline() == 2 && (m_limelight.getBoundingBoxHeight() > maxUseableTargetHeight)
          && !changeLocked) {

        changeToZoom1Counter++;

      } else {

        changeToZoom1Counter = 0;
      }

      if (changeToZoom1Counter >= counterLimit) {

        m_limelight.setPipeline(1);

        changeLocked = true;

        changeLockedCounter = 10;

        changeToZoom1Counter = 0;
      }

      // if a changeover is in progress, wait for period of time
      if (changeLocked) {
        m_limelight.useVision = false;
        changeLockedCounter--;
      }

      if (changeLocked && changeLockedCounter <= 0) {

        changeLocked = false;
        changeLockedCounter = 0;
        m_limelight.useVision = true;

      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
