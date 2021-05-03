// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PotonVision */
  public final PhotonCamera m_photonCamera;;

  static final double kCameraHeight = 0.51; // meters
  static final double kCameraPitch = 0.436; // radians
  static final double kTargetHeight = 2.44; // meters

  public PhotonVision() {

    // Creates a new PhotonCamera.
    m_photonCamera = new PhotonCamera("Main");

  }

  public PhotonPipelineResult result() {
    return m_photonCamera.getLatestResult();
  }

  public List<PhotonTrackedTarget> getListTargets() {
    return result().getTargets();
  }

  public boolean hasTargets() {
    return result().hasTargets();
  }

  public double getYaw() {
    return result().getBestTarget().getYaw();
  }

  public double getPitch() {
    return result().getBestTarget().getPitch();
  }

  public void setDriverMode(boolean on) {
    m_photonCamera.setDriverMode(on);
  }

  public void setPipeline(int number) {
    m_photonCamera.setPipelineIndex(number);
  }

  public double getPipelineLatency() {
    return result().getLatencyMillis() / 1000;
  }

  public double getTargetArea() {
    return result().getBestTarget().getArea();
  }

  public double getTargetSkew() {
    return result().getBestTarget().getSkew();
  }

  public Transform2d getTargetPose() {
    return result().getBestTarget().getCameraToTarget();
  }

  public void setLeds(LEDMode mode) {
    m_photonCamera.setLED(mode);
  }

  // Get distance to target.
  public double getDistanceMeters() {
    return PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight, kTargetHeight, kCameraPitch,
        Math.toRadians(result().getBestTarget().getPitch()));
  }

  public Translation2d getTranslation2d() {
    return PhotonUtils.estimateCameraToTargetTranslation(getDistanceMeters(),
        Rotation2d.fromDegrees(-result().getBestTarget().getYaw()));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
