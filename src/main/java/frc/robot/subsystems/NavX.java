// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * 
 * TEMP GYRO FOR TESTING
 * 
 */
public class NavX extends SubsystemBase {

  private final AHRS gyro;
  private double yawAdjustment = 0.0;
  /** Creates a new NavX. */
  public NavX() {
    gyro = new AHRS(SPI.Port.kMXP);
  }


  public void zeroYaw() {
    gyro.zeroYaw();
  }

  public void setYaw(double yaw) {
    yawAdjustment = yaw - getYaw();
  }

  /** */
  public double getYaw() {
    // Negate the gyro yaw so that ccw is positive
    // Gyros are cw positive by default
    return -gyro.getYaw() + yawAdjustment;
  }

  /** */
  public Rotation2d getYawAsRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.telemetry == false) return;

    builder.setSmartDashboardType("NavX");
    builder.addDoubleProperty("Yaw", this::getYaw, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
