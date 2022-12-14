// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDS;

public class Pigeon extends SubsystemBase {

  private final Pigeon2 gyro;
  /** Creates a new Gyro. */
  public Pigeon() {
    gyro = new Pigeon2(CANIDS.PIGEON2_ID);
  }

  public void zeroYaw() {
    setYaw(0);
  }

  public void setYaw(double yaw) {
    gyro.setYaw(-yaw);
  }

  /** */
  public double getYaw() {
    // Negate the gyro yaw so that ccw is positive
    // Gyros are cw positive by default
    return -gyro.getYaw();
  }

  /** */
  public Rotation2d getYawAsRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.telemetry == false) return;

    builder.setSmartDashboardType("Pigeon");
    builder.addDoubleProperty("Yaw", this::getYaw, null);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
