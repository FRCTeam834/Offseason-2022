// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyro extends SubsystemBase {
  private static final Gyro instance = new Gyro();

  public static final Gyro getInstance() {
    return instance;
  }

  // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/sensors/Pigeon2.html
  private Pigeon2 pigeon;
  
  private Gyro() {
    pigeon = new Pigeon2(Constants.PIGEON_ID);
  }

  public void setYaw(double deg) {
    pigeon.setYaw(deg);
  }

  public double getYaw() {
    return -pigeon.getYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
