// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Localization.localization;

public class vision extends SubsystemBase {
  /** Creates a new vision. */
  public vision() {
    builder.setTable(
        NetworkTableInstance.getDefault()
            .getTable("MyField")
    );
    localization.getField2d().initSendable(builder);
    builder.startListeners();
  }
  localization localization = new localization();
  SendableBuilderImpl builder = new SendableBuilderImpl();
  

  @Override
  public void periodic() {
    System.out.println("UPDATING/VANCE STINKS");
    // This method will be called once per scheduler run
    localization.updateOdometry("limelight-barbuda");
  }
}
