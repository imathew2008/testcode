// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Robot extends TimedRobot {

  WPI_TalonSRX Left_Talon = new WPI_TalonSRX(0);
  WPI_TalonSRX Right_Talon = new WPI_TalonSRX(3);
  WPI_VictorSPX Left_VictorA = new WPI_VictorSPX(1);
  WPI_VictorSPX Left_VictorB = new WPI_VictorSPX(2);
  WPI_VictorSPX Right_VictorA = new WPI_VictorSPX(4);
  WPI_VictorSPX Right_VictorB = new WPI_VictorSPX(5);

  public final MotorControllerGroup Left_Motors = new MotorControllerGroup(Left_Talon, Left_VictorA, Left_VictorB);
  public final MotorControllerGroup Right_Motors = new MotorControllerGroup(Right_Talon, Right_VictorA, Right_VictorB);

  public final DifferentialDrive robot_Drive = new DifferentialDrive(Left_Motors, Right_Motors);
  public final Joystick Left_Stick = new Joystick(0);
  public final Joystick Right_Stick = new Joystick(1);

  private final double SCALE_FACTOR = 0.8;

  @Override
  public void robotInit() {

    Right_Motors.setInverted(true);

    Left_Talon.set(ControlMode.PercentOutput, 0);
    Right_Talon.set(ControlMode.PercentOutput, 0);
    Left_VictorA.set(ControlMode.PercentOutput, 0);
    Left_VictorB.set(ControlMode.PercentOutput, 0);
    Right_VictorA.set(ControlMode.PercentOutput, 0);
    Right_VictorB.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void teleopPeriodic() {
   robot_Drive.tankDrive(Left_Stick.getY() * SCALE_FACTOR, Right_Stick.getY() * SCALE_FACTOR);

  }
}