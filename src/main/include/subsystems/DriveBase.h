// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <rev/SparkMax.h>

#include <rev/sim/SparkMaxSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

class DriveBaseSim;

class DriveBase : public frc2::SubsystemBase {
 public:
  DriveBase();
  ~DriveBase();

  void MoveLeftMotor(double speed);
  void MoveRightMotor(double speed);
  void Move(double Lspeed, double Rspeed);


  void SimulationPeriodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkMax m_leftMotorLeader;
  rev::spark::SparkMax m_leftMotorFollower;
  
  rev::spark::SparkMax m_rightMotorLeader;
  rev::spark::SparkMax m_rightMotorFollower;

  //simulation
  friend class DriveBaseSim;
  std::unique_ptr<DriveBaseSim> m_sim_state;
};
