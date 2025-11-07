// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveBase.h"

class DriveBaseSim {
public:
  friend class DriveBase; 

public:
  // Constructor for DriveBaseSim
  DriveBaseSim(DriveBase &driveBase);
  ~DriveBaseSim();

  rev::spark::SparkMaxSim m_simLeftMotorLeader;
  rev::spark::SparkMaxSim m_simLeftMotorFollower;
  rev::spark::SparkMaxSim m_simRightMotorLeader;
  rev::spark::SparkMaxSim m_simRightMotorFollower;

  frc::DCMotor maxGearbox = frc::DCMotor::NEO(1);

  frc::sim::DifferentialDrivetrainSim m_driveSim{
    frc::DCMotor::NEO(2),
    7.29,
    7.5_kg_sq_m,
    70_kg,
    3_in,
    0.7112_in,
    //standard deviation for measurmnt noise
    {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}
  };

  frc::Field2d m_field;
};

DriveBase::DriveBase() : 
m_leftMotorLeader{20, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_leftMotorFollower{21, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_rightMotorLeader{22, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_rightMotorFollower{23, rev::spark::SparkLowLevel::MotorType::kBrushless},
m_sim_state{new DriveBaseSim{*this}}
{
  // Implementation of subsystem constructor goes here.
  
  rev::spark::SparkBaseConfig configLeft;
  configLeft.Follow(m_leftMotorLeader); //set to leader
  m_leftMotorFollower.Configure(configLeft, 
    rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
    rev::spark::SparkBase::PersistMode::kNoPersistParameters);

  rev::spark::SparkBaseConfig configRight;
  configRight.Follow(m_rightMotorLeader); //set to leader
  m_rightMotorFollower.Configure(configRight, 
    rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
    rev::spark::SparkBase::PersistMode::kNoPersistParameters);
}

DriveBase::~DriveBase() {}

void DriveBase::MoveLeftMotor(double speed) {
  m_leftMotorLeader.Set(speed);
}

void DriveBase::MoveRightMotor(double speed) {
  m_rightMotorLeader.Set(speed);
}

void DriveBase::Move(double Lspeed, double Rspeed) {
  this->MoveLeftMotor(Lspeed);
  this->MoveRightMotor(Rspeed);
}

DriveBaseSim::DriveBaseSim(DriveBase& drivebase):
  m_simLeftMotorLeader{&drivebase.m_leftMotorLeader, &maxGearbox},
  m_simLeftMotorFollower{&drivebase.m_leftMotorLeader, &maxGearbox},
  m_simRightMotorLeader{&drivebase.m_leftMotorLeader, &maxGearbox},
  m_simRightMotorFollower{&drivebase.m_leftMotorLeader, &maxGearbox}
{
  frc::SmartDashboard::PutData("Field", &m_field);
}

DriveBaseSim::~DriveBaseSim() {}

void DriveBase::SimulationPeriodic() {
  rotation += units::degree_t(2 * (m_leftMotorLeader.Get() - m_rightMotorLeader.Get()));

  rotation = units::degree_t(static_cast<int>(rotation) % 360);
  
  if ((double)rotation <= 0) { 
      rotation = units::degree_t(360 + (double)rotation); 
  }

  double dist = 0;
  if (abs(m_leftMotorLeader.Get()) > abs(m_rightMotorLeader.Get())) {
      dist = m_rightMotorLeader.Get() / 15;
  } else {
      dist = m_leftMotorLeader.Get() / 15;
  }

  tranX += units::length::meter_t(  
      dist * std::cos((double)rotation * 3.1415 / 180.0));
  tranY += units::length::meter_t(
      dist * std::sin((double)rotation * 3.1415 / 180.0));

  m_sim_state->m_field.SetRobotPose(frc::Pose2d({tranX, tranY}, rotation));
}
