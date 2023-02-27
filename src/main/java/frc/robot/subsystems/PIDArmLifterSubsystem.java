// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class PIDArmLifterSubsystem extends PIDSubsystem {
 
  static double kP = 0.025;
  static double kI = 0.0;
  static double kD = 0.005;

  public double armliftMotorEncoderValue = 0;
  double armliftMotorEncoderVelocity = 0;
  
  SlewRateLimiter speedLimiter = new SlewRateLimiter(Constants.ArmLifterConstants.kArmLifterSlewRate);
  private final CANSparkMax armLiftMotor = new CANSparkMax(Constants.ArmLifterConstants.kArmLifterSparkMaxCanID,MotorType.kBrushless);
  private RelativeEncoder armLiftMotor_encoder; 

  public PIDArmLifterSubsystem() {
    super(new PIDController(kP, kI, kD));
    setSetpoint(0);
    armLiftMotor_encoder = armLiftMotor.getEncoder();

    armLiftMotor_encoder.setPosition(0);
    armLiftMotor.setInverted(true);
    //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
    armLiftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ArmLifterConstants.kmaxEncoderValue);
    armLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ArmLifterConstants.kminEncoderValue);
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    //armLiftMotor.setOpenLoopRampRate(.1);
    armLiftMotor.setClosedLoopRampRate(.05);
    enable();//enable the pidcontroller of this subsystem
  }

   @Override
  public double getMeasurement() {
    armliftMotorEncoderValue = armLiftMotor_encoder.getPosition();

    SmartDashboard.putNumber("ArmLift PID Encoder Position",armliftMotorEncoderValue);
    return armliftMotorEncoderValue;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    SetSpeed(output);
    SmartDashboard.putNumber("ArmLift PID output",output);
    SmartDashboard.putNumber("ArmLift SetPoint",setpoint);
    //m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

    @Override
    public void periodic() {
      getEncoderData();
      super.periodic();// This is a PidSubsystem, we have orridden the periodic method to get encoder data... So we need to call the super periodic method to get the PID stuff to work.
    }

    public void setSetpointAtCurrentPoint() {
      armliftMotorEncoderVelocity = armLiftMotor_encoder.getVelocity();
      double absGain = 2;
      double outputgain = 0;
      if(armliftMotorEncoderVelocity >= 0.0)
      {
        outputgain = absGain;
      }
      else
      {
        outputgain = absGain * -1.0;
      }
      double totaloutput = ((armliftMotorEncoderVelocity/60)/60) * outputgain;
      SmartDashboard.putNumber("ArmLift velocity add",totaloutput);
      setSetpoint(armLiftMotor_encoder.getPosition() + totaloutput );
    }
  
  public void setSetpointGround() {
    setSetpoint(80);
  }
  public void setSetpointScore() {
    setSetpoint(Constants.ArmLifterConstants.kGoalScoringEncoderValue);
  }
  public void setSetpointVertical() {
    setSetpoint(0);
  }

  public void slowWindInBeyondSoftLimit() {
    disable();
    double kslowretractspeed = -.2;
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    SetSpeed(speedLimiter.calculate(kslowretractspeed));
  }
  public void resetEncoder() {
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    setSetpointVertical();//might as well set the setpoint to 0 to it doesnt appear to run away after finding 0.
    armLiftMotor_encoder.setPosition(0);
    enable();
  }

    public void getEncoderData()
  {
    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    //extensionMotorEncoderValue = extensionMotor_encoder.getPosition();
    //SmartDashboard.putNumber("ArmExtension Encoder Position",extensionMotorEncoderValue);

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    armliftMotorEncoderVelocity = armLiftMotor_encoder.getVelocity();
    SmartDashboard.putNumber("ArmLift Encoder Velocity", armliftMotorEncoderVelocity);

  }
  public void LiftArmUP() {
    SetSpeed(speedLimiter.calculate(Constants.ArmLifterConstants.kArmLifterUpSpeed));
  }
  public void LiftArmDown() {
    SetSpeed(speedLimiter.calculate(Constants.ArmLifterConstants.kArmLifterDownSpeed));
  }
  public void LiftArmStop() {
    SetSpeed(0);
  }


  public void SetSpeed(double thisspeed) {
    armLiftMotor.set(thisspeed);
  } 
}
  
 