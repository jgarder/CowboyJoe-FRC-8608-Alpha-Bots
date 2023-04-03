// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class PIDArmLifterSubsystem extends PIDSubsystem {
 
  public enum ArmLifterState {
    STARTUP,
    CALIBRATED,
    VERITCALREADY,
    SUBSTATIONHUNT,
    SUBSTATIONGRAB,
    SCOREHUNT,
    FLOORHUNT,
    FLOORGRAB
    
  }
  static double kP = 0.025;
  static double kI = 0.0001;
  static double kD = 0.005;

  public double armliftMotorEncoderValue = 0;
  double armliftMotorEncoderVelocity = 0;
  
  SlewRateLimiter speedLimiter = new SlewRateLimiter(Constants.ArmLifterConstants.kArmLifterSlewRate);
  private final CANSparkMax armLiftMotor = new CANSparkMax(Constants.ArmLifterConstants.kArmLifterSparkMaxCanID,MotorType.kBrushless);
  private RelativeEncoder armLiftMotor_encoder; 

  private BooleanSupplier sup_isLassoOpen;
  public PIDArmLifterSubsystem(BooleanSupplier isLassoOpen) {
    super(new PIDController(kP, kI, kD));
    setSetpoint(0);
    sup_isLassoOpen = isLassoOpen;
    armLiftMotor_encoder = armLiftMotor.getEncoder();

    armLiftMotor_encoder.setPosition(0);
    armLiftMotor.setInverted(true);
    //armLiftMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
    armLiftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ArmLifterConstants.kEncoderValueMax);
    armLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ArmLifterConstants.kEncoderValueMin);
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    armLiftMotor.setOpenLoopRampRate(.1);//small ramp rate becuase this will reverse instantly. 
    armLiftMotor.setClosedLoopRampRate(.1);//orig
    armLiftMotor.setSmartCurrentLimit(Constants.NeoBrushless.neo1650safelimitAmps);

    //limit everything on this motor controller to 500ms except the status 0 frame which is 10ms and does faults and applied output. 
    armLiftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
    armLiftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);  //Default Rate: 20ms ,Motor Position
    armLiftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
    armLiftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
    armLiftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
    armLiftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency
    
    enable();//enable the pidcontroller of this subsystem
  }

   @Override
  public double getMeasurement() {
    armliftMotorEncoderValue = armLiftMotor_encoder.getPosition();

    SmartDashboard.putNumber("ArmLift PID Encoder Position",armliftMotorEncoderValue);
    return armliftMotorEncoderValue;
  }
  public double getEncoderPosition()
  {
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
      disablePidWhenParked();
    
    }
    public void disablePidWhenParked(){
      double chainStretchMagicNumber = 1.25;
      if(getSetpoint() <= Constants.ArmLifterConstants.kEncoderValueMin && Math.abs(getEncoderPosition()) < Constants.ArmLifterConstants.kEncoderValueMin + chainStretchMagicNumber){
        disable();
      }
      else{
        if(!isEnabled())
        {
          enable();
        }
        
      }
    }

    double TempForOverTemp = 37;
    public boolean isMotorOvertemp()
    {
      if(MotorTemp >TempForOverTemp)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    public boolean isLiftArmVerticalOrCloser()
    {
      //if we are already vertical then we dont need to fold back further. 
      boolean istrue = (getEncoderPosition() <= Constants.ArmLifterConstants.kEncoderValueVertical);
      SmartDashboard.putBoolean("LiftArmVeritcal", istrue);
      if(istrue) {
        return true;
      }
      else {
        return false; // we are not vertical or less (closer)
      }
    }
    public boolean getSuppliersaysLassoisOpen(){
      return sup_isLassoOpen.getAsBoolean();
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
      double finaloutput = 0;
      if(sup_isLassoOpen.getAsBoolean()){
        finaloutput = MathUtil.clamp((armLiftMotor_encoder.getPosition() + totaloutput), Constants.ArmLifterConstants.kEncoderValueMin, Constants.ArmLifterConstants.kEncoderValueGroundPickupGRAB) ;
        armLiftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ArmLifterConstants.kEncoderValueGroundPickupGRAB);
      }else{
        finaloutput = MathUtil.clamp((armLiftMotor_encoder.getPosition() + totaloutput), Constants.ArmLifterConstants.kEncoderValueMin, Constants.ArmLifterConstants.kEncoderValueGroundPickupHUNT) ;
        armLiftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ArmLifterConstants.kEncoderValueGroundPickupHUNT);
      }
      setSetpoint(finaloutput);
    }
  public void setSetpointFloorHunt() {
    enable();
    setSetpoint(Constants.ArmLifterConstants.kEncoderValueGroundPickupHUNT);
  }
  public void setSetpointFloorGrab() {
    enable();
    setSetpoint(Constants.ArmLifterConstants.kEncoderValueGroundPickupGRAB);
  }
  public void setSetpointScore() {
    enable();
    setSetpoint(Constants.ArmLifterConstants.kEncoderValueGoalScoring);
  }
  public void setSetpointVertical() {
    enable();
    setSetpoint(Constants.ArmLifterConstants.kEncoderValueVertical);
  }
  public void setSetpointStartingConfig() {
    setSetpoint(Constants.ArmLifterConstants.kEncoderValueStartingConfig);
  }
  public void setSetpointSubstationHunt() {
    setSetpoint(Constants.ArmLifterConstants.kEncoderValueSubStationHunt);
  }
  public void setSetpointSubstationGrab() {
    setSetpoint(Constants.ArmLifterConstants.kEncoderValueSubStationGrab);
  }
  public void setSetpointSubstationGrabCone() {
    setSetpoint(Constants.ArmLifterConstants.kEncoderValueSubStationGrab+2.5);
  }

  public void slowWindInBeyondSoftLimit() {
    WindInBeyondSoftLimit(-.3); //Constants.ArmLifterConstants.kslowretractspeed
  }
  public void slowerWindInBeyondSoftLimit() {
    WindInBeyondSoftLimit(-.1);
  }
  public void WindInBeyondSoftLimit(double retractSpeed) {
    disable();
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    SetSpeed(retractSpeed);
  }


  public void resetEncoder() {
    SetSpeed(0);
    armLiftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    setSetpointStartingConfig();//might as well set the setpoint to verticle to it doesnt appear to run away after finding veritcle. 
    Timer.delay(.1);//TODO MAKE THIS IN THREAD
    armLiftMotor_encoder.setPosition(Constants.ArmLifterConstants.kEncoderValueStartingConfig);
    enable();
  }


  double OutputCurrent = 0;
  double MotorTemp = 0;
  public double getMotorAmps()
  {
    OutputCurrent = armLiftMotor.getOutputCurrent();
    return OutputCurrent;
  }

    public void getEncoderData()
  {

    getMotorAmps();
    SmartDashboard.putNumber("Lifter Amps",OutputCurrent);

    MotorTemp = armLiftMotor.getMotorTemperature();
    SmartDashboard.putNumber("Lifter Temp",MotorTemp);
    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    armliftMotorEncoderValue = armLiftMotor_encoder.getPosition();
    SmartDashboard.putNumber("ArmLift Encoder Position",armliftMotorEncoderValue);

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
    //if we are already vertical then we dont need to fold back further. 
    //if(armliftMotorEncoderValue <= Constants.ArmLifterConstants.kEncoderValueVertical) {
      //return;
    //}
    //else {
      SetSpeed(speedLimiter.calculate(Constants.ArmLifterConstants.kArmLifterUpSpeed));
    //}
    
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
  
 