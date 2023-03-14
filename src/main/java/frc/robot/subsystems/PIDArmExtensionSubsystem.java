package frc.robot.Subsystems;


import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PIDArmExtensionSubsystem extends PIDSubsystem {

  static double kP = 0.015;
  static double kI = 0.0;
  static double kD = 0.0;

    double extensionMotorEncoderValue = 0;
    double extensionMotorEncoderVelocity = 0;
    
    SlewRateLimiter speedLimiter = new SlewRateLimiter(Constants.ArmExtensionConstants.kArmExtensionSlewRate);
    private final CANSparkMax extensionMotor = new CANSparkMax(Constants.ArmExtensionConstants.kArmExtensionSparkMaxCanID,MotorType.kBrushless);
    private RelativeEncoder extensionMotor_encoder; 

    public PIDArmExtensionSubsystem() {
      super(new PIDController(kP, kI, kD));
      setSetpoint(0);
      extensionMotor_encoder = extensionMotor.getEncoder();
     

      extensionMotor.setSmartCurrentLimit(Constants.NeoBrushless.neo550safelimitAmps);
    
      extensionMotor_encoder.setPosition(0);
      extensionMotor.setInverted(false);
      extensionStage = 0;
      //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
      extensionMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ArmExtensionConstants.kmaxEncoderValue);
      extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ArmExtensionConstants.kminEncoderValue);
      extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

      //limit everything on this motor controller to 500ms except the status 0 frame which is 10ms and does faults and applied output. 
      extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
      extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);  //Default Rate: 20ms ,Motor Position
      extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
      extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
      extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
      extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency
      
      getController().setTolerance(100);
      enable();//enable the pidcontroller of this subsystem
    }

  @Override
  public double getMeasurement() {
    extensionMotorEncoderValue = extensionMotor_encoder.getPosition();
    SmartDashboard.putNumber("ArmExtension PID Encoder Position",extensionMotorEncoderValue);
    return extensionMotorEncoderValue;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    SetSpeed(output);
    
    SmartDashboard.putNumber("ArmExtension PID output",output);
    SmartDashboard.putNumber("ArmExtension SetPoint",setpoint);
    //m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }

    @Override
    public void periodic() {
      getEncoderData();
      super.periodic();// This is a PidSubsystem, we have orridden the periodic method to get encoder data... So we need to call the super periodic method to get the PID stuff to work.
      disablePidWhenParked();
    }
  
  public double getEncoderPosition()
  {
    return extensionMotorEncoderValue;
  }
  
  public void disablePidWhenParked(){
    double chainStretchMagicNumber = 1.25;
    if(getSetpoint() <= Constants.ArmExtensionConstants.kminEncoderValue && Math.abs(getEncoderPosition()) < Constants.ArmExtensionConstants.kminEncoderValue + chainStretchMagicNumber){
      disable();
    }
    else{
      if(!isEnabled())
      {
        enable();
      }
      
    }
  }

  int extensionStage = -1; // -1 = unknown postion, 0 = zeroed , 1= manual mode (could be anywhere), 2 = Mid Score extension , 3 High score extension, 4 substation pickup
  
  public void runArmExtensionStages()
  {
    switch (extensionStage){
      case -1 : //we are in pre setup
        extensionMotor_encoder.setPosition(0);
        setSetpointIn();
        break;
      case 0  :
        setSetpointMidScore();
        extensionStage = 2;
        break;
      case 1  :
        break;
      case 2  :
        setSetpointHighestScore();
        extensionStage = 3;
        break;
      case 3  :
        setSetpointIn();
        break;
      case 4  :
        break;
    }
  }

  public void setSetpointHighestScore() {
    enable();
    setSetpoint(Constants.ArmExtensionConstants.kHighestGoalEncoderValue);
  }
  public void setSetpointMidScore() {
    enable();
    setSetpoint(Constants.ArmExtensionConstants.kMidestGoalEncoderValue);
  }
  public void setSetpointLowScore() {
    enable();
    setSetpoint(Constants.ArmExtensionConstants.kLowestGoalEncoderValue);
  }
  public void setSetpointIn() {
    extensionStage = 0;
    setSetpoint(Constants.ArmExtensionConstants.kminEncoderValue);
  }

  public void slowWindInBeyondSoftLimit() {
    WindInBeyondSoftLimit(-.3);
  }
  public void slowerWindInBeyondSoftLimit() {
    WindInBeyondSoftLimit(-.1);
  }
  public void WindInBeyondSoftLimit(double retractSpeed) {
    disable(); //disable the pidcontroller of this subsystem
    extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    SetSpeed(speedLimiter.calculate(retractSpeed));
  }
  public void resetEncoder() {
    SetSpeed(0);
    
    extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    setSetpointIn();//might as well set the setpoint to 0 to it doesnt appear to run away after finding 0.
    enable();
    extensionMotor_encoder.setPosition(Constants.ArmExtensionConstants.kminEncoderValue);
  }

  double OutputCurrent = 0;
  double MotorTemp = 0;
  public double getMotorAmps()
  {
    OutputCurrent = extensionMotor.getOutputCurrent();
    return OutputCurrent;
  }

    public void getEncoderData()
  {
    getMotorAmps();
    SmartDashboard.putNumber("Extension Amps",OutputCurrent);

    MotorTemp = extensionMotor.getMotorTemperature();
    SmartDashboard.putNumber("Extension Temp",MotorTemp);
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
    extensionMotorEncoderVelocity = extensionMotor_encoder.getVelocity();
    SmartDashboard.putNumber("ArmExtension Encoder Velocity", extensionMotorEncoderVelocity);

  }
  public void ExtArmOut() {
    SetSpeed(speedLimiter.calculate(Constants.ArmExtensionConstants.kArmOutSpeed));
  }
  public void ExtArmIn() {
    SetSpeed(speedLimiter.calculate(Constants.ArmExtensionConstants.kArmInSpeed));
  }
  public void ExtArmStop() {
    SetSpeed(0);
  }


  public void SetSpeed(double thisspeed) {
    extensionMotor.set(thisspeed);
  } 
}