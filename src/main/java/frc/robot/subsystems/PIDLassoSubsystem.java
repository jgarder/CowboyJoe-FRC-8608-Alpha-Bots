package frc.robot.Subsystems;


import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PIDLassoSubsystem extends PIDSubsystem {

  static double kP = 0.060;
  static double kI = 0.0;
  static double kD = 0.0;
    
  //double Currentlassolength = 0; // this is the current length of the lasso in inches that is 'out' from the motor. 0 is retracted to tightest position (slight slack)
    
  double lassoEncoderValue = 0;
  double lassoEncoderVelocity = 0;
  private final CANSparkMax lassoMotor = new CANSparkMax(Constants.LassoConstants.klassoMotorCanID,MotorType.kBrushless);
  private RelativeEncoder lassoMotor_encoder; 
  JoeColorSensor colorSensor;

  public PIDLassoSubsystem(JoeColorSensor thisSensor) {
      super(new PIDController(kP, kI, kD));
      setSetpoint(0);
      lassoMotor_encoder = lassoMotor.getEncoder();
      lassoMotor_encoder.setPosition(0);
      lassoState = 0;// our state, says lasso is in 0 position
      this.colorSensor =  thisSensor;
 
      lassoMotor.setInverted(true);
      //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
      lassoMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      lassoMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      lassoMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.LassoConstants.kmaxEncoderValue);
      lassoMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.LassoConstants.kminEncoderValue);
      lassoMotor.setIdleMode(IdleMode.kBrake);
      //lassoMotor.setClosedLoopRampRate(.5);
      lassoMotor.setOpenLoopRampRate(.05);//small ramp rate becuase this will reverse instantly. 
      lassoMotor.setSmartCurrentLimit(Constants.NeoBrushless.neo550safelimitAmps);

      //limit everything on this motor controller to 500ms except the status 0 frame which is 10ms and does faults and applied output. 
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);  //Default Rate: 20ms ,Motor Position
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency
      enable();//enable the pidcontroller of this subsystem
  }

    @Override
  public double getMeasurement() {
    lassoEncoderValue = lassoMotor_encoder.getPosition();
    SmartDashboard.putNumber("Lasso PID Encoder Position",lassoEncoderValue);
    return lassoEncoderValue;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    SetSpeed(output);
    SmartDashboard.putNumber("Lasso PID output",output);
    SmartDashboard.putNumber("Lasso SetPoint",setpoint);
    //m_shooterMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
  }
  public void SetSpeed(double thisspeed) {
    lassoMotor.set(thisspeed);
  }

  @Override
  public void periodic() {
    getEncoderData();
    super.periodic();// This is a PidSubsystem, we have orridden the periodic method to get encoder data... So we need to call the super periodic method to get the PID stuff to work.
    setMinLassoEncoderValueBasedOnColor();
    double beltStretchMagicNumber = 1.25;
    if(getSetpoint() == Constants.LassoConstants.kminEncoderValue && lassoEncoderValue < Constants.LassoConstants.kminEncoderValue + beltStretchMagicNumber){
      disable();
    }
    else{
      if(!isEnabled())
      {
        enable();
      }
      
    }
  }
  public void setMinLassoEncoderValueBasedOnColor()
  { 
    if (lassoEncoderVelocity < 100)
    {
      return;
    }
    boolean isLassoHeadedTowardsConeSize = (getSetpoint() <= Constants.LassoConstants.kminEncoderValueWithCube);
    if (colorSensor.lastdetectedColor == "Cube" && isLassoHeadedTowardsConeSize)
    {
      //lassospeed = thisspeed/2;
      setSetpointLassoCube();
      // if we are pulling in a cube, then we can only retract the lasso to a certain point
      //lassoMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.LassoConstants.kminEncoderValueWithCube);

    }

  }
  SlewRateLimiter speedLimiter = new SlewRateLimiter(.5);
  public void slowWindInBeyondSoftLimit() {
    disable(); //disable the pidcontroller of this subsystem
    double slowretractspeed = -.6;//-.1;
    lassoMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    SetSpeed(speedLimiter.calculate(slowretractspeed));
  }
  public void slowerWindInBeyondSoftLimit() {
    disable(); //disable the pidcontroller of this subsystem
    double slowretractspeed = -.1;//-.1;
    lassoMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    SetSpeed(speedLimiter.calculate(slowretractspeed));
  }

  public void resetEncoder() {
    SetSpeed(0);
    setSetpointLassoZero();
    
    lassoMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    enable();//reactivate the pidcontroller of this subsystem
    lassoMotor_encoder.setPosition(Constants.LassoConstants.kminEncoderValue);
  }
  double OutputCurrent = 0;
  double LassoTemp = 0;
  public double getLassoAmps()
  {
    OutputCurrent = lassoMotor.getOutputCurrent();
    return OutputCurrent;
  }
  public void getEncoderData()
  {
    getLassoAmps();
    SmartDashboard.putNumber("Lasso Amps",OutputCurrent);

    LassoTemp = lassoMotor.getMotorTemperature();
    SmartDashboard.putNumber("Lasso Temp",LassoTemp);
    /**
     * Encoder position is read from a RelativeEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    lassoEncoderValue = lassoMotor_encoder.getPosition();
    SmartDashboard.putNumber("Lasso Encoder Position",lassoEncoderValue);

    /**
     * Encoder velocity is read from a RelativeEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    lassoEncoderVelocity = lassoMotor_encoder.getVelocity();
    SmartDashboard.putNumber("Lasso Encoder Velocity", lassoEncoderVelocity);

  }

  public void setSetpointLassoOut() {
    enable();
    setSetpoint(Constants.LassoConstants.kEncoderValueLoopOut);
  }
  public void setSetpointLassoZero() {
    enable();
    setSetpoint(Constants.LassoConstants.kminEncoderValue);
    lassoState = 0;
  }
  public void setSetpointLassoCone() {
    enable();
    setSetpoint(Constants.LassoConstants.kminEncoderValueWithCone);
  }
  public void setSetpointLassoCube() {
    enable();
    setSetpoint(Constants.LassoConstants.kminEncoderValueWithCube);
  }

   // -1 = start state, 0 = zeroed, 1= go(ing) for cone, 2 = cone color/distance detected,
   // 3=going for cube, 4 = cube went for and color/distance detected, 5 = open position
   int lassoState = -1;
  public void RunLasso()
  {
    switch (lassoState){
      case -1 : // if -1 we are in start state and this will just zero us out and sit. 
        setSetpointLassoZero();
        break;
      case 0 : // 0 if zeroed when button is pressed then send lasso out.  
        setSetpointLassoOut();
        lassoState = 5;
        break;
      case 1 :// we are pressing button while go(ing) for a cone (we may just have a cone)
        //we dont have cone then backup
        //we do have a cone then we are scoring so open lasso
        setSetpointLassoOut();
        lassoState = 5;
        break;
      case 2 :
        break;
      case 3 : // we are pressing button while go(ing) for a cube
        //we dont have cube then backup and open lasso
        //we do have a cube then we are scoring so open lasso
        setSetpointLassoOut();
        lassoState = 5;
        break;
      case 4 :
        break;
      case 5 : // if open lasso when run lasso is ran. 
        //detect cone or cube
        if (ObjectInLasso() == "Cube")
        {
          lassoState = 3;
          setSetpointLassoCube();
        }
        else if(ObjectInLasso() == "Cone")
        {
          lassoState = 1;
          setSetpointLassoCone();
        }
        else if(ObjectInLasso() == "RoomLight")
        {
          //nothing detected
          //retractSlowly();
          setSetpointLassoCone();
          lassoState = 1;
        }
        break;


    }
  }

  boolean Zeroed = false;
  private void retractSlowly() {
    int reduction = 10;
    if(lassoEncoderValue > reduction)
    {
      setSetpoint(lassoEncoderValue-reduction);
    }
  }

  public String ObjectInLasso()
  {
    return colorSensor.lastdetectedColor;
  }
  /**
   * 
   * feed this a color sensor and speed from an axis of -1 to 1
   */
  public void SetlassoSpeed(double thisspeed) {
    double lassospeed = 0;

    if(thisspeed > 0.2)//if we are letting the lasso out at all
    {
      setSetpointLassoOut();
    }

    //final speed divider if you are pulling in a cube. This is a safety feature to prevent the lasso from pulling in the cube too fast and breaking it. 
    // todo make this look at amperage on motor and stop if it is too high (ie the motor is stalled or the lasso is pulled tight enough)
    if (thisspeed < -0.2 && colorSensor.lastdetectedColor == "Cube")
    {
      //lassospeed = thisspeed/2;
      setSetpointLassoCube();
      // if we are pulling in a cube, then we can only retract the lasso to a certain point
      //lassoMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.LassoConstants.kminEncoderValueWithCube);

    }
    else if (thisspeed < -0.2 )
    {
      setSetpointLassoCone();
      //lassospeed = thisspeed;
    }
    else 
    {
      // if(Math.abs(thisspeed) > .5)//if outside of the deadband range .2 to -.2
      // {
      //   // if we are not pulling in a cube or Cone, then we can retract the lasso all the way
      //   lassospeed = thisspeed;
      //   SetSpeed(lassospeed);
      //   setSetpoint(lassoMotor_encoder.getPosition());
      // }

    }
    //SetSpeed(lassospeed);
  }
}