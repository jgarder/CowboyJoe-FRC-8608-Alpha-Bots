package frc.robot.Subsystems;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.ZeroLassoCmd;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PIDLassoSubsystem extends PIDSubsystem {

  static double kP = 0.040;
  static double kI = 0.0;
  static double kD = 0.0;
    
  //double Currentlassolength = 0; // this is the current length of the lasso in inches that is 'out' from the motor. 0 is retracted to tightest position (slight slack)
    
  public double lassoEncoderValue = 0;
  double lassoEncoderVelocity = 0;
  private final CANSparkMax lassoMotor = new CANSparkMax(Constants.LassoConstants.klassoMotorCanID,MotorType.kBrushless);
  private RelativeEncoder lassoMotor_encoder; 
  JoeColorSensor colorSensor;

  public PIDLassoSubsystem(JoeColorSensor thisSensor) {
      super(new PIDController(kP, kI, kD));
      setSetpoint(0);
      lassoMotor_encoder = lassoMotor.getEncoder();
      lassoMotor_encoder.setPosition(0);
      lassoState = LassoState.STARTUP;// our state, says lasso is in Startup position
      this.colorSensor =  thisSensor;
 
      lassoMotor.setInverted(true);
      //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
      lassoMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      lassoMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      lassoMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.LassoConstants.kmaxEncoderValue);
      lassoMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.LassoConstants.kminEncoderValue);
      lassoMotor.setIdleMode(IdleMode.kBrake);
      lassoMotor.setClosedLoopRampRate(.05);
      lassoMotor.setOpenLoopRampRate(.05);//small ramp rate becuase this will reverse instantly. 
      lassoMotor.setSmartCurrentLimit(Constants.NeoBrushless.neo550safelimitAmps);

      //limit everything on this motor controller to 500ms except the status 0 frame which is 10ms and does faults and applied output. 
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);  //Default Rate: 20ms ,Motor Velocity,Motor Temperature,Motor VoltageMotor Current
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);  //Default Rate: 20ms ,Motor Position
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500); //Default Rate: 50ms ,Analog Sensor Voltage ,Analog Sensor Velocity ,Analog Sensor Position
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 500); //Default Rate: 20ms, Alternate Encoder Velocity,Alternate Encoder Position
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Position,Duty Cycle Absolute Encoder Absolute Angle
      lassoMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500); //Default Rate: 200ms, Duty Cycle Absolute Encoder Velocity,Duty Cycle Absolute Encoder Frequency
      //enable();//enable the pidcontroller of this subsystem
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
    //setMinLassoEncoderValueBasedOnColor();
    
    // double beltStretchMagicNumber = 1.25;
    // if(getSetpoint() == Constants.LassoConstants.kminEncoderValue && Math.abs(lassoEncoderValue) < Constants.LassoConstants.kminEncoderValue + beltStretchMagicNumber){
    //   disable();
    // }
    // else{
    //   if(!isEnabled())
    //   {
    //     enable();
    //   }
      
    // }
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
    WindInBeyondSoftLimit(-.6);
  }
  public void slowerWindInBeyondSoftLimit() {
    WindInBeyondSoftLimit(-.1);
  }
  public void WindInBeyondSoftLimit(double retractSpeed) {
    disable(); //disable the pidcontroller of this subsystem
    lassoMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    SetSpeed(speedLimiter.calculate(retractSpeed));
  }

  public void resetEncoder() {
    SetSpeed(0);
    setSetpointLassoZero();
    
    lassoMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    enable();//reactivate the pidcontroller of this subsystem
    lassoMotor_encoder.setPosition(Constants.LassoConstants.kminEncoderValue);
  }
  double OutputCurrent = 0;
  double MotorTemp = 0;
  public double getMotorAmps()
  {
    OutputCurrent = lassoMotor.getOutputCurrent();
    return OutputCurrent;
  }
  public void getEncoderData()
  {
    getMotorAmps();
    SmartDashboard.putNumber("Lasso Amps",OutputCurrent);

    MotorTemp = lassoMotor.getMotorTemperature();
    SmartDashboard.putNumber("Lasso Temp",MotorTemp);
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
    lassoState = LassoState.OPEN;
  }
  public void setSetpointLassoZero() {
    enable();
    setSetpoint(Constants.LassoConstants.kminEncoderValue);
    lassoState = LassoState.ZERO;
  }
  public double setSetpointLassoCone() {
    enable();
    setSetpoint(Constants.LassoConstants.kminEncoderValueWithCone);
    return Constants.LassoConstants.kminEncoderValueWithCone;
  }
  public double setSetpointLassoCube() {
    enable();
    setSetpoint(Constants.LassoConstants.kminEncoderValueWithCube);
    return Constants.LassoConstants.kminEncoderValueWithCube;
  }
  public void HoldAutoLoaded(){
    disable();
    lassoState = LassoState.AUTON;
    double position = lassoMotor_encoder.getPosition();
    lassoMotor_encoder.setPosition(position);
    setSetpoint(position-3.0);
    enable();
  }


  public enum LassoState{
    STARTUP,
    ZERO,
    AUTON,
    GOCONE,
    CONEIN,
    GOCUBE,
    CUBEIN,
    OPEN,


  }
   // -1 = start state, 0 = zeroed, 1= go(ing) for cone, 2 = cone color/distance detected,
   // 3=going for cube, 4 = cube went for and color/distance detected, 5 = open position
  public LassoState lassoState = LassoState.STARTUP;
  public void RunLasso()
  {
    switch (lassoState){
      case STARTUP : // if -1 we are in start state and this will just zero us out and sit. 
        new ZeroLassoCmd(this).schedule();
        setSetpointLassoZero();
        break;
      case ZERO : // 0 if zeroed when button is pressed then send lasso out.  
        setSetpointLassoOut();
        lassoState = LassoState.OPEN;
        break;
      case GOCONE :// we are pressing button while go(ing) for a cone (we may just have a cone)
        //we dont have cone then backup
        //we do have a cone then we are scoring so open lasso
        setSetpointLassoOut();
        lassoState = LassoState.OPEN;
        break;
      case CONEIN :
        break;
      case GOCUBE : // we are pressing button while go(ing) for a cube
        //we dont have cube then backup and open lasso
        //we do have a cube then we are scoring so open lasso
        setSetpointLassoOut();
        
        break;
      case CUBEIN :
        break;
      case OPEN : // if open lasso when run lasso is ran. 
        //detect cone or cube
        if (ObjectInLasso() == "Cube")
        {
          lassoState = LassoState.GOCUBE;
          setSetpointLassoCube();
        }
        else if(ObjectInLasso() == "Cone")
        {
          lassoState = LassoState.GOCONE;
          setSetpointLassoCone();
        }
        else if(ObjectInLasso() == "RoomLight")
        {
          //nothing detected
          //retractSlowly();
          setSetpointLassoCone();
          lassoState = LassoState.GOCONE;
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
  int beltTolerance = 3;
  public boolean isLassoOut() {
      if(lassoEncoderValue > Constants.LassoConstants.kEncoderValueLoopOut - beltTolerance)
      {
        lassoState = LassoState.OPEN;
        return true;
      }
      else
      {
        return false;
      }
  }
  public boolean isLassoinOpenState(){
    if (lassoState == LassoState.OPEN){
      return true;
    }
    return false;
  }
  public double getWantedSetpoint(){
    double wantedsetpoint = Constants.LassoConstants.kminEncoderValue;// default
    if (ObjectInLasso() == "Cube")
        {
          lassoState = LassoState.GOCUBE;
          wantedsetpoint = Constants.LassoConstants.kminEncoderValueWithCube;//setSetpointLassoCube();
        }
        else if(ObjectInLasso() == "Cone")
        {
          lassoState = LassoState.GOCONE;
          wantedsetpoint = Constants.LassoConstants.kminEncoderValueWithCone;//setSetpointLassoCone();
        }
        else if(ObjectInLasso() == "RoomLight")
        {
          //nothing detected
          //retractSlowly();
          wantedsetpoint = Constants.LassoConstants.kminEncoderValueWithCone;//setSetpointLassoCone();
          lassoState = LassoState.GOCONE;
        }
    return wantedsetpoint;
  }
  public boolean getIsLassoIn() {
    double wantedsetpoint = getWantedSetpoint();

    if(lassoEncoderValue < wantedsetpoint + beltTolerance)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
  public boolean isLassoIn() {
    double wantedsetpoint = Constants.LassoConstants.kminEncoderValue;// default
    if (ObjectInLasso() == "Cube")
        {
          lassoState = LassoState.GOCUBE;
          wantedsetpoint = setSetpointLassoCube();
        }
        else if(ObjectInLasso() == "Cone")
        {
          lassoState = LassoState.GOCONE;
          wantedsetpoint = setSetpointLassoCone();
        }
        else if(ObjectInLasso() == "RoomLight")
        {
          //nothing detected
          //retractSlowly();
          wantedsetpoint = setSetpointLassoCone();
          lassoState = LassoState.GOCONE;
        }

    if(lassoEncoderValue < wantedsetpoint + beltTolerance)
      {
        return true;
      }
      else
      {
        return false;
      }
  }
  public boolean isLassoZerod() {
    if(lassoEncoderValue < Constants.LassoConstants.kminEncoderValue + beltTolerance)
      {
        lassoState = LassoState.ZERO;
        return true;
      }
      else
      {
        return false;
      }
  }
}