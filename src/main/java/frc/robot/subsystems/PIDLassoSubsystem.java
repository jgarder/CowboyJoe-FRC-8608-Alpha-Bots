package frc.robot.Subsystems;


import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PIDLassoSubsystem extends PIDSubsystem {

  static double kP = 0.030;
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
      this.colorSensor =  thisSensor;
      lassoMotor_encoder.setPosition(0);
      lassoMotor.setInverted(true);
      //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
      lassoMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      lassoMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      lassoMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.LassoConstants.kmaxEncoderValue);
      lassoMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.LassoConstants.kminEncoderValue);
      lassoMotor.setIdleMode(IdleMode.kBrake);
      lassoMotor.setClosedLoopRampRate(.25);
      
      lassoMotor.setSmartCurrentLimit(Constants.NeoBrushless.neo550safelimitAmps);
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
  }
  public void setMinLassoEncoderValueBasedOnColor()
  { 
    if (lassoEncoderVelocity < 100)
    {
      return;
    }
    boolean isLassoHeadedTowardsConeSize = (getSetpoint() <= Constants.LassoConstants.kminEncoderValueWithCone);
    if (colorSensor.lastdetectedColor == "Cube" && isLassoHeadedTowardsConeSize)
    {
      //lassospeed = thisspeed/2;
      setSetpointLassoCube();
      // if we are pulling in a cube, then we can only retract the lasso to a certain point
      //lassoMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.LassoConstants.kminEncoderValueWithCube);

    }

  }
  public void slowWindInBeyondSoftLimit() {
    disable(); //disable the pidcontroller of this subsystem
    double slowretractspeed = -.1;
    setSetpointLassoZero();
    lassoMotor_encoder.setPosition(Constants.LassoConstants.kmaxEncoderValue);//we could disable soft limit here but this is "safer" because you ALWAYS call reset encoder after.
    SetSpeed(slowretractspeed);
  }

  public void resetEncoder() {
    lassoMotor_encoder.setPosition(0);
    enable();//reactivate the pidcontroller of this subsystem
  }

  public void getEncoderData()
  {
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
    setSetpoint(Constants.LassoConstants.kEncoderValueLoopOut);
  }
  public void setSetpointLassoZero() {
    setSetpoint(Constants.LassoConstants.kminEncoderValue);
  }
  public void setSetpointLassoCone() {
    setSetpoint(Constants.LassoConstants.kminEncoderValueWithCone);
  }
  public void setSetpointLassoCube() {
    setSetpoint(Constants.LassoConstants.kminEncoderValueWithCube);
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