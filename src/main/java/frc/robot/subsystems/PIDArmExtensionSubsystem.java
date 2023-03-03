package frc.robot.Subsystems;


import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.SimDevice.Direction;
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

      extensionMotor_encoder.setPosition(0);
      extensionMotor.setInverted(false);
      ;
      //lassoMotor_encoder.setVelocityConversionFactor(lassoencodercountsperinch);
      extensionMotor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ArmExtensionConstants.kmaxEncoderValue);
      extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ArmExtensionConstants.kminEncoderValue);
      extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
      extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      enable();//enable the pidcontroller of this subsystem
      getController().setTolerance(100);
      extensionMotor.setSmartCurrentLimit(Constants.NeoBrushless.neo550safelimitAmps);
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
    }


  public void setSetpointHighestScore() {
    setSetpoint(Constants.ArmExtensionConstants.kHighestGoalEncoderValue);
  }
  public void setSetpointMidScore() {
    setSetpoint(Constants.ArmExtensionConstants.kMidestGoalEncoderValue);
  }
  public void setSetpointLowScore() {
    setSetpoint(Constants.ArmExtensionConstants.kLowestGoalEncoderValue);
  }
  public void setSetpointIn() {
    setSetpoint(Constants.ArmExtensionConstants.kminEncoderValue);
  }

  public void slowWindInBeyondSoftLimit() {
    disable();
    double slowretractspeed = -.2;
    extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    //extensionMotor_encoder.setPosition(Constants.ArmExtensionConstants.kmaxEncoderValue);//we could disable soft limit here but this is "safer" because you ALWAYS call reset encoder after.
    SetSpeed(slowretractspeed);
  }
  public void resetEncoder() {
    extensionMotor_encoder.setPosition(0);
    extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    setSetpointIn();//might as well set the setpoint to 0 to it doesnt appear to run away after finding 0.
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