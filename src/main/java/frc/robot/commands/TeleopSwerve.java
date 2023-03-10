package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    public double getcurrentspeedmultiplier()
    {
        // subscribe to the topic in "datatable" called "Y"
        // default value is 0
        
        //ySub = datatable.getDoubleTopic("Y").subscribe(0.0);
        //return RobotContainer.SpeedAdjustSlider.getDouble(0.0);
        double clamped = MathUtil.clamp(SmartDashboard.getNumber("Jow Speed Multiplier", 0.0), 0.0, 1.0);
        return clamped;
    }
    public double getcurrentRotationMultiplier()
    {
        return MathUtil.clamp(SmartDashboard.getNumber("Jow Rotation Multiplier", 0.0), 0.0, 1.0);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed * getcurrentspeedmultiplier()), 
            rotationVal * Constants.Swerve.maxAngularVelocity * getcurrentRotationMultiplier(), 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}