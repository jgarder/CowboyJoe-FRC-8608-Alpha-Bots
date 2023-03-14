package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Subsystems.JoeColorSensor;
import frc.robot.Subsystems.PIDLassoSubsystem;



public class LassoJoystickCmd extends CommandBase {

    private final PIDLassoSubsystem PIDLassoSubsystem;
    private final JoeColorSensor Colorsensor;
    private final Supplier<Double> speedFunction;

    public LassoJoystickCmd(PIDLassoSubsystem PIDLassoSubsystem, 
            JoeColorSensor Colorsensor, Supplier<Double> speedFunction) {
        this.Colorsensor = Colorsensor;
        this.speedFunction = speedFunction;
        this.PIDLassoSubsystem = PIDLassoSubsystem;
        addRequirements(PIDLassoSubsystem);
    }

    @Override
    public void initialize() {
        //System.out.println("LassoJoystickCmd started!");
    }

    @Override
    public void execute() {
        PIDLassoSubsystem.SetlassoSpeed(MathUtil.applyDeadband(speedFunction.get(), Constants.stickDeadband));
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("LassoJoystickCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
