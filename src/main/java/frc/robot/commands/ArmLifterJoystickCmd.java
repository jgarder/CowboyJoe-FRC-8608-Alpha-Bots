package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.PIDArmLifterSubsystem;



public class ArmLifterJoystickCmd extends CommandBase {

    private final PIDArmLifterSubsystem PIDArmLifterSubsystem;
    private final Supplier<Double> speedFunction;

    public ArmLifterJoystickCmd(PIDArmLifterSubsystem PIDArmLifterSubsystem, 
             Supplier<Double> speedFunction) {
        this.speedFunction = speedFunction;
        this.PIDArmLifterSubsystem = PIDArmLifterSubsystem;
        addRequirements(PIDArmLifterSubsystem);
    }
    @Override
    public void initialize() {
        //System.out.println("LassoJoystickCmd started!");
    }

    @Override
    public void execute() {
        //PIDArmLifterSubsystem.SetSpeed(speedFunction.get());
        double SetpointGain = 20;
        PIDArmLifterSubsystem.setSetpoint(PIDArmLifterSubsystem.getMeasurement() + (SetpointGain * speedFunction.get()));
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
