package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.settings.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommand extends Command{
    Climber climber;
    DoubleSupplier translationYSupplierLeft;
    DoubleSupplier translationYSupplierRight;
    BooleanSupplier climberDowen; 

    public ClimberCommand(Climber climber, DoubleSupplier translationYSupplierRight,DoubleSupplier translationYSupplierLeft,BooleanSupplier climberDown){
     
      this.climber  = climber;
      this.translationYSupplierLeft = translationYSupplierLeft;
      this.translationYSupplierRight = translationYSupplierRight;
      this.climberDowen = climberDown;
      addRequirements(climber);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if (climberDowen.getAsBoolean()) {
            climber.climberGoLeft(ClimberConstants.CLIMBER_SPEED_DOWN);
            climber.climberGoRigt(ClimberConstants.CLIMBER_SPEED_DOWN);
        }
        else {
            climber.climberGoLeft(translationYSupplierLeft.getAsDouble());
            climber.climberGoRigt(translationYSupplierRight.getAsDouble());
        }
        }
    @Override
    public void end(boolean interrupt) {
        climber.climberStop();
    }
    @Override
    public boolean isFinished(){
        return false; 
    }
} 
