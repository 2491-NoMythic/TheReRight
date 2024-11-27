package frc.robot.subsystems;


import java.util.Currency;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import frc.robot.settings.Constants.ClimberConstants;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
public class Climber extends SubsystemBase {
double runSpeedL;
double runsSpeedR;
CANSparkMax climbMotorL;
CANSparkMax climbMotorR;
SparkLimitSwitch limitSwitchR; 
SparkLimitSwitch limitSwitchL; 
RelativeEncoder climbEncoderR;
RelativeEncoder climbEncoderL;
double initialEncoderRotationsL;
double initialEncoderRotationsR;
double currentEncoderRotationsL;
double currentEncoderRotationsR;

public Climber (){
    runSpeedL = 0;
    runsSpeedR = 0;
    climbMotorL = new CANSparkMax (ClimberConstants.CLIMBER_MOTOR_LEFT, MotorType.kBrushless);
    climbMotorR = new CANSparkMax (ClimberConstants.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);
    climbMotorL.setInverted(true);
    climbMotorR.setInverted(true);
    limitSwitchL = climbMotorL.getForwardLimitSwitch(Type.kNormallyOpen);
    limitSwitchR = climbMotorR.getForwardLimitSwitch(Type.kNormallyOpen);
    climbEncoderL = climbMotorL.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    climbEncoderR = climbMotorR.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    climbMotorL.setIdleMode(IdleMode.kBrake);
    climbMotorR.setIdleMode(IdleMode.kBrake);
    initialEncoderRotationsL = Math.abs(climbEncoderL.getPosition());
    initialEncoderRotationsR = Math.abs(climbEncoderL.getPosition());
    climbMotorL.burnFlash();
    climbMotorR.burnFlash();

}
public void climberGoLeft(double speed){
    runSpeedL = speed;
}

public void climberGoRigt(double speed){
    runsSpeedR = speed;
}

public double getRightRPM(){
    return climbEncoderR.getVelocity();
}

public double getLeftRPM(){
    return climbEncoderL.getVelocity();
}

public boolean isClimberIn(){
    return (limitSwitchL.isPressed() && limitSwitchR.isPressed());
}

public void resetInitial(){
    initialEncoderRotationsL = climbEncoderL.getPosition();
    initialEncoderRotationsR = climbEncoderR.getPosition();
}

public void climberStop(){
    climbMotorL.set(0);
    climbMotorR.set(0);
}
@Override
public void periodic(){
    currentEncoderRotationsL = Math.abs(Math.abs(climbEncoderL.getPosition()) - initialEncoderRotationsL);
    currentEncoderRotationsR = Math.abs(Math.abs(climbEncoderR.getPosition()) - initialEncoderRotationsR);
    SmartDashboard.putNumber("left rotaions since start", currentEncoderRotationsL);
    SmartDashboard.putNumber("right rotaions since start", currentEncoderRotationsR);
    SmartDashboard.putBoolean("left hall effect value", isClimberIn());
    SmartDashboard.putBoolean("right hall effect value", isClimberIn());
    double lSpeed = 0;
    double rSpeed = 0;

    if (runSpeedL>0) {
        if (limitSwitchL.isPressed()) {
            lSpeed = runSpeedL;     
        }     
    }
    else{ 
        if(currentEncoderRotationsL< ClimberConstants.MAX_MOTOR_ROTATIONS);{
        lSpeed = runSpeedL;
        }
    }
    if (runsSpeedR>0) {
        if (limitSwitchR.isPressed()) {
            rSpeed = runsSpeedR;
        }
    } 
    else{
        if(currentEncoderRotationsR<ClimberConstants.MAX_MOTOR_ROTATIONS);{
            rSpeed = runsSpeedR;
        }
    }   
    climbMotorL.set(lSpeed);
    climbMotorR.set(rSpeed);
}
}