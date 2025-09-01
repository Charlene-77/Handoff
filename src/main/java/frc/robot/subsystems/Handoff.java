package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import static edu.wpi.first.units.Units.*;


public class Handoff extends SubsystemBase{
    private final TalonFX handoffMotor;
    private VoltageOut handoffRequest;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;
    private final StatusSignal<AngularVelocity> RPS;
    private double setpointVolts;

  
    public Handoff (){
        handoffMotor = new TalonFX(Constants.canIDConstants.handoffMotor, "canivore");
        handoffMotor.setNeutralMode(NeutralModeValue.Coast); 
        handoffRequest = new VoltageOut(0).withEnableFOC(true);
        current = handoffMotor.getStatorCurrent();
        temp = handoffMotor.getDeviceTemp();
        RPS = handoffMotor.getRotorVelocity();
        
        var handoffConfigs = new TalonFXConfiguration();
        var handoffCurrentLimitConfigs = handoffConfigs.CurrentLimits;
        handoffCurrentLimitConfigs.StatorCurrentLimit = 60;
        handoffCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        handoffConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        handoffMotor.getConfigurator().apply(handoffConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(50,current,temp, RPS);
        handoffMotor.optimizeBusUtilization();

        setpointVolts = 0.0;

    }

      public void runHandoff(double voltage){
        setpointVolts = voltage;
        handoffMotor.setControl(handoffRequest.withOutput(voltage));
    }

    public double getStatorCurrent(){
        return current.getValueAsDouble();
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(current, temp, RPS);
        SmartDashboard.putNumber("Handoff Voltage", setpointVolts);
        SmartDashboard.putNumber("Handoff Current", current.getValue().in(Amps));
        SmartDashboard.putNumber("Handoff Temperature", temp.getValueAsDouble());
        SmartDashboard.putNumber("Handoff Speed (RPS)", RPS.getValue().in(RotationsPerSecond));
    }
}
        
        
  


