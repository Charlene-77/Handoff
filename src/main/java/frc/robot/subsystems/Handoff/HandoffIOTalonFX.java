package frc.robot.subsystems.Handoff;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.robot.Constants.canIDConstants;
import frc.robot.subsystems.Handoff.HandoffIO.HandoffIOInputs;
public class HandoffIOTalonFX {
    private final TalonFX handoff = new TalonFX(canIDConstants.handoffMotor, "canivore");
    private final TalonFXConfiguration handoffConfigs = new TalonFXConfiguration();

    private VoltageOut handoffRequest = new VoltageOut(0).withEnableFOC(true);

    private final StatusSignal<Current> current = handoff.getStatorCurrent();
    private final StatusSignal<Temperature> temp = handoff.getDeviceTemp();
    private final StatusSignal<AngularVelocity> RPS = handoff.getRotorVelocity();

    private double setpointVolts;

    public HandoffIOTalonFX() {
        handoffConfigs.CurrentLimits.StatorCurrentLimit = 60;
        handoffConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        handoffConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        handoff.getConfigurator().apply(handoffConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(50,current,temp, RPS);

        handoff.optimizeBusUtilization();

        setpointVolts = 0;
    }

    public void updateInputs(HandoffIOInputs inputs) {
        BaseStatusSignal.refreshAll(current, temp, RPS);
        inputs.appliedVolts = handoffRequest.Output;
        inputs.setpointVolts = this.setpointVolts;
        inputs.currentAmps = current.getValueAsDouble();
        inputs.tempF = current.getValueAsDouble();
        inputs.handoffRPS = RPS.getValueAsDouble();
    }

    public void runHandoff(double output) {
        this.setpointVolts = output;
        handoff.setControl(handoffRequest.withOutput(output));
    }
}
