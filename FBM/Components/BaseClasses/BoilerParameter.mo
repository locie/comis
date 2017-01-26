within FBM.Components.BaseClasses;
record BoilerParameter
  parameter Modelica.SIunits.Power QBoiler "Nominal heating power"
                                                                  annotation(Dialog(group= "Boiler parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler));
  parameter Modelica.SIunits.Temperature T_nominal = 353.15
    "Temperature used to compute nominal efficiency (only used if efficiency curve depends on temperature)" annotation(Dialog(group= "Boiler parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler));
  parameter Buildings.Fluid.Types.EfficiencyCurves effCur=Buildings.Fluid.Types.EfficiencyCurves.Constant
    "Curve used to compute the efficiency" annotation(Dialog(group= "Boiler parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler or   source == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler));
  parameter Real a[:] = {0.9} "Coefficients for efficiency curve" annotation(Dialog(group= "Boiler parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler));
  parameter Modelica.SIunits.ThermalConductance UA=0.05*QBoiler/30
    "Overall UA value" annotation(Dialog(group= "Boiler parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler or                       source == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler or
                                                                                                   source == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler));
  parameter Modelica.SIunits.Volume VWat = 1.5E-6*QBoiler
    "Water volume of boiler"
    annotation(Dialog(group= "Boiler parameters", tab = "Dynamics", enable = not (energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) and (source == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler)));
  parameter Modelica.SIunits.Mass mDry =   1.5E-3*QBoiler
    "Mass of boiler that will be lumped to water heat capacity"
    annotation(Dialog(group= "Boiler parameters", tab = "Dynamics", enable = not (energyDynamics == Modelica.Fluid.Types.Dynamics.SteadyState) and (source == FBM.HeatingDHWsystems.Types.HeatSourceWoodBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSourceCondensingBoiler or
                                                                                                    source == FBM.HeatingDHWsystems.Types.HeatSourceGasBoiler)));
 parameter Modelica.SIunits.TemperatureDifference dTSupRetNom=10
    "Nominal DT in the heating system" annotation(Dialog(group= "Boiler parameters", enable= source == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler or
                                                                 source == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler or
                                                                 source == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler));
 parameter Modelica.SIunits.PressureDifference dp_nominal(min=0,
                                                           displayUnit="Pa")=6000
    "Boiler Pressure difference"
    annotation(Dialog(group= "Boiler parameters", enable= source == FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler or
                                                                 source == FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler or
                                                                 source == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler));
 parameter Modelica.SIunits.Temperature TBoiRet_min = 273.15+60
    "Boiler minimum return water temperature" annotation(Dialog(group= "Boiler parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.WoodBoiler));
   annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BoilerParameter;
