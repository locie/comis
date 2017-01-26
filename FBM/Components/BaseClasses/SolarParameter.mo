within FBM.Components.BaseClasses;
record SolarParameter

  parameter Buildings.Fluid.SolarCollectors.Data.GenericSolarCollector perColector = Buildings.Fluid.SolarCollectors.Data.GlazedFlatPlate.FP_SolahartKf()
    "Partial performance data"
    annotation(choicesAllMatching=true,Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

  parameter Modelica.SIunits.MassFlowRate mPrim_flow_nominal(min=0)=1
    "Nominal mass flow rate"
    annotation(Dialog(group= "Solar parameters",enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

  parameter Boolean includePipesSol=true
    "Set to true to include pipes in the basecircuit"
    annotation(Dialog(group = "Settings",enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

  parameter Modelica.SIunits.PressureDifference dpHexPrim=12000 "Pressure drop in the heat exchanger in the glycol side" annotation(Dialog(group= "Solar parameters",enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Modelica.SIunits.PressureDifference dpHexSecon=12000 "Pressure drop in the heat exchanger in the water side" annotation(Dialog(group= "Solar parameters",enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

  parameter Modelica.SIunits.Angle lat(displayUnit="degree")=0.759 "Latitude"
                                                                             annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Modelica.SIunits.Angle azi(displayUnit="degree")=0.0
    "Surface azimuth (0 for south-facing; -90 degree for east-facing; +90 degree for west facing"
                                                                                                 annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Modelica.SIunits.Angle til(displayUnit="degree")=0.52
    "Surface tilt (0 for horizontally mounted collector)"
                                                         annotation(Dialog(group= "Solar parameters",enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Real rho=0.2 "Ground reflectance"
                                             annotation(Dialog(group= "Solar parameters",enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

  parameter Integer nPar(min=2) = 2 "Number of parallele solar collector branch"
                                                                                annotation(Dialog(group= "Solar parameters",enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Integer nSer= 1  "Number of solar collector in each branch"
                                                                       annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

  parameter Modelica.SIunits.Thickness InsuPipeThicknessSol=0.01
                                                             "Thickness of the pipe insulation for solarPannelField"
                                                                                                                    annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Modelica.SIunits.Length PipelengthSol=5
                                                "pipe length of the supply OR return branch for solarPannelField"
                                                                                                                 annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Modelica.SIunits.ThermalConductivity InsuHeatConduSol=0.04 "for solarPannelField"
                                                                   annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

  parameter Modelica.SIunits.Temperature T_a1_nominal= 273.15+90
                                                                "Nominal temperature of glycol in the primary loop at the inlet of the heat exchanger "
                                                                                                                                                       annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Modelica.SIunits.Temperature T_a2_nominal= 273.15+35
                                                                "Nominal temperature of water in the secondary loop at the inlet of the heat exchanger "
                                                                                                                                                        annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

  parameter Modelica.SIunits.Irradiance HOn = 200
                                                 "Minimum global tilted insulation to turn on the pump"
                                                                                                       annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Modelica.SIunits.Irradiance HOff = 150
                                                  "Minimum global tilted insulation to turn off the pump"
                                                                                                         annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

  parameter Integer nbrNodes=10 "Number of nodes in the tank"
                                                             annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField and isMulti == false));

  parameter Modelica.SIunits.Volume VTan=5 "Tank volume"
                                                        annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField and isMulti == false));
  parameter Modelica.SIunits.Length hTan=2 "Height of tank (without insulation)"
                                                                                annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField and isMulti == false));
  parameter Modelica.SIunits.Length dInsTan=0.2 "Thickness of insulation"
                                                                         annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField and isMulti == false));

  /// -- useless parameters

  parameter Real dp_nominalSol=0 "USELESS BUT NECESSARY"
                                                        annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Real Q_Solar_nominal=0
                                 "USELESS BUT NECESSARY"
                                                        annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));
  parameter Buildings.Fluid.Data.Fuels.Generic fue=Buildings.Fluid.Data.Fuels.NaturalGasHigherHeatingValue() "USELESS BUT NECESSARY" annotation(Dialog(group= "Solar parameters", enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField));

   parameter Integer posTBot(max=nbrNodes) = nbrNodes - 2
    "Position of the bottom temperature sensor"
                                               annotation(Dialog(enable = source == FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField and isMulti == false));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end SolarParameter;
