within FBM;
package ElementaryBlocs
  extends Modelica.Icons.VariantsPackage;
  model BalancingValve
    //Extensions
    extends Interfaces.PartialCircuitBalancingValve(final useBalancingValve=true);
  equation
    if not includePipes then
      if not measureSupplyT then
        connect(port_a1, port_b1);
      else
        connect(senTemSup.port_a, port_a1) annotation (Line(
        points={{60,60},{-100,60}},
        color={0,127,255},
        smooth=Smooth.None));
      end if;
    end if;
    if includePipes then
      if not measureSupplyT then
        connect(port_b1, pipeSupply.port_b);
      end if;
    end if;
    annotation (Documentation(info="<html><p>
  This model is the base circuit implementation of a simple balancing valve. The valve is modelled using the <a href=\"modelica://Buildings.Fluid.Actuators.Valves.TwoWayLinear\">Buildings.Fluid.Actuators.Valves.TwoWayLinear</a> model with a constant opening of 1. <p>The valve is characterized by a fixed Kv value.</p></html>",  revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"),   Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics));
  end BalancingValve;

  model CollectorUnit "Collector unit"
    replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
      "Medium in the component"
      annotation (__Dymola_choicesAllMatching=true);
    //Extensions
    extends Buildings.Fluid.Interfaces.PartialFourPort(
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      final allowFlowReversal1 = allowFlowReversal,
      final allowFlowReversal2 = allowFlowReversal);
    extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations;
    parameter Boolean allowFlowReversal=true
      "= true to allow flow reversal, false restricts to design direction (port_a -> port_b)"
      annotation(Dialog(tab="Assumptions"));
    parameter Modelica.SIunits.MassFlowRate m_flow_nominal
      "Nominal mass flow rate"
      annotation(Dialog(group = "Nominal condition"));
    parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
      "Small mass flow rate for regularization of zero flow";
    Modelica.Fluid.Interfaces.FluidPort_a port_a3(
                       redeclare final package Medium = Medium,
                       m_flow(min=if allowFlowReversal1 then -Modelica.Constants.inf else 0),
                       h_outflow(start=Medium.h_default))
      "Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)"
      annotation (Placement(transformation(extent={{50,-114},{70,-94}},
              rotation=0), iconTransformation(extent={{50,-114},{70,-94}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b3(
                       redeclare final package Medium = Medium,
                       m_flow(max=if allowFlowReversal2 then +Modelica.Constants.inf else 0),
                       h_outflow(start=Medium.h_default))
      "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)"
      annotation (Placement(transformation(extent={{-50,90},{-70,110}},
                            rotation=0),
                  iconTransformation(extent={{-50,90},{-70,110}})));
    Buildings.Fluid.FixedResistances.Junction spl_supply(
      redeclare package Medium = Medium,
      m_flow_nominal={m_flow_nominal,-m_flow_nominal,-m_flow_nominal},
      dp_nominal={0,0,0},
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
      annotation (Placement(transformation(extent={{-70,70},{-50,50}})));
    Buildings.Fluid.FixedResistances.Junction spl_return(
      redeclare package Medium = Medium,
      m_flow_nominal={-m_flow_nominal,m_flow_nominal,m_flow_nominal},
      dp_nominal={0,0,0},
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial)
      annotation (Placement(transformation(extent={{50,-50},{70,-70}})));
  equation
    connect(port_a1, spl_supply.port_1) annotation (Line(
        points={{-100,60},{-70,60}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(port_b3, spl_supply.port_3) annotation (Line(
        points={{-60,100},{-60,70}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(spl_supply.port_2, port_b1) annotation (Line(
        points={{-50,60},{100,60}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(port_b2, spl_return.port_1) annotation (Line(
        points={{-100,-60},{50,-60}},
        color={0,127,255},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(spl_return.port_2, port_a2) annotation (Line(
        points={{70,-60},{100,-60}},
        color={0,127,255},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(port_a3, spl_return.port_3) annotation (Line(
        points={{60,-104},{60,-50}},
        color={0,127,255},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}),
          graphics={
          Line(
            points={{-100,-60},{100,-60}},
            color={0,128,255}),
          Line(
            points={{-100,60},{-60,60},{-60,100}},
            color={255,85,85}),
          Line(
            points={{-60,60},{100,60}},
            color={255,85,85}),
          Line(
            points={{60,-70},{60,-104}},
            color={0,128,255}),
          Polygon(
            points={{-80,68},{-68,68},{-68,80},{-52,80},{-52,68},{-20,68},{-20,52},
                {-80,52},{-80,68}},
            lineColor={255,85,85},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{80,-68},{68,-68},{68,-80},{52,-80},{52,-68},{20,-68},{20,-52},
                {80,-52},{80,-68}},
            lineColor={0,128,255},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid)}));
  end CollectorUnit;

  model FlowController
    //Extensions
    extends Interfaces.PartialValveCircuit(
      redeclare Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage flowRegulator,
      final measurePower=false);
  equation
    connect(u, flowRegulator.y) annotation (Line(
        points={{0,108},{0,72}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (
    Documentation(info="<html><p>
  This model is the base circuit implementation of a combination of a regulation and balancing valve to control a flow in a pressurizeµd hydraulic circuit. The regulation valve is an equal-percentage opening valve and is modelled using the <a href=\"modelica://Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage\">Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage</a> model with a variable opening to control the flow. 
  <p>The balancing valve is characterized by a fixed Kv value which can be adjusted to obtain the desired flow through the circuit dependent on the pressure head of the circuit.</p></html>"),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
              100}}),            graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Line(
            points={{0,102},{6,80},{0,60}},
            color={0,255,128},
            smooth=Smooth.None)}));
  end FlowController;

  model FluidCollector "Collects m fluid flows"
    replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
      "Medium in the component"
      annotation (__Dymola_choicesAllMatching=true);
    parameter Integer m(min=1)=3 "Number of collected heat flows";
    FBM.ElementaryBlocs.BaseClasses.FlowPort_a port_a[m](redeclare final
        package Medium = Medium)
      annotation (Placement(transformation(extent={{-10,110},{10,90}})));
    FBM.ElementaryBlocs.BaseClasses.FlowPort_b port_b(redeclare final package
        Medium = Medium)
      annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
  equation
    for i in 1:m loop
      connect(port_a[i], port_b)
      annotation (Line(points={{0,100},{0,-100},{0,-100}},     color={0,128,255}));
    end for;
    annotation (        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
              100,100}}), graphics={
          Text(
            extent={{-150,-30},{150,-70}},
            textString="%name",
            lineColor={0,0,255}),
          Text(
            extent={{-150,80},{150,50}},
            lineColor={0,0,0},
            textString="m=%m"),
          Line(
            points={{0,90},{0,40}},
            color={0,128,255}),
          Rectangle(
            extent={{-60,40},{60,30}},
            lineColor={0,128,255},
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-60,30},{0,-30},{0,-90}},
            color={0,128,255}),
          Line(
            points={{0,-30},{-20,30}},
            color={0,128,255}),
          Line(
            points={{0,-30},{20,30}},
            color={0,128,255}),
          Line(
            points={{0,-30},{60,30}},
            color={0,128,255})}),
      Documentation(info="<html>
<p>
This is a model to collect the heat flows from <i>m</i> heatports to one single heatport.
</p>
</html>"));
  end FluidCollector;

  model MixingCircuit_EqualPercentage
    "Mixing circuit with equal percentage three way valve"
    extends Interfaces.PartialMixingCircuit(redeclare
        Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear partialThreeWayValve(final R=R,
          final delta0=delta0));
    parameter Real R=50 "Rangeability, R=50...100 typically";
    parameter Real delta0=0.01
      "Range of significant deviation from equal percentage law";
  equation
    connect(y, partialThreeWayValve.y)
      annotation (Line(points={{0,104},{0,72},{0,72}}, color={0,0,127}));
  end MixingCircuit_EqualPercentage;

  model MixingCircuit_Linear "Mixing circuit with linear three way valve"
    extends Interfaces.PartialMixingCircuit(redeclare
        Buildings.Fluid.Actuators.Valves.ThreeWayLinear partialThreeWayValve);
  equation
    connect(y, partialThreeWayValve.y)
      annotation (Line(points={{0,104},{0,72},{0,72}}, color={0,0,127}));
  end MixingCircuit_Linear;

  model MixingCircuit_Tset
    "Mixing circuit with outlet temperature setpoint - assuming ideal mixing without pressure simulation"


    //Extensions
    extends FBM.ElementaryBlocs.Interfaces.PartialCircuitBalancingValve(measureSupplyT=true);


    Buildings.Fluid.Actuators.Valves.ThreeWayLinear
      thermostatic3WayValve(
      m_flow_nominal=m_flow_nominal,
      redeclare package Medium = Medium,
      CvData=Buildings.Fluid.Types.CvTypes.Kv,
      Kv=50)
      annotation (Placement(transformation(extent={{-10,50},{10,70}})));

    Modelica.Blocks.Interfaces.RealInput TMixedSet
      "Setpoint for the supply temperature"                                              annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={0,104}),   iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={0,100})));
  Modelica.Blocks.Continuous.LimPID conPID(
        Ti=600,
      yMax=1,
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      yMin=0)   annotation (Placement(transformation(extent={{12,78},{32,98}})));
  equation

    if not measureSupplyT then
      connect(thermostatic3WayValve.port_2, port_b1);
    end if;

    if not includePipes then
      connect(thermostatic3WayValve.port_1, port_a1);
    end if;

    connect(thermostatic3WayValve.port_1, pipeSupply.port_b) annotation (Line(
        points={{-10,60},{-70,60}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(thermostatic3WayValve.port_2, senTemSup.port_a) annotation (Line(
        points={{10,60},{60,60}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(thermostatic3WayValve.port_3, port_a2) annotation (Line(
        points={{0,50},{0,-2},{100,-2},{100,-60}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(TMixedSet, conPID.u_s) annotation (Line(points={{0,104},{0,104},{0,88},
            {10,88}}, color={0,0,127}));
    connect(conPID.y, thermostatic3WayValve.y) annotation (Line(points={{33,88},{34,
            88},{34,72},{0,72}}, color={0,0,127}));
    connect(conPID.u_m, senTemSup.T) annotation (Line(points={{22,76},{22,76},{22,
            71},{70,71}}, color={0,0,127}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}})),           Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Polygon(
            points={{-20,70},{-20,50},{0,60},{-20,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{20,70},{20,50},{0,60},{20,70}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{0,40},{0,60}},
            color={0,0,127},
            smooth=Smooth.None),
          Line(
            points={{0,100},{-6,80},{0,60}},
            color={0,255,128},
            smooth=Smooth.None),
          Polygon(
            points={{-10,10},{-10,-10},{10,0},{-10,10}},
            lineColor={0,0,127},
            smooth=Smooth.None,
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            origin={0,50},
            rotation=90),
          Line(
            points={{0,40},{0,0},{20,-40},{60,-60}},
            color={0,127,255},
            pattern=LinePattern.Dash)}));
  end MixingCircuit_Tset;

  model PumpSupply
    //Parameters
    parameter Boolean filteredSpeed=true
      "= true, if speed is filtered with a 2nd order CriticalDamping filter"
      annotation(Dialog(tab="Dynamics", group="Filtered speed"));
    parameter Modelica.SIunits.Time riseTime=30
      "Rise time of the filter (time to reach 99.6 % of the speed)"
      annotation(Dialog(tab="Dynamics", group="Filtered speed",enable=filteredSpeed));
    parameter Modelica.Blocks.Types.Init init=Modelica.Blocks.Types.Init.InitialOutput
      "Type of initialization (no init/steady state/initial state/initial output)"
      annotation(Dialog(tab="Dynamics", group="Filtered speed",enable=filteredSpeed));
    //Extensions
    extends Interfaces.PartialPumpCircuit(redeclare
        Buildings.Fluid.Movers.FlowControlled_m_flow
        flowRegulator(
        filteredSpeed=filteredSpeed,
        riseTime=riseTime,
        init=init,
        massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        allowFlowReversal=true,
        nominalValuesDefineDefaultPressureCurve=true,
        addPowerToMedium=true),                   final useBalancingValve=true,
      balancingValve(show_T=true),
      booleanInput = true,
      realInput = false);
    Modelica.Blocks.Math.BooleanToReal booleanToReal(realTrue=m_flow_nominal) if booleanInput
      annotation (Placement(transformation(extent={{-34,68},{-14,88}})));
  equation
    connect(flowRegulator.P, power) annotation (Line(
        points={{11,68},{40,68},{40,108}},
        color={0,0,127},
        smooth=Smooth.None));

         if booleanInput then
       connect(booleanToReal.y, flowRegulator.m_flow_in)
      annotation (Line(points={{-13,78},{-0.2,78},{-0.2,72}}, color={0,0,127}));
    connect(booleanToReal.u, u2) annotation (Line(points={{-36,78},{-40,78},{
            -40,108},{0,108}},  color={255,0,255}));
         else
           connect(u, flowRegulator.m_flow_in)
      annotation (Line(points={{0,108},{-0.2,108},{-0.2,72}}, color={0,0,127}));

         end if;
    annotation (Documentation(info="<html><p>
            This model is the base circuit implementation of a pressure head controlled pump and makes use of <a href=\"modelica://Buildings.Fluid.Movers.FlowControlled_dp\">Buildings.Fluid.Movers.FlowControlled_dp</a>. The flow can be regulated by changing the Kv value of the balancing valve.
            </p><p>Note that an hydronic optimization might be necessary to obtain a meaningfull value for the Kv parameter.</p></html>"),   Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}})),
          Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Polygon(
            points={{-10,10},{-10,-22},{22,-6},{-10,10}},
            lineColor={0,0,255},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            origin={-2,66},
            rotation=360),
          Text(
            extent={{-10,70},{8,50}},
            lineColor={0,0,255},
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            textString="dP")}));
  end PumpSupply;

  model PumpSupply_dp
    //Parameters
    parameter Boolean filteredSpeed=true
      "= true, if speed is filtered with a 2nd order CriticalDamping filter"
      annotation(Dialog(tab="Dynamics", group="Filtered speed"));
    parameter Modelica.SIunits.Time riseTime=30
      "Rise time of the filter (time to reach 99.6 % of the speed)"
      annotation(Dialog(tab="Dynamics", group="Filtered speed",enable=filteredSpeed));
    parameter Modelica.Blocks.Types.Init init=Modelica.Blocks.Types.Init.InitialOutput
      "Type of initialization (no init/steady state/initial state/initial output)"
      annotation(Dialog(tab="Dynamics", group="Filtered speed",enable=filteredSpeed));
    //Extensions
    extends Interfaces.PartialPumpCircuit(redeclare
        Buildings.Fluid.Movers.FlowControlled_dp
        flowRegulator(
        filteredSpeed=filteredSpeed,
        riseTime=riseTime,
        init=init,
        massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        addPowerToMedium=false,
        allowFlowReversal=true),                  final useBalancingValve=true,
      balancingValve(show_T=true));
  equation
    connect(flowRegulator.P, power) annotation (Line(
        points={{11,68},{40,68},{40,108}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(u, flowRegulator.dp_in) annotation (Line(
        points={{0,108},{0,72},{-0.2,72}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation (Documentation(info="<html><p>
            This model is the base circuit implementation of a pressure head controlled pump and makes use of <a href=\"modelica://Buildings.Fluid.Movers.FlowControlled_dp\">Buildings.Fluid.Movers.FlowControlled_dp</a>. The flow can be regulated by changing the Kv value of the balancing valve.
            </p><p>Note that an hydronic optimization might be necessary to obtain a meaningfull value for the Kv parameter.</p></html>"),   Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics),
          Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Polygon(
            points={{-10,10},{-10,-22},{22,-6},{-10,10}},
            lineColor={0,0,255},
            smooth=Smooth.None,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            origin={-2,66},
            rotation=360),
          Text(
            extent={{-10,70},{8,50}},
            lineColor={0,0,255},
            fillColor={0,128,255},
            fillPattern=FillPattern.Solid,
            textString="dP")}));
  end PumpSupply_dp;

  model PumpSupply_m_flow "Pump on supply duct"
    //Parameters
    parameter Boolean filteredSpeed=true
      "= true, if speed is filtered with a 2nd order CriticalDamping filter"
      annotation(Dialog(tab="Dynamics", group="Filtered speed"));
    parameter Modelica.SIunits.Time riseTime=30
      "Rise time of the filter (time to reach 99.6 % of the speed)"
      annotation(Dialog(tab="Dynamics", group="Filtered speed",enable=filteredSpeed));
    parameter Modelica.Blocks.Types.Init init=Modelica.Blocks.Types.Init.InitialOutput
      "Type of initialization (no init/steady state/initial state/initial output)"
      annotation(Dialog(tab="Dynamics", group="Filtered speed",enable=filteredSpeed));
    //Extensions
    extends Interfaces.PartialPumpCircuit(redeclare
        Buildings.Fluid.Movers.FlowControlled_m_flow
        flowRegulator(
          filteredSpeed=filteredSpeed,
          riseTime=riseTime,
          init=init));
  equation
    connect(u, flowRegulator.m_flow_in) annotation (Line(
        points={{0,108},{0,68},{-0.2,68},{-0.2,72}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(flowRegulator.P, power) annotation (Line(
        points={{11,68},{40,68},{40,108}},
        color={0,0,127},
        smooth=Smooth.None));
      annotation(Dialog(group="Pump parameters"),
                Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}})),           Documentation(
              info="<html><p>
            This model is the base circuit implementation of a mass-flow controlled pump and makes use of <a href=\"modelica://Buildings.Fluid.Movers.FlowControlled_m_flow\">Buildings.Fluid.Movers.FlowControlled_m_flow</a>.
</p></html>"),
      Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
          graphics));
  end PumpSupply_m_flow;

  model ThermostaticRadiatorValve
    "Simple TRV model approximated by a P-control action"
    //Extensions
    extends Interfaces.PartialValveCircuit(
      redeclare Buildings.Fluid.Actuators.Valves.TwoWayQuickOpening flowRegulator(
        allowFlowReversal=false,
        show_T=false,
        filteredOpening=true));
    //Parameters
    parameter Real K = 1 "Gain of the TRV";
    //Components
    Modelica.Blocks.Interfaces.RealInput u1 "measurement signal"
      annotation (Placement(transformation(
          extent={{-20,-20},{20,20}},
          rotation=270,
          origin={-40,110})));
    Modelica.Blocks.Continuous.LimPID PID(controllerType=Modelica.Blocks.Types.SimpleController.P,
      k=K,
      yMax=1,
      yMin=0) annotation (Placement(transformation(extent={{20,72},{40,92}})));
  equation
    connect(PID.u_m, u1) annotation (Line(
        points={{30,70},{30,66},{-40,66},{-40,110}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(PID.y, flowRegulator.y) annotation (Line(
        points={{41,82},{44,82},{44,62},{18,62},{18,74},{0,74},{0,72}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(u, PID.u_s) annotation (Line(
        points={{0,108},{0,82},{18,82}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(flowRegulator.y_actual, power) annotation (Line(
        points={{5,67},{40,67},{40,108}},
        color={0,0,127},
        smooth=Smooth.None));
    annotation(Dialog(group = "Thermostatic valve parameters",
                      enable = (CvData==IDEAS.Fluid.Types.CvTypes.Kv)),
                Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}), graphics), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
          Line(
            points={{0,100},{6,80},{0,60}},
            color={0,255,128},
            smooth=Smooth.None),
          Text(
            extent={{4,44},{24,24}},
            lineColor={0,0,127},
            fillColor={95,95,95},
            fillPattern=FillPattern.Solid,
            textString="T"),
          Line(
            points={{-40,100},{-16,86},{0,60}},
            color={255,0,0},
            smooth=Smooth.None)}));
  end ThermostaticRadiatorValve;

  package Interfaces
      extends Modelica.Icons.InterfacesPackage;
    partial model CircuitInterface "Partial circuit for base circuits"
      replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
        "Medium in the component"
        annotation (__Dymola_choicesAllMatching=true);
      //Extensions
      extends Buildings.Fluid.Interfaces.PartialFourPort(
        redeclare package Medium1 = Medium,
        redeclare package Medium2 = Medium,
        final allowFlowReversal1 = allowFlowReversal,
        final allowFlowReversal2 = allowFlowReversal);
      extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations(
        massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState);
      //Parameters
      parameter Integer tauTSensor = 120 "Time constant of the temperature sensors";
      //----Settings
      parameter Boolean includePipes=false
        "Set to true to include pipes in the basecircuit"
        annotation(Dialog(group = "Settings"));
      parameter Boolean measureSupplyT=false
        "Set to true to measure the supply temperature"
        annotation(Dialog(group = "Settings"));
      parameter Boolean measureReturnT=false
        "Set to true to measure the return temperature"
        annotation(Dialog(group = "Settings"));
      //----if includePipes
      parameter Modelica.SIunits.Thickness InsuPipeThickness=0.01
                                                                 "Thickness of the pipe insulation";
      parameter Modelica.SIunits.Length Pipelength=5
                                                    "pipe length of the supply OR return branch";
      parameter Modelica.SIunits.Pressure dp=0 "Pressure drop over a single pipe"
        annotation(Dialog(group = "Pipes",
                         enable = includePipes));
      parameter Modelica.SIunits.ThermalConductivity InsuHeatCondu=0.04;
      //----Fluid parameters
      parameter Boolean dynamicBalance=true
        "Set to true to use a dynamic balance, which often leads to smaller systems of equations"
        annotation(Dialog(tab="Dynamics", group="Equations"));
      parameter Boolean allowFlowReversal=true
        "= true to allow flow reversal, false restricts to design direction (port_a -> port_b)"
        annotation(Dialog(tab="Assumptions"));
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal
        "Nominal mass flow rate"
        annotation(Dialog(group = "Nominal condition"));
      parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
        "Small mass flow rate for regularization of zero flow";
      // Components ----------------------------------------------------------------
    public
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort if includePipes
        annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
      Modelica.Blocks.Interfaces.RealOutput Tsup if measureSupplyT
        "Supply temperature" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={70,108}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={76,104})));
      Modelica.Blocks.Interfaces.RealOutput Tret if measureReturnT
        "Return temperature" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-70,-108}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-76,-104})));
    protected
      Buildings.Fluid.FixedResistances.Pipe pipeSupply(
        dp_nominal=dp,
        energyDynamics=energyDynamics,
        m_flow_nominal=m_flow_nominal,
        redeclare package Medium = Medium,
        massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        thicknessIns=InsuPipeThickness,
        lambdaIns=InsuHeatCondu,
        nSeg=3,
        length=Pipelength) if                                      includePipes
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=180,
            origin={-80,60})), choicesAllMatching=true);
      Buildings.Fluid.FixedResistances.Pipe pipeReturn(
        dp_nominal=dp,
        energyDynamics=energyDynamics,
        massDynamics=massDynamics,
        m_flow_nominal=m_flow_nominal,
        redeclare package Medium = Medium,
        thicknessIns=InsuPipeThickness,
        lambdaIns=InsuHeatCondu,
        nSeg=3,
        length=Pipelength) if                 includePipes annotation (Placement(
            transformation(
            extent={{10,-10},{-10,10}},
            rotation=0,
            origin={-40,-60})), choicesAllMatching=true);
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemSup(
        m_flow_nominal=m_flow_nominal,
        tau=tauTSensor,
        redeclare package Medium = Medium) if measureSupplyT
        annotation (Placement(transformation(extent={{60,50},{80,70}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemRet(
        m_flow_nominal=m_flow_nominal,
        tau=tauTSensor,
        redeclare package Medium = Medium) if measureReturnT
        annotation (Placement(transformation(extent={{-60,-50},{-80,-70}})));
    equation
      connect(port_a1, pipeSupply.port_a) annotation (Line(
          points={{-100,60},{-90,60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeSupply.heatPort, heatPort) annotation (Line(
          points={{-80,55},{-80,-40},{-24,-40},{-24,-100},{0,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(pipeReturn.heatPort, heatPort) annotation (Line(
          points={{-40,-55},{-40,-40},{-24,-40},{-24,-100},{0,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(port_b1, senTemSup.port_b) annotation (Line(
          points={{100,60},{80,60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(Tsup, Tsup) annotation (Line(
          points={{70,108},{70,108}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(senTemSup.T, Tsup) annotation (Line(
          points={{70,71},{70,108}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(port_b2, senTemRet.port_b) annotation (Line(
          points={{-100,-60},{-80,-60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemRet.port_a, pipeReturn.port_b) annotation (Line(
          points={{-60,-60},{-50,-60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemRet.T, Tret) annotation (Line(
          points={{-70,-71},{-70,-108}},
          color={0,0,127},
          smooth=Smooth.None));
        annotation (Placement(transformation(extent={{60,10},{80,30}})),
                  Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                {100,100}}), graphics={
            Rectangle(extent={{-100,100},{100,-100}}, lineColor={135,135,135}),
                                   Line(
              points={{-100,-60},{100,-60}},
              color={0,127,255},
              thickness=0.5),          Line(
              points={{-100,60},{100,60}},
              color={255,85,85},
              thickness=0.5),
            Rectangle(
              extent={{-100,-50},{-80,-70}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              visible=includePipes),
            Rectangle(
              extent={{-10,10},{10,-10}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              origin={90,60},
              rotation=180,
              visible=includePipes),
            Rectangle(
              extent={{-10,10},{10,-10}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              origin={90,-60},
              rotation=180,
              visible=includePipes),
            Rectangle(
              extent={{-100,70},{-80,50}},
              lineColor={0,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              visible=includePipes),
            Polygon(
              points={{-80,70},{-80,50},{-72,60},{-80,70}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              visible=includePipes),
            Polygon(
              points={{-80,-50},{-80,-70},{-72,-60},{-80,-50}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              visible=includePipes),
            Polygon(
              points={{-4,10},{-4,-10},{4,0},{-4,10}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              origin={76,60},
              rotation=180,
              visible=includePipes),
            Polygon(
              points={{80,-50},{80,-70},{72,-60},{80,-50}},
              lineColor={0,0,0},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              visible=includePipes)}),
                                     Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}})),Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end CircuitInterface;

    partial model PartialBaseCircuit "Partial for a mixing circuit"
      // Extensions ----------------------------------------------------------------
      extends CircuitInterface;
    equation
      if includePipes then
        if not measureReturnT then
          connect(pipeReturn.port_b, port_b2);
        end if;
      end if;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
            Line(
              points={{74,100},{80,80},{78,60}},
              color={255,0,0},
              smooth=Smooth.None,
              visible=measureSupplyT),
            Ellipse(
              extent={{76,62},{80,58}},
              lineColor={255,0,0},
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid,
              visible=measureSupplyT),
            Line(
              points={{-3,20},{3,0},{1,-20}},
              color={255,0,0},
              smooth=Smooth.None,
              origin={-77,-80},
              rotation=180,
              visible=measureReturnT),
            Ellipse(
              extent={{-80,-58},{-76,-62}},
              lineColor={255,0,0},
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid,
              visible=measureReturnT)}),Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end PartialBaseCircuit;

    partial model PartialCircuitBalancingValve
      // Extensions ----------------------------------------------------------------
      extends ValveParametersReturn;
      extends PartialBaseCircuit( pipeReturn(dp_nominal=0));
      // Parameter -----------------------------------------------------------------
      parameter Boolean useBalancingValve=false
        "Set to true to include a balancing valve"
        annotation(Dialog(group = "Settings"));
      // Components ----------------------------------------------------------------
    //protected
      Buildings.Fluid.FixedResistances.PressureDrop balancingValve(
        final deltaM=deltaMReturn,
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        dp_nominal=m_flow_nominal^2/KvReturn^2*1e5,
        allowFlowReversal=false) if useBalancingValve
        annotation (Placement(transformation(extent={{10,-70},{-10,-50}})));
    equation
      if not useBalancingValve then
        if includePipes then
          connect(pipeReturn.port_a, port_a2);
        else
          if measureReturnT then
            connect(senTemRet.port_a, port_a2);
          else
            connect(port_b2, port_a2);
          end if;
        end if;
      else
        if not includePipes then
          if measureReturnT then
            connect(senTemRet.port_a, balancingValve.port_b);
          end if;
        end if;
      end if;
      if not measureReturnT then
        if includePipes then
          connect(pipeReturn.port_b, port_b2);
        else
          if useBalancingValve then
            connect(balancingValve.port_b, port_b2);
          end if;
        end if;
      end if;
      connect(balancingValve.port_b, pipeReturn.port_a) annotation (Line(
          points={{-10,-60},{-30,-60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(port_a2, balancingValve.port_a) annotation (Line(
          points={{100,-60},{10,-60}},
          color={0,127,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(graphics={
            Polygon(
              points={{-20,-50},{-20,-70},{0,-60},{-20,-50}},
              lineColor={0,0,127},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              visible=useBalancingValve),
            Polygon(
              points={{20,-50},{20,-70},{0,-60},{20,-50}},
              lineColor={0,0,127},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid,
              visible=useBalancingValve),
            Line(
              points={{0,-60},{0,-40}},
              color={0,0,127},
              smooth=Smooth.None,
              visible=useBalancingValve),
            Line(
              points={{-10,-40},{10,-40}},
              color={0,0,127},
              smooth=Smooth.None,
              visible=useBalancingValve)}),Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end PartialCircuitBalancingValve;

    model PartialFlowCircuit
      // Extensions ----------------------------------------------------------------
      extends PartialCircuitBalancingValve;
      // Parameters ----------------------------------------------------------------
      parameter Boolean measurePower=true
        "Set to false to remove the power consumption measurement of the flow regulator"
        annotation(Dialog(group = "Settings"));
      parameter Boolean realInput = true;
      parameter Boolean booleanInput = false;
      // Components ----------------------------------------------------------------
    protected
      replaceable Buildings.Fluid.Interfaces.PartialTwoPortInterface flowRegulator(
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal)
        annotation (Placement(transformation(extent={{-10,50},{10,70}})));
    public
      Modelica.Blocks.Interfaces.RealInput u if realInput
        "Setpoint of the flow regulator"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,108})));
      Modelica.Blocks.Interfaces.RealOutput power if measurePower
        "Power consumption of the flow regulator"
                                                 annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={40,108}), iconTransformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={42,104})));
      Modelica.Blocks.Interfaces.BooleanInput u2 if booleanInput
        "Setpoint of the flow regulator"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,108})));
    equation
      if not includePipes then
        connect(flowRegulator.port_a, port_a1);
      end if;
      if not measureSupplyT then
        connect(flowRegulator.port_b, port_b1);
      end if;
      connect(flowRegulator.port_b, senTemSup.port_a) annotation (Line(
          points={{10,60},{60,60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeSupply.port_b, flowRegulator.port_a) annotation (Line(
          points={{-70,60},{-10,60}},
          color={0,127,255},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}})),           Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                                                   graphics={
            Line(
              points={{42,100},{48,80},{46,60}},
              color={255,0,0},
              smooth=Smooth.None,
              visible=measurePower),
            Ellipse(
              extent={{44,62},{48,58}},
              lineColor={255,0,0},
              fillColor={255,0,0},
              fillPattern=FillPattern.Solid,
              visible=measurePower)}),Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end PartialFlowCircuit;

    model PartialMixingCircuit "Partial for a circuit containing a three way valve"
      // Extensions ----------------------------------------------------------------
      extends ValveParametersSupply(
        rhoStdSupply=Medium.density_pTX(101325, 273.15+4, Medium.X_default));
      extends FBM.ElementaryBlocs.Interfaces.PartialCircuitBalancingValve;
      // Parameters ----------------------------------------------------------------
      parameter Real fraKSupply(min=0, max=1) = 0.7
        "Fraction Kv(port_3->port_2)/Kv(port_1->port_2)";
      parameter Real[2] lSupply(each min=0, each max=1) = {0.01, 0.01}
        "Valve leakage, l=Kv(y=0)/Kv(y=1)";
      // Components ----------------------------------------------------------------
    protected
      replaceable Buildings.Fluid.Actuators.BaseClasses.PartialThreeWayValve partialThreeWayValve
      constrainedby Buildings.Fluid.Actuators.BaseClasses.PartialThreeWayValve(
        redeclare package Medium = Medium,
        energyDynamics=energyDynamics,
        massDynamics=massDynamics,
        final CvData=Buildings.Fluid.Types.CvTypes.Kv,
        final Kv=KvSupply,
        final deltaM=deltaMSupply,
        final m_flow_nominal=m_flow_nominal,
        final fraK=fraKSupply,
        final l=lSupply)
        annotation (Placement(transformation(extent={{-10,50},{10,70}})));
    public
             Modelica.Blocks.Interfaces.RealInput y "Three way valve position setpoint"
        annotation (Placement(transformation(
            extent={{-20,-20},{20,20}},
            rotation=270,
            origin={0,104})));

    equation
      if not measureSupplyT then
        connect(partialThreeWayValve.port_2, port_b1);
      end if;
      connect(partialThreeWayValve.port_2, senTemSup.port_a) annotation (Line(
          points={{10,60},{60,60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeSupply.port_b, partialThreeWayValve.port_1) annotation (Line(
          points={{-70,60},{-10,60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(partialThreeWayValve.port_3, balancingValve.port_a) annotation (
          Line(points={{0,50},{20,50},{20,-60},{10,-60}}, color={0,127,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}),
                             graphics={
            Polygon(
              points={{-20,70},{-20,50},{0,60},{-20,70}},
              lineColor={0,0,127},
              fillColor={0,127,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{20,70},{20,50},{0,60},{20,70}},
              lineColor={0,0,127},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{0,40},{0,60}},
              color={0,0,127},
              smooth=Smooth.None),
            Line(
              points={{0,100},{-6,80},{0,60}},
              color={0,255,128},
              smooth=Smooth.None),
            Polygon(
              points={{-10,10},{-10,-10},{10,0},{-10,10}},
              lineColor={0,0,127},
              fillColor={0,127,255},
              fillPattern=FillPattern.Solid,
              origin={0,50},
              rotation=90),
            Line(
              points={{0,40},{0,0},{20,-40},{60,-60}},
              color={0,127,255},
              pattern=LinePattern.Dash)}),
                                     Diagram(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}})),          Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end PartialMixingCircuit;

    model PartialPumpCircuit
      // Extensions ----------------------------------------------------------------
      extends PartialFlowCircuit(redeclare
          Buildings.Fluid.Movers.BaseClasses.PartialFlowMachine flowRegulator(
          tau=tauPump,
          energyDynamics=energyDynamics,
          massDynamics=massDynamics,
          addPowerToMedium=addPowerToMedium));
      extends PumpParameters;
      annotation (Icon(graphics={
            Ellipse(extent={{-20,80},{20,40}},lineColor={0,0,127},
              fillColor={0,127,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{0,94},{4,80},{0,64}},
              color={0,255,128},
              smooth=Smooth.None),
            Polygon(
              points={{-12,76},{-12,44},{20,60},{-12,76}},
              lineColor={0,0,127},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end PartialPumpCircuit;

    model PartialValveCircuit
      // Extensions ----------------------------------------------------------------
      extends ValveParametersSupply(
          rhoStdSupply=Medium.density_pTX(101325, 273.15+4, Medium.X_default));
      extends PartialFlowCircuit(redeclare
          Buildings.Fluid.Actuators.BaseClasses.PartialTwoWayValve flowRegulator(
          Kv=KvSupply,
          rhoStd=rhoStdSupply,
          deltaM=deltaMSupply,
          CvData=Buildings.Fluid.Types.CvTypes.Kv));
    equation
      connect(flowRegulator.y_actual, power) annotation (Line(
          points={{5,67},{40,67},{40,108}},
          color={0,0,127},
          smooth=Smooth.None));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Icon(graphics={
            Polygon(
              points={{-20,70},{-20,50},{0,60},{-20,70}},
              lineColor={0,0,127},
              fillColor={0,127,255},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{20,70},{20,50},{0,60},{20,70}},
              lineColor={0,0,127},
              smooth=Smooth.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Line(
              points={{0,40},{0,60}},
              color={0,0,127},
              smooth=Smooth.None),
            Line(
              points={{0,102},{6,80},{0,60}},
              color={0,255,128},
              smooth=Smooth.None),
            Rectangle(
              extent={{-6,44},{6,32}},
              lineColor={0,0,127},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end PartialValveCircuit;

    partial model PumpParameters
      "Partial circuit for base circuits with pump parameters"
      parameter Integer tauPump = 1
        "Time constant of the pump if dynamicBalance is true"
        annotation(Dialog(
                       group = "Pump parameters"));
      parameter Boolean addPowerToMedium = false "Add heat to the medium"
                                 annotation(Dialog(
                       group = "Pump parameters"));
      parameter Boolean use_powerCharacteristic = false
        "Use powerCharacteristic (vs. efficiencyCharacteristic)" annotation(Dialog(
                       group = "Pump parameters"));
      parameter Boolean motorCooledByFluid = true
        "If true, then motor heat is added to fluid stream" annotation(Dialog(
                       group = "Pump parameters"));
    end PumpParameters;

    model ValveParametersReturn
      parameter Buildings.Fluid.Types.CvTypes CvDataReturn=Buildings.Fluid.Types.CvTypes.Kv
        "Selection of flow coefficient"
       annotation(Dialog(group = "Flow Coefficient Return Valve"));
      parameter Real KvReturn(
        fixed= if CvDataReturn==Buildings.Fluid.Types.CvTypes.Kv then true else false)
        "Kv (metric) flow coefficient [m3/h/(bar)^(1/2)]"
      annotation(Dialog(group = "Flow Coefficient Return Valve",
                        enable = (CvDataReturn==Buildings.Fluid.Types.CvTypes.Kv)));
      parameter Real deltaMReturn = 0.02
        "Fraction of nominal flow rate where linearization starts, if y=1"
        annotation(Dialog(group="Pressure-flow linearization"));
    end ValveParametersReturn;

    model ValveParametersSupply
      parameter Buildings.Fluid.Types.CvTypes CvDataSupply=Buildings.Fluid.Types.CvTypes.OpPoint
        "Selection of flow coefficient"
       annotation(Dialog(group = "Flow Coefficient Supply Valve"));
      parameter Real KvSupply(
        fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.Kv then true else false)
        "Kv (metric) flow coefficient [m3/h/(bar)^(1/2)]"
      annotation(Dialog(group = "Flow Coefficient Supply Valve",
                        enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Kv)));
      parameter Real CvSupply(
        fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.Cv then true else false)
        "Cv (US) flow coefficient [USG/min/(psi)^(1/2)]"
      annotation(Dialog(group = "Flow Coefficient Supply Valve",
                        enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Cv)));
      parameter Modelica.SIunits.Area AvSupply(
        fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.Av then true else false)
        "Av (metric) flow coefficient"
       annotation(Dialog(group = "Flow Coefficient Supply Valve",
                         enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.OpPoint.Av)));
      parameter Real deltaMSupply = 0.02
        "Fraction of nominal flow rate where linearization starts, if y=1"
        annotation(Dialog(group="Pressure-flow linearization"));
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal
        "Nominal mass flow rate"
        annotation(Dialog(group = "Nominal condition"));
      parameter Modelica.SIunits.Pressure dpValve_nominalSupply(displayUnit="Pa",
                                                          min=0,
                                                          fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.OpPoint then true else false)
        "Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint"
        annotation(Dialog(group="Nominal condition",
                   enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      parameter Modelica.SIunits.Density rhoStdSupply
        "Inlet density for which valve coefficients are defined"
      annotation(Dialog(group="Nominal condition", tab="Advanced"));
    protected
      parameter Real Kv_SISupply(
        min=0,
        fixed= false)
        "Flow coefficient for fully open valve in SI units, Kv=m_flow/sqrt(dp) [kg/s/(Pa)^(1/2)]"
      annotation(Dialog(group = "Flow Coefficient Supply Valve",
                        enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
    initial equation
      if  CvDataSupply == Buildings.Fluid.Types.CvTypes.OpPoint then
        Kv_SISupply =           m_flow_nominal/sqrt(dpValve_nominalSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Kv then
        Kv_SISupply =           KvSupply*rhoStdSupply/3600/sqrt(1E5)
          "Unit conversion m3/(h*sqrt(bar)) to kg/(s*sqrt(Pa))";
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (m_flow_nominal/Kv_SISupply)^2;
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Cv then
        Kv_SISupply =           CvSupply*rhoStdSupply*0.0631/1000/sqrt(6895)
          "Unit conversion USG/(min*sqrt(psi)) to kg/(s*sqrt(Pa))";
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (m_flow_nominal/Kv_SISupply)^2;
      else
        assert(CvDataSupply == Buildings.Fluid.Types.CvTypes.Av, "Invalid value for CvData.
Obtained CvData = "     + String(CvDataSupply) + ".");
        Kv_SISupply =           AvSupply*sqrt(rhoStdSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        dpValve_nominalSupply =  (m_flow_nominal/Kv_SISupply)^2;
      end if;
    end ValveParametersSupply;
  end Interfaces;

  package BaseClasses
    extends Modelica.Icons.VariantsPackage;
    connector FluidTwoPort "For automatically connecting supply and return pipes"
      replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
        "Medium model" annotation (choicesAllMatching=true);
      Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium=Medium);
      Modelica.Fluid.Interfaces.FluidPort_a port_b(redeclare package Medium=Medium);
      annotation (Icon(graphics={             Ellipse(
              extent={{-100,50},{0,-50}},
              lineColor={0,0,0},
              fillColor={0,127,255},
              fillPattern=FillPattern.Solid), Ellipse(
              extent={{0,50},{100,-50}},
              lineColor={0,0,0},
              fillColor={0,127,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{8,42},{92,-42}},
              lineColor={0,127,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}), Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end FluidTwoPort;

    partial model FourPort
      parameter Boolean enableFourPort1 = true;
      parameter Boolean enableFourPort2 = true;
      FluidTwoPort fluidTwoPort1(redeclare package Medium = Medium) if not enableFourPort1 annotation (
        Placement(transformation(extent={{100,100},{100,100}}), iconTransformation(
              extent={{-20,80},{20,120}})));
      FluidTwoPort fluidTwoPort2(redeclare package Medium = Medium) if not enableFourPort2
        annotation (Placement(transformation(extent={{100,100},{100,100}}),iconTransformation(extent={{-20,
                -120},{20,-80}})));
      replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
        "Medium in the component"
        annotation (__Dymola_choicesAllMatching=true);
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal
        "Nominal mass flow rate"
        annotation(Dialog(group = "Nominal condition"));
      parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
        "Small mass flow rate for regularization of zero flow"
        annotation(Dialog(tab = "Advanced"));
      Modelica.Fluid.Interfaces.FluidPort_a port_a1 if enableFourPort1
        annotation (Placement(transformation(extent={{100,100},{100,100}}),
            iconTransformation(extent={{-70,90},{-50,110}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_b1 if enableFourPort1
        annotation (Placement(transformation(extent={{100,100},{100,100}}),
            iconTransformation(extent={{50,90},{70,110}})));
      Modelica.Fluid.Interfaces.FluidPort_a port_a2 if enableFourPort2
        annotation (Placement(transformation(extent={{100,100},{100,100}}),
            iconTransformation(extent={{-70,-110},{-50,-90}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_b2 if enableFourPort2
        annotation (Placement(transformation(extent={{100,100},{100,100}}),
            iconTransformation(extent={{50,-110},{70,-90}})));
    protected
      Modelica.Fluid.Interfaces.FluidPort_a port_A1
        annotation (Placement(transformation(extent={{-70,90},{-50,110}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_B1
        annotation (Placement(transformation(extent={{50,90},{70,110}})));
      Modelica.Fluid.Interfaces.FluidPort_a port_A2
        annotation (Placement(transformation(extent={{-70,-110},{-50,-90}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_B2
        annotation (Placement(transformation(extent={{50,-110},{70,-90}})));
    equation
      connect(port_A1, port_a1);
      connect(port_A1, fluidTwoPort1.port_a);
      connect(port_B1, port_b1);
      connect(port_B1, fluidTwoPort1.port_b);
      connect(port_A2, port_a2);
      connect(port_A2, fluidTwoPort2.port_a);
      connect(port_B2, port_b2);
      connect(port_B2, fluidTwoPort2.port_b);
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Documentation(info="<html>
<p>By convention the &apos;a&apos; side of the model and its connectors are used as the supply side (when appropriate).</p>
</html>",     revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
            graphics));
    end FourPort;

    partial model TwoPort
      FluidTwoPort fluidTwoPort_a(redeclare package Medium = Medium) annotation (
          Placement(transformation(extent={{10,90},{-10,110}}), iconTransformation(
              extent={{-10,90},{10,110}})));
      replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
        "Medium in the component"
        annotation (__Dymola_choicesAllMatching=true);
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal
        "Nominal mass flow rate"
        annotation(Dialog(group = "Nominal condition"));
      parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
        "Small mass flow rate for regularization of zero flow"
        annotation(Dialog(tab = "Advanced"));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end TwoPort;

    partial model TwoPortComponent
      extends FBM.ElementaryBlocs.BaseClasses.TwoPort;
      replaceable Buildings.Fluid.Interfaces.PartialTwoPortInterface partialTwoPortInterface
        annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      parameter Boolean inletPortA
        "Set to true to use port A as the fluid inlet port"
        annotation(Dialog(tab="Advanced"));
    equation
      if inletPortA then
        connect(partialTwoPortInterface.port_a, fluidTwoPort_a.port_b) annotation (
          Line(
          points={{-10,0},{-40,0},{-40,100.05},{-0.05,100.05}},
          color={0,127,255},
          smooth=Smooth.None));
        connect(partialTwoPortInterface.port_b, fluidTwoPort_a.port_a) annotation (
          Line(
          points={{10,0},{40,0},{40,100.05},{-0.05,100.05}},
          color={0,127,255},
          smooth=Smooth.None));
      else
        connect(partialTwoPortInterface.port_a, fluidTwoPort_a.port_a) annotation (
          Line(
          points={{-10,0},{-40,0},{-40,100.05},{-0.05,100.05}},
          color={0,127,255},
          smooth=Smooth.None));
        connect(partialTwoPortInterface.port_b, fluidTwoPort_a.port_b) annotation (
          Line(
          points={{10,0},{40,0},{40,100.05},{-0.05,100.05}},
          color={0,127,255},
          smooth=Smooth.None));
      end if;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                -100},{100,100}}), graphics), Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end TwoPortComponent;

    partial model TwoPortSink
      "Component where the inlet of the component is considered to be the supply side"
      extends TwoPortComponent(inletPortA=true);
      annotation (Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end TwoPortSink;

    partial model TwoPortSource
      "Component where the outlet of the component is considered to be the supply side"
      extends TwoPortComponent(inletPortA=false);
      annotation (Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
    end TwoPortSource;

    connector FlowPort_a "Filled flow port (used upstream)"
      extends FBM.ElementaryBlocs.BaseClasses.FlowPort;
      annotation (
        Documentation(info="<HTML>
Same as FlowPort, but icon allows to differentiate direction of flow.
</HTML>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={0,127,255},
              fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Rectangle(
              extent={{-50,50},{50,-50}},
              lineColor={255,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-48,48},{48,-48}},
              lineColor={0,0,255},
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid),
            Text(
              extent={{-100,110},{100,50}},
              lineColor={0,0,255},
              textString="%name")}));
    end FlowPort_a;

    connector FlowPort_b "Hollow flow port (used downstream)"
      extends FBM.ElementaryBlocs.BaseClasses.FlowPort;
      annotation (
        Documentation(info="<HTML>
Same as FlowPort, but icon allows to differentiate direction of flow.
</HTML>"),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Ellipse(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={0,127,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-80,80},{80,-80}},
              lineColor={0,127,255},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid)}),
        Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                100}}), graphics={
            Rectangle(
              extent={{-50,50},{50,-50}},
              lineColor={255,0,0},
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(extent={{-48,48},{48,-48}}, lineColor={0,0,255}),
            Text(
              extent={{-100,110},{100,50}},
              lineColor={0,0,255},
              textString="%name")}));
    end FlowPort_b;

    connector FlowPort "Connector flow port"
      extends Modelica.Fluid.Interfaces.FluidPort;
      annotation (Documentation(info="<HTML>
Basic definition of the connector.<br>
<b>Variables:</b>
<ul>
<li>Pressure p</li>
<li>flow MassFlowRate m_flow</li>
<li>Specific Enthalpy h</li>
<li>flow EnthaplyFlowRate H_flow</li>
</ul>
If ports with different media are connected, the simulation is asserted due to the check of parameter.
</HTML>"));
    end FlowPort;
  end BaseClasses;

  model CollectorUnitParra "collector unit for parallele branch "
    parameter Integer nEle(min=2)=1
                                   "number of splitter in series";
      replaceable package Medium = Modelica.Media.Interfaces.PartialMedium
      "Medium in the component"
      annotation (__Dymola_choicesAllMatching=true);
    //Extensions
    extends Buildings.Fluid.Interfaces.PartialFourPort(
      redeclare package Medium1 = Medium,
      redeclare package Medium2 = Medium,
      final allowFlowReversal1 = allowFlowReversal,
      final allowFlowReversal2 = allowFlowReversal);
    extends Buildings.Fluid.Interfaces.LumpedVolumeDeclarations;
    parameter Boolean allowFlowReversal=true
      "= true to allow flow reversal, false restricts to design direction (port_a -> port_b)"
      annotation(Dialog(tab="Assumptions"));
    parameter Modelica.SIunits.MassFlowRate m_flow_nominal
      "Nominal mass flow rate"
      annotation(Dialog(group = "Nominal condition"));
    parameter Modelica.SIunits.MassFlowRate m_flow_small(min=0) = 1E-4*abs(m_flow_nominal)
      "Small mass flow rate for regularization of zero flow";
    Modelica.Fluid.Interfaces.FluidPort_a[ nEle] port_a3(
                       redeclare final package Medium = Medium,
                       m_flow(min=if allowFlowReversal1 then -Modelica.Constants.inf else 0),
                       h_outflow(start=Medium.h_default))
      "Fluid connector a1 (positive design flow direction is from port_a1 to port_b1)"
      annotation (Placement(transformation(extent={{50,-114},{70,-94}},
              rotation=0), iconTransformation(extent={{50,-114},{70,-94}})));
    Modelica.Fluid.Interfaces.FluidPort_b[nEle] port_b3(
                       redeclare final package Medium = Medium,
                       m_flow(max=if allowFlowReversal2 then +Modelica.Constants.inf else 0),
                       h_outflow(start=Medium.h_default))
      "Fluid connector b2 (positive design flow direction is from port_a2 to port_b2)"
      annotation (Placement(transformation(extent={{-50,90},{-70,110}},
                            rotation=0),
                  iconTransformation(extent={{-50,90},{-70,110}})));
    Buildings.Fluid.FixedResistances.Junction[nEle] spl_supply(
      redeclare package Medium = Medium,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      m_flow_nominal=m_flow_nominal,
      dp_nominal=0)
      annotation (Placement(transformation(extent={{-70,70},{-50,50}})));
    Buildings.Fluid.FixedResistances.Junction[nEle] spl_return(
      redeclare package Medium = Medium,
      massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
      energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      m_flow_nominal=m_flow_nominal,
      dp_nominal=0)
      annotation (Placement(transformation(extent={{50,-50},{70,-70}})));
  equation
    for i in 1:nEle-1 loop
    connect(spl_supply[i].port_2, spl_supply[i+1].port_1)
      annotation (Line(points={{-50,60},{-50,60},{-70,60}}, color={0,127,255}));
   connect(spl_return[i].port_2, spl_return[i+1].port_1)
      annotation (Line(points={{70,-60},{70,-60},{50,-60}}, color={0,127,255}));
    end for;
    for i in 1:nEle loop
      connect(port_b3[i], spl_supply[i].port_3)
      annotation (Line(points={{-60,100},{-60,85},{-60,70}}, color={0,127,255}));
      connect(spl_return[i].port_3, port_a3[i]) annotation (Line(points={{60,-50},
              {60,-104}},     color={0,127,255}));
       end for;
    connect(port_a1, spl_supply[1].port_1)
      annotation (Line(points={{-100,60},{-85,60},{-70,60}}, color={0,127,255}));
    connect(spl_supply[nEle].port_2, port_b1)
      annotation (Line(points={{-50,60},{26,60},{100,60}}, color={0,127,255}));
    connect(port_b2, spl_return[1].port_1) annotation (Line(points={{-100,-60},{-26,
            -60},{50,-60}}, color={0,127,255}));
    connect(spl_return[nEle].port_2, port_a2) annotation (Line(points={{70,-60},{100,
            -60}},           color={0,127,255}));
      annotation (Line(points={{60,-50},{60,-50},{60,-104}}, color={0,127,255}),
                       Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end CollectorUnitParra;
end ElementaryBlocs;
