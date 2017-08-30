within FBM.Components;
package ActiveBeams
  extends Modelica.Icons.VariantsPackage;

  package UsersGuide "User's Guide"
    extends Modelica.Icons.Information;
    annotation (preferredView="info",
    Documentation(info="<html>
<p>
This package contains models of active beams.
Active beams are devices used for heating, cooling and ventilation of spaces.
A schematic diagram of an active beam unit is given below.
</p>
<p align=\"center\" >
<img alt=\"image\" src=\"modelica://IDEAS/Resources/Images/Fluid/HeatExchangers/ActiveBeams/schematicAB.png\" border=\"1\"/>
</p>
<p>
The active beam unit consists of a primary air plenum, a mixing chamber, a heat exchanger (coil) and several nozzles.
Typically, an air-handling unit supplies primary air to the active beams.
The primary air is discharged to the mixing chamber through the nozzles.
This generates a low-pressure region which induces air from the room up through the heat exchanger,
where hot or cold water is circulating.
The conditioned induced air is then mixed with primary air, and the mixture descents back to the space.
</p>
<p>
This package contains two models. The model
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.Cooling\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.Cooling</a>
is for cooling only, while the model
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating</a>
has two water streams, one for heating and one for cooling.
</p>

<h4>Model equations for cooling</h4>
<p>
The performance of the model
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.Cooling\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.Cooling</a>
is computed based on manufacturer data
specified in the package
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.Data\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.Data</a>.
</p>
<p>
For off-design conditions, the performance is adjusted using modification factors
that account for changes in water flow rate,
primary air flow rate and temperature difference.
The total heat flow rate of the active beam unit is the sum of the heat flow rate provided by the primary air supply
<i>Q<sub>sa</sub></i> and the cooling heat flow rate provided by the beam convector <i>Q<sub>c,Beam</sub></i>
which injects room air and mixes it with the primary air.
</p>
<p>
The heat flow rate
<i>Q<sub>sa</sub> </i> is delivered to a thermal zone
through the fluid ports, while the heat flow rate from the convector <i>Q<sub>c,Beam</sub></i>
is coupled directly to the heat port.
See for example
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.Examples.CoolingOnly\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.Examples.CoolingOnly</a>
for how to connect these heat flow rates to a control volume.
</p>
<p>
The primary air contribution is
</p>
<p align=\"center\" style=\"font-style:italic;\">
  Q<sub>sa</sub> = &#7745;<sub>sa</sub> c<sub>p,sa</sub> (T<sub>sa</sub>-T<sub>z</sub>)
</p>
<p>
where <i>&#7745;<sub>sa</sub></i> is the primary air mass flow rate,
<i>c<sub>p,sa</sub></i> is the air specific heat capacity,
<i>T<sub>sa</sub></i> is the primary air temperature
and <i>T<sub>z</sub></i> is the zone air temperature.
</p>
<p>
The heat flow rate of the beam convector <i>Q<sub>c,Beam</sub></i> is determined using
the rated capacity which is modified by three separate functions as
</p>
<p align=\"center\" style=\"font-style:italic;\">
  Q<sub>c,Beam</sub> = Q<sub>c,nominal</sub>
f<sub>&#916;T</sub> ( &#916;T<sub>c</sub> &frasl; &#916;T<sub>c,nominal</sub> )
f<sub>sa</sub>( &#7745;<sub>sa</sub> &frasl; &#7745;<sub>sa,nominal</sub> )
f<sub>w</sub>( &#7745;<sub>c,w</sub> ),
</p>
<p>
the modification factors are as follows:
The modification factor <i>f<sub>&#916;T</sub>(&middot;)</i>
describes how the capacity is adjusted to account for the temperature difference
between the zone air and the water entering the convector.
The independent variable is the ratio between the current temperature difference
<i>&#916;T<sub>c</sub></i> and the temperature difference used to rate beam performance <i>&#916;T<sub>c,nominal</sub></i>.
The temperature difference is
</p>
<p align=\"center\" style=\"font-style:italic;\">
    &#916;T<sub>c</sub> = T<sub>cw</sub>-T<sub>z</sub>,
</p>
<p>
where <i>T<sub>cw</sub></i> is the chilled water temperature entering the convector.

The modification factor <i>f<sub>sa</sub>(&middot;)</i> adjusts the cooling capacity to account for varying primary air flow rate.
The independent variable is the ratio between the current primary air flow rate <i>&#7745;<sub>sa</sub></i>
and the nominal air flow rate used to rate the beam performance.

The modification factor <i>f<sub>w</sub>(&middot;)</i> adjusts the cooling capacity for changes in water flow rate through the convector.
The independent variable is the ratio between the current water flow rate <i>&#7745;<sub>w</sub></i>
and the nominal water flow rate used to rate the beam performance.
</p>

<h4>Model equations for heating</h4>
<p>
The performance of the model
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating</a>
is computed identical to the above described model that only provides cooling,
with the exception that this model contains an additional water stream that
can be used to provide heating.
</p>
<p>
For the heating water stream, the temperature difference <i><code>&#916;</code>T<sub>h</sub></i>
used for the calculation of the modification factor <i>f<sub><code>&#916;</code>T</sub>(&middot;)</i> is
</p>
<p align=\"center\" style=\"font-style:italic;\">
&#916;T<sub>h</sub> = T<sub>hw</sub>-T<sub>z</sub>,
</p>
<p>
where <i>T<sub>hw</sub></i> is the hot water temperature entering the convector in heating mode
and <i>T<sub>z</sub></i> is the zone air temperature.
</p>

<h4>Dynamics</h4>
<p>
The model can be configured to be steady-state or dynamic.
If configured as dynamic, then a dynamic conservation equation is applied to the water streams
for heating and for cooling.
However, because the capacity of the beam depends on its inlet temperature, and is independent of the
outlet temperature, the heat transferred
to the room at the port <code>heaPor.Q_flow</code>, as well as the heat added to or removed from the
water streams, will instantaneously change.
The only dynamic responses are the water outlet temperatures, which change with a first
order response, parameterized with the time constant <code>tau</code>.
</p>

<h4>Energy balance</h4>
<p>
All heat flow rate that is added to or extracted from the room is transmitted through the heat port
<code>heaPor</code>. Hence, this model does not cool the supply air between the ports
<code>air_a</code> and <code>air_b</code>. Rather, it adds this heat flow rate
to the heat port <code>heaPor</code>.
The rationale for this implementation is that the beam transfers heat by convection directly to the room, and
by induction of room air into the supply air. As this split of heat flow rate is generally not known,
and because the amount of inducted air is also unknown,
it was decided to transfer all heat through the heat port <code>heaPor</code>.
This also avoids having to add an extra air flow path for the air induced from the room.
</p>
</html>"));
  end UsersGuide;

  model Cooling "Active beam unit for cooling"

    replaceable package MediumWat = Modelica.Media.Interfaces.PartialMedium
      "Medium 1 in the component"
      annotation (choicesAllMatching = true);
    replaceable package MediumAir = Modelica.Media.Interfaces.PartialMedium
      "Medium 2 in the component"
      annotation (choicesAllMatching = true);

    replaceable parameter Data.Generic perCoo "Performance data for cooling"
      annotation (
        Dialog(group="Nominal condition"),
        choicesAllMatching=true,
        Placement(transformation(extent={{102,-98},{118,-82}})));

    parameter Integer nBeams(min=1)=1 "Number of beams in parallel";

    parameter Boolean allowFlowReversalWat=true
      "= true to allow flow reversal in water circuit, false restricts to design direction (port_a -> port_b)"
      annotation(Dialog(tab="Assumptions"), Evaluate=true);
    parameter Boolean allowFlowReversalAir=true
      "= true to allow flow reversal in air circuit, false restricts to design direction (port_a -> port_b)"
      annotation(Dialog(tab="Assumptions"), Evaluate=true);

    parameter Modelica.SIunits.Time tau = 30
      "Time constant at nominal flow (if energyDynamics <> SteadyState)"
       annotation (Dialog(tab = "Dynamics", group="Nominal condition"));

    // Flow resistance
    parameter Boolean from_dpWat = false
      "= true, use m_flow = f(dp) else dp = f(m_flow)"
      annotation (Evaluate=true, Dialog(enable = perCoo.dpAir_nominal > 0,
                  tab="Flow resistance"));
    parameter Boolean linearizeFlowResistanceWat = false
      "= true, use linear relation between m_flow and dp for any flow rate"
      annotation(Dialog(tab="Flow resistance"));
    parameter Real deltaMWat = 0.1
      "Fraction of nominal flow rate where flow transitions to laminar"
      annotation(Dialog(tab="Flow resistance"));
    // Advanced
    parameter Boolean homotopyInitialization = true "= true, use homotopy method"
      annotation(Evaluate=true, Dialog(tab="Advanced"));

    // Dynamics
    parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
      "Type of energy balance: dynamic (3 initialization options) or steady state"
      annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));
    parameter Modelica.Fluid.Types.Dynamics massDynamics=energyDynamics
      "Type of mass balance: dynamic (3 initialization options) or steady state"
      annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));

    // Initialization
    parameter MediumWat.AbsolutePressure pWatCoo_start = MediumWat.p_default
      "Start value of pressure"
      annotation(Dialog(tab = "Initialization", group = "Cooling"));
    parameter MediumWat.Temperature TWatCoo_start = MediumWat.T_default
      "Start value of temperature"
      annotation(Dialog(tab = "Initialization", group = "Cooling"));

    parameter MediumWat.MassFlowRate mWat_flow_small(min=0) = 1E-4*abs(perCoo.mWat_flow_nominal)
      "Small mass flow rate for regularization of zero flow"
      annotation(Dialog(tab = "Advanced"));
    parameter MediumAir.MassFlowRate mAir_flow_small(min=0) = 1E-4*abs(perCoo.mAir_flow_nominal)
      "Small mass flow rate for regularization of zero flow"
      annotation(Dialog(tab = "Advanced"));

    // Diagnostics
    parameter Boolean show_T = false
      "= true, if actual temperature at port is computed"
      annotation(Dialog(tab="Advanced",group="Diagnostics"));

    // Ports
    Modelica.Fluid.Interfaces.FluidPort_a watCoo_a(
      redeclare final package Medium = MediumWat,
      m_flow(min=if allowFlowReversalWat then -Modelica.Constants.inf else 0),
      h_outflow(start=MediumWat.h_default))
      "Fluid connector watCoo_a (positive design flow direction is from watCoo_a to watCoo_b)"
      annotation (Placement(transformation(extent={{-150,50},{-130,70}})));
    Modelica.Fluid.Interfaces.FluidPort_b watCoo_b(
      redeclare final package Medium = MediumWat,
      m_flow(max=if allowFlowReversalWat then +Modelica.Constants.inf else 0),
      h_outflow(start=MediumWat.h_default))
      "Fluid connector watCoo_b (positive design flow direction is from watCoo_a to watCoo_b)"
      annotation (Placement(transformation(extent={{150,50},{130,70}})));

    Modelica.Fluid.Interfaces.FluidPort_a air_a(
      redeclare final package Medium = MediumAir,
      m_flow(min=if allowFlowReversalAir then -Modelica.Constants.inf else 0),
      h_outflow(start=MediumAir.h_default))
      "Fluid connector air_a (positive design flow direction is from air_a to air_b)"
      annotation (Placement(transformation(extent={{130,-70},{150,-50}})));
    Modelica.Fluid.Interfaces.FluidPort_b air_b(
      redeclare final package Medium = MediumAir,
      m_flow(max=if allowFlowReversalAir then +Modelica.Constants.inf else 0),
      h_outflow(start=MediumAir.h_default))
      "Fluid connector air_b (positive design flow direction is from air_a to air_b)"
      annotation (Placement(transformation(extent={{-130,-70},{-150,-50}})));

    Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heaPor
      "Heat port, to be connected to room air"
      annotation (Placement(transformation(extent={{-10,-130},{10,-110}})));

    MediumWat.ThermodynamicState staWatCoo_a=
        MediumWat.setState_phX(watCoo_a.p,
                             noEvent(actualStream(watCoo_a.h_outflow)),
                             noEvent(actualStream(watCoo_a.Xi_outflow))) if
           show_T "Medium properties in port watCoo_a";
    MediumWat.ThermodynamicState staWatCoo_b=
        MediumWat.setState_phX(watCoo_b.p,
                             noEvent(actualStream(watCoo_b.h_outflow)),
                             noEvent(actualStream(watCoo_b.Xi_outflow))) if
           show_T "Medium properties in port watCoo_b";
    MediumAir.ThermodynamicState staAir_a=
        MediumAir.setState_phX(air_a.p,
                             noEvent(actualStream(air_a.h_outflow)),
                             noEvent(actualStream(air_a.Xi_outflow))) if
           show_T "Medium properties in port air_a";
    MediumAir.ThermodynamicState staAir_b=
        MediumAir.setState_phX(air_b.p,
                             noEvent(actualStream(air_b.h_outflow)),
                             noEvent(actualStream(air_b.Xi_outflow))) if
           show_T "Medium properties in port air_b";

    Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow heaToRoo(
      final alpha=0)
      "Heat tranferred to the room (in addition to heat from supply air)" annotation (
        Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={0,-36})));

    // Pressure drop
    Modelica.SIunits.PressureDifference dpWatCoo(displayUnit="Pa") = watCoo_a.p - watCoo_b.p
      "Pressure difference watCoo_a minus watCoo_b";

    Modelica.SIunits.PressureDifference dpAir(displayUnit="Pa") = air_a.p - air_b.p
      "Pressure difference air_a minus air_b";

    Buildings.Fluid.FixedResistances.PressureDrop res(
      redeclare final package Medium = MediumAir,
      final m_flow_nominal=perCoo.mAir_flow_nominal*nBeams,
      final dp_nominal=perCoo.dpAir_nominal)
      annotation (Placement(transformation(extent={{40,-70},{20,-50}})));

  protected
    BaseClasses.Convector conCoo(
      redeclare final package Medium = MediumWat,
      final per=perCoo,
      final allowFlowReversal=allowFlowReversalWat,
      final m_flow_small=mWat_flow_small,
      final show_T=false,
      final homotopyInitialization=homotopyInitialization,
      final from_dp=from_dpWat,
      final linearizeFlowResistance=linearizeFlowResistanceWat,
      final deltaM=deltaMWat,
      final tau=tau,
      final energyDynamics=energyDynamics,
      final massDynamics=massDynamics,
      final p_start=pWatCoo_start,
      final T_start=TWatCoo_start,
      final nBeams=nBeams) "Cooling beam"
      annotation (Placement(transformation(extent={{-10,50},{10,70}})));

    Modelica.Blocks.Math.Sum sum "Connector for heating and cooling mode"
      annotation (Placement(transformation(extent={{40,20},{60,40}})));

    Modelica.Blocks.Math.Gain gaiSig(
      final k=-1,
      u(final unit="W"),
      y(final unit="W")) "Gain to reverse the sign" annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          origin={50,-20})));

    Buildings.Fluid.Sensors.MassFlowRate senFloAir(redeclare final package
        Medium =
          MediumAir) "Mass flow rate sensor"
      annotation (Placement(transformation(extent={{-80,-70},{-100,-50}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor senTemRooAir
      "Temperature sensor for room air"
      annotation (Placement(transformation(extent={{-20,-50},{-40,-30}})));

  initial equation
    assert(perCoo.primaryAir.r_V[1]<=0.000001 and perCoo.primaryAir.f[1]<=0.00001,
      "Performance curve perCoo.primaryAir must pass through (0,0).");
    assert(perCoo.water.r_V[1]<=0.000001      and perCoo.water.f[1]<=0.00001,
      "Performance curve perCoo.water must pass through (0,0).");
    assert(perCoo.dT.r_dT[1]<=0.000001      and perCoo.dT.f[1]<=0.00001,
      "Performance curve perCoo.dT must pass through (0,0).");



  equation
    connect(heaToRoo.port, heaPor)
      annotation (Line(points={{0,-46},{0,-120}}, color={191,0,0}));
    connect(sum.y, gaiSig.u) annotation (Line(points={{61,30},{66,30},{70,30},{70,
            -20},{62,-20}}, color={0,0,127}));
    connect(gaiSig.y, heaToRoo.Q_flow)
      annotation (Line(points={{39,-20},{0,-20},{0,-26}}, color={0,0,127}));
    connect(senTemRooAir.port, heaPor) annotation (Line(points={{-20,-40},{-14,-40},
            {-14,-52},{0,-52},{0,-120}}, color={191,0,0}));
    connect(air_b, senFloAir.port_b)
      annotation (Line(points={{-140,-60},{-100,-60}}, color={0,127,255}));
    connect(conCoo.port_b, watCoo_b)
      annotation (Line(points={{10,60},{140,60}}, color={0,127,255}));
    connect(conCoo.Q_flow, sum.u[1]) annotation (Line(points={{11,67},{20,67},{20,
            30},{38,30}}, color={0,0,127}));
    connect(senTemRooAir.T, conCoo.TRoo) annotation (Line(points={{-40,-40},{-50,-40},
            {-50,54},{-12,54}}, color={0,0,127}));

    connect(air_a, res.port_a)
      annotation (Line(points={{140,-60},{90,-60},{40,-60}}, color={0,127,255}));
    connect(senFloAir.port_a, res.port_b) annotation (Line(points={{-80,-60},{-30,
            -60},{20,-60}}, color={0,127,255}));
    connect(watCoo_a, conCoo.port_a)
      annotation (Line(points={{-140,60},{-76,60},{-10,60}}, color={0,127,255}));
    connect(senFloAir.m_flow, conCoo.mAir_flow) annotation (Line(points={{-90,-49},
            {-90,-49},{-90,64},{-12,64}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false,  extent={{-140,
              -120},{140,120}}), graphics={Rectangle(
            extent={{-120,100},{120,-100}},
            fillPattern=FillPattern.Solid,
            fillColor={95,95,95},
            pattern=LinePattern.None,
            lineColor={0,0,0}),       Ellipse(
            extent={{48,78},{-48,-18}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-38,-34},{42,-80}},
            fillColor={0,128,255},
            fillPattern=FillPattern.VerticalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Rectangle(
            extent={{-120,66},{-132,54}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{132,66},{120,54}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-120,-54},{-134,-66}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{134,-54},{120,-66}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-149,141},{151,101}},
            lineColor={0,0,255},
            fillPattern=FillPattern.HorizontalCylinder,
            fillColor={0,127,255},
            textString="%name"),
          Line(points={{-114,60},{-68,60},{-84,70}}, color={0,0,255}),
          Line(points={{-68,60},{-84,52}}, color={0,0,255}),
          Line(points={{114,-60},{70,-60},{84,-52}}, color={0,127,127}),
          Line(points={{70,-60},{82,-68}}, color={0,127,127})}),
  defaultComponentName="actBea",
  Diagram(coordinateSystem(
            preserveAspectRatio=false, extent={{-140,-120},{140,120}})),
  Documentation(info="<html>
<p>
Model of an active beam, based on the EnergyPlus beam model  <code>AirTerminal:SingleDuct:ConstantVolume:FourPipeBeam</code>.
</p>
<p>
This model operates only in cooling mode. For a model that operates in both heating and cooling mode,
use <a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating</a>.
</p>
<p>
For a description of the equations, see the
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.UsersGuide\">
User's Guide</a>.
</p>
<p>
Performance data are available from
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.Data\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.Data</a>.
</p>
<h4>References</h4>
<ul>
<li>
DOE(2015) EnergyPlus documentation v8.4.0 - Engineering Reference.
</li>
</ul>
</html>",   revisions="<html>
<ul>
<li>
November 3, 2016, by Michael Wetter:<br/>
Set <code>final alpha=0</code> for prescribed heat flow rate.
</li>
<li>
September 17, 2016, by Michael Wetter:<br/>
Corrected wrong annotation to avoid an error in the pedantic model check
in Dymola 2017 FD01 beta2.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/557\">issue 557</a>.
</li>
<li>
June 14, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
  end Cooling;

  model CoolingAndHeating "Active beam unit for heating and cooling"
    extends FBM.Components.ActiveBeams.Cooling(sum(nin=2));

    replaceable parameter Data.Generic perHea "Performance data for heating"
      annotation (
        Dialog(group="Nominal condition"),
        choicesAllMatching=true,
        Placement(transformation(extent={{62,-98},{78,-82}})));

    // Initialization
    parameter MediumWat.AbsolutePressure pWatHea_start = pWatCoo_start
      "Start value of pressure"
      annotation(Dialog(tab = "Initialization", group = "Heating"));

    parameter MediumWat.Temperature TWatHea_start = TWatCoo_start
      "Start value of temperature"
      annotation(Dialog(tab = "Initialization", group = "Heating"));

    Modelica.Fluid.Interfaces.FluidPort_a watHea_a(
      redeclare final package Medium = MediumWat,
      m_flow(min=if allowFlowReversalWat then -Modelica.Constants.inf else 0),
      h_outflow(start=MediumWat.h_default))
      "Fluid connector a (positive design flow direction is from watHea_a to watHea_b)"
      annotation (Placement(transformation(extent={{-150,-10},{-130,10}})));
    Modelica.Fluid.Interfaces.FluidPort_b watHea_b(
      redeclare final package Medium = MediumWat,
      m_flow(max=if allowFlowReversalWat then +Modelica.Constants.inf else 0),
      h_outflow(start=MediumWat.h_default))
      "Fluid connector b (positive design flow direction is from watHea_a to watHea_b)"
      annotation (Placement(transformation(extent={{150,-10},{130,10}})));

    MediumWat.ThermodynamicState staHea_a=
        MediumWat.setState_phX(watHea_a.p,
                            noEvent(actualStream(watHea_a.h_outflow)),
                            noEvent(actualStream(watHea_a.Xi_outflow))) if
           show_T "Medium properties in port watHea_a";

    MediumWat.ThermodynamicState staHea_b=
        MediumWat.setState_phX(watHea_b.p,
                            noEvent(actualStream(watHea_b.h_outflow)),
                            noEvent(actualStream(watHea_b.Xi_outflow))) if
            show_T "Medium properties in port watHea_b";

    Modelica.SIunits.PressureDifference dpWatHea(displayUnit="Pa") = watHea_a.p - watHea_b.p
      "Pressure difference between watHea_a and watHea_b";

  protected
    BaseClasses.Convector conHea(
      redeclare final package Medium = MediumWat,
      final per=perHea,
      final allowFlowReversal=allowFlowReversalWat,
      final m_flow_small=mWat_flow_small,
      final show_T=false,
      final homotopyInitialization=homotopyInitialization,
      final from_dp=from_dpWat,
      final linearizeFlowResistance=linearizeFlowResistanceWat,
      final deltaM=deltaMWat,
      final tau=tau,
      final energyDynamics=energyDynamics,
      final massDynamics=massDynamics,
      final p_start=pWatHea_start,
      final T_start=TWatHea_start,
      final nBeams=nBeams) "Heating beam"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

  initial equation
    assert(perHea.primaryAir.r_V[1]<=0.000001 and perHea.primaryAir.f[1]<=0.00001,
      "Performance curve perHea.primaryAir must pass through (0,0).");
    assert(perHea.water.r_V[1]<=0.000001      and perHea.water.f[1]<=0.00001,
      "Performance curve perHea.water must pass through (0,0).");
    assert(perHea.dT.r_dT[1]<=0.000001        and perHea.dT.f[1]<=0.00001,
      "Performance curve perHea.dT must pass through (0,0).");


  equation
    connect(conHea.port_b, watHea_b)
      annotation (Line(points={{10,0},{140,0}}, color={0,127,255}));
    connect(conHea.Q_flow, sum.u[2])
      annotation (Line(points={{11,7},{20,7},{20,30},{38,30}}, color={0,0,127}));
    connect(conHea.TRoo, senTemRooAir.T) annotation (Line(points={{-12,-6},{-26,-6},
            {-50,-6},{-50,-40},{-40,-40}}, color={0,0,127}));
    connect(watHea_a, conHea.port_a)
      annotation (Line(points={{-140,0},{-76,0},{-10,0}}, color={0,127,255}));
    connect(conHea.mAir_flow, senFloAir.m_flow) annotation (Line(points={{-12,4},
            {-46,4},{-90,4},{-90,-49}}, color={0,0,127}));
    annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
              -120},{140,120}})), defaultComponentName="beaCooHea",Icon(
          coordinateSystem(extent={{-140,-120},{140,120}}),             graphics={
          Rectangle(
            extent={{-120,6},{-138,-6}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{138,6},{120,-6}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-60,-34},{0,-80}},
            fillColor={255,0,0},
            fillPattern=FillPattern.VerticalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Rectangle(
            extent={{0,-34},{64,-80}},
            fillColor={0,128,255},
            fillPattern=FillPattern.VerticalCylinder,
            pattern=LinePattern.None,
            lineColor={0,0,0}),
          Line(points={{-112,0},{-66,0},{-82,10}}, color={255,0,0}),
          Line(points={{-66,0},{-82,-8}}, color={255,0,0})}),
            Documentation(info="<html>
<p>
This model is identical to
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.Cooling\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.Cooling</a>,
except that an additional water stream and convector is added to allow for heating
in addition to cooling.
</p>
<p>
For a description of the equations, see the
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.UsersGuide\">
User's Guide</a>.
</p>
<p>
Performance data are available from
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.Data\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.Data</a>.
</p>
</html>",   revisions="<html>
<ul>
<li>
June 14, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
  end CoolingAndHeating;

  package Data "Package with performance data"
    extends Modelica.Icons.MaterialPropertiesPackage;

    package Trox "Performance data for Trox"
      record DID632A_nozzleH_length6ft_cooling =
        FBM.Components.ActiveBeams.Data.Generic (
          primaryAir(r_V={0,0.714286,1,1.2857}, f={0,0.823403,1,1.1256}),
          dT(f={0,0.5,1}, r_dT={0,0.5,1}),
          water(r_V={0,0.33333,0.5,0.666667,0.833333,1,1.333333}, f={0,0.71,
                0.85,0.92,0.97,1,1.04}),
          mAir_flow_nominal=0.0792,
          mWat_flow_nominal=0.094,
          dpWat_nominal=10000,
          dpAir_nominal=100,
          dT_nominal=-10,
          Q_flow_nominal=-1092)
      "Performance data for Trox DID 632A for cooling mode"
           annotation (
        Documentation(revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>",     info="<html>
<p>
Performance data for Trox active beam for cooling mode.
</p>
</html>"));

      record DID632A_nozzleH_length6ft_heating =
          FBM.Components.ActiveBeams.Data.Generic (
          dT(f={0,0.5,1}, r_dT={0,0.5,1}),
          primaryAir(r_V={0,0.714286,1,1.2857}, f={0,0.8554,1,1.0778}),
          water(r_V={0,0.33333,0.5,0.666667,0.833333,1,1.333333}, f={0,0.71,
                0.85,0.92,0.97,1,1.04}),
          mAir_flow_nominal=0.0792,
          mWat_flow_nominal=0.094,
          dpWat_nominal=10000,
          dpAir_nominal=100,
          dT_nominal=27.8,
          Q_flow_nominal=2832)
      "Performance data for Trox DID 632A for heating mode"
          annotation (
        Documentation(revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>",     info="<html>
<p>
Performance data for Trox active beam for heating mode.
</p>
</html>"));

    annotation (Documentation(revisions="", info="<html>
<p>
Package with performance data for active beams from Trox.
</p>
</html>"));
    end Trox;

    record Generic "Generic data record for active beam"
       extends Modelica.Icons.Record;

      parameter BaseClasses.AirFlow primaryAir(
        r_V={0,0.2,1},
        f={0,0.5,1}) "Performance data for primary air";
      parameter FBM.Components.ActiveBeams.Data.BaseClasses.WaterFlow water(r_V={0,
            0.5,1}, f={0,0.7,1}) "Performance data for water";

      parameter BaseClasses.TemperatureDifference dT(
        r_dT={0,0.5,1},
        f={0,0.5,1})
        "Performance data for normalized temperature difference room minus water inlet";

      parameter Modelica.SIunits.MassFlowRate mAir_flow_nominal
        "Nominal air mass flow rate per beam"
        annotation (Dialog(group="Nominal condition"));

      parameter Modelica.SIunits.MassFlowRate mWat_flow_nominal
        "Nominal water mass flow rate per beam"
        annotation (Dialog(group="Nominal condition"));
      parameter Modelica.SIunits.PressureDifference dpWat_nominal(displayUnit="Pa")
        "Water-side nominal pressure drop per beam"
        annotation (Dialog(group="Nominal condition"));

      parameter Modelica.SIunits.PressureDifference dpAir_nominal(displayUnit="Pa")
        "Air-side nominal pressure drop"
        annotation (Dialog(group="Nominal condition"));

      parameter Modelica.SIunits.TemperatureDifference dT_nominal
        "Nominal temperature difference water inlet minus room air"
        annotation (Dialog(group="Nominal condition"));
      parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal
        "Nominal capacity per beam"
        annotation (Dialog(group="Nominal condition"));

      annotation (defaultComponentName="per",
    Documentation(revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>",     info="<html>
<p>
Performance data for a generic active beam.
</p>
</html>"));
    end Generic;

    package BaseClasses "Base classes for performance data"

      record AirFlow "Record for primary air parameters"
        extends Modelica.Icons.Record;
        parameter Real r_V[:](each min=0, each final unit="1")
          "Normalized air volume flow rate at user-selected operating points";
        parameter Real f[size(r_V, 1)](each min=0, each final unit="1")
          "Normalized performance factor at these flow rates";

        annotation (Documentation(info="<html>
<p>
Data record for performance data that describe the air volume flow rate versus
the change in the rate of heating or cooling.
</p>
<p>
The normamlized volume flow rate <i>r<sub>V</sub></i> must be strictly increasing, i.e.,
<i>r<sub>V</sub><sup>i</sup> &lt; r<sub>V</sub><sup>i+1</sup></i>.
Both vectors, <i>r<sub>V</sub></i> and <i>f</i>
must have the same size.
</p>
</html>",
      revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
      end AirFlow;

      record TemperatureDifference "Record for temperature difference"
        extends Modelica.Icons.Record;
        parameter Real r_dT[:](each min=0, each final unit="1")
         "Normalized temperature difference, e.g., temperature difference at
 user-selected operating points divided by nominal temperature difference.
 Must be positive.";
        parameter Real f[size(r_dT, 1)](each min=0, each final unit="1")
          "Normalized performance factor at these normalized temperature differences";

        annotation (Documentation(info="<html>
<p>
Data record for performance data that describe the normalized
temperature difference
versus the change in the rate of heating or cooling.
The normalized temperature difference is defined as
</p>
<p align=\"center\" style=\"font-style:italic;\">
r<sub>&#916;T</sub><sup>i</sup>=
&#916;T<sup>i</sup> &frasl; &#916;T<sub>nominal</sub>
=
(T<sub>w</sub><sup>i</sup>-T<sub>z</sub>)
&frasl;
(T<sub>w,nominal</sub>-T<sub>z</sub>),
</p>
<p>
where
<i>T<sub>w</sub><sup>i</sup></i> is the water inlet temperature,
<i>T<sub>z</sub></i> is the zone air temperature and
<i>T<sub>w,nominal</sub></i> is the nominal water inlet temperature.
</p>
<p>
The normalized temperature difference <i>r<sub>&#916;T</sub></i> must be strictly increasing, i.e.,
<i>r<sub>&#916;T</sub><sup>i</sup> &lt; r<sub>&#916;T</sub><sup>i+1</sup></i>.
Both vectors, <i>r<sub>&#916;T</sub></i> and <i>f</i>
must have the same size.
</p>
</html>",
      revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
      end TemperatureDifference;

      record WaterFlow "Record for water parameters"
        extends Modelica.Icons.Record;
        parameter Real r_V[:](each min=0, each final unit="1")
          "Normalized water volume flow rate at user-selected operating points";
        parameter Real f[size(r_V, 1)](each min=0, each unit="1")
          "Normalized performance factor at these flow rates";

        annotation (Documentation(info="<html>
<p>
Data record for performance data that describe the water volume flow rate versus
the change in the rate of heating or cooling.
</p>
<p>
The normamlized volume flow rate <i>r<sub>V</sub></i> must be strictly increasing, i.e.,
<i>r<sub>V</sub><sup>i</sup> &lt; r<sub>V</sub><sup>i+1</sup></i>.
Both vectors, <i>r<sub>V</sub></i> and <i>f</i>
must have the same size.
</p>
</html>",
      revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
      end WaterFlow;
    annotation (Documentation(info="<html>
<p>
This package contains performance curves for
the active beam models.
</p>
</html>"));
    end BaseClasses;
  annotation (Documentation(info="<html>
<p>
Package with performance data for active beams.
</p>
</html>"));
  end Data;

  package Examples "Package with examples of active beam models"
    extends Modelica.Icons.ExamplesPackage;

    model CoolingAndHeating
      extends Modelica.Icons.Example;

      package MediumA = Buildings.Media.Air "Medium model for air";

      package MediumW = Buildings.Media.Water "Medium model for water";

      Buildings.Fluid.Sources.FixedBoundary sin_1(
        redeclare package Medium = MediumW,
        nPorts=1) "Sink chilled water"
        annotation (Placement(transformation(extent={{100,90},{80,110}})));
      Buildings.Fluid.Sources.MassFlowSource_T souAir(
        redeclare package Medium = MediumA,
        m_flow=0.0792,
        use_m_flow_in=false,
        nPorts=1,
        T=285.85) "Source air"
        annotation (Placement(transformation(extent={{100,10},{80,30}})));
      Buildings.Fluid.Sources.FixedBoundary bou(
        redeclare package Medium = MediumA,
        nPorts=1) "Sink air"
        annotation (Placement(transformation(extent={{100,-110},{80,-90}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor theConWal(G=200)
        "Thermal conductor for wall"
        annotation (Placement(transformation(extent={{-60,-110},{-40,-90}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow heaFlo
        "Thermal loads"
        annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
      Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 22)
        "Heating set-point temperature"
        annotation (Placement(transformation(extent={{-110,-20},{-90,0}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature TOut(T=301.15)
        "Outdoor air temperature"
        annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
      Buildings.Controls.Continuous.LimPID conHea(
        yMax=0.094,
        Td=0,
        reverseAction=false,
        Ti=100,
        k=0.1,
        controllerType=Modelica.Blocks.Types.SimpleController.PI)
        "Controller for heating"
             annotation (Placement(transformation(extent={{-70,-20},{-50,0}})));
      Buildings.Fluid.Sources.MassFlowSource_T pumCoo(
        redeclare package Medium = MediumW,
        use_m_flow_in=true,
        nPorts=1,
        T=288.15) "Source chilled water"
        annotation (Placement(transformation(extent={{-20,90},{0,110}})));
      Modelica.Blocks.Math.Gain gain(k=1200) "Gain for thermal loads"
        annotation (Placement(transformation(extent={{-68,-70},{-48,-50}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor senTem
        "Room air temperature sensor"
        annotation (Placement(transformation(extent={{-20,-40},{-40,-20}})));
      Buildings.Fluid.Sources.MassFlowSource_T pumHea(
        redeclare package Medium = MediumW,
        use_m_flow_in=true,
        nPorts=1,
        T=320.95) "Source hot water"
        annotation (Placement(transformation(extent={{-20,50},{0,70}})));
      Buildings.Fluid.Sources.FixedBoundary sin_2(
        redeclare package Medium = MediumW,
        nPorts=1) "Sink hot water"
        annotation (Placement(transformation(extent={{100,50},{80,70}})));
      Modelica.Blocks.Sources.Sine sine(
        freqHz=1/86400,
        amplitude=1,
        phase=-1.5707963267949) "Source for thermal loads"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Buildings.Fluid.MixingVolumes.MixingVolume vol(nPorts=2,
        redeclare package Medium = MediumA,
        m_flow_nominal=0.1,
        V=30,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
        "Air volume for room"
        annotation (Placement(transformation(extent={{50,-70},{70,-50}})));
      Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 25)
        "Cooling set-point temperature"
        annotation (Placement(transformation(extent={{-110,20},{-90,40}})));
      Buildings.Controls.Continuous.LimPID conCoo(
        yMax=0.094,
        reverseAction=true,
        Td=0,
        k=0.5,
        Ti=70,
        controllerType=Modelica.Blocks.Types.SimpleController.PI)
        "Controller for cooling"
        annotation (Placement(transformation(extent={{-70,20},{-50,40}})));

      FBM.Components.ActiveBeams.CoolingAndHeating beaCooHea(
        redeclare package MediumWat = MediumW,
        redeclare package MediumAir = MediumA,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_cooling perCoo,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_heating perHea,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
        "Active Beam"
        annotation (Placement(transformation(extent={{26,48},{54,72}})));
    equation
      connect(TOut.port, theConWal.port_a)
        annotation (Line(points={{-90,-100},{-60,-100}}, color={191,0,0}));
      connect(gain.y, heaFlo.Q_flow)
        annotation (Line(points={{-47,-60},{-30,-60}}, color={0,0,127}));
      connect(senTem.T,conHea. u_m) annotation (Line(points={{-40,-30},{-50,-30},{-60,
              -30},{-60,-22}}, color={0,0,127}));
      connect(sine.y, gain.u)
        annotation (Line(points={{-89,-60},{-70,-60}}, color={0,0,127}));
      connect(bou.ports[1], vol.ports[1]) annotation (Line(points={{80,-100},{58,
              -100},{58,-70}}, color={0,127,255}));
      connect(theConWal.port_b, vol.heatPort) annotation (Line(points={{-40,-100},{
              -2,-100},{40,-100},{40,-60},{50,-60}}, color={191,0,0}));
      connect(heaFlo.port, vol.heatPort)
        annotation (Line(points={{-10,-60},{20,-60},{50,-60}}, color={191,0,0}));
      connect(senTem.port, vol.heatPort) annotation (Line(points={{-20,-30},{40,-30},
              {40,-60},{50,-60}}, color={191,0,0}));
      connect(TSetHea.y, conHea.u_s) annotation (Line(points={{-89,-10},{-80.5,-10},
              {-72,-10}}, color={0,0,127}));
      connect(TSetCoo.y, conCoo.u_s)
        annotation (Line(points={{-89,30},{-72,30}}, color={0,0,127}));
      connect(conCoo.u_m, senTem.T) annotation (Line(points={{-60,18},{-60,10},{-80,
              10},{-80,-30},{-40,-30}}, color={0,0,127}));
      connect(beaCooHea.watCoo_b, sin_1.ports[1]) annotation (Line(points={{54,66},
              {60,66},{60,100},{80,100}}, color={0,127,255}));
      connect(sin_2.ports[1], beaCooHea.watHea_b)
        annotation (Line(points={{80,60},{68,60},{54,60}}, color={0,127,255}));
      connect(souAir.ports[1], beaCooHea.air_a) annotation (Line(points={{80,20},{
              60,20},{60,54},{54,54}}, color={0,127,255}));
      connect(beaCooHea.air_b, vol.ports[2]) annotation (Line(points={{26,54},{20,
              54},{20,-80},{62,-80},{62,-70}}, color={0,127,255}));
      connect(beaCooHea.heaPor, vol.heatPort) annotation (Line(points={{40,48},{40,
              48},{40,16},{40,-60},{50,-60}}, color={191,0,0}));
      connect(conCoo.y, pumCoo.m_flow_in) annotation (Line(points={{-49,30},{-40,30},
              {-40,108},{-20,108}}, color={0,0,127}));
      connect(conHea.y, pumHea.m_flow_in) annotation (Line(points={{-49,-10},{-32,
              -10},{-32,68},{-20,68}}, color={0,0,127}));
      connect(pumCoo.ports[1], beaCooHea.watCoo_a) annotation (Line(points={{0,100},
              {12,100},{20,100},{20,66},{26,66}}, color={0,127,255}));
      connect(pumHea.ports[1], beaCooHea.watHea_a)
        annotation (Line(points={{0,60},{13,60},{26,60}}, color={0,127,255}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -120},{120,120}})),experiment(StopTime=172800),__Dymola_Commands(file="modelica://FBM/Resources/Scripts/Dymola/Components/ActiveBeams/Examples/CoolingAndHeating.mos"
            "Simulate and plot"),
        Icon(coordinateSystem(extent={{-120,-120},{120,120}})),
         Documentation(info="<html>
<p>
This example tests the implementation of
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating</a>
for both heating and cooling mode. An air volume is maintained at a temperature between <i>22&circ;</i>C and
<i>25&circ;</i>C by two controllers that regulate the water flow rate in the active beam.
</p>
</html>",     revisions="<html>
<ul>
<li>
June 25, 2016, by Michael Wetter:<br/>
Changed medium start temperature to avoid conflicting
start values of the same precedence in Dymola 2016.
See
<a href=\"https://github.com/iea-annex60/modelica-annex60/issues/485\">
issue 485</a>.
</li>
<li>
June 14, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
    end CoolingAndHeating;

    model CoolingOnly
      extends Modelica.Icons.Example;

      package MediumA = Buildings.Media.Air "Medium model for air";

      package MediumW = Buildings.Media.Water "Medium model for water";

      Buildings.Fluid.Sources.FixedBoundary sin_1(
        redeclare package Medium = MediumW,
        nPorts=1) "Sink for water"
        annotation (Placement(transformation(extent={{100,56},{80,76}})));
      Buildings.Fluid.Sources.MassFlowSource_T souAir(
        redeclare package Medium = MediumA,
        use_m_flow_in=false,
        m_flow=0.0792,
        nPorts=1,
        T=285.85) "Source for air"
        annotation (Placement(transformation(extent={{100,10},{80,30}})));
      Buildings.Fluid.Sources.FixedBoundary bou(
        redeclare package Medium = MediumA,
        nPorts=1) "Sink for air"
        annotation (Placement(transformation(extent={{100,-110},{80,-90}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor theConWal(G=200)
        "Thermal conductor for wall"
        annotation (Placement(transformation(extent={{-60,-110},{-40,-90}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow heaFlo
        "Thermal loads"
        annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
      Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 25)
        "Set-point temperature"
        annotation (Placement(transformation(extent={{-110,-20},{-90,0}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature TOut(T=301.15)
        "Outdoor air temperature"
        annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
      Buildings.Controls.Continuous.LimPID conPID(
        reverseAction=true,
        Td=0,
        k=0.5,
        Ti=70,
        controllerType=Modelica.Blocks.Types.SimpleController.PI,
        yMax=0.094) "Controller"
             annotation (Placement(transformation(extent={{-70,-20},{-50,0}})));
      Buildings.Fluid.Sources.MassFlowSource_T pum(
        redeclare package Medium = MediumW,
        use_m_flow_in=true,
        nPorts=1,
        T=288.15) "Source for water"
        annotation (Placement(transformation(extent={{-22,56},{-2,76}})));
      Modelica.Blocks.Math.Gain gain(k=1200) "Gain thermal loads"
        annotation (Placement(transformation(extent={{-68,-70},{-48,-50}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor senTem
        "Room air temperature sensor"
        annotation (Placement(transformation(extent={{-20,-40},{-40,-20}})));
      Modelica.Blocks.Sources.Sine sine(
        freqHz=1/86400,
        amplitude=1,
        phase=-1.5707963267949) "Source for thermal loads"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Buildings.Fluid.MixingVolumes.MixingVolume vol(
        nPorts=2,
        redeclare package Medium = MediumA,
        m_flow_nominal=0.1,
        V=30,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        T_start=293.15) "Air volume for room"
        annotation (Placement(transformation(extent={{50,-70},{70,-50}})));
      FBM.Components.ActiveBeams.Cooling beaCoo(
        redeclare package MediumWat = MediumW,
        redeclare package MediumAir = MediumA,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_cooling perCoo,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Active beam"
        annotation (Placement(transformation(extent={{26,48},{54,72}})));
    equation
      connect(TOut.port, theConWal.port_a)
        annotation (Line(points={{-90,-100},{-60,-100}}, color={191,0,0}));
      connect(gain.y, heaFlo.Q_flow)
        annotation (Line(points={{-47,-60},{-30,-60}}, color={0,0,127}));
      connect(TSetCoo.y, conPID.u_s)
        annotation (Line(points={{-89,-10},{-72,-10}}, color={0,0,127}));
      connect(senTem.T, conPID.u_m) annotation (Line(points={{-40,-30},{-50,-30},{-60,
              -30},{-60,-22}}, color={0,0,127}));
      connect(vol.ports[1], bou.ports[1]) annotation (Line(points={{58,-70},{60,-70},
              {60,-100},{80,-100}}, color={0,127,255}));
      connect(heaFlo.port, vol.heatPort)
        annotation (Line(points={{-10,-60},{20,-60},{50,-60}}, color={191,0,0}));
      connect(theConWal.port_b, vol.heatPort) annotation (Line(points={{-40,-100},{
              40,-100},{40,-60},{50,-60}}, color={191,0,0}));
      connect(senTem.port, vol.heatPort) annotation (Line(points={{-20,-30},{40,-30},
              {40,-60},{50,-60}}, color={191,0,0}));
      connect(sine.y, gain.u)
        annotation (Line(points={{-89,-60},{-89,-60},{-70,-60}}, color={0,0,127}));
      connect(beaCoo.watCoo_b, sin_1.ports[1]) annotation (Line(points={{54,66},{70,
              66},{80,66}},           color={0,127,255}));
      connect(beaCoo.heaPor, vol.heatPort) annotation (Line(points={{40,48},{40,48},
              {40,26},{40,-60},{50,-60}}, color={191,0,0}));
      connect(souAir.ports[1], beaCoo.air_a) annotation (Line(points={{80,20},{70,
              20},{70,54},{54,54}}, color={0,127,255}));
      connect(beaCoo.air_b, vol.ports[2]) annotation (Line(points={{26,54},{12,54},
              {12,-80},{62,-80},{62,-70}}, color={0,127,255}));
      connect(pum.ports[1], beaCoo.watCoo_a)
        annotation (Line(points={{-2,66},{12,66},{26,66}}, color={0,127,255}));
      connect(pum.m_flow_in, conPID.y) annotation (Line(points={{-22,74},{-40,74},{
              -40,-10},{-49,-10}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -120},{120,120}})),experiment(StopTime=172800),
                __Dymola_Commands(file="modelica://FBM/Resources/Scripts/Dymola/Components/ActiveBeams/Examples/CoolingOnly.mos"
            "Simulate and plot"),
        Icon(coordinateSystem(extent={{-120,-120},{120,120}})),
         Documentation(info="<html>
<p>
This example tests the implementation of <a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.Cooling\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.Cooling</a>.
An air volume is maintained at a temperature below <i>25&circ;</i>C by a controller
that regulates the water flow rate in the active beam.
</p>
</html>",     revisions="<html>
<ul>
<li>
June 14, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
    end CoolingOnly;

    model HeatingOnly
      extends Modelica.Icons.Example;

      package MediumA = Buildings.Media.Air "Medium model for air";

      package MediumW = Buildings.Media.Water "Medium model for water";

      Buildings.Fluid.Sources.FixedBoundary sin_1(
        redeclare package Medium = MediumW,
        nPorts=1) "Sink for chilled water"
        annotation (Placement(transformation(extent={{100,90},{80,110}})));
      Buildings.Fluid.Sources.MassFlowSource_T souAir(
        redeclare package Medium = MediumA,
        m_flow=0.0792,
        use_m_flow_in=false,
        nPorts=1,
        T=285.85) "Source for air"
        annotation (Placement(transformation(extent={{100,10},{80,30}})));
      Buildings.Fluid.Sources.FixedBoundary bou(
        redeclare package Medium = MediumA,
        nPorts=1) "Sink for air"
        annotation (Placement(transformation(extent={{100,-110},{80,-90}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor theConWal(G=200)
        "Thermal conductor for wall"
        annotation (Placement(transformation(extent={{-60,-110},{-40,-90}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow heaFlo
        "Thermal loads"
        annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
      Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 22)
        "Heating set-point temperature"
        annotation (Placement(transformation(extent={{-110,-20},{-90,0}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature TOut(T=301.15)
        "Outdoor air temperature"
        annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
      Buildings.Controls.Continuous.LimPID conPID(
        yMax=0.094,
        Td=0,
        reverseAction=false,
        Ti=100,
        k=0.1,
        controllerType=Modelica.Blocks.Types.SimpleController.PI) "Controller"
             annotation (Placement(transformation(extent={{-70,-20},{-50,0}})));
      Buildings.Fluid.Sources.FixedBoundary sou_1(
        redeclare package Medium = MediumW,
        T=288.15,
        nPorts=1) "Soure chilled water"
        annotation (Placement(transformation(extent={{-40,90},{-20,110}})));
      Modelica.Blocks.Math.Gain gain(k=1200)
        annotation (Placement(transformation(extent={{-68,-70},{-48,-50}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor senTem
        "Room air temperature sensor"
        annotation (Placement(transformation(extent={{-20,-40},{-40,-20}})));
      Buildings.Fluid.Sources.MassFlowSource_T pumHea(
        redeclare package Medium = MediumW,
        use_m_flow_in=true,
        nPorts=1,
        T=320.95) "Source for heating"
        annotation (Placement(transformation(extent={{-20,50},{0,70}})));
      Buildings.Fluid.Sources.FixedBoundary sin_2(
        redeclare package Medium = MediumW,
        nPorts=1) "Sink for hot water"
        annotation (Placement(transformation(extent={{100,50},{80,70}})));
      Modelica.Blocks.Sources.Sine sine(
        freqHz=1/86400,
        amplitude=1,
        phase=-1.5707963267949) "Source for thermal loads"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Buildings.Fluid.MixingVolumes.MixingVolume vol(nPorts=2,
        redeclare package Medium = MediumA,
        m_flow_nominal=0.1,
        V=30,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        T_start=293.15) "Air volume for room"
        annotation (Placement(transformation(extent={{50,-70},{70,-50}})));

      FBM.Components.ActiveBeams.CoolingAndHeating beaCooHea(
        redeclare package MediumWat = MediumW,
        redeclare package MediumAir = MediumA,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_cooling perCoo,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_heating perHea,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial) "Active beam"
        annotation (Placement(transformation(extent={{26,48},{54,72}})));
    equation
      connect(TOut.port, theConWal.port_a)
        annotation (Line(points={{-90,-100},{-60,-100}}, color={191,0,0}));
      connect(gain.y, heaFlo.Q_flow)
        annotation (Line(points={{-47,-60},{-30,-60}}, color={0,0,127}));
      connect(TSetHea.y, conPID.u_s)
        annotation (Line(points={{-89,-10},{-72,-10}}, color={0,0,127}));
      connect(senTem.T, conPID.u_m) annotation (Line(points={{-40,-30},{-50,-30},{-60,
              -30},{-60,-22}}, color={0,0,127}));
      connect(sine.y, gain.u)
        annotation (Line(points={{-89,-60},{-70,-60}}, color={0,0,127}));
      connect(bou.ports[1], vol.ports[1]) annotation (Line(points={{80,-100},{58,
              -100},{58,-70}}, color={0,127,255}));
      connect(theConWal.port_b, vol.heatPort) annotation (Line(points={{-40,-100},{
              -2,-100},{40,-100},{40,-60},{50,-60}}, color={191,0,0}));
      connect(heaFlo.port, vol.heatPort)
        annotation (Line(points={{-10,-60},{20,-60},{50,-60}}, color={191,0,0}));
      connect(senTem.port, vol.heatPort) annotation (Line(points={{-20,-30},{40,-30},
              {40,-60},{50,-60}}, color={191,0,0}));
      connect(beaCooHea.watHea_b, sin_2.ports[1])
        annotation (Line(points={{54,60},{80,60}}, color={0,127,255}));
      connect(beaCooHea.heaPor, vol.heatPort)
        annotation (Line(points={{40,48},{40,-60},{50,-60}}, color={191,0,0}));
      connect(souAir.ports[1], beaCooHea.air_a) annotation (Line(points={{80,20},{
              70,20},{70,54},{54,54}}, color={0,127,255}));
      connect(sin_1.ports[1], beaCooHea.watCoo_b) annotation (Line(points={{80,100},
              {70,100},{70,66},{54,66}}, color={0,127,255}));
      connect(sou_1.ports[1], beaCooHea.watCoo_a) annotation (Line(points={{-20,100},
              {-20,100},{12,100},{12,66},{26,66}}, color={0,127,255}));
      connect(beaCooHea.air_b, vol.ports[2]) annotation (Line(points={{26,54},{12,
              54},{12,-80},{62,-80},{62,-70}}, color={0,127,255}));
      connect(pumHea.ports[1], beaCooHea.watHea_a)
        annotation (Line(points={{0,60},{0,60},{26,60}}, color={0,127,255}));
      connect(pumHea.m_flow_in, conPID.y) annotation (Line(points={{-20,68},{-40,68},
              {-40,-10},{-49,-10}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -120},{120,120}})),experiment(StopTime=172800),__Dymola_Commands(file="modelica://FBM/Resources/Scripts/Dymola/Components/ActiveBeams/Examples/HeatingOnly.mos"
            "Simulate and plot"),
        Icon(coordinateSystem(extent={{-120,-120},{120,120}})),
         Documentation(info="<html>
<p>
This example tests the implementation of <a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating</a>, but operates it only in heating mode.
An air volume is maintained at a temperature above <i>22&circ;</i>C by a controller
that regulates the water flow rate in the active beam.
</p>
</html>",     revisions="<html>
<ul>
<li>
June 14, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
    end HeatingOnly;

    model CoolingAndHeatingRoom
      extends Modelica.Icons.Example;

      package MediumA = Buildings.Media.Air "Medium model for air";

      package MediumW = Buildings.Media.Water "Medium model for water";

      Buildings.Fluid.Sources.FixedBoundary sin_1(
        redeclare package Medium = MediumW,
        nPorts=1) "Sink chilled water"
        annotation (Placement(transformation(extent={{100,90},{80,110}})));
      Buildings.Fluid.Sources.FixedBoundary bou(
        redeclare package Medium = MediumA,
        nPorts=1) "Sink air"
        annotation (Placement(transformation(extent={{100,-110},{80,-90}})));
      Modelica.Thermal.HeatTransfer.Components.ThermalConductor theConWal(G=200)
        "Thermal conductor for wall"
        annotation (Placement(transformation(extent={{-60,-110},{-40,-90}})));
      Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow heaFlo
        "Thermal loads"
        annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
      Modelica.Blocks.Sources.Constant TSetHea(k=273.15 + 22)
        "Heating set-point temperature"
        annotation (Placement(transformation(extent={{-110,-20},{-90,0}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature TOut(T=301.15)
        "Outdoor air temperature"
        annotation (Placement(transformation(extent={{-110,-110},{-90,-90}})));
      Buildings.Controls.Continuous.LimPID conHea(
        yMax=0.094,
        Td=0,
        reverseAction=false,
        Ti=100,
        k=0.1,
        controllerType=Modelica.Blocks.Types.SimpleController.PI)
        "Controller for heating"
             annotation (Placement(transformation(extent={{-70,-20},{-50,0}})));
      Buildings.Fluid.Sources.MassFlowSource_T pumCoo(
        redeclare package Medium = MediumW,
        use_m_flow_in=true,
        nPorts=1,
        T=288.15) "Source chilled water"
        annotation (Placement(transformation(extent={{-20,90},{0,110}})));
      Modelica.Blocks.Math.Gain gain(k=800)  "Gain for thermal loads"
        annotation (Placement(transformation(extent={{-68,-70},{-48,-50}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor senTem
        "Room air temperature sensor"
        annotation (Placement(transformation(extent={{-20,-40},{-40,-20}})));
      Buildings.Fluid.Sources.MassFlowSource_T pumHea(
        redeclare package Medium = MediumW,
        use_m_flow_in=true,
        nPorts=1,
        T=320.95) "Source hot water"
        annotation (Placement(transformation(extent={{-20,50},{0,70}})));
      Buildings.Fluid.Sources.FixedBoundary sin_2(
        redeclare package Medium = MediumW,
        nPorts=1) "Sink hot water"
        annotation (Placement(transformation(extent={{100,50},{80,70}})));
      Modelica.Blocks.Sources.Sine sine(
        freqHz=1/86400,
        amplitude=1,
        phase=-1.5707963267949) "Source for thermal loads"
        annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
      Buildings.Fluid.MixingVolumes.MixingVolume vol(nPorts=3,
        redeclare package Medium = MediumA,
        m_flow_nominal=0.1,
        V=30,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
        "Air volume for room"
        annotation (Placement(transformation(extent={{50,-70},{70,-50}})));
      Modelica.Blocks.Sources.Constant TSetCoo(k=273.15 + 25)
        "Cooling set-point temperature"
        annotation (Placement(transformation(extent={{-110,20},{-90,40}})));
      Buildings.Controls.Continuous.LimPID conCoo(
        yMax=0.094,
        reverseAction=true,
        Td=0,
        k=0.5,
        Ti=70,
        controllerType=Modelica.Blocks.Types.SimpleController.PI)
        "Controller for cooling"
        annotation (Placement(transformation(extent={{-70,20},{-50,40}})));

      FBM.Components.ActiveBeams.CoolingAndHeating beaCooHea(
        redeclare package MediumWat = MediumW,
        redeclare package MediumAir = MediumA,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_cooling perCoo,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_heating perHea,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
        "Active Beam"
        annotation (Placement(transformation(extent={{26,48},{54,72}})));
      Buildings.Fluid.Movers.FlowControlled_m_flow fan(redeclare package Medium =
            MediumA, m_flow_nominal=0.125) annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=90,
            origin={72,-14})));
      Modelica.Blocks.Sources.Constant MFlow(k=0.125)
        "Cooling set-point temperature"
        annotation (Placement(transformation(extent={{116,-24},{96,-4}})));
    equation
      connect(TOut.port, theConWal.port_a)
        annotation (Line(points={{-90,-100},{-60,-100}}, color={191,0,0}));
      connect(gain.y, heaFlo.Q_flow)
        annotation (Line(points={{-47,-60},{-30,-60}}, color={0,0,127}));
      connect(senTem.T,conHea. u_m) annotation (Line(points={{-40,-30},{-50,-30},{-60,
              -30},{-60,-22}}, color={0,0,127}));
      connect(sine.y, gain.u)
        annotation (Line(points={{-89,-60},{-70,-60}}, color={0,0,127}));
      connect(bou.ports[1], vol.ports[1]) annotation (Line(points={{80,-100},{
              57.3333,-100},{57.3333,-70}},
                               color={0,127,255}));
      connect(theConWal.port_b, vol.heatPort) annotation (Line(points={{-40,-100},{
              -2,-100},{40,-100},{40,-60},{50,-60}}, color={191,0,0}));
      connect(heaFlo.port, vol.heatPort)
        annotation (Line(points={{-10,-60},{20,-60},{50,-60}}, color={191,0,0}));
      connect(senTem.port, vol.heatPort) annotation (Line(points={{-20,-30},{40,-30},
              {40,-60},{50,-60}}, color={191,0,0}));
      connect(TSetHea.y, conHea.u_s) annotation (Line(points={{-89,-10},{-80.5,-10},
              {-72,-10}}, color={0,0,127}));
      connect(TSetCoo.y, conCoo.u_s)
        annotation (Line(points={{-89,30},{-72,30}}, color={0,0,127}));
      connect(conCoo.u_m, senTem.T) annotation (Line(points={{-60,18},{-60,10},{-80,
              10},{-80,-30},{-40,-30}}, color={0,0,127}));
      connect(beaCooHea.watCoo_b, sin_1.ports[1]) annotation (Line(points={{54,66},
              {60,66},{60,100},{80,100}}, color={0,127,255}));
      connect(sin_2.ports[1], beaCooHea.watHea_b)
        annotation (Line(points={{80,60},{68,60},{54,60}}, color={0,127,255}));
      connect(beaCooHea.air_b, vol.ports[2]) annotation (Line(points={{26,54},{
              20,54},{20,-80},{60,-80},{60,-70}},
                                               color={0,127,255}));
      connect(beaCooHea.heaPor, vol.heatPort) annotation (Line(points={{40,48},{40,
              48},{40,16},{40,-60},{50,-60}}, color={191,0,0}));
      connect(conCoo.y, pumCoo.m_flow_in) annotation (Line(points={{-49,30},{-40,30},
              {-40,108},{-20,108}}, color={0,0,127}));
      connect(conHea.y, pumHea.m_flow_in) annotation (Line(points={{-49,-10},{-32,
              -10},{-32,68},{-20,68}}, color={0,0,127}));
      connect(pumCoo.ports[1], beaCooHea.watCoo_a) annotation (Line(points={{0,100},
              {12,100},{20,100},{20,66},{26,66}}, color={0,127,255}));
      connect(pumHea.ports[1], beaCooHea.watHea_a)
        annotation (Line(points={{0,60},{13,60},{26,60}}, color={0,127,255}));
      connect(fan.port_a, vol.ports[3]) annotation (Line(points={{72,-24},{72,
              -24},{72,-70},{62.6667,-70}}, color={0,127,255}));
      connect(beaCooHea.air_a, fan.port_b)
        annotation (Line(points={{54,54},{72,54},{72,-4}}, color={0,127,255}));
      connect(fan.m_flow_in, MFlow.y) annotation (Line(points={{84,-14.2},{88,
              -14.2},{88,-14},{95,-14}}, color={0,0,127}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -120},{120,120}})),experiment(StopTime=172800),__Dymola_Commands(file="modelica://FBM/Resources/Scripts/Dymola/Components/ActiveBeams/Examples/CoolingAndHeating.mos"
            "Simulate and plot"),
        Icon(coordinateSystem(extent={{-120,-120},{120,120}})),
         Documentation(info="<html>
<p>
This example tests the implementation of
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating</a>
for both heating and cooling mode. An air volume is maintained at a temperature between <i>22&circ;</i>C and
<i>25&circ;</i>C by two controllers that regulate the water flow rate in the active beam.
</p>
</html>",     revisions="<html>
<ul>
<li>
June 25, 2016, by Michael Wetter:<br/>
Changed medium start temperature to avoid conflicting
start values of the same precedence in Dymola 2016.
See
<a href=\"https://github.com/iea-annex60/modelica-annex60/issues/485\">
issue 485</a>.
</li>
<li>
June 14, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
    end CoolingAndHeatingRoom;
  annotation (Documentation(info="<html>
<p>
This package contains examples for the use of models that can be found in
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams\">
IDEAS.Fluid.HeatExchangers.ActiveBeams</a>.
</p>
</html>"));
  end Examples;

  package Validation "Collection of validation models"
    extends Modelica.Icons.ExamplesPackage;

    model NumberOfBeams
      extends Modelica.Icons.Example;

      package MediumA = IDEAS.Media.Air "Medium model for air";

      package MediumW = IDEAS.Media.Water "Medium model for water";

      parameter Integer nBeams(min=1) = 10 "Number of beams";

      IDEAS.Fluid.Sources.FixedBoundary sin_1(
        redeclare package Medium = MediumW,
        nPorts=2) "Sink for chilled water"
        annotation (Placement(transformation(extent={{80,70},{60,90}})));
      IDEAS.Fluid.Sources.MassFlowSource_T souAir(
        redeclare package Medium = MediumA,
        use_m_flow_in=false,
        nPorts=1,
        m_flow=0.0792,
        T=285.85) "Source for air"
        annotation (Placement(transformation(extent={{80,-10},{60,10}})));
      IDEAS.Fluid.Sources.FixedBoundary sin_3(
        redeclare package Medium = MediumA,
        nPorts=2)
        annotation (Placement(transformation(extent={{-120,-10},{-100,10}})));
      IDEAS.Fluid.Sources.FixedBoundary sou_1(
        redeclare package Medium = MediumW,
        nPorts=2,
        T=288.15) "Source for chilled water"
        annotation (Placement(transformation(extent={{-120,68},{-100,88}})));
      IDEAS.Fluid.Movers.FlowControlled_m_flow pumHotWat(
        redeclare package Medium = MediumW,
        m_flow_nominal=0.094,
        addPowerToMedium=false,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        filteredSpeed=false,
        nominalValuesDefineDefaultPressureCurve=true)
        "Pump for hot water"
        annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
      IDEAS.Fluid.Sources.FixedBoundary sou_2(
        redeclare package Medium = MediumW,
        nPorts=2,
        T=320.95) "Source for hot water" annotation (Placement(transformation(extent={{-120,28},{-100,48}})));
      IDEAS.Fluid.Sources.FixedBoundary sin_2(
        redeclare package Medium = MediumW,
        nPorts=2) "Sink for hot water"
        annotation (Placement(transformation(extent={{80,30},{60,50}})));
      IDEAS.Fluid.Movers.FlowControlled_m_flow pumChiWat(
        redeclare package Medium = MediumW,
        m_flow_nominal=0.094,
        addPowerToMedium=false,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        filteredSpeed=false,
        nominalValuesDefineDefaultPressureCurve=true)
        "Pump for chilled water"
        annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
      FBM.Components.ActiveBeams.CoolingAndHeating beaCooHea(
        redeclare package MediumWat = MediumW,
        redeclare package MediumAir = MediumA,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_cooling perCoo,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_heating perHea,
        nBeams=1,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Active beam"
        annotation (Placement(transformation(extent={{-14,28},{14,52}})));

      Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature
        prescribedTemperature "Room temperature"
        annotation (Placement(transformation(extent={{-60,-160},{-40,-140}})));

      IDEAS.Fluid.Sources.MassFlowSource_T souAir10(
        redeclare package Medium = MediumA,
        use_m_flow_in=false,
        nPorts=1,
        m_flow=0.0792*nBeams,
        T=285.85) "Source for air"
        annotation (Placement(transformation(extent={{80,-130},{60,-110}})));

      IDEAS.Fluid.Movers.FlowControlled_m_flow pumHotWat10(
        redeclare package Medium = MediumW,
        addPowerToMedium=false,
        m_flow_nominal=0.094*nBeams,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        filteredSpeed=false,
        nominalValuesDefineDefaultPressureCurve=true)
        "Pump for hot water"
        annotation (Placement(transformation(extent={{-60,-90},{-40,-70}})));

      IDEAS.Fluid.Movers.FlowControlled_m_flow pumChiWat10(
        redeclare package Medium = MediumW,
        addPowerToMedium=false,
        m_flow_nominal=0.094*nBeams,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        filteredSpeed=false,
        nominalValuesDefineDefaultPressureCurve=true)
       "Pump for chilled water"
        annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));

      FBM.Components.ActiveBeams.CoolingAndHeating beaCooHea10(
        redeclare package MediumWat = MediumW,
        redeclare package MediumAir = MediumA,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_cooling perCoo,
        redeclare Data.Trox.DID632A_nozzleH_length6ft_heating perHea,
        nBeams=nBeams,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState) "Active beam"
        annotation (Placement(transformation(extent={{-14,-92},{14,-68}})));
      Modelica.Blocks.Sources.Step step(
        height=-0.094,
        offset=0.094,
        startTime=2000) "Chilled water mass flow rate"
        annotation (Placement(transformation(extent={{-180,90},{-160,110}})));
      Modelica.Blocks.Sources.Step step1(
        height=0.094,
        startTime=3000)
        "Hot water mass flow rate"
        annotation (Placement(transformation(extent={{-180,50},{-160,70}})));
      Modelica.Blocks.Sources.Step step2(
        height=0.094*nBeams,
        startTime=3000) "Hot water mass flow rate"
        annotation (Placement(transformation(extent={{-180,-70},{-160,-50}})));
      Modelica.Blocks.Sources.Step step3(
        height=-0.094*nBeams,
        offset=0.094*nBeams,
        startTime=2000) "Chilled water mass flow rate"
        annotation (Placement(transformation(extent={{-180,-30},{-160,-10}})));
      Modelica.Blocks.Sources.Step step4(
        offset=273.15 + 25,
        height=-5,
        startTime=2500) "Room air temperature variation"
        annotation (Placement(transformation(extent={{-120,-160},{-100,-140}})));
    equation
      connect(sou_2.ports[1], pumHotWat.port_a)
        annotation (Line(points={{-100,40},{-80,40},{-60,40}}, color={0,127,255}));
      connect(pumChiWat.port_a, sou_1.ports[1])
        annotation (Line(points={{-60,80},{-80,80},{-100,80}}, color={0,127,255}));
      connect(pumChiWat.port_b, beaCooHea.watCoo_a) annotation (Line(points={{-40,
              80},{-32,80},{-20,80},{-20,46},{-14,46}}, color={0,127,255}));
      connect(beaCooHea.watCoo_b, sin_1.ports[1]) annotation (Line(points={{14,46},{
              20,46},{20,82},{60,82}}, color={0,127,255}));
      connect(sin_2.ports[1], beaCooHea.watHea_b)
        annotation (Line(points={{60,42},{60,40},{14,40}}, color={0,127,255}));
      connect(beaCooHea.watHea_a, pumHotWat.port_b)
        annotation (Line(points={{-14,40},{-27,40},{-40,40}}, color={0,127,255}));
      connect(souAir.ports[1], beaCooHea.air_a) annotation (Line(points={{60,0},{20,
              0},{20,34},{14,34}}, color={0,127,255}));
      connect(sin_3.ports[1], beaCooHea.air_b) annotation (Line(points={{-100,2},{-64,
              2},{-20,2},{-20,34},{-14,34}}, color={0,127,255}));
      connect(pumChiWat10.port_b, beaCooHea10.watCoo_a) annotation (Line(points={{-40,
              -40},{-32,-40},{-20,-40},{-20,-74},{-14,-74}}, color={0,127,255}));
      connect(beaCooHea10.watHea_a, pumHotWat10.port_b) annotation (Line(points={{-14,
              -80},{-27,-80},{-40,-80}}, color={0,127,255}));
      connect(souAir10.ports[1], beaCooHea10.air_a) annotation (Line(points={{60,-120},
              {20,-120},{20,-86},{14,-86}}, color={0,127,255}));
      connect(pumChiWat10.port_a, sou_1.ports[2]) annotation (Line(points={{-60,-40},
              {-70,-40},{-70,76},{-100,76}}, color={0,127,255}));
      connect(pumHotWat10.port_a, sou_2.ports[2]) annotation (Line(points={{-60,-80},
              {-80,-80},{-80,36},{-100,36}}, color={0,127,255}));
      connect(beaCooHea10.air_b, sin_3.ports[2]) annotation (Line(points={{-14,-86},
              {-20,-86},{-20,-108},{-90,-108},{-90,-2},{-100,-2}}, color={0,127,255}));
      connect(beaCooHea10.watCoo_b, sin_1.ports[2]) annotation (Line(points={{14,-74},
              {28,-74},{28,78},{60,78}}, color={0,127,255}));
      connect(beaCooHea10.watHea_b, sin_2.ports[2]) annotation (Line(points={{14,-80},
              {28,-80},{40,-80},{40,38},{60,38}}, color={0,127,255}));
      connect(step.y, pumChiWat.m_flow_in) annotation (Line(points={{-159,100},{-108,
              100},{-50.2,100},{-50.2,92}}, color={0,0,127}));
      connect(step1.y, pumHotWat.m_flow_in) annotation (Line(points={{-159,60},{-110,
              60},{-50.2,60},{-50.2,52}}, color={0,0,127}));
      connect(step3.y, pumChiWat10.m_flow_in) annotation (Line(points={{-159,-20},{
              -104,-20},{-50.2,-20},{-50.2,-28}}, color={0,0,127}));
      connect(step2.y, pumHotWat10.m_flow_in) annotation (Line(points={{-159,-60},{
              -104,-60},{-50.2,-60},{-50.2,-68}}, color={0,0,127}));
      connect(step4.y, prescribedTemperature.T)
        annotation (Line(points={{-99,-150},{-62,-150}}, color={0,0,127}));
      connect(prescribedTemperature.port, beaCooHea10.heaPor) annotation (Line(
            points={{-40,-150},{-20,-150},{0,-150},{0,-92}}, color={191,0,0}));
      connect(beaCooHea.heaPor, prescribedTemperature.port) annotation (Line(points=
             {{0,28},{0,28},{0,-10},{0,-40},{50,-40},{50,-150},{-40,-150}}, color={191,
              0,0}));
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -180},{120,120}})),experiment(StopTime=5000),
       __Dymola_Commands(file="modelica://IDEAS/Resources/Scripts/Dymola/Fluid/HeatExchangers/ActiveBeams/Validation/NumberOfBeams.mos"
            "Simulate and plot"),
         Documentation(info="<html>
<p>
This model validates the scaling of the heat tranfer and pressure drop for
<code>nBeams &gt; 1</code>.
</p>
<p>
It uses two instances of
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.CoolingAndHeating</a>,
one with
<code>nBeams = 1</code> and one with
<code>nBeams = 10</code>.
</p>
</html>",     revisions="<html>
<ul>
<li>
June 14, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
    end NumberOfBeams;

    model NumberOfBeamsDynamics
      "Validation model for the dynamic response for one and multiple beams"
      extends NumberOfBeams(
        nBeams = 2,
        beaCooHea(
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          show_T=true,
          tau=120,
          TWatCoo_start=278.15,
          TWatHea_start=303.15),
        beaCooHea10(
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          show_T=true,
          tau=120,
          TWatCoo_start=278.15,
          TWatHea_start=303.15),
        step(startTime=200),
        step1(startTime=300),
        step3(startTime=200),
        step2(startTime=300));
      annotation (
    experiment(StopTime=500),
       __Dymola_Commands(file="modelica://IDEAS/Resources/Scripts/Dymola/Fluid/HeatExchangers/ActiveBeams/Validation/NumberOfBeamsDynamics.mos"
            "Simulate and plot"),
      Documentation(info="<html>
<p>
This model validates whether the transient response is indeed
independent of the number of beams.
The model is similar to
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.Validation.NumberOfBeams\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.Validation.NumberOfBeams</a>,
except that it is configured with a dynamic balance and non-default initial conditions.
</p>
</html>",     revisions="<html>
<ul>
<li>
June 24, 2016, by Michael Wetter:<br/>
First implementation.
</li>
</ul>
</html>"));

    end NumberOfBeamsDynamics;
  annotation (preferredView="info", Documentation(info="<html>
<p>
This package contains validation models for the classes in
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams\">
IDEAS.Fluid.HeatExchangers.ActiveBeams</a>.
</p>
<p>
Note that most validation models contain simple input data
which may not be realistic, but for which the correct
output can be obtained through an analytic solution.
The examples plot various outputs, which have been verified against these
solutions. These model outputs are stored as reference data and
used for continuous validation whenever models in the library change.
</p>
</html>"));
  end Validation;

  package BaseClasses "Base classes for active beam models"
    extends Modelica.Icons.BasesPackage;

    model Convector "Heat exchanger for the water stream"
      extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(
        final m_flow_nominal = per.mWat_flow_nominal*nBeams);
      extends Buildings.Fluid.Interfaces.TwoPortFlowResistanceParameters(
        final computeFlowResistance=true,
        final dp_nominal = per.dpWat_nominal "Don't multiply with nBeams, as the beams are in parallel");

      parameter Data.Generic per "Performance data"
        annotation (choicesAllMatching = true,
        Placement(transformation(extent={{60,-80},{80,-60}})));

      parameter Integer nBeams(min=1) "Number of beams in parallel";

      parameter Modelica.SIunits.Time tau = 30
        "Time constant at nominal flow (if energyDynamics <> SteadyState)"
         annotation (Dialog(tab = "Dynamics", group="Nominal condition"));

      // Advanced
      parameter Boolean homotopyInitialization = true "= true, use homotopy method"
        annotation(Evaluate=true, Dialog(tab="Advanced"));

      // Dynamics
      parameter Modelica.Fluid.Types.Dynamics energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial
        "Type of energy balance: dynamic (3 initialization options) or steady state"
        annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));
      parameter Modelica.Fluid.Types.Dynamics massDynamics=energyDynamics
        "Type of mass balance: dynamic (3 initialization options) or steady state"
        annotation(Evaluate=true, Dialog(tab = "Dynamics", group="Equations"));

      // Initialization
      parameter Medium.AbsolutePressure p_start = Medium.p_default
        "Start value of pressure"
        annotation(Dialog(tab = "Initialization"));
      parameter Medium.Temperature T_start = Medium.T_default
        "Start value of temperature"
        annotation(Dialog(tab = "Initialization"));
      parameter Medium.MassFraction X_start[Medium.nX](
        final quantity=Medium.substanceNames) = Medium.X_default
        "Start value of mass fractions m_i/m"
        annotation (Dialog(tab="Initialization", enable=Medium.nXi > 0));
      parameter Medium.ExtraProperty C_start[Medium.nC](
        final quantity=Medium.extraPropertiesNames)=fill(0, Medium.nC)
        "Start value of trace substances"
        annotation (Dialog(tab="Initialization", enable=Medium.nC > 0));

      Modelica.Blocks.Interfaces.RealInput mAir_flow(
        final unit="kg/s") "Air mass flow rate of a single beam"
        annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
      Modelica.Blocks.Interfaces.RealInput TRoo(
        final unit="K",
        displayUnit="degC") "Room air temperature"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Modelica.Blocks.Interfaces.RealOutput Q_flow(final unit="W")
        "Actual capacity of a single beam"
        annotation (Placement(transformation(extent={{100,60},{120,80}})));

    protected
      Buildings.Fluid.HeatExchangers.HeaterCooler_u hex(
        redeclare final package Medium = Medium,
        final allowFlowReversal=allowFlowReversal,
        final m_flow_nominal=m_flow_nominal,
        final m_flow_small=m_flow_small,
        final show_T=false,
        final from_dp=from_dp,
        final dp_nominal=dp_nominal,
        final linearizeFlowResistance=linearizeFlowResistance,
        final deltaM=deltaM,
        final tau=tau,
        final homotopyInitialization=homotopyInitialization,
        final energyDynamics=energyDynamics,
        final massDynamics=massDynamics,
        final p_start=p_start,
        final T_start=T_start,
        final X_start=X_start,
        final C_start=C_start,
        final Q_flow_nominal=-nBeams*per.Q_flow_nominal)
        "Heat exchanger for the water stream"
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));

      ModificationFactor mod(
        final nBeams=nBeams,
        final per=per) "Performance modification for part load"
        annotation (Placement(transformation(extent={{-10,50},{10,70}})));

      Modelica.Blocks.Sources.RealExpression senTem(final y=Medium.temperature(port_a_inflow))
        "Actual water temperature entering the beam"
        annotation (Placement(transformation(extent={{-60,18},{-40,38}})));

      Medium.ThermodynamicState port_a_inflow=
        Medium.setState_phX(port_a.p, inStream(port_a.h_outflow), inStream(port_a.Xi_outflow))
        "state for medium inflowing through port_a";

      Buildings.Fluid.Sensors.MassFlowRate senFloWatCoo(redeclare final package
                                                                                Medium =
            Medium) "Mass flow rate sensor"
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    equation
      connect(hex.Q_flow, Q_flow) annotation (Line(points={{61,6},{70,6},{70,70},{
              110,70}},
                    color={0,0,127}));
      connect(hex.port_b, port_b)
        annotation (Line(points={{60,0},{82,0},{100,0}},
                                                  color={0,127,255}));
      connect(mod.y, hex.u)
        annotation (Line(points={{11,60},{20,60},{20,6},{38,6}}, color={0,0,127}));
      connect(senTem.y, mod.TWat) annotation (Line(points={{-39,28},{-28,28},{-28,57},
              {-12,57}}, color={0,0,127}));
      connect(TRoo, mod.TRoo) annotation (Line(points={{-120,-60},{-20,-60},{-20,51.2},
              {-12,51.2}}, color={0,0,127}));
      connect(mAir_flow, mod.mAir_flow) annotation (Line(points={{-120,40},{-120,40},
              {-34,40},{-34,63},{-12,63}}, color={0,0,127}));
      connect(senFloWatCoo.port_a, port_a)
        annotation (Line(points={{-80,0},{-100,0}},          color={0,127,255}));
      connect(senFloWatCoo.port_b, hex.port_a)
        annotation (Line(points={{-60,0},{40,0}}, color={0,127,255}));
      connect(senFloWatCoo.m_flow, mod.mWat_flow)
        annotation (Line(points={{-70,11},{-70,69},{-12,69}}, color={0,0,127}));
      annotation ( Icon(coordinateSystem(preserveAspectRatio=false,
              extent={{-100,-100},{100,100}}), graphics={Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={0,0,0},
              fillColor={215,215,215},
              fillPattern=FillPattern.Solid),
            Rectangle(
              extent={{-70,100},{-50,-100}},
              lineColor={95,95,95},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={95,95,95}),
            Rectangle(
              extent={{-10,100},{10,-100}},
              lineColor={95,95,95},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={95,95,95}),
            Rectangle(
              extent={{50,100},{70,-100}},
              lineColor={95,95,95},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={95,95,95})}), defaultComponentName="con",
               Documentation(info="<html>
<p>
In cooling mode, this model adds heat to the water stream. The heat added is equal to:
</p>
<p align=\"center\" style=\"font-style:italic;\">
Q<sub>Beam</sub> = Q<sub>rated</sub> f<sub><code>&#916;</code>T</sub> f<sub>SA</sub> f<sub>W</sub>
</p>
<p>
In heating mode, the heat is removed from the water stream.
</p>
</html>",     revisions="<html>
<ul>
<li>
November 2, 2016, by Michael Wetter:<br/>
Made assignment of <code>senTem.y</code> final.
</li>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
    end Convector;

    model DerivativesCubicSpline "Cubic spline for interpolation"
      extends Modelica.Blocks.Icons.Block;
      parameter Real[:] xd={0,0.5,1};
      parameter Real[size(xd, 1)] yd={0,0.75,1};

      Modelica.Blocks.Interfaces.RealInput u
        "Independent variable for interpolation"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Modelica.Blocks.Interfaces.RealOutput y "Interpolated value"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    protected
      parameter Real[size(xd, 1)] dMonotone(each fixed=false) "Derivatives";
      Integer i "Counter to pick the interpolation interval";

    initial algorithm
      // Get the derivative values at the support points

      dMonotone := Buildings.Utilities.Math.Functions.splineDerivatives(
        x=xd,
        y=yd,
        ensureMonotonicity=true);

    algorithm
      i := 1;
      for j in 1:size(xd, 1) - 1 loop
        if u > xd[j] then
          i := j;
        end if;
      end for;
      // Extrapolate or interpolate the data
      y :=
        Buildings.Utilities.Math.Functions.cubicHermiteLinearExtrapolation(
        x=u,
        x1=xd[i],
        x2=xd[i + 1],
        y1=yd[i],
        y2=yd[i + 1],
        y1d=dMonotone[i],
        y2d=dMonotone[i + 1]);
    annotation (
      defaultComponentName="cubSpl",
      Documentation(info="<html>
<p>
This model calculates the output based on the cubic hermite interpolation
and linear extrapolation of predefined values. The predefined values must create a monotone curve.
</p>
</html>",     revisions="<html>
<ul>
<li>
December 15, 2016, by Michael Wetter:<br/>
Removed wrong annotations.<br/>
This is for
<a href=\"https://github.com/iea-annex60/modelica-annex60/issues/629\">#629</a>.
</li>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"),
        Icon(graphics={
        Line(points={{46,-76},{46,58}},
                                      color={192,192,192}),
        Line(points={{-84,-72},{84,-72}},  color={192,192,192}),
        Line(points={{-40,-78},{-40,-66}},
                                      color={192,192,192}),
        Line(points={{0,-88},{0,86}}, color={192,192,192}),
        Polygon(
          points={{0,90},{-6,74},{6,74},{0,90}},
          lineColor={192,192,192},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid),
            Line(
              points={{-82,-72},{-40,-72},{-18,-56},{-6,-32},{0,-8},{14,26},{32,46},
                  {46,50},{80,50}},
              color={0,0,0},
              smooth=Smooth.Bezier)}));
    end DerivativesCubicSpline;

    model ModificationFactor "Factor to modify nominal capacity"
      extends Modelica.Blocks.Icons.Block;

      parameter Integer nBeams(min=1) "Number of beams in parallel";

      parameter Data.Generic per "Performance data"
        annotation (choicesAllMatching = true,
        Placement(transformation(extent={{60,-80},{80,-60}})));

      Modelica.Blocks.Interfaces.RealInput TWat(
        final unit="K",
        displayUnit="degC") "Temperature of the water entering the beams"
        annotation (Placement(transformation(extent={{-140,-50},{-100,-10}})));
      Modelica.Blocks.Interfaces.RealInput mWat_flow(
        final unit="kg/s") "Mass flow rate of the water entering the beams"
        annotation (Placement(transformation(extent={{-140,70},{-100,110}})));
      Modelica.Blocks.Interfaces.RealInput mAir_flow(
        final unit="kg/s") "Mass flow rate of the primary air entering the beams"
        annotation (Placement(transformation(extent={{-140,10},{-100,50}})));

      Modelica.Blocks.Interfaces.RealInput TRoo(
        final unit="K",
        displayUnit="degC") "Room air temperature"
        annotation (Placement(transformation(extent={{-140,-108},{-100,-68}})));

      Modelica.Blocks.Interfaces.RealOutput y(final unit="1")
        "Total modification factor"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));

    protected
      Modelica.Blocks.Sources.Constant temDif_nom(
        final k=1/per.dT_nominal)
        "Nominal temperature difference between water and room air"
        annotation (Placement(transformation(extent={{-70,-80},{-50,-60}})));
      Modelica.Blocks.Sources.Constant watFlo_nom(final k=1/(nBeams*per.mWat_flow_nominal))
                                         "Nominal water mass flow rate"
        annotation (Placement(transformation(extent={{-70,50},{-50,70}})));
      Modelica.Blocks.Sources.Constant airFlo_nom(final k=1/(nBeams*per.mAir_flow_nominal))
                                         "Nominal water mass flow rate"
        annotation (Placement(transformation(extent={{-70,-10},{-50,10}})));

      DerivativesCubicSpline temDif_mod(
        final xd=per.dT.r_dT,
        final yd=per.dT.f)
        "Derivatives of the cubic spline for the temperature difference between room and water"
        annotation (Placement(transformation(extent={{20,-50},{40,-30}})));
      DerivativesCubicSpline watFlo_mod(
        final xd=per.water.r_V,
        final yd=per.water.f) "Derivatives of the cubic spline for the water flow"
        annotation (Placement(transformation(extent={{20,70},{40,90}})));
      DerivativesCubicSpline airFlo_mod(
        final xd=per.primaryAir.r_V,
        final yd=per.primaryAir.f)
        "Derivatives of the cubic spline for the air flow"
        annotation (Placement(transformation(extent={{20,10},{40,30}})));

      Modelica.Blocks.Math.Product pro_3
        "Ratio of actual/nominal temperature difference"
        annotation (Placement(transformation(extent={{-10,-50},{10,-30}})));
      Modelica.Blocks.Math.Product pro_2 "Ratio of actual/nominal water flow rate"
        annotation (Placement(transformation(extent={{-10,70},{10,90}})));
      Modelica.Blocks.Math.Product pro_1 "Ratio of actual/nominal air flow rate"
        annotation (Placement(transformation(extent={{-10,10},{10,30}})));

      Modelica.Blocks.Math.MultiProduct mulPro(final nu=3)
        "Product of the three modification factors"
        annotation (Placement(transformation(extent={{64,-6},{76,6}})));

      Modelica.Blocks.Math.Add add(final k1=+1, final k2=-1)
        "Temperature difference between water and room air"
        annotation (Placement(transformation(extent={{-50,-50},{-30,-30}})));

    equation
      connect(mAir_flow, pro_1.u1) annotation (Line(points={{-120,30},{-72,30},{-20,
              30},{-20,26},{-12,26}}, color={0,0,127}));
      connect(airFlo_nom.y, pro_1.u2) annotation (Line(points={{-49,0},{-20,0},{-20,
              14},{-12,14}},     color={0,0,127}));
      connect(pro_1.y, airFlo_mod.u)
        annotation (Line(points={{11,20},{14.5,20},{18,20}}, color={0,0,127}));
      connect(mulPro.y, y)
        annotation (Line(points={{77.02,0},{110,0}}, color={0,0,127}));
      connect(mWat_flow, pro_2.u1) annotation (Line(points={{-120,90},{-72,90},{-20,
              90},{-20,86},{-12,86}}, color={0,0,127}));
      connect(pro_2.y, watFlo_mod.u)
        annotation (Line(points={{11,80},{18,80}}, color={0,0,127}));
      connect(watFlo_nom.y, pro_2.u2) annotation (Line(points={{-49,60},{-20,60},{
              -20,74},{-12,74}},
                             color={0,0,127}));
      connect(TWat, add.u1) annotation (Line(points={{-120,-30},{-60,-30},{-60,-34},
              {-52,-34}}, color={0,0,127}));
      connect(add.u2, TRoo) annotation (Line(points={{-52,-46},{-66,-46},{-80,-46},{
              -80,-88},{-120,-88}}, color={0,0,127}));
      connect(temDif_nom.y, pro_3.u2) annotation (Line(points={{-49,-70},{-20,-70},
              {-20,-46},{-12,-46}}, color={0,0,127}));
      connect(add.y, pro_3.u1) annotation (Line(points={{-29,-40},{-20,-40},{-20,
              -34},{-12,-34}}, color={0,0,127}));
      connect(pro_3.y, temDif_mod.u)
        annotation (Line(points={{11,-40},{14.5,-40},{18,-40}}, color={0,0,127}));
      connect(watFlo_mod.y, mulPro.u[1]) annotation (Line(points={{41,80},{60,80},{60,
              2.8},{64,2.8}}, color={0,0,127}));
      connect(airFlo_mod.y, mulPro.u[2]) annotation (Line(points={{41,20},{56,20},{56,
              4.44089e-16},{64,4.44089e-16}}, color={0,0,127}));
      connect(temDif_mod.y, mulPro.u[3]) annotation (Line(points={{41,-40},{50,-40},
              {60,-40},{60,-2.8},{64,-2.8}}, color={0,0,127}));
      annotation ( defaultComponentName="mod",
                Documentation(info="<html>
<p>
This model determines the three modification factors described in
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.UsersGuide\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.UsersGuide</a>
by comparing the actual values of air mass flow rate,
water mass flow rate and room-water temperature difference with the nominal values.
The three modification factors are then multiplied.
Input to this model are the total mass flow rates of all parallel beams combined.
</p>
</html>",     revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                100}}), graphics={Text(
              extent={{100,100},{-100,-100}},
              lineColor={0,0,0},
              textString="f")}));
    end ModificationFactor;

    package Examples
      extends Modelica.Icons.ExamplesPackage;

      model Convector
        extends Modelica.Icons.Example;

        package Medium = IDEAS.Media.Water "Water model";

        IDEAS.Fluid.Sources.MassFlowSource_T wat(
          redeclare package Medium = Medium,
          m_flow=0.094,
          T=288.15,
          nPorts=1)
          annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
        IDEAS.Fluid.Sources.FixedBoundary bou(
          redeclare package Medium = Medium, nPorts=1) "Pressure boundary condition"
          annotation (Placement(transformation(extent={{80,-10},{60,10}})));
        Modelica.Blocks.Sources.Ramp airFlo(height=0.0792, duration=4)
          "Air mass flow rate"
          annotation (Placement(transformation(extent={{-80,70},{-60,90}})));
        Modelica.Blocks.Sources.Constant rooTem(k=273.15 + 25) "Room air temperature"
          annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
        FBM.Components.ActiveBeams.BaseClasses.Convector con(
          redeclare package Medium = Medium,
          nBeams=1,
          energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
          per(
            mAir_flow_nominal=0.0792,
            mWat_flow_nominal=0.094,
            dT_nominal=-10,
            Q_flow_nominal=1092,
            dpWat_nominal=10000,
            dpAir_nominal=100)) "Convector model"
          annotation (Placement(transformation(extent={{0,-10},{20,10}})));
        IDEAS.Fluid.Sensors.TemperatureTwoPort senTem(
          redeclare package Medium = Medium,
          m_flow_nominal=0.094) "Temperature sensor"
          annotation (Placement(transformation(extent={{30,-10},{50,10}})));
      equation
        connect(airFlo.y, con.mAir_flow) annotation (Line(points={{-59,80},{-10,80},{-10,
                4},{-2,4}},  color={0,0,127}));
        connect(rooTem.y, con.TRoo) annotation (Line(points={{-59,40},{-14,40},{-14,-6},
                {-2,-6}},  color={0,0,127}));
        connect(con.port_b, senTem.port_a)
          annotation (Line(points={{20,0},{26,0},{30,0}},
                                                   color={0,127,255}));
        connect(senTem.port_b, bou.ports[1])
          annotation (Line(points={{50,0},{60,0}}, color={0,127,255}));
        connect(wat.ports[1], con.port_a)
          annotation (Line(points={{-60,0},{-30,0},{0,0}}, color={0,127,255}));
        annotation (experiment(StopTime=10),__Dymola_Commands(file="modelica://IDEAS/Resources/Scripts/Dymola/Fluid/HeatExchangers/ActiveBeams/BaseClasses/Examples/Convector.mos"
              "Simulate and plot"),
          Documentation(info="<html>
<p>
The example tests the implementation of
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.BaseClasses.Convector\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.BaseClasses.Convector</a>.
The room air temperature and the water mass flow rate are constant while the air flow rate varys with a ramp.
</p>
</html>",       revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
      end Convector;

      model DerivateCubicSpline
        extends Modelica.Icons.Example;

        Modelica.Blocks.Sources.Clock clock "Clock"
          annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
        FBM.Components.ActiveBeams.BaseClasses.DerivativesCubicSpline cubSpl
          "Derivatives of cubic spline"
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        connect(clock.y, cubSpl.u)
          annotation (Line(points={{-39,0},{-12,0}},         color={0,0,127}));
        annotation (experiment(StopTime=1),__Dymola_Commands(file="modelica://IDEAS/Resources/Scripts/Dymola/Fluid/HeatExchangers/ActiveBeams/BaseClasses/Examples/DerivateCubicSpline.mos"
              "Simulate and plot"),
              Documentation(info="<html>
<p>
The example tests the implementation of
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.BaseClasses.DerivatesCubicSpline\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.BaseClasses.DerivatesCubicSpline</a>.
Default vectors are: <i>x=[0,0.5,1]</i> and <i>y=[0,0.75,1]</i>.
Input to the model is the simulation time.
</p>
</html>",       revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
      end DerivateCubicSpline;

      model ModificationFactor
         extends Modelica.Icons.Example;

        Modelica.Blocks.Sources.Constant const1(k=20) "Constant input signal"
          annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
        Modelica.Blocks.Sources.Ramp ramp(height=0.0792, duration=1)
          "Ramp input signal"
          annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
        Modelica.Blocks.Sources.Ramp ramp1(height=0.094, duration=1)
          "Ramp input signal"
          annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
        Modelica.Blocks.Sources.Ramp ramp2(
          height=27.8,
          duration=1,
          offset=20) "Ramp input signal"
          annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
        FBM.Components.ActiveBeams.BaseClasses.ModificationFactor mod(nBeams=1,
            per(
            Q_flow_nominal=0.094*2*4200,
            mAir_flow_nominal=0.0792,
            mWat_flow_nominal=0.094,
            dT_nominal=27.8,
            dpWat_nominal=10000,
            dpAir_nominal=100)) "Modification factor"
          annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
      equation
        connect(ramp.y, mod.mAir_flow) annotation (Line(points={{-59,30},{-59,30},{-52,
                30},{-52,3},{-12,3}}, color={0,0,127}));
        connect(ramp1.y, mod.mWat_flow) annotation (Line(points={{-59,70},{-40,70},{-40,
                9},{-12,9}}, color={0,0,127}));
        connect(ramp2.y, mod.TWat) annotation (Line(points={{-59,-30},{-40,-30},{-40,-3},
                {-12,-3}}, color={0,0,127}));
        connect(const1.y, mod.TRoo) annotation (Line(points={{-59,-70},{-20,-70},{-20,
                -8.8},{-12,-8.8}}, color={0,0,127}));
        annotation (            experiment(StopTime=1),
                  __Dymola_Commands(file="modelica://IDEAS/Resources/Scripts/Dymola/Fluid/HeatExchangers/ActiveBeams/BaseClasses/Examples/ModificationFactor.mos"
              "Simulate and plot"),
              Documentation(info="<html>
<p>
This example tests the implementation of
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.BaseClasses.ModificationFactor\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.BaseClasses.ModificationFactor</a>.
</p>
</html>",       revisions="<html>
<ul>
<li>
June 13, 2016, by Michael Wetter:<br/>
Revised implementation.
</li>
<li>
May 20, 2016, by Alessandro Maccarini:<br/>
First implementation.
</li>
</ul>
</html>"));
      end ModificationFactor;
    annotation (Documentation(info="<html>
<p>
This package contains examples for the use of models that can be found in
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams.BaseClasses\">
IDEAS.Fluid.HeatExchangers.ActiveBeams.BaseClasses</a>.
</p>
</html>"));
    end Examples;
  annotation (Documentation(info="<html>
<p>
This package contains base classes that are used to construct the models in
<a href=\"modelica://IDEAS.Fluid.HeatExchangers.ActiveBeams\">IDEAS.Fluid.HeatExchangers.ActiveBeams</a>.
</p>
</html>"));
  end BaseClasses;
annotation (Documentation(info="<html>
<p>
<strong>This package is a duplicate of the IDEAS package ActiveBeams !!</strong></p>
<p>This package contains models of active beams.
See the
<a href=\"modelica://FBM.Components.ActiveBeams.UsersGuide\">
User's Guide</a> for more information.
</p>
</html>"));
end ActiveBeams;
