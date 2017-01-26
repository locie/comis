within FBM.Interfaces.BaseClasses;
partial model HeatingSystem "Partial heating/cooling system"
  extends FBM.Interfaces.BaseClasses.PartialSystem;
  replaceable package Medium=Buildings.Media.Water;
  // *********** Building characteristics and  interface ***********
  // --- General
  parameter Integer nZones(min=1)    "Number of conditioned thermal zones deserved by the system" annotation(Dialog(group= "Settings parameters"));
  parameter Boolean includePipes=false
    "Set to true to include pipes in the basecircuit" annotation(Dialog(group= "Settings parameters"));
  // --- Boolean declarations
  parameter Boolean isAHU=false "true if system have an AHU loop";
  parameter Boolean isMulti=false "true if system have multi heatsource";
  parameter Boolean isHea=true "true if system is able to heat";
  parameter Boolean isCoo=false "true if system is able to cool";
  parameter Boolean isDH=false "true if the system is connected to a DH grid";
  parameter Boolean InInterface = false;
  // --- Ports
  parameter Integer nConvPorts(min=0) = nZones
    "Number of ports in building for convective heating/cooling";
  parameter Integer nRadPorts(min=0) = nZones
    "Number of ports in building for radiative heating/cooling";
  parameter Integer nEmbPorts(min=0) = nZones
    "Number of ports in building for embedded systems";
  // --- Sensor
  parameter Integer nTemSen(min=0) = nZones
    "number of temperature inputs for the system";
  // *********** Outputs ***********
  // --- Thermal
  Modelica.SIunits.Power QHeaSys if isHea
    "Total energy use forspace heating + DHW, if present)";
  Modelica.SIunits.Power QCooTotal if isCoo "Total cooling energy use";
  // *********** Interface ***********
  // --- thermal
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nConvPorts] heatPortCon
    "Nodes for convective heat gains"
    annotation (Placement(transformation(extent={{-128,10},{-108,30}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nRadPorts] heatPortRad
    "Nodes for radiative heat gains"
    annotation (Placement(transformation(extent={{-130,-30},{-110,-10}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b[nEmbPorts] heatPortEmb
    "Construction nodes for heat gains by embedded layers"
    annotation (Placement(transformation(extent={{-70,-110},{-50,-90}}),
        iconTransformation(extent={{-70,-110},{-50,-90}})));
  // --- Sensor
  Modelica.Blocks.Interfaces.RealInput[nTemSen] TSensor(
    final quantity="ThermodynamicTemperature",
    unit="K",
    displayUnit="degC",
    min=0) "Sensor temperature" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-124,-60})));
  Modelica.Blocks.Interfaces.RealInput mDHW60C
    "mFlow for domestic hot water, at 60 degC" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={80,-104}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={60,-102})));
  Modelica.Blocks.Interfaces.RealInput[nZones] TSet(
    final quantity="ThermodynamicTemperature",
    unit="K",
    displayUnit="degC",
    min=0) "Setpoint temperature for the zones" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-40,104}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,-102})));
  // --- fluid
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,
            100}}), graphics={
        Rectangle(
          extent={{-180,100},{160,-100}},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid,
          lineColor={255,0,0}),
        Line(
          points={{50,-20},{30,0}},
          color={0,0,127}),
        Line(
          points={{30,0},{0,-30}},
          color={0,0,127},
          pattern=LinePattern.Dash),
        Line(
          points={{30,0},{-8,0}},
          color={191,0,0},
          thickness=0.5),
        Line(
          points={{-28,-20},{-128,-20}},
          color={191,0,0},
          thickness=0.5),
        Line(
          points={{-28,20},{-128,20}},
          color={191,0,0},
          thickness=0.5),
        Line(
          points={{-8,0},{-28,-20}},
          color={191,0,0},
          thickness=0.5),
        Line(
          points={{-8,0},{-28,20}},
          color={191,0,0},
          thickness=0.5),
        Polygon(
          points={{-128,0},{-128,40},{-158,20},{-128,0}},
          lineColor={191,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-128,-40},{-128,0},{-158,-20},{-128,-40}},
          lineColor={191,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-158,40},{-178,-40}},
          lineColor={191,0,0},
          fillColor={191,0,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{60,-30},{50,-20}},
          color={0,0,127},
          pattern=LinePattern.Dash),
        Line(
          points={{0,-100},{0,-30}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash),
        Line(
          points={{60,-100},{60,-30}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash),
      Text(extent={{-150.0,-150.0},{150.0,-110.0}},
          textString="Ti=%Ti"),
        Rectangle(
          extent={{-2,40},{60,-18}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
      Line(points={{4,-12},{48,-12}},
          color={0,0,0}),
      Polygon(lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid,
          points={{52,-12},{48,-10},{48,-14},{52,-12}}),
        Line(points={{4,30},{4,-12}},
            color={0,0,0}),
      Polygon(lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Solid,
          points={{4,34},{2,30},{6,30},{4,34}}),
      Line(points={{10,4},{10,14},{16,18}},        color={0,0,0})}),
    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{
            120,100}})),
    Documentation(info="<html>
<p><b>Description</b> </p>
<p>Interface model for a complete multi-zone heating system (with our without domestic hot water and solar system).</p>
<p>This model defines the ports used to link a heating system with a building, and the basic parameters that most heating systems will need to have. The model is modular as a function of the number of zones <i>nZones. </i></p>
<p>Two sets of heatPorts are defined:</p>
<p><ol>
<li><i>heatPortCon[nZones]</i> and <i>heatPortRad[nZones]</i> for convective respectively radiative heat transfer to the building. </li>
<li><i>heatPortEmb[nZones]</i> for heat transfer to TABS elements in the building. </li>
</ol></p>
<p>The model also defines <i>TSensor[nZones]</i> and <i>TSet[nZones]</i> for the control, and a nominal power <i>QNom[nZones].</i></p>
<p>There is also an input for the DHW flow rate, <i>mDHW60C</i>, but this can be unconnected if the system only includes heating and no DHW.</p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>See the different extensions of this model in <a href=\"modelica://IDEAS.Thermal.HeatingSystems\">IDEAS.Thermal.HeatingSystems</a></li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> and <i>plugLoad. </i></li>
</ol></p>
<p><h4>Validation </h4></p>
<p>No validation performed.</p>
</html>"));
end HeatingSystem;
