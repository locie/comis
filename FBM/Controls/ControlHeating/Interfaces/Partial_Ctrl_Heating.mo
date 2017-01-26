within FBM.Controls.ControlHeating.Interfaces;
partial model Partial_Ctrl_Heating
  "Partial for a heating control algorithm without TES"
  /* 
  This partial class contains the temperature control algorithm. It has to be extended
  in order to be complete controller.  
  */
  extends FBM.Controls.ControlHeating.Interfaces.ControlPara;
  HeatingCurves.HeatingCurveFilter heatingCurve(
    timeFilter=timeFilter,
    TSup_nominal=TSupNom,
    TRet_nominal=TSupNom - dTSupRetNom,
    redeclare FBM.Utilities.Math.MovingAverage filter(period=timeFilter),
    TSupMin=TSupMin,
    minSup=minSup,
    TRoo_nominal=TRoo_nominal,
    TOut_nominal=TOut_nominal,
    use_TRoo_in=true,
    dTOutHeaBal=dTOutHeaBal)
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  Modelica.Blocks.Interfaces.RealOutput THeaterSet(final quantity="ThermodynamicTemperature",unit="K",displayUnit="degC", min=0,start=283.15)
    "Heat pump set temperature" annotation (Placement(transformation(extent={{80,0},{
            100,20}}),          iconTransformation(extent={{80,0},{100,20}})));
  Modelica.Blocks.Interfaces.RealOutput THeaCur(final quantity="ThermodynamicTemperature",unit="K",displayUnit="degC", min=0)
    "Heating curve setpoint"
    annotation (Placement(transformation(extent={{90,30},{110,50}}),
        iconTransformation(extent={{90,30},{110,50}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=heatingCurve.TSup + dTHeaterSet)
    annotation (Placement(transformation(extent={{-44,0},{0,20}})));
  Modelica.Blocks.Interfaces.RealInput TRoo_in1(final quantity="ThermodynamicTemperature",unit="K",displayUnit="degC", min=0)
    "Room air temperature set point"
    annotation (Placement(transformation(extent={{-120,24},{-80,64}}),
        iconTransformation(extent={{-110,30},{-90,50}})));
  Modelica.Blocks.Sources.RealExpression corHeaCur(y=corFac_val)
    "Correction term on the heating curve"
    annotation (Placement(transformation(extent={{-20,20},{0,40}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
        transformation(extent={{-90,60},{-50,100}}), iconTransformation(extent={
            {-186,52},{-166,72}})));
equation
  connect(heatingCurve.TRoo_in, TRoo_in1) annotation (Line(
      points={{-21.9,44},{-100,44}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(weaBus.TDryBul, heatingCurve.TOut) annotation (Line(
      points={{-70,80},{-48,80},{-48,56},{-22,56}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  annotation (Icon(coordinateSystem(extent={{-100,-100},{100,100}},
          preserveAspectRatio=false),                             graphics={
        Text(
          extent={{-151,147},{149,107}},
          lineColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255},
          textString="%name"),
          Rectangle(
          extent={{120,60},{-80,-60}},
          lineColor={135,135,135},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Polygon(
          points={{-80,60},{-80,-60},{-40,0},{-80,60}},
          smooth=Smooth.None,
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None,
          lineColor={135,135,135})}),
                               Diagram(coordinateSystem(extent={{-100,-100},{
            100,100}},
                  preserveAspectRatio=false)),Documentation(revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
Redefining for compatibility with the Buildings library</li>
</ul></p>
</html>"));
end Partial_Ctrl_Heating;
