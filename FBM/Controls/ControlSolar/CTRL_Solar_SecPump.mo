within FBM.Controls.ControlSolar;
block CTRL_Solar_SecPump "Controler for secondary loop pump"
  extends Modelica.Blocks.Icons.Block;
  Modelica.Blocks.Math.Add add(final k2=-1)
    "Compares the current insolation to the critical insolation"
    annotation (Placement(transformation(extent={{-82,-18},{-60,4}})));
  Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=TLow, uHigh=THigh)
    annotation (Placement(transformation(extent={{-52,-18},{-32,2}})));
  Modelica.Blocks.Interfaces.RealInput THotSol
    "Outlet temperature of solar collector"
    annotation (Placement(transformation(extent={{-130,-20},{-90,20}})));
  Modelica.Blocks.Interfaces.RealInput TBoTank
    "Temperature at the bottom of the tank"
    annotation (Placement(transformation(extent={{-126,-100},{-86,-60}})));
  Modelica.Blocks.Logical.And and1
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
  Modelica.Blocks.Logical.Switch switch3
    annotation (Placement(transformation(extent={{38,-10},{58,10}})));
  Modelica.Blocks.Sources.Constant pumOff(k=0) "Pump off signal"
    annotation (Placement(transformation(extent={{-20,-48},{0,-28}})));
  parameter Modelica.SIunits.Time riseTime=30
    "Rise time of the filter (time to reach 99.6 % of the speed)";
parameter Modelica.SIunits.TemperatureDifference THigh = 5 "Minimum difference temperature to turn on the pump";
parameter Modelica.SIunits.TemperatureDifference TLow = 2 "Minimum difference temperature to turn off the pump";
  Modelica.Blocks.Interfaces.BooleanInput P0 "Primary pump state"
    annotation (Placement(transformation(extent={{-130,46},{-90,86}})));
  Modelica.Blocks.Sources.Constant pumOn(k=1) "Pump off signal"
    annotation (Placement(transformation(extent={{-20,30},{0,50}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
protected
  Modelica.Blocks.Continuous.Filter filter(
     order=2,
     f_cut=5/(2*Modelica.Constants.pi*riseTime),
     x(each stateSelect=StateSelect.always),
     final filterType=Modelica.Blocks.Types.FilterType.LowPass,
    final analogFilter=Modelica.Blocks.Types.AnalogFilter.Bessel) "Second order filter to improve numerics"
    annotation (Placement(transformation(extent={{72,-7},{86,7}})));
equation
  connect(add.y,hysteresis. u) annotation (Line(
      points={{-58.9,-7},{-54,-7},{-54,-8}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(THotSol, add.u1) annotation (Line(
      points={{-110,0},{-88,0},{-88,-0.4},{-84.2,-0.4}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(TBoTank, add.u2) annotation (Line(
      points={{-106,-80},{-98,-80},{-98,-13.6},{-84.2,-13.6}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(hysteresis.y, and1.u2) annotation (Line(
      points={{-31,-8},{-24,-8},{-22,-8}},
      color={255,0,255},
      smooth=Smooth.Bezier));
  connect(and1.y, switch3.u2) annotation (Line(
      points={{1,0},{1,0},{36,0}},
      color={255,0,255},
      smooth=Smooth.Bezier));
  connect(pumOff.y, switch3.u3) annotation (Line(
      points={{1,-38},{18,-38},{18,-8},{36,-8}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(P0, and1.u1)
    annotation (Line(points={{-110,66},{-28,66},{-28,0},{-22,0}},
                                                  color={255,0,255},
      smooth=Smooth.Bezier));
  connect(pumOn.y, switch3.u1) annotation (Line(
      points={{1,40},{18,40},{18,8},{36,8}},
      color={0,0,127},
      smooth=Smooth.Bezier));
  connect(switch3.y, filter.u) annotation (Line(points={{59,0},{64.8,0},{64.8,8.88178e-16},
          {70.6,8.88178e-16}}, color={0,0,127}));
  connect(filter.y, y) annotation (Line(points={{86.7,8.88178e-16},{98.35,8.88178e-16},
          {98.35,0},{110,0}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),Documentation(info="<html>
    <p>
    This component is a controler for a pump placed into a secondary loop of a solar panels installation (Between a heat exchanger and a storage tank).
    </p>
     <p>
     The pump is On if the primary pump is On and if the difference between the bottom storage tank temperature and the primary loop temperature is higher than <i>THigh</i>
     </p>
          <p>
     The pump is Off if the primary pump is Off or if the difference between the bottom storage tank temperature and the primary loop temperature is lower than <i>TLow</i>
    </p>
    
   </html>",
  revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
end CTRL_Solar_SecPump;
