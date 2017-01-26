within FBM.Controls.ControlSolar;
block CTRL_Solar_Prim
  "Primary loop pump controler for sollar installation based on global insulation"
  extends Modelica.Blocks.Icons.Block;

   parameter Modelica.SIunits.Angle lat(displayUnit="degree") "Latitude";
  parameter Modelica.SIunits.Angle azi(displayUnit="degree")
    "Surface azimuth (0 for south-facing; -90 degree for east-facing; +90 degree for west facing";
  parameter Modelica.SIunits.Angle til(displayUnit="degree")
    "Surface tilt (0 for horizontally mounted collector)";
  parameter Real rho "Ground reflectance";
  parameter Modelica.SIunits.Irradiance HOn = 200
                                                 "Minimum global tilted insulation to turn on the pump";
  parameter Modelica.SIunits.Irradiance HOff = 150
                                                  "Minimum global tilted insulation to turn off the pump";

  parameter Modelica.SIunits.Time riseTime=30
    "Rise time of the filter (time to reach 99.6 % of the speed)";

  Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil(
    til=til,
    lat=lat,
    azi=azi) annotation (Placement(transformation(extent={{-68,70},{-48,90}})));
  Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil(
    til=til,
    lat=lat,
    azi=azi,
    rho=rho) annotation (Placement(transformation(extent={{-68,42},{-48,62}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
        transformation(extent={{-108,70},{-68,110}}), iconTransformation(extent=
           {{-168,32},{-148,52}})));
  BaseClasses.HRefTil hRefTil(rho=rho, til=til)
    annotation (Placement(transformation(extent={{-68,12},{-48,32}})));
  Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=HOff, uHigh=HOn)
    annotation (Placement(transformation(extent={{6,38},{26,58}})));
  Modelica.Blocks.Math.Sum sum1(nin=3)
    annotation (Placement(transformation(extent={{-24,38},{-4,58}})));
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Math.BooleanToReal booleanToReal
    annotation (Placement(transformation(extent={{36,38},{56,58}})));
protected
  Modelica.Blocks.Continuous.Filter filter(
     order=2,
     f_cut=5/(2*Modelica.Constants.pi*riseTime),
     x(each stateSelect=StateSelect.always),
     final filterType=Modelica.Blocks.Types.FilterType.LowPass,
    final analogFilter=Modelica.Blocks.Types.AnalogFilter.Bessel) "Second order filter to improve numerics"
    annotation (Placement(transformation(extent={{64,41},{78,55}})));
public
  Modelica.Blocks.Interfaces.BooleanOutput y1
    annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
equation
  connect(HDirTil.weaBus, weaBus) annotation (Line(
      points={{-68,80},{-68,80},{-88,80},{-88,90}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.Bezier),
                      Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(weaBus, HDifTil.weaBus) annotation (Line(
      points={{-88,90},{-88,90},{-88,52},{-68,52}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.Bezier),
                      Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(weaBus, hRefTil.weaBus) annotation (Line(
      points={{-88,90},{-88,90},{-88,30},{-65.6,30}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.Bezier),
                      Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}}));
  connect(HDirTil.H, sum1.u[1]) annotation (Line(points={{-47,80},{-40,80},{-40,
          46.6667},{-26,46.6667}}, color={0,0,127},
      smooth=Smooth.Bezier));
  connect(HDifTil.H, sum1.u[2]) annotation (Line(points={{-47,52},{-40,52},{-40,
          48},{-26,48}}, color={0,0,127},
      smooth=Smooth.Bezier));
  connect(hRefTil.y, sum1.u[3]) annotation (Line(points={{-47,22},{-40,22},{-40,
          49.3333},{-26,49.3333}}, color={0,0,127},
      smooth=Smooth.Bezier));
  connect(sum1.y, hysteresis.u)
    annotation (Line(points={{-3,48},{-2,48},{4,48}},
                                              color={0,0,127}));
  connect(hysteresis.y, booleanToReal.u)
    annotation (Line(points={{27,48},{30,48},{34,48}}, color={255,0,255}));
  connect(booleanToReal.y, filter.u)
    annotation (Line(points={{57,48},{62.6,48}}, color={0,0,127}));
  connect(filter.y, y) annotation (Line(points={{78.7,48},{92,48},{92,0},{110,0}},
        color={0,0,127}));
  connect(hysteresis.y, y1) annotation (Line(points={{27,48},{30,48},{30,-40},{
          110,-40}},
                 color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),Documentation(info="<html>
    <p>
    This component is a controler for a pump placed into a primary loop of a solar installation.
    </p>
     <p>
     The pump is On if the global tilted insulation is higher than the threeshold <i>HOn</i> and Off if the insulation pass below <i>HOff</i>
    </p>
    
   </html>",
  revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
end CTRL_Solar_Prim;
