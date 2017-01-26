within FBM.Controls.ControlSolar;
package BaseClasses
    extends Modelica.Icons.VariantsPackage;

  block HRefTil "Reflected insulation on a tilted surface"
   extends Modelica.Blocks.Icons.Block;
   parameter Real rho=0.2 "Ground reflectance";
    parameter Modelica.SIunits.Angle til
                                        "Surface tilt angle";

protected
    Modelica.Blocks.Interfaces.RealInput u
      annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));

public
    Modelica.Blocks.Interfaces.RealOutput y
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
          transformation(extent={{-106,50},{-66,90}}), iconTransformation(extent={
              {-86,70},{-66,90}})));

  equation

    y=rho*(1-Modelica.Math.cos(til))*u;

    connect(weaBus.HGloHor, u) annotation (Line(
        points={{-86,70},{-92,70},{-92,0},{-100,0}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),Documentation(info="<html>
    <p>
    This component calculates the reflected tilted solar radiation.
    </p>
    <p>
    The reflected tilted solar radiation is calculated using Equation :
    </p>
    <p align=\"center\" style=\"font-style:italic;\">
      H<sub>ref</sub>=Rho*(1-cos(til))*H<sub>GloHor</sub>
         </p>
    <p>
      where <i>H<sub>ref</sub></i> is the reflected tilted solar radiation,
      <i>Rho</i> is the ground reflectance coefficient,
      <i>til</i> is the tilted of the installation; and <i>H<sub>GloHor</sub></i> is the global horizontal solar radiation.
    </p>
   </html>",
    revisions="<html>
<p><ul>
<li>November 2016 by Wilfried Thomaré:<br> 
First implementationn</li>
</ul></p>
</html>"));
  end HRefTil;
end BaseClasses;
