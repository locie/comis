within FBM.Controls.Discrete;
block Timer_Off
  "This timer gives 0 during timing and 1 outside of timing"
  extends Modelica.Blocks.Icons.Block;
  parameter Modelica.SIunits.Time timing = 0 "Interval of time during the output is 0 then switch to 1";
protected
  Modelica.SIunits.Time entryTime "Time instant when u became true";
  Modelica.Blocks.Interfaces.RealOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput u
    annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
  Modelica.Blocks.Math.RealToBoolean Translate
    annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
  Modelica.Blocks.Logical.Switch switch1
    annotation (Placement(transformation(extent={{34,-10},{54,10}})));
  Modelica.Blocks.Sources.Constant On(k=0)
    annotation (Placement(transformation(extent={{-64,58},{-44,78}})));
  Modelica.Blocks.Sources.Constant Off(k=1)
    annotation (Placement(transformation(extent={{-12,-40},{8,-20}})));
  Buildings.Controls.Continuous.OffTimer
           offHys
    annotation (Placement(transformation(extent={{-48,26},{-28,46}})));
protected
  Modelica.Blocks.Logical.And and3
    annotation (Placement(transformation(extent={{-4,-10},{16,10}})));
public
  Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold(threshold=
       timing)  annotation (Placement(transformation(extent={{-20,26},{0,46}})));
equation
 when (not Translate.y) then
    entryTime = time;
  end when;
  connect(u, Translate.u)
    annotation (Line(points={{-100,0},{-76,0}},         color={0,0,127}));
  connect(On.y, switch1.u1) annotation (Line(points={{-43,68},{18,68},{18,8},{32,
          8}}, color={0,0,127}));
  connect(Off.y, switch1.u3) annotation (Line(points={{9,-30},{16,-30},{16,-8},{
          32,-8}}, color={0,0,127}));
  connect(switch1.y, y)
    annotation (Line(points={{55,0},{110,0}}, color={0,0,127}));
  connect(Translate.y, offHys.u)
    annotation (Line(points={{-53,0},{-50,0},{-50,36}}, color={255,0,255}));
  connect(Translate.y, and3.u1)
    annotation (Line(points={{-53,0},{-6,0}}, color={255,0,255}));
  connect(and3.y, switch1.u2)
    annotation (Line(points={{17,0},{32,0}}, color={255,0,255}));
  connect(offHys.y, lessEqualThreshold.u)
    annotation (Line(points={{-27,36},{-22,36}}, color={0,0,127}));
  connect(lessEqualThreshold.y, and3.u2) annotation (Line(points={{1,36},{-4,36},
          {-4,-8},{-6,-8}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Timer_Off;
