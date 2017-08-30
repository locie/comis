within FBM.Heating;
model None "No heating or cooling system"
  extends FBM.Interfaces.BaseClasses.HeatingSystem(
    isHea = false,
    isCoo = false,
    nConvPorts = nZones,
    nRadPorts = nZones,
    nTemSen = nZones,
    nEmbPorts=nZones,
    nZones=1);
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlowCon[
    nZones](each Q_flow=0) if nConvPorts >=1
    annotation (Placement(transformation(extent={{-160,10},{-180,30}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlowRad[
    nZones](each Q_flow=0) if nRadPorts >=1
    annotation (Placement(transformation(extent={{-160,-30},{-180,-10}})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlowEmb[
    nEmbPorts](each Q_flow=0) if nEmbPorts >=1
    annotation (Placement(transformation(extent={{-160,50},{-180,70}})));
equation
  if nConvPorts >=1 then
      connect(prescribedHeatFlowCon.port, heatPortCon) annotation (Line(
      points={{-180,20},{-200,20}},
      color={191,0,0},
      smooth=Smooth.None));
  end if;
  if nEmbPorts >=1 then
    connect(prescribedHeatFlowEmb.port, heatPortEmb) annotation (Line(
      points={{-180,60},{-200,60}},
      color={191,0,0},
      smooth=Smooth.None));
  end if;
  if nRadPorts >=1 then
    connect(prescribedHeatFlowRad.port, heatPortRad) annotation (Line(
      points={{-180,-20},{-200,-20}},
      color={191,0,0},
      smooth=Smooth.None));
  end if;
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
            -100},{200,100}}), graphics));
end None;
