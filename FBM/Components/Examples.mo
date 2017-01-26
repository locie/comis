within FBM.Components;
package Examples
    extends Modelica.Icons.ExamplesPackage;
  model SolarPannelField

    extends Modelica.Icons.Example;

      package MediumA = Buildings.Media.Air
                                           "Medium model for air";
      package MediumW = Buildings.Media.Water "Medium model";
      package MediumG =FBM.Media.Glycol20Water80 "Medium model of Glycol";
    FBM.Components.SolarPannelFieldWithTes solarPannelField(
      perColector=
          Buildings.Fluid.SolarCollectors.Data.GlazedFlatPlate.FP_SolahartKf(),
      redeclare package Medium = MediumW,
      includePipes=true,
      nPar=10,
      nSer=10,
      dpHexPrim(displayUnit="Pa") = 19000,
      dpHexSecon(displayUnit="Pa") = 19000,
      Pipelength=20,
      InsuHeatCondu=0.032,
      T_a1_nominal(displayUnit="K"),
      T_a2_nominal(displayUnit="K") = 273.15 + 35,
      redeclare package MediumPrim = MediumG,
      VTan=25,
      hTan=1.8,
      m_flow_nominal=1,
      mPrim_flow_nominal=2)
      annotation (Placement(transformation(extent={{18,-32},{64,14}})));

    Buildings.Fluid.Sources.Boundary_pT bou(
                        redeclare package Medium = MediumW, nPorts=1)
                        "Outlet for hot water draw"
      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-26,-10})));
    Buildings.Fluid.Sources.MassFlowSource_T bou1(
      use_m_flow_in=false,
      redeclare package Medium = MediumW,
      nPorts=1,
      m_flow=0.25,
      T=308.15) "Inlet and flow rate for hot water draw"
      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        origin={90,-10})));
    Buildings.HeatTransfer.Sources.FixedTemperature rooT(T=290.15)
      "Room temperature"
      annotation (Placement(transformation(extent={{-30,46},{-10,66}})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
      "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos",
      computeWetBulbTemperature=false) "Weather data file reader"
      annotation (Placement(transformation(extent={{-30,18},{-10,38}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
          MediumW, m_flow_nominal=0.1)
      annotation (Placement(transformation(extent={{10,-20},{-10,0}})));
  equation
    connect(rooT.port, solarPannelField.heatPort)
      annotation (Line(points={{-10,56},{41,56},{41,13.54}}, color={191,0,0}));
    connect(weaDat.weaBus, solarPannelField.weaBus) annotation (Line(
        points={{-10,28},{24,28},{24,9.86},{23.52,9.86}},
        color={255,204,51},
        thickness=0.5));

        if solarPannelField.includePipes then
      connect(rooT.port, solarPannelField.heatPort);
        end if;

    connect(bou1.ports[1], solarPannelField.port_b)
      annotation (Line(points={{80,-10},{64,-10},{64,-9}}, color={0,127,255}));
    connect(bou.ports[1], senTem.port_b) annotation (Line(points={{-16,-10},{
            -14,-10},{-10,-10}}, color={0,127,255}));
    connect(senTem.port_a, solarPannelField.port_a) annotation (Line(points={{
            10,-10},{10,-10},{10,-9},{18,-9}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -120},{100,100}})),                                  Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-40,-60},{100,80}})));
  end SolarPannelField;

  model BalancedTap
    import FBM;
   extends Modelica.Icons.Example;

      package MediumA = Buildings.Media.Air
                                           "Medium model for air";
      package MediumW = Buildings.Media.Water "Medium model";
    FBM.Components.SolarPannelFieldWithTes solarPannelField(
      perColector=
          Buildings.Fluid.SolarCollectors.Data.GlazedFlatPlate.FP_SolahartKf(),
      redeclare package Medium = MediumW,
      redeclare package MediumPrim = MediumW,
      includePipes=true,
      nPar=10,
      dpHexPrim(displayUnit="Pa") = 19000,
      dpHexSecon(displayUnit="Pa") = 19000,
      Pipelength=20,
      InsuHeatCondu=0.032,
      T_a1_nominal(displayUnit="K"),
      m_flow_nominal=1,
      mPrim_flow_nominal=1,
      mSecon_flow_nominal=1,
      T_a2_nominal(displayUnit="K") = 273.15 + 35,
      nSer=20)
      annotation (Placement(transformation(extent={{46,-96},{92,-50}})));

    Buildings.Fluid.Storage.StratifiedEnhancedInternalHex
     tan(
      CHex=200,
      dExtHex=0.01905,
      hHex_a=0.9,
      hHex_b=0.65,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      Q_flow_nominal=3000,
      mHex_flow_nominal=3000/20/4200,
      energyDynamicsHex=Modelica.Fluid.Types.Dynamics.FixedInitial,
      redeclare package Medium = MediumW,
      redeclare package MediumHex = MediumW,
      VTan=25,
      m_flow_nominal=1,
      dIns=0.10,
      hTan=1.8,
      T_start=323.15,
      TTan_nominal=293.15,
      THex_nominal=323.15)
      "Storage tank model"
      annotation (Placement(transformation(
        extent={{-15,-15},{15,15}},
        origin={29,51})));
    Buildings.HeatTransfer.Sources.FixedTemperature rooT(T=293.15)
      "Room temperature"
      annotation (Placement(transformation(extent={{16,-36},{36,-16}})));
    Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor TTan
      "Temperature in the tank water that surrounds the heat exchanger"
      annotation (Placement(transformation(extent={{68,4},{80,16}})));
    Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
      "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos",
      computeWetBulbTemperature=false) "Weather data file reader"
      annotation (Placement(transformation(extent={{16,-64},{36,-44}})));
    Buildings.Fluid.Storage.ExpansionVessel exp(redeclare package Medium =
          MediumW, V_start=1,
      p=300000)                                         "Expansion tank"
      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-6,76})));

    FBM.Components.BalancedTap balancedTap(
      redeclare package Medium = MediumW,
      VDayAvg=5,
      profileType=2,
      m_flow_nominal=0.25)
                 annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={-20,40})));
    Buildings.Fluid.Storage.ExpansionVessel exp1(
                                                redeclare package Medium =
          MediumW, V_start=1,
      p=100000)                                         "Expansion tank"
      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        origin={-62,64})));
  equation
    connect(solarPannelField.port_b, tan.portHex_a) annotation (Line(points={{92,-73},
            {96,-73},{96,-34},{4,-34},{4,45.3},{14,45.3}},    color={0,127,255}));
    connect(solarPannelField.port_a, tan.portHex_b) annotation (Line(points={{46,-73},
            {10,-73},{10,39},{14,39}},  color={0,127,255}));
    connect(rooT.port, tan.heaPorBot)
      annotation (Line(points={{36,-26},{32,-26},{32,39.9}},
                                                          color={191,0,0}));
    connect(rooT.port, tan.heaPorTop)
      annotation (Line(points={{36,-26},{32,-26},{32,62.1}},
                                                          color={191,0,0}));
    connect(rooT.port, tan.heaPorSid) annotation (Line(points={{36,-26},{40,-26},{
            40,51},{37.4,51}},
                            color={191,0,0}));
    connect(tan.heaPorVol[4], TTan.port) annotation (Line(points={{29,51},{29,
            10.5},{68,10.5},{68,10}},
                                color={191,0,0}));
    connect(rooT.port, solarPannelField.heatPort) annotation (Line(points={{36,
            -26},{69,-26},{69,-50.46}}, color={191,0,0}));
    connect(TTan.T, solarPannelField.TbotTank) annotation (Line(points={{80,10},{96,
            10},{96,-86.8},{93.38,-86.8}},   color={0,0,127}));
    connect(weaDat.weaBus, solarPannelField.weaBus) annotation (Line(
        points={{36,-54},{36,-56},{44,-56},{44,-54.14},{51.52,-54.14}},
        color={255,204,51},
        thickness=0.5));

        if solarPannelField.includePipes then
      connect(rooT.port, solarPannelField.heatPort);
        end if;

    connect(exp.port_a, tan.portHex_b) annotation (Line(points={{-6,66},{-6,39},
            {14,39}}, color={0,127,255}));
    connect(balancedTap.port_hot, tan.port_a)
      annotation (Line(points={{-20,50},{14,50},{14,51}}, color={0,127,255}));
    connect(balancedTap.port_cold, tan.port_b) annotation (Line(points={{-20,30},
            {48,30},{48,51},{44,51}}, color={0,127,255}));
    connect(exp1.port_a, tan.port_b) annotation (Line(points={{-62,54},{-8,54},
            {-8,51},{44,51}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)),
                Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},
              {100,100}})),                                        Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-40,-60},{100,80}})));
  end BalancedTap;
end Examples;
