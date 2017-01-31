within FBM;
package HeatingDHWsystems "Fully functional multi-zone heating and DHW systems"
  package ModularProductionSystem
    "Package with modular source of heat production but fix emited production"
    model SingleProduction_Radiator
      extends
        FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_SingleSource(
        QBoiler=sum(QNom),
        final isHea=true,
        final isCoo=false,
        nConvPorts=nZones,
        nRadPorts=nZones,
        nTemSen=nZones,
        nEmbPorts=0,
        nZones=1,
        nSources=1,
        minSup=true,
        TSupMin=273.15 + 25,
        redeclare Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2
          emission[nZones](
          each T_a_nominal=TSupNom,
          each T_b_nominal=TSupNom - dTSupRetNom,
          TAir_nominal=TRoomNom,
          Q_flow_nominal=QNom,
          redeclare each package Medium = Medium),
        pipeSupply(redeclare package Medium = Medium),
        pipeReturn(redeclare package Medium = Medium),
        pipeReturnEmission(redeclare package Medium = Medium),
        pumpRad(redeclare package Medium = Medium),
        vol(redeclare package Medium = Medium),
        senTemHea_out(redeclare package Medium = Medium),
        senTemEm_in(redeclare package Medium = Medium),
        idealCtrlMixer(redeclare package Medium = Medium),
        senTemEm_out(redeclare package Medium = Medium),
        spl(redeclare package Medium = Medium),
        absolutePressure(redeclare package Medium = Medium),
        conVal(
          initType=Modelica.Blocks.Types.InitPID.InitialState,
          xi_start=1,
          controllerType=Modelica.Blocks.Types.SimpleController.PI,
          Ti=600,
          k=0.2),
        onOffController(bandwidth=20),
        conHeatSource(
          initType=Modelica.Blocks.Types.InitPID.InitialState,
          k=1,
          Ti=1200,
          controllerType=Modelica.Blocks.Types.SimpleController.P),
        heater(redeclare package Medium = Medium));
      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{-200,-50},{-220,-30}})));
    equation
      QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
      connect(senTemHea_out.T, y) annotation (Line(points={{-52,69},{-106,69},{-106,
              -40},{-210,-40}}, color={0,0,127}));
      connect(heatPortCon, emission.heatPortCon) annotation (Line(points={{-118,20},
              {-200,20},{-200,100},{152,100},{152,67.2},{152,67.2}},                                                                       color={191,0,0}));
      connect(emission.heatPortRad, heatPortRad) annotation (Line(points={{158,67.2},
              {158,67.2},{158,100},{-120,100},{-120,-20}},                                                                        color={191,0,0}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{
                200,100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-200,-100},{200,100}})));
    end SingleProduction_Radiator;

    model SingleProduction_Radiator_DHW
     replaceable parameter
        FBM.Components.BaseClasses.RadiantSlabChar[nEmbPorts] RadSlaCha constrainedby
        FBM.Components.BaseClasses.RadiantSlabChar
        "Properties of the floor heating or TABS, if present";
      extends FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_SingleSource(
        QBoiler=sum(QNom),
        final isHea=true,
        final isCoo=false,
        nConvPorts=nZones,
        nRadPorts=nZones,
        nTemSen=nZones,
        nEmbPorts=0,
        nZones=1,
        nSources=1,
        minSup=true,
        TSupMin=273.15 + 25,
        redeclare Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2
          emission[nZones](
          each T_a_nominal=TSupNom,
          each T_b_nominal=TSupNom - dTSupRetNom,
          TAir_nominal=TRoomNom,
          Q_flow_nominal=QNom,
          redeclare each package Medium = Medium),
          redeclare FBM.Controls.ControlHeating.Ctrl_Heating_DHW ctrl_Heating(
          TDHWSet=TDHWSet,
          TColdWaterNom=TColdWaterNom,
          dTHPTankSet=dTHPTankSet),
        pipeSupply(redeclare package Medium = Medium),
        pipeReturn(redeclare package Medium = Medium),
        pipeReturnEmission(redeclare package Medium = Medium),
        pumpRad(redeclare package Medium = Medium),
        vol(redeclare package Medium = Medium),
        senTemHea_out(redeclare package Medium = Medium),
        senTemEm_in(redeclare package Medium = Medium),
        idealCtrlMixer(redeclare package Medium = Medium),
        senTemEm_out(redeclare package Medium = Medium),
        spl(redeclare package Medium = Medium),
        absolutePressure(redeclare package Medium = Medium),
        conVal(
          initType=Modelica.Blocks.Types.InitPID.InitialState,
          xi_start=1,
          controllerType=Modelica.Blocks.Types.SimpleController.PI,
          Ti=600,
          k=0.2),
        onOffController(bandwidth=20),
        conHeatSource(
          initType=Modelica.Blocks.Types.InitPID.InitialState,
          k=1,
          controllerType=Modelica.Blocks.Types.SimpleController.PI,
          Ti=1200),
        heater(redeclare package Medium = Medium));
        // --- Domestic Hot Water (DHW) Parameters
      parameter Integer nOcc = 1 "Number of occupants";
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal_DHW = nOcc*nOcc*0.045*983/(3600*24)*10
        "nominal mass flow rate of DHW";
      parameter Modelica.SIunits.Temperature TDHWSet(max=273.15 + 60) = 273.15 + 45
        "DHW temperature setpoint";
      parameter Modelica.SIunits.Temperature TColdWaterNom=273.15 + 10
        "Nominal tap (cold) water temperature";
      // --- Storage Tank Parameters
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal_stoHX = m_flow_nominal_DHW * (TDHWSet - TColdWaterNom)/dTHPTankSet
        "nominal mass flow rate of HX of storage tank";
      parameter Modelica.SIunits.TemperatureDifference dTHPTankSet(min=1)=2
        "Difference between tank setpoint and heat pump setpoint";
      parameter Modelica.SIunits.Volume volumeTank=0.25;
      parameter Integer nbrNodes=10 "Number of nodes in the tank";
      parameter Integer posTTop(max=nbrNodes) = 1
        "Position of the top temperature sensor";
      parameter Integer posTBot(max=nbrNodes) = nbrNodes - 2
        "Position of the bottom temperature sensor";
      parameter Integer posOutHP(max=nbrNodes + 1) = if solSys then nbrNodes - 1
         else nbrNodes + 1 "Position of extraction of TES to HP";
      parameter Integer posInSTS(max=nbrNodes) = nbrNodes - 1
        "Position of injection of STS in TES";
      parameter Boolean solSys(fixed=true) = false;
          // --- Storage tank and hydraulic circuit connect to its heat exchanger
      Buildings.Fluid.Movers.FlowControlled_m_flow pumpSto(
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal_stoHX,
        tau=30,
        filteredSpeed=true,
        riseTime=100) "Pump for loading the storage tank"
        annotation (Placement(transformation(extent={{-30,-56},{-44,-44}})));
      Buildings.Fluid.Storage.StratifiedEnhancedInternalHex DHWTank(
        port_a(m_flow(start=0)),
        redeclare package Medium = Medium,
        hTan=1.8,
        VTan=volumeTank,
        mHex_flow_nominal=m_flow_nominal_stoHX,
        nSeg=nbrNodes,
        dIns=InsuPipeThickness,
        kIns=InsuHeatCondu,
        m_flow_nominal=m_flow_nominal_DHW,
        redeclare package MediumHex = Medium,
        hHex_a=volumeTank/2,
        hHex_b=volumeTank/3,
        THex_nominal=heater.T_nominal,
        Q_flow_nominal=m_flow_nominal_stoHX*4180*(heater.T_nominal - 60),
        T_start=323.15,
        TTan_nominal=333.15)                   annotation (Placement(transformation(
            extent={{-14,-20},{14,20}},
            rotation=0,
            origin={-8,6})));
          // --- Domestic Hot Water and it hydraulic circuit
      replaceable FBM.Components.BalancedTap_m_flow dHW(
        TDHWSet=TDHWSet,
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal_DHW) constrainedby
        FBM.Components.BaseClasses.BalancedTap( redeclare package Medium = Medium,
          TDHWSet=TDHWSet) annotation (Placement(transformation(
            extent={{-9,5},{9,-5}},
            rotation=-90,
            origin={-47,1})));
      Buildings.Fluid.FixedResistances.Pipe pipeDHW(redeclare package Medium =  Medium,
        m_flow_nominal=m_flow_nominal_DHW,
        nSeg=3,
        length=Pipelength,
        lambdaIns=InsuHeatCondu,
        thicknessIns=InsuPipeThickness)
        annotation (Placement(transformation(extent={{-30,-32},{-46,-26}})));
      Buildings.Fluid.Sources.FixedBoundary absolutePressure1(
        redeclare package Medium = Medium,
        use_T=false,
        nPorts=1) annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={8,-42})));
     // --- Temperature sensors
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemSto_top(redeclare
          package                                                                Medium =
                   Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the top outlet of the storage tank (port_a)"
        annotation (Placement(transformation(extent={{-28,18},{-36,26}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemSto_bot(redeclare
          package                                                                Medium =
                   Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the bottom inlet of the storage tank (port_b)"
        annotation (Placement(transformation(extent={{-6,-34},{-16,-24}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemStoHX_out(redeclare
          package                                                                  Medium =
                   Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the outlet of the storage tank heat exchanger (port_bHX)"
        annotation (Placement(transformation(extent={{-66,-54},{-78,-42}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nbrNodes] TTank
        annotation (Placement(transformation(extent={{10,2},{20,12}})));
      Modelica.Blocks.Math.Gain gain(k=m_flow_nominal_stoHX) annotation (Placement(
            transformation(
            extent={{-8,-8},{8,8}},
            rotation=270,
            origin={-84,-16})));
    equation
      QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
      connect(heatPortCon, emission.heatPortCon) annotation (Line(points={{-118,20},
              {-200,20},{-200,100},{152,100},{152,67.2},{152,67.2}},                                                                       color={191,0,0}));
      connect(emission.heatPortRad, heatPortRad) annotation (Line(points={{158,
              67.2},{158,67.2},{158,100},{-120,100},{-120,-20}},                                                                  color={191,0,0}));
      connect(pipeDHW.heatPort, fixedTemperature.port) annotation (Line(
          points={{-38,-27.5},{-56,-27.5},{-56,-60},{-102,-60},{-102,-4},{-142,-4},{-142,45}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(pumpSto.heatPort, fixedTemperature.port) annotation (Line(
          points={{-37,-54.08},{-26,-54.08},{-26,-60},{-102,-60},{-102,-4},{-142,-4},{-142,45}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(dHW.port_cold, pipeDHW.port_b) annotation (Line(
          points={{-47,-8},{-47,-29},{-46,-29}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(dHW.port_hot,senTemSto_top. port_b) annotation (Line(
          points={{-47,10},{-47,22},{-36,22}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemSto_top.port_a,DHWTank. port_a) annotation (Line(
          points={{-28,22},{-22,22},{-22,6}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeDHW.port_a,senTemSto_bot. port_b) annotation (Line(
          points={{-30,-29},{-16,-29}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemSto_bot.port_a,DHWTank. port_b) annotation (Line(
          points={{-6,-29},{6,-29},{6,6}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(absolutePressure1.ports[1],DHWTank. port_b) annotation (Line(
          points={{8,-36},{8,6},{6,6}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pumpSto.port_b, senTemStoHX_out.port_a) annotation (Line(
          points={{-44,-50},{-56,-50},{-56,-48},{-66,-48}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemStoHX_out.port_b, heater.port_a) annotation (Line(
          points={{-78,-48},{-134,-48},{-134,18.25}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(ctrl_Heating.onOff, gain.u)
        annotation (Line(points={{-158,64},{-84,64},{-84,-6.4}},color={0,0,127}));
      connect(gain.y, pumpSto.m_flow_in) annotation (Line(points={{-84,-24.8},{-84,-38},{-36.86,-38},{-36.86,-42.8}},
                                                 color={0,0,127}));
      connect(DHWTank.portHex_b, pumpSto.port_a) annotation (Line(points={{-22,-10},{-26,-10},{-26,-50},{-30,-50}},
                                              color={0,127,255}));
      connect(fixedTemperature.port,DHWTank. heaPorBot) annotation (Line(points={{-142,45},{-38,45},{-38,-8.8},{-5.2,-8.8}},
                                                      color={191,0,0}));
      connect(fixedTemperature.port,DHWTank. heaPorTop) annotation (Line(points={{-142,45},{-6,45},{-6,20.8},{-5.2,20.8}},
                                                  color={191,0,0}));
      connect(fixedTemperature.port,DHWTank. heaPorSid) annotation (Line(points={{-142,45},{0,45},{0,6},{-0.16,6}},
                                             color={191,0,0}));
      connect(senTemHea_out.port_b,DHWTank. portHex_a) annotation (Line(points={{-42,58},{-28,58},{-28,-1.6},{-22,-1.6}},
                                                   color={0,127,255}));
      connect(DHWTank.heaPorVol, TTank.port) annotation (Line(points={{-8,6},{-8,6},{-8,8},{2,8},{2,7},{10,7}},
                               color={191,0,0}));
      connect(TTank[posTBot].T, ctrl_Heating.TTankBot) annotation (Line(points={{20,7},{
              20,7},{20,62},{-176,62}},
                                      color={0,0,127}));
      connect(TTank[posTTop].T, ctrl_Heating.TTankTop) annotation (Line(
          points={{20,7},{20,50},{-176,50},{-176,66}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(mDHW60C, dHW.mDHW60C) annotation (Line(points={{80,-104},{-60,-104},{-60,
              -2},{-60,-0.8},{-52,-0.8}}, color={0,0,127}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{
                200,100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-200,-100},{200,100}})));
    end SingleProduction_Radiator_DHW;

    model MultiProduction_Radiator
      extends
        FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_MultiSource(
        QBoiler=sum(QNom),
        final isMulti=true,
        final isHea=true,
        final isCoo=false,
        nConvPorts=nZones,
        nRadPorts=nZones,
        nTemSen=nZones,
        nEmbPorts=0,
        nZones=1,
        minSup=true,
        TSupMin=273.15 + 25,
        redeclare Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2
          emission[nZones](
          each T_a_nominal=TSupNom,
          each T_b_nominal=TSupNom - dTSupRetNom,
          TAir_nominal=TRoomNom,
          Q_flow_nominal=QNom,
          redeclare each package Medium = Medium),
        pipeSupply(redeclare package Medium = Medium),
        pipeReturn(redeclare package Medium = Medium),
        pipeReturnEmission(redeclare package Medium = Medium),
        pumpRad(redeclare package Medium = Medium),
        vol(redeclare package Medium = Medium),
        senTemHea_out(redeclare package Medium = Medium),
        senTemEm_in(redeclare package Medium = Medium),
        idealCtrlMixer(redeclare package Medium = Medium),
        senTemEm_out(redeclare package Medium = Medium),
        spl(redeclare package Medium = Medium),
        absolutePressure(redeclare package Medium = Medium),
        conVal(
          k=2,
          controllerType=Modelica.Blocks.Types.SimpleController.PI,
          Ti=1200),
          heater(redeclare package Medium = Medium),
        ctrl_Heating(
          THeaterSet(start=353.15),
          TBotSet(start=333.15),
          TTopSet(start=343.15)),
        add(k1=-1, k2=+1));
      Modelica.Blocks.Interfaces.RealOutput y
        annotation (Placement(transformation(extent={{-200,-50},{-220,-30}})));
    equation
      QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
      connect(heatPortCon, emission.heatPortCon) annotation (Line(points={{-118,20},
              {-214,20},{-214,100},{120,100},{120,65.2}},                                                                                  color={191,0,0}));
      connect(emission.heatPortRad, heatPortRad) annotation (Line(points={{126,
              65.2},{144,65.2},{144,100},{-120,100},{-120,-20}},                                                                  color={191,0,0}));
      connect(y, TTank[3].T) annotation (Line(points={{-210,-40},{-86,-40},{-86,23}},
                    color={0,0,127}));
      for i in 1:nSource loop
      connect(heater[i].port_b, TESTank.port_a) annotation (Line(points={{-116,
                -3.75},{-98,-3.75},{-98,-16},{-54,-16}},
                                           color={0,127,255}));
      connect(heater[i].port_a, TESTank.port_b) annotation (Line(points={{-136,
                -3.75},{-142,-3.75},{-142,-26},{-96,-26},{-22,-26},{-22,-16}},
                                                                            color={0,
              127,255}));
      end for;
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{
                200,100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-200,-100},{200,100}})));
    end MultiProduction_Radiator;

    model MultiProduction_Radiator_DHW
      extends FBM.HeatingDHWsystems.BaseClasses.DHWPara;
      extends FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_MultiSource(
        QBoiler=sum(QNom),
        final isMulti=true,
        final isHea=true,
        final isCoo=false,
        nConvPorts=nZones,
        nRadPorts=nZones,
        nTemSen=nZones,
        nEmbPorts=0,
        nZones=1,
        minSup=true,
        TSupMin=273.15 + 25,
        redeclare FBM.Controls.ControlHeating.Ctrl_Heating_combiTES ctrl_Heating(
          corFac_val=corFac_val,
          TSupMin=TSupMin,
          minSup=minSup,
          THeaterSet(start=353.15),
          TBotSet(start=333.15),
          TTopSet(start=343.15)),
        redeclare Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2
          emission[nZones](
          each T_a_nominal=TSupNom,
          each T_b_nominal=TSupNom - dTSupRetNom,
          TAir_nominal=TRoomNom,
          Q_flow_nominal=QNom,
          redeclare each package Medium = Medium),
        pipeSupply(redeclare package Medium = Medium),
        pipeReturn(redeclare package Medium = Medium),
        pipeReturnEmission(redeclare package Medium = Medium),
        pumpRad(redeclare package Medium = Medium),
        vol(redeclare package Medium = Medium),
        senTemHea_out(redeclare package Medium = Medium),
        senTemEm_in(redeclare package Medium = Medium),
        idealCtrlMixer(redeclare package Medium = Medium),
        senTemEm_out(redeclare package Medium = Medium),
        spl(redeclare package Medium = Medium),
        conVal(
          k=1,
          Ti=600,
          controllerType=Modelica.Blocks.Types.SimpleController.PI),
          heater(redeclare package Medium = Medium),
        add(k1=-1, k2=+1),
        exp(p=300000),
        TESTank(p_start=300000));
              // --- Domestic Hot Water and it hydraulic circuit
      replaceable FBM.Components.BalancedTap_m_flow dHW(
        TDHWSet=TDHWSet,
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal_DHW,
        InsuPipeThickness=InsuPipeThickness,
        InsuHeatCondu=InsuHeatCondu)       constrainedby
        FBM.Components.BaseClasses.BalancedTap( redeclare package Medium = Medium,
          TDHWSet=TDHWSet) annotation (Placement(transformation(
            extent={{-9,-5},{9,5}},
            rotation=-90,
            origin={31,-15})));
      Buildings.Fluid.FixedResistances.Pipe pipeDHW(redeclare package Medium =  Medium,
        m_flow_nominal=m_flow_nominal_DHW,
        nSeg=3,
        length=Pipelength,
        lambdaIns=InsuHeatCondu,
        thicknessIns=InsuPipeThickness)
        annotation (Placement(transformation(extent={{72,-34},{56,-40}})));
      Buildings.Fluid.Movers.FlowControlled_m_flow pumpSto(
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal_stoHX,
        tau=30,
        filteredSpeed=true,
        riseTime=100) "Pump for loading the storage tank"
        annotation (Placement(transformation(extent={{52,-46},{38,-34}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemStoHX_out(redeclare
          package
          Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the outlet of the storage tank heat exchanger (port_bHX)"
        annotation (Placement(transformation(extent={{34,-46},{22,-34}})));
      Modelica.Blocks.Math.Gain gain(k=m_flow_nominal_stoHX) annotation (Placement(
            transformation(
            extent={{-8,-8},{8,8}},
            rotation=270,
            origin={24,10})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemSto_top(redeclare
          package
          Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the top outlet of the storage tank (port_a)"
        annotation (Placement(transformation(extent={{46,-6},{38,2}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemSto_bot(redeclare
          package
          Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the bottom inlet of the storage tank (port_b)"
        annotation (Placement(transformation(extent={{80,-42},{70,-32}})));
      Buildings.Fluid.Storage.StratifiedEnhancedInternalHex DHWTank(
        port_a(m_flow(start=0)),
        redeclare package Medium = Medium,
        hTan=1.8,
        VTan=volumeTank,
        mHex_flow_nominal=m_flow_nominal_stoHX,
        nSeg=nbrNodes,
        dIns=InsuPipeThickness,
        kIns=InsuHeatCondu,
        m_flow_nominal=m_flow_nominal_DHW,
        redeclare package MediumHex = Medium,
        hHex_a=volumeTank/2,
        hHex_b=volumeTank/3,
        Q_flow_nominal=m_flow_nominal_stoHX*4180*(90 - 60),
        p_start=300000,
        T_start=333.15,
        TTan_nominal=333.15,
        THex_nominal=363.15)                   annotation (Placement(transformation(
            extent={{-14,-20},{14,20}},
            rotation=0,
            origin={66,-4})));
      Buildings.Fluid.Storage.ExpansionVessel exp1(
        redeclare package Medium = Medium,
        V_start=1,
        p=300000)
        annotation (Placement(transformation(extent={{66,-60},{76,-48}})));
    equation
      QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
      connect(heatPortCon, emission.heatPortCon) annotation (Line(points={{-118,20},
              {-200,20},{-200,100},{152,100},{152,65.2},{120,65.2}},                                                                       color={191,0,0}));
      connect(emission.heatPortRad, heatPortRad) annotation (Line(points={{126,
              65.2},{174,65.2},{174,100},{-120,100},{-120,-20}},                                                                  color={191,0,0}));
      for i in 1:nSource loop
          connect(heater[i].port_b, TESTank.port_a) annotation (Line(points={{-116,-3.75},
                {-54,-3.75},{-54,-16}},            color={0,127,255}));
      connect(heater[i].port_a, TESTank.port_b) annotation (Line(points={{-136,-3.75},
                {-136,-24},{-22,-24},{-22,-16}},
                                         color={0,127,255}));
      end for;
      connect(pipeDHW.heatPort, fixedTemperature.port) annotation (Line(
          points={{64,-38.5},{64,-60},{-102,-60},{-102,-4},{-158,-4},{-158,45}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(pumpSto.heatPort, fixedTemperature.port) annotation (Line(
          points={{45,-44.08},{36,-44.08},{36,-60},{-102,-60},{-102,-4},{-158,-4},{-158,
              45}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(dHW.port_cold, pipeDHW.port_b) annotation (Line(
          points={{31,-24},{31,-37},{56,-37}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(dHW.port_hot,senTemSto_top. port_b) annotation (Line(
          points={{31,-6},{32,-6},{32,-2},{38,-2}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemSto_top.port_a,DHWTank. port_a) annotation (Line(
          points={{46,-2},{52,-2},{52,-4}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeDHW.port_a,senTemSto_bot. port_b) annotation (Line(
          points={{72,-37},{70,-37}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemSto_bot.port_a,DHWTank. port_b) annotation (Line(
          points={{80,-37},{80,-4}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pumpSto.port_b, senTemStoHX_out.port_a) annotation (Line(
          points={{38,-40},{34,-40}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(ctrl_Heating.onOff, gain.u)
        annotation (Line(points={{-106.222,79.5},{-102,79.5},{-102,76},{-100,76},
              {-100,36},{24,36},{24,19.6}},                     color={0,0,127}));
      connect(gain.y, pumpSto.m_flow_in) annotation (Line(points={{24,1.2},{24,-26},
              {24,-30},{45.14,-30},{45.14,-32.8}},
                                                 color={0,0,127}));
      connect(DHWTank.portHex_b, pumpSto.port_a) annotation (Line(points={{52,-20},{
              52,-40}},                       color={0,127,255}));
      connect(mDHW60C, dHW.mDHW60C) annotation (Line(points={{80,-104},{40,-104},{40,
              -18},{40,-16.8},{36,-16.8}},color={0,0,127}));
      connect(fixedTemperature.port, DHWTank.heaPorTop) annotation (Line(points={{-158,45},
              {64,45},{64,10.8},{68.8,10.8}},     color={191,0,0}));
      connect(fixedTemperature.port, DHWTank.heaPorSid) annotation (Line(points={{-158,45},
              {70,45},{70,-4},{73.84,-4}},     color={191,0,0}));
      connect(fixedTemperature.port, DHWTank.heaPorBot) annotation (Line(points={{-158,45},
              {64,45},{64,-18.8},{68.8,-18.8}},     color={191,0,0}));
      connect(senTemHea_out.T, ctrl_Heating.TTankEmiOut) annotation (Line(points={{-38,69},
              {-78,69},{-78,71},{-115.556,71}},     color={0,0,127}));
      connect(exp1.port_a, DHWTank.port_b) annotation (Line(points={{71,-60},{
              80,-60},{80,-4}}, color={0,127,255}));
      connect(TESTank.port_b, DHWTank.portHex_a) annotation (Line(points={{-22,
              -16},{-22,-16},{-22,-11.6},{52,-11.6}}, color={0,127,255}));
      connect(senTemStoHX_out.port_b, TESTank.port_a) annotation (Line(points={
              {22,-40},{-54,-40},{-54,-16}}, color={0,127,255}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,-100},{
                200,100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-200,-100},{200,100}})));
    end MultiProduction_Radiator_DHW;
  end ModularProductionSystem;

  package SimpleHeatingSystem
    "First implementation for heating systems with only single source and output possibility and with or without DHW and TES"
    model Boiler_FloorHeating
      replaceable parameter
        FBM.Components.BaseClasses.RadiantSlabChar[nEmbPorts] RadSlaCha constrainedby
        FBM.Components.BaseClasses.RadiantSlabChar
        "Properties of the floor heating or TABS, if present";
      parameter Modelica.SIunits.Area AEmb[nEmbPorts]
        "surface of each embedded circuit";
      extends FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_Test(
        final isHea=true,
        final isCoo=false,
        nConvPorts=nZones,
        nRadPorts=nZones,
        nTemSen=nZones,
        nEmbPorts=nZones,
        nZones=1,
        minSup=true,
        TSupMin=273.15 + 25,
        redeclare FBM.Components.EmbeddedPipe emission[nEmbPorts](
          redeclare each package Medium = Medium,
          m_flow_nominal=m_flow_nominal,
          m_flowMin=m_flow_nominal/3,
          RadSlaCha=RadSlaCha,
          A_floor=AEmb,
          each nParCir=1),
        heater(redeclare package Medium = Medium, dp_nominal=sum(m_flow_nominal)/
              30,
          Q_flow_nominal=sum(QNom)),
        pumpSupply_m_flow(measurePower=false));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlow[
        nConvPorts](each Q_flow=0)
        annotation (Placement(transformation(extent={{-84,10},{-104,30}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlow1[
        nRadPorts](each Q_flow=0)
        annotation (Placement(transformation(extent={{-82,-30},{-102,-10}})));
    equation
      QHeaSys = -sum(emission.heatPortEmb.Q_flow);
      connect(emission[:].heatPortEmb[1], heatPortEmb[:]) annotation (Line(
          points={{97,30},{88,30},{88,40},{112,40},{112,-100},{-60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(prescribedHeatFlow.port, heatPortCon) annotation (Line(
          points={{-104,20},{-118,20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(prescribedHeatFlow1.port, heatPortRad) annotation (Line(
          points={{-102,-20},{-120,-20}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{120,
                100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-120,-100},{120,100}})),
        Documentation(info="<html>
<p><b>Description</b> </p>
<p>Multi-zone Hydraulic heating system with <a href=\"modelica://FBM.Components.EmbeddedPipe\">Floor heating</a>.  There is no thermal energy storage tank and no domestic hot water system. A schematic hydraulic scheme is given below:</p>
<p><img src=\"modelica://FBM/images/HydraulicScheme_Heating_Emisision_low.png\"/></p>
<p><br/>For multizone systems, the components <i>pumpRad</i>, <i>emission</i> and <i>pipeReturn</i> are arrays of size <i>nZones</i>. In this model, the <i>emission</i> is a embedded pipe, the <i>heater</i> is a replaceable component and can be a boiler or heat pump or anything </p>
<p>There are two controllers in the model (not represented in the hydraulic scheme): one for the heater set temperature (<a href=\"modelica://FBM.Controls.Ctrl_Heating\">Ctrl_Heating</a>), and another one for the on/off signal of <i>pumpRad</i> (= thermostat). The system is controlled based on a temperature measurement in each zone, a set temperature for each zone, and a general heating curve (not per zone). The heater will produce hot water at a temperature slightly above the heating curve, and the <i>3-Way valve</i> will mix it with return water to reach the heating curve set point. Right after the <i>3-Way valve</i>, the flow is splitted in <i>nZones</i> flows and each <i>pumpRad</i> will set the flowrate in the zonal distribution circuit based on the zone temperature and set point. </p>
<p>The heat losses of the heater and all the pipes are connected to a central fix temperature. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>Controllers try to limit or avoid events for faster simulation</li>
<li>Single heating curve for all zones</li>
<li>Heat emitted through <i>heatPortRad</i> and <i>heatPortCon</i> </li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding surBou surface of a <a href=\"modelica://Buildings.Rooms.MixedAir\">MixedAir</a> room. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
<li>Set all parameters that are required. </li>
<li>Not all parameters of the sublevel components are ported to the uppermost level. Therefore, it might be required to modify these components deeper down the hierarchy. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>This is a system level model, no validation performed.</p>
<p><h4>Example </h4></p>
<p>An example of the use of this model can be found in <a href=\"modelica://FBM.HeatingDHWsystems.Examples.Heating_Embedded\">FBM.HeatingDHWsystems.Examples.Heating_Embedded</a>.</p>
</html>",     revisions="<html>
<p><ul>
<li>2016 November, Wilfried Thomaré: first implementation</li>
</ul></p>
</html>"));
    end Boiler_FloorHeating;

    model Boiler_FloorHeating_DHW
      "Hydraulic heating with embedded emission, DHW (with STS), no TES for heating"
      // fixme: no solar system is implemeted so far (adapt documentation)
      replaceable parameter
        FBM.Components.BaseClasses.RadiantSlabChar[nEmbPorts] RadSlaCha constrainedby
        FBM.Components.BaseClasses.RadiantSlabChar
        "Properties of the floor heating or TABS, if present";
       parameter Modelica.SIunits.Area AEmb[nEmbPorts]
        "surface of each embedded circuit";
       extends FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_Test(
        final isHea=true,
        final isCoo=false,
        final nConvPorts=nZones,
        final nRadPorts=nZones,
        final nTemSen=nZones,
        final nEmbPorts=nZones,
        nZones=1,
        minSup=true,
        TSupMin=273.15 + 25,
        redeclare FBM.Components.EmbeddedPipe emission[nEmbPorts](
          redeclare each package Medium = Medium,
          m_flow_nominal=m_flow_nominal,
          m_flowMin=m_flow_nominal/3,
          RadSlaCha=RadSlaCha,
          A_floor=AEmb,
          nParCir=1),
        redeclare FBM.Controls.ControlHeating.Ctrl_Heating_DHW ctrl_Heating(
          TDHWSet=TDHWSet,
          TColdWaterNom=TColdWaterNom,
          dTHPTankSet=dTHPTankSet),
        heater(m_flow_nominal=sum(m_flow_nominal) + m_flow_nominal_stoHX,
          dp_nominal=sum(m_flow_nominal)/30,
          Q_flow_nominal=sum(QNom),
          T_nominal=333.15));
      // --- Domestic Hot Water (DHW) Parameters
      parameter Integer nOcc = 1 "Number of occupants";
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal_DHW = nOcc*nOcc*0.045*983/(3600*24)*10
        "nominal mass flow rate of DHW";
      parameter Modelica.SIunits.Temperature TDHWSet(max=273.15 + 60) = 273.15 + 45
        "DHW temperature setpoint";
      parameter Modelica.SIunits.Temperature TColdWaterNom=273.15 + 10
        "Nominal tap (cold) water temperature";
      // --- Storage Tank Parameters
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal_stoHX = m_flow_nominal_DHW * (TDHWSet - TColdWaterNom)/dTHPTankSet
        "nominal mass flow rate of HX of storage tank";
      parameter Modelica.SIunits.TemperatureDifference dTHPTankSet(min=1)=2
        "Difference between tank setpoint and heat pump setpoint";
      parameter Modelica.SIunits.Volume volumeTank=0.25;
      parameter Integer nbrNodes=10 "Number of nodes in the tank";
      parameter Integer posTTop(max=nbrNodes) = 1
        "Position of the top temperature sensor";
      parameter Integer posTBot(max=nbrNodes) = nbrNodes - 2
        "Position of the bottom temperature sensor";
      parameter Integer posOutHP(max=nbrNodes + 1) = if solSys then nbrNodes - 1
         else nbrNodes + 1 "Position of extraction of TES to HP";
      parameter Integer posInSTS(max=nbrNodes) = nbrNodes - 1
        "Position of injection of STS in TES";
      parameter Boolean solSys(fixed=true) = false;
          // -- Pipes and valve parameters
      parameter Modelica.SIunits.Thickness InsuPipeThickness=0.02
                                                                 "Thickness of the pipe insulation";
      parameter Modelica.SIunits.Length Pipelength=5
                                                    "pipe length of the supply OR return branch";
      parameter Modelica.SIunits.Pressure dp=3000 "Pressure drop over a single pipe"
        annotation(Dialog(group = "Pipes",
                         enable = includePipes));
     parameter Modelica.SIunits.ThermalConductivity InsuHeatCondu=0.04;
      // --- Storage tank and hydraulic circuit connect to its heat exchanger
      Buildings.Fluid.Movers.FlowControlled_m_flow pumpSto(
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal_stoHX,
        tau=30,
        filteredSpeed=true,
        riseTime=100) "Pump for loading the storage tank"
        annotation (Placement(transformation(extent={{108,-84},{94,-72}})));
      Buildings.Fluid.Storage.StratifiedEnhancedInternalHex DHWTank(
        port_a(m_flow(start=0)),
        redeclare package Medium = Medium,
        hTan=1.8,
        VTan=volumeTank,
        mHex_flow_nominal=m_flow_nominal_stoHX,
        nSeg=nbrNodes,
        dIns=InsuPipeThickness,
        kIns=InsuHeatCondu,
        m_flow_nominal=m_flow_nominal_DHW,
        redeclare package MediumHex = Medium,
        hHex_a=volumeTank/2,
        hHex_b=volumeTank/3,
        THex_nominal=heater.T_nominal,
        Q_flow_nominal=m_flow_nominal_stoHX*4180*(heater.T_nominal - 60),
        T_start=323.15,
        TTan_nominal=333.15)                   annotation (Placement(transformation(
            extent={{-14,-20},{14,20}},
            rotation=0,
            origin={124,-32})));
      // --- Domestic Hot Water and it hydraulic circuit
      replaceable FBM.Components.BalancedTap_m_flow dHW(
        TDHWSet=TDHWSet,
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal_DHW) constrainedby
        FBM.Components.BaseClasses.BalancedTap( redeclare package Medium = Medium,
          TDHWSet=TDHWSet) annotation (Placement(transformation(
            extent={{-9,5},{9,-5}},
            rotation=-90,
            origin={85,-29})));
      Buildings.Fluid.FixedResistances.Pipe pipeDHW(redeclare package Medium =  Medium,
        m_flow_nominal=m_flow_nominal_DHW,
        thicknessIns=InsuPipeThickness,
        nSeg=3,
        length=Pipelength,
        lambdaIns=InsuHeatCondu)
        annotation (Placement(transformation(extent={{106,-62},{90,-56}})));
      Buildings.Fluid.Sources.FixedBoundary absolutePressure1(
        redeclare package Medium = Medium,
        use_T=false,
        nPorts=1) annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={134,-78})));
      // --- Result (Output) variables
      Modelica.SIunits.Temperature[nbrNodes] TSto=TTank.T;
      Modelica.SIunits.Temperature TTankTopSet;
      Modelica.SIunits.Temperature TTankBotIn;
      Modelica.SIunits.MassFlowRate m_flowDHW;
      Modelica.SIunits.Power QDHW;
      Modelica.SIunits.Temperature TDHW;
      Real SOCTank;
      // --- Temperature sensors
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemSto_top(redeclare
          package
          Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the top outlet of the storage tank (port_a)"
        annotation (Placement(transformation(extent={{104,-18},{96,-10}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemSto_bot(redeclare
          package
          Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the bottom inlet of the storage tank (port_b)"
        annotation (Placement(transformation(extent={{132,-64},{122,-54}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemStoHX_out(redeclare
          package
          Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the outlet of the storage tank heat exchanger (port_bHX)"
        annotation (Placement(transformation(extent={{84,-84},{72,-72}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlow1[
        nRadPorts](Q_flow=0)
        annotation (Placement(transformation(extent={{-88,-30},{-108,-10}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlow[
        nConvPorts](Q_flow=0)
        annotation (Placement(transformation(extent={{-86,10},{-106,30}})));
      Modelica.Blocks.Math.Gain gain(k=m_flow_nominal_stoHX) annotation (Placement(
            transformation(
            extent={{-8,-8},{8,8}},
            rotation=0,
            origin={70,70})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nbrNodes] TTank
        annotation (Placement(transformation(extent={{124,-2},{114,8}})));
    equation
      QHeaSys = -sum(emission.heatPortEmb.Q_flow) + QDHW;
      // Result variables
      TTankTopSet = ctrl_Heating.TTopSet;
      TDHW = dHW.TDHW_actual;
      TTankBotIn = senTemSto_bot.T;
      m_flowDHW = dHW.idealSource.m_flow_in;
      SOCTank = ctrl_Heating.SOC;
      QDHW = -dHW.pipe_HeatPort.heatPort.Q_flow;
      connect(dHW.port_cold, pipeDHW.port_b) annotation (Line(
          points={{85,-38},{85,-59},{90,-59}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(dHW.port_hot,senTemSto_top. port_b) annotation (Line(
          points={{85,-20},{85,-14},{96,-14}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemSto_top.port_a,DHWTank. port_a) annotation (Line(
          points={{104,-14},{110,-14},{110,-32}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeDHW.port_a,senTemSto_bot. port_b) annotation (Line(
          points={{106,-59},{122,-59}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemSto_bot.port_a,DHWTank. port_b) annotation (Line(
          points={{132,-59},{138,-59},{138,-32}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(absolutePressure1.ports[1],DHWTank. port_b) annotation (Line(
          points={{134,-72},{134,-32},{138,-32}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pumpSto.port_b, senTemStoHX_out.port_a) annotation (Line(
          points={{94,-78},{84,-78}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemStoHX_out.port_b, heater.port_a) annotation (Line(
          points={{72,-78},{-66,-78},{-66,2}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(prescribedHeatFlow1.port, heatPortRad) annotation (Line(
          points={{-108,-20},{-120,-20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(prescribedHeatFlow.port, heatPortCon) annotation (Line(
          points={{-106,20},{-118,20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(ctrl_Heating.onOff, gain.u)
        annotation (Line(points={{-92,70},{60.4,70}},           color={0,0,127}));
      connect(gain.y, pumpSto.m_flow_in) annotation (Line(points={{78.8,70},{78.8,-68},
              {101.14,-68},{101.14,-70.8}},      color={0,0,127}));
      connect(heatPortEmb, emission.heatPortEmb[1]) annotation (Line(points={{-60,
              -100},{-60,-98},{64,-98},{64,42},{82,42},{88,42},{88,30},{97,30}},
            color={191,0,0}));
      connect(DHWTank.portHex_b, pumpSto.port_a) annotation (Line(points={{110,-48},
              {110,-48},{110,-78},{108,-78}}, color={0,127,255}));
      connect(DHWTank.heaPorVol, TTank.port) annotation (Line(points={{124,-32},{124,
              -32},{124,-6},{124,3}},
                               color={191,0,0}));
      connect(TTank[posTBot].T, ctrl_Heating.TTankBot) annotation (Line(points={{114,3},
              {114,3},{114,68},{-110,68}},
                                      color={0,0,127}));
      connect(TTank[posTTop].T, ctrl_Heating.TTankTop) annotation (Line(
          points={{114,3},{114,72},{-110,72}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(mDHW60C, dHW.mDHW60C) annotation (Line(points={{80,-104},{80,-104},{80,
              -32},{80,-30.8}},           color={0,0,127}));
      connect(prescribedTemperature.port, DHWTank.heaPorTop) annotation (Line(
            points={{12,-8},{126,-8},{126,-17.2},{126.8,-17.2}}, color={191,0,0}));
      connect(prescribedTemperature.port, DHWTank.heaPorBot) annotation (Line(
            points={{12,-8},{126,-8},{126,-46.8},{126.8,-46.8}}, color={191,0,0}));
      connect(prescribedTemperature.port, DHWTank.heaPorSid) annotation (Line(
            points={{12,-8},{132,-8},{132,-32},{131.84,-32}}, color={191,0,0}));
      connect(balancingValve.port_b1, DHWTank.portHex_a) annotation (Line(points={{-30,
              21.2},{2,21.2},{2,-39.6},{110,-39.6}}, color={0,127,255}));
      connect(prescribedTemperature.port, pipeDHW.heatPort) annotation (Line(
            points={{12,-8},{98,-8},{98,-57.5}}, color={191,0,0}));
      connect(prescribedTemperature.port, pumpSto.heatPort) annotation (Line(
            points={{12,-8},{98,-8},{98,-82.08},{101,-82.08}}, color={191,0,0}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{140,
                100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-120,-100},{140,100}})),
        Documentation(info="<html>
<p><b>Description</b> </p>
<p>Multi-zone Hydraulic heating system with <a href=\"modelica://FBM.Components.EmbeddedPipe\">embedded pipe</a> emission system (TABS). There is no thermal energy storage tank for the heating, but the domestic hot water (DHW) system has a storage tank with internal heat exchanger. An optional solar thermal system is foreseen to (pre)heat the DHW storage tank.. A schematic hydraulic scheme is given below:</p>
<p><img src=\"modelica://FBM/images/HydraulicScheme_Heating_Embedded_DHW_STS.png\"/></p>
<p>For multizone systems, the components <i>pumpRad</i>, <i>emission</i> and <i>pipeReturn</i> are arrays of size <i>nZones</i>. In this model, the <i>emission</i> is a an embedded pipe, the <i>heater</i> is a replaceable component and can be a boiler.</p>
<p>There are two controllers in the model (not represented in the hydraulic scheme): one for the heater set temperature and control signal of the pump for charging the DHW storage tank (<a href=\"modelica://FBM.Controls.ControlHeating.Ctrl_Heating_DHW\">Ctrl_Heating_DHW</a>), and another one for the on/off signal of <i>pumpRad</i> (= thermostat). The system is controlled based on a temperature measurement in each zone, a set temperature for each zone, temperature measurements in the storage tank and a general heating curve (not per zone). The heater will produce hot water at a temperature slightly above the required temperature, depending on the heat demand (space heating or DHW). The <i>3-Way valve</i> will mix the supply flow rate with return water to reach the heating curve set point. Right after the <i>3-Way valve</i>, the flow is splitted in <i>nZones</i> flows and each <i>pumpRad</i> will set the flowrate in the zonal distribution circuit based on the zone temperature and set point. </p>
<p>The heat losses of the heater and all the pipes are connected to a central fix temperature. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>Controllers try to limit or avoid events for faster simulation</li>
<li>Single heating curve for all zones</li>
<li>Heat emitted through <i>heatPortEmb</i> (to the core of a building construction layer or a <a href=\"FBM.Components.NakedTabs\">nakedTabs</a>)</li>
<li>All pumps are on/off</li>
<li>No priority: both pumps can run simultaneously (could be improved).</li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding surBou surface of a <a href=\"modelica://Buildings.Rooms.MixedAir\">MixedAir</a> room. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
<li>Set all parameters that are required. </li>
<li>Not all parameters of the sublevel components are ported to the uppermost level. Therefore, it might be required to modify these components deeper down the hierarchy. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>This is a system level model, no validation performed. If the solar thermal system is used, the controller of the heat pump might be unsufficient, and the internal heat exchanger in the storage tank should be reconfigured (only in top of tank). To be correct, a second internal heat exchanger should be foreseen to connect the primary circuit of the solar thermal system to the tank. </p>
<p><h4>Example </h4></p>
<p>An example of the use of this model can be found in<a href=\"modelica://FBM.HeatingDHWsystems.Examples.Heating_Embedded\"> FBM.HeatingDHWsystems.ExamplesHeating_Embedded</a>.</p>
</html>",     revisions="<html>
<p><ul>
<li>2016 November, Wilfried Thomaré: First implementation</li>
</ul></p>
</html>"));
    end Boiler_FloorHeating_DHW;

    model Boiler_Radiator
      "Basic hydraulic heating (with heating curve) with radiator. No TES, no DHW"
       extends FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_Test(
        final isHea=true,
        final isCoo=false,
        nConvPorts=nZones,
        nRadPorts=nZones,
        nTemSen=nZones,
        nEmbPorts=0,
        nZones=1,
        minSup=true,
        TSupMin=273.15 + 30,
        redeclare Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2
          emission[nZones](
          each T_a_nominal=TSupNom,
          each T_b_nominal=TSupNom - dTSupRetNom,
          TAir_nominal=TRoomNom,
          Q_flow_nominal=QNom,
          redeclare each package Medium = Medium),
        ctrl_Heating(
          TSupNom=TSupNom,
          dTSupRetNom=dTSupRetNom,
          TSupMin=TSupMin,
          dTHeaterSet=dTHeaterSet,
          timeFilter=timeFilter,
          TOut_nominal=TOut_nominal,
          corFac_val=corFac_val),
        heater(dp_nominal=sum(m_flow_nominal)/30, Q_flow_nominal=sum(QNom)),
        mixingCircuit_Emit(
          InsuPipeThickness=InsuPipeThickness,
          Pipelength=Pipelength,
          InsuHeatCondu=InsuHeatCondu),
        pumpSupply_m_flow(
          InsuPipeThickness=InsuPipeThickness,
          Pipelength=Pipelength,
          dp=dp,
          InsuHeatCondu=InsuHeatCondu,
          measurePower=false));
    equation
      QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
      connect(emission.heatPortCon, heatPortCon) annotation (Line(
          points={{94,27.2},{94,38},{86,38},{86,38},{-120,38},{-120,20},{-118,
              20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(emission.heatPortRad, heatPortRad) annotation (Line(
          points={{100,27.2},{100,38},{92,38},{92,38},{-120,38},{-120,-20},{
              -120,-20}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
                {120,100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-120,-100},{
                120,100}})),
        Documentation(info="<html>
<p><b>Description</b> </p>
<p>Multi-zone Hydraulic heating system with <a href=\"modelica://IDEAS.Thermal.Components.Emission.EmbeddedPipe\">embedded pipe</a> emission system (TABS). Except for the emission system, this model is identical to <a href=\"modelica://FBM.HeatingDHWsystems.SimpleHeatingSystem.Boiler_FloorHeating\">Boiler_FloorHeating</a>. There is no thermal energy storage tank and no domestic hot water system. A schematic hydraulic scheme is given below:</p>
<p><img src=\"modelica://FBM/images/HydraulicScheme_Heating_Emisision_low.png\"/></p>
<p><br/>For multizone systems, the components <i>pumpRad</i>, <i>emission</i> and <i>pipeReturn</i> are arrays of size <i>nZones</i>. In this model, the <i>emission</i> is a an embedded pipe, the <i>heater</i> is a replaceable component and can be a boiler or heat pump.</p>
<p>There are two controllers in the model (not represented in the hydraulic scheme): one for the heater set temperature (<a href=\"modelica://FBM.Controls.ControlHeating.Ctrl_Heating\">Ctrl_Heating</a>), and another one for the on/off signal of <i>pumpRad</i> (= thermostat). The system is controlled based on a temperature measurement in each zone, a set temperature for each zone, and a general heating curve (not per zone). The heater will produce hot water at a temperature slightly above the heating curve, and the <i>3-Way valve</i> will mix it with return water to reach the heating curve set point. Right after the <i>3-Way valve</i>, the flow is splitted in <i>nZones</i> flows and each <i>pumpRad</i> will set the flowrate in the zonal distribution circuit based on the zone temperature and set point. </p>
<p>The heat losses of the heater and all the pipes are connected to a central fix temperature. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>Controllers try to limit or avoid events for faster simulation</li>
<li>Single heating curve for all zones</li>
<li>Heat emitted through <i>heatPortEmb</i> (to the core of a building construction layer or a <a href=\"modelica://IDEAS.Thermal.Components.Emission.NakedTabs\">nakedTabs</a>)</li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts of a <a href=\"modelica://Buildings.Rooms.MixedAir\">MixedAir</a> room. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
<li>Set all parameters that are required. </li>
<li>Not all parameters of the sublevel components are ported to the uppermost level. Therefore, it might be required to modify these components deeper down the hierarchy. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>This is a system level model, no validation performed.</p>
<p><h4>Example </h4></p>
<p>An example of the use of this model can be found in<a href=\"modelica://FBM.HeatingDHWsystems.Examples.Heating_Radiator\"> FBM.HeatingDHWsystems.Examples.Heating_Radiator</a>.</p>
</html>",     revisions="<html>
<p><ul>
<li>2016 November, Wilfried Thomaré: first version</li>
</ul></p>
</html>"));
    end Boiler_Radiator;

    model Boiler_Radiator_DHW
      "Hydraulic heating with embedded emission, DHW (with STS), no TES for heating"
      // fixme: no solar system is implemeted so far (adapt documentation)
       extends FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_Test(
        final isHea=true,
        final isCoo=false,
        final nConvPorts=nZones,
        final nRadPorts=nZones,
        final nTemSen=nZones,
        final nEmbPorts=nZones,
        nZones=1,
        minSup=true,
        TSupMin=273.15 + 25,
        redeclare Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2
          emission[nZones](
          each T_a_nominal=TSupNom,
          each T_b_nominal=TSupNom - dTSupRetNom,
          TAir_nominal=TRoomNom,
          Q_flow_nominal=QNom,
          redeclare each package Medium = Medium),
        redeclare FBM.Controls.ControlHeating.Ctrl_Heating_DHW ctrl_Heating(
          TDHWSet=TDHWSet,
          TColdWaterNom=TColdWaterNom,
          dTHPTankSet=dTHPTankSet),
        heater(m_flow_nominal=sum(m_flow_nominal) + m_flow_nominal_stoHX,
          dp_nominal=sum(m_flow_nominal)/30,
          Q_flow_nominal=sum(QNom)));
      // --- Domestic Hot Water (DHW) Parameters
      parameter Integer nOcc = 1 "Number of occupants";
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal_DHW = nOcc*nOcc*0.045*983/(3600*24)*10
        "nominal mass flow rate of DHW";
      parameter Modelica.SIunits.Temperature TDHWSet(max=273.15 + 60) = 273.15 + 45
        "DHW temperature setpoint";
      parameter Modelica.SIunits.Temperature TColdWaterNom=273.15 + 10
        "Nominal tap (cold) water temperature";
      // --- Storage Tank Parameters
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal_stoHX = m_flow_nominal_DHW * (TDHWSet - TColdWaterNom)/dTHPTankSet
        "nominal mass flow rate of HX of storage tank";
      parameter Modelica.SIunits.TemperatureDifference dTHPTankSet(min=1)=2
        "Difference between tank setpoint and heat pump setpoint";
      parameter Modelica.SIunits.Volume volumeTank=0.25;
      parameter Integer nbrNodes=10 "Number of nodes in the tank";
      parameter Integer posTTop(max=nbrNodes) = 1
        "Position of the top temperature sensor";
      parameter Integer posTBot(max=nbrNodes) = nbrNodes - 2
        "Position of the bottom temperature sensor";
      parameter Integer posOutHP(max=nbrNodes + 1) = if solSys then nbrNodes - 1
         else nbrNodes + 1 "Position of extraction of TES to HP";
      parameter Integer posInSTS(max=nbrNodes) = nbrNodes - 1
        "Position of injection of STS in TES";
      parameter Boolean solSys(fixed=true) = false;
          // -- Pipes and valve parameters
      parameter Modelica.SIunits.Thickness InsuPipeThickness=0.02
                                                                 "Thickness of the pipe insulation";
      parameter Modelica.SIunits.Length Pipelength=5
                                                    "pipe length of the supply OR return branch";
      parameter Modelica.SIunits.Pressure dp=3000 "Pressure drop over a single pipe"
        annotation(Dialog(group = "Pipes",
                         enable = includePipes));
     parameter Modelica.SIunits.ThermalConductivity InsuHeatCondu=0.04;
       // --- Storage tank and hydraulic circuit connect to its heat exchanger
      Buildings.Fluid.Movers.FlowControlled_m_flow pumpSto(
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal_stoHX,
        tau=30,
        filteredSpeed=true,
        riseTime=100) "Pump for loading the storage tank"
        annotation (Placement(transformation(extent={{108,-84},{94,-72}})));
      Buildings.Fluid.Storage.StratifiedEnhancedInternalHex tesTank(
        port_a(m_flow(start=0)),
        redeclare package Medium = Medium,
        hTan=1.8,
        VTan=volumeTank,
        mHex_flow_nominal=m_flow_nominal_stoHX,
        nSeg=nbrNodes,
        dIns=InsuPipeThickness,
        kIns=InsuHeatCondu,
        m_flow_nominal=m_flow_nominal_DHW,
        redeclare package MediumHex = Medium,
        hHex_a=volumeTank/2,
        hHex_b=volumeTank/3,
        THex_nominal=heater.T_nominal,
        Q_flow_nominal=m_flow_nominal_stoHX*4180*(heater.T_nominal - 60),
        T_start=323.15,
        TTan_nominal=333.15)                   annotation (Placement(transformation(
            extent={{-14,-20},{14,20}},
            rotation=0,
            origin={124,-32})));
      // --- Domestic Hot Water and it hydraulic circuit
      replaceable FBM.Components.BalancedTap_m_flow dHW(
        TDHWSet=TDHWSet,
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal_DHW) constrainedby
        FBM.Components.BaseClasses.BalancedTap( redeclare package Medium = Medium,
          TDHWSet=TDHWSet) annotation (Placement(transformation(
            extent={{-9,5},{9,-5}},
            rotation=-90,
            origin={85,-29})));
      Buildings.Fluid.FixedResistances.Pipe pipeDHW(redeclare package Medium =  Medium,
        m_flow_nominal=m_flow_nominal_DHW,
        thicknessIns=InsuPipeThickness,
        nSeg=3,
        length=Pipelength,
        lambdaIns=InsuHeatCondu)
        annotation (Placement(transformation(extent={{106,-62},{90,-56}})));
      Buildings.Fluid.Sources.FixedBoundary absolutePressure1(
        redeclare package Medium = Medium,
        use_T=false,
        nPorts=1) annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={134,-78})));
      // --- Result (Output) variables
      Modelica.SIunits.Temperature[nbrNodes] TSto=TTank.T;
      Modelica.SIunits.Temperature TTankTopSet;
      Modelica.SIunits.Temperature TTankBotIn;
      Modelica.SIunits.MassFlowRate m_flowDHW;
      Modelica.SIunits.Power QDHW;
      Modelica.SIunits.Temperature TDHW;
      Real SOCTank;
      // --- Temperature sensors
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemSto_top(redeclare
          package
          Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the top outlet of the storage tank (port_a)"
        annotation (Placement(transformation(extent={{104,-18},{96,-10}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemSto_bot(redeclare
          package
          Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the bottom inlet of the storage tank (port_b)"
        annotation (Placement(transformation(extent={{132,-64},{122,-54}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemStoHX_out(redeclare
          package
          Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Temperature at the outlet of the storage tank heat exchanger (port_bHX)"
        annotation (Placement(transformation(extent={{84,-84},{72,-72}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlow1[
        nRadPorts](Q_flow=0)
        annotation (Placement(transformation(extent={{-88,-30},{-108,-10}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlow[
        nConvPorts](Q_flow=0)
        annotation (Placement(transformation(extent={{-86,10},{-106,30}})));
      Modelica.Blocks.Math.Gain gain(k=m_flow_nominal_stoHX) annotation (Placement(
            transformation(
            extent={{-8,-8},{8,8}},
            rotation=0,
            origin={70,70})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nbrNodes] TTank
        annotation (Placement(transformation(extent={{124,-2},{114,8}})));
    equation
    QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow)+ QDHW;
      // Result variables
      TTankTopSet = ctrl_Heating.TTopSet;
      TDHW = dHW.TDHW_actual;
      TTankBotIn = senTemSto_bot.T;
      m_flowDHW = dHW.idealSource.m_flow_in;
      SOCTank = ctrl_Heating.SOC;
      QDHW = -dHW.pipe_HeatPort.heatPort.Q_flow;
      connect(dHW.port_cold, pipeDHW.port_b) annotation (Line(
          points={{85,-38},{85,-59},{90,-59}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(dHW.port_hot,senTemSto_top. port_b) annotation (Line(
          points={{85,-20},{85,-14},{96,-14}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemSto_top.port_a, tesTank.port_a) annotation (Line(
          points={{104,-14},{110,-14},{110,-32}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeDHW.port_a,senTemSto_bot. port_b) annotation (Line(
          points={{106,-59},{122,-59}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemSto_bot.port_a, tesTank.port_b) annotation (Line(
          points={{132,-59},{138,-59},{138,-32}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(absolutePressure1.ports[1], tesTank.port_b) annotation (Line(
          points={{134,-72},{134,-66},{138,-66},{138,-32},{138,-32}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pumpSto.port_b, senTemStoHX_out.port_a) annotation (Line(
          points={{94,-78},{84,-78}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemStoHX_out.port_b, heater.port_a) annotation (Line(
          points={{72,-78},{-66,-78},{-66,2}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(ctrl_Heating.onOff, gain.u)
        annotation (Line(points={{-92,70},{60.4,70},{60.4,70}}, color={0,0,127}));
      connect(gain.y, pumpSto.m_flow_in) annotation (Line(points={{78.8,70},{
              78.8,-48},{101.14,-48},{101.14,-70.8}},
                                                 color={0,0,127}));
      connect(tesTank.portHex_b, pumpSto.port_a) annotation (Line(points={{110,-48},
              {110,-50},{108,-50},{108,-78}}, color={0,127,255}));
      connect(tesTank.heaPorVol, TTank.port) annotation (Line(points={{124,-32},
              {124,-32},{124,-32},{124,3}},
                               color={191,0,0}));
      connect(TTank[posTBot].T, ctrl_Heating.TTankBot) annotation (Line(points={{114,3},
              {114,68},{20,68},{-110,68}},
                                      color={0,0,127}));
      connect(TTank[posTTop].T, ctrl_Heating.TTankTop) annotation (Line(
          points={{114,3},{114,72},{-110,72}},
          color={0,0,127},
          smooth=Smooth.None));
        connect(emission.heatPortCon, heatPortCon) annotation (Line(
          points={{94,27.2},{94,40},{-120,40},{-120,20},{-118,20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(emission.heatPortRad, heatPortRad) annotation (Line(
          points={{100,27.2},{100,40},{-120,40},{-120,-20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(prescribedHeatFlow.port, heatPortEmb)
        annotation (Line(points={{-106,20},{-106,-100},{-60,-100}},
                                                                 color={191,0,0}));
      connect(mDHW60C, dHW.mDHW60C) annotation (Line(points={{80,-104},{-60,-104},{-60,
              -30.8},{80,-30.8}},           color={0,0,127}));
      connect(balancingValve.port_b1, tesTank.portHex_a) annotation (Line(
            points={{-30,21.2},{2,21.2},{2,-39.6},{110,-39.6}},color={0,127,255}));
      connect(prescribedTemperature.port, tesTank.heaPorTop) annotation (Line(
            points={{12,-8},{126,-8},{126,-17.2},{126.8,-17.2}}, color={191,0,0}));
      connect(prescribedTemperature.port, tesTank.heaPorSid) annotation (Line(
            points={{12,-8},{132,-8},{132,-32},{131.84,-32}}, color={191,0,0}));
      connect(prescribedTemperature.port, tesTank.heaPorBot) annotation (Line(
            points={{12,-8},{132,-8},{132,-46.8},{126.8,-46.8}}, color={191,0,0}));
      connect(prescribedTemperature.port, pumpSto.heatPort) annotation (Line(
            points={{12,-8},{102,-8},{102,-82.08},{101,-82.08}}, color={191,0,0}));
      connect(prescribedTemperature.port, pipeDHW.heatPort) annotation (Line(
            points={{12,-8},{102,-8},{102,-57.5},{98,-57.5}}, color={191,0,0}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
                {140,100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-120,-100},{
                140,100}})),
        Documentation(info="<html>
<p><b>Description</b> </p>
<p>Multi-zone Hydraulic heating system with <a href=\"modelica://Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2\">radiators</a> emission system. There is no thermal energy storage tank for the heating, but the domestic hot water (DHW) system has a storage tank with internal heat exchanger. A schematic hydraulic scheme is given below:</p>
<p><img src=\"modelica://FBM/images/HydraulicScheme_Heating_Embedded_DHW_STS.png\"/></p>
<p>For multizone systems, the components <i>pumpRad</i>, <i>emission</i> and <i>pipeReturn</i> are arrays of size <i>nZones</i>. In this model, the <i>emission</i> is a an embedded pipe, the <i>heater</i> is a replaceable component and can be a boiler or heat pump.</p>
<p>There are two controllers in the model (not represented in the hydraulic scheme): one for the heater set temperature and control signal of the pump for charging the DHW storage tank (<a href=\"modelica://FBM.Controls.ControlHeating.Ctrl_Heating_DHW\">Ctrl_Heating_DHW</a>), and another one for the on/off signal of <i>pumpRad</i> (= thermostat). The system is controlled based on a temperature measurement in each zone, a set temperature for each zone, temperature measurements in the storage tank and a general heating curve (not per zone). The heater will produce hot water at a temperature slightly above the required temperature, depending on the heat demand (space heating or DHW). The <i>3-Way valve</i> will mix the supply flow rate with return water to reach the heating curve set point. Right after the <i>3-Way valve</i>, the flow is splitted in <i>nZones</i> flows and each <i>pumpRad</i> will set the flowrate in the zonal distribution circuit based on the zone temperature and set point. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>Controllers try to limit or avoid events for faster simulation</li>
<li>Single heating curve for all zones</li>
<li>Heat emitted through <i>heatPortEmb</i> (to the core of a building construction layer or a <a href=\"modelica://FBM.Components.NakedTabs\">nakedTabs</a>)</li>
<li>All pumps are on/off</li>
<li>No priority: both pumps can run simultaneously (could be improved).</li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts of a <a href=\"modelica://IDEAS.Templates.Interfaces.BaseClasses.Structure\">structure</a>. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
<li>Set all parameters that are required. </li>
<li>Not all parameters of the sublevel components are ported to the uppermost level. Therefore, it might be required to modify these components deeper down the hierarchy. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>This is a system level model, no validation performed. </p>
<p><h4>Example </h4></p>
<p>An example of the use of this model can be found in<a href=\"modelica://FBM.HeatingDHWsystems.ExamplesHeating_Embedded\"> FBM.HeatingDHWsystems.ExamplesHeating_Embedded</a>.</p>
</html>",     revisions="<html>
<p><ul>
<li>2016 November, Wilfried Thomaré: First implementation</li>
</ul></p>
</html>"));
    end Boiler_Radiator_DHW;

    model Ideal_FloorHeating
      "Ideal heating, no DHW, with embedded system (eg. floor heating) "
      extends FBM.HeatingDHWsystems.Interfaces.Partial_IdealHeating;
      extends FBM.Interfaces.BaseClasses.HeatingSystem(
        final isHea = true,
        final isCoo = false,
        nConvPorts = nZones,
        nRadPorts = nZones,
        nTemSen = nZones,
        nEmbPorts=nZones);
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlow1[
        nRadPorts](Q_flow=0)
        annotation (Placement(transformation(extent={{-82,-30},{-102,-10}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlow[
        nConvPorts](Q_flow=0)
        annotation (Placement(transformation(extent={{-82,10},{-102,30}})));
    equation
      for i in 1:nZones loop
        if noEvent((TSet[i] - TSensor[i]) > 0) then
          QHeatZone[i] = Buildings.Utilities.Math.Functions.smoothMin(x1=C[i]*(TSet[i] - TSensor[i])/t, x2=QNom[i],deltaX=1);
        else
          QHeatZone[i] = 0;
        end if;
        heatPortEmb[i].Q_flow = -QHeatZone[i];
      end for;
      QHeaSys = sum(QHeatZone);
      connect(prescribedHeatFlow1.port, heatPortRad) annotation (Line(
          points={{-102,-20},{-120,-20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(prescribedHeatFlow.port, heatPortCon) annotation (Line(
          points={{-102,20},{-118,20}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (Documentation(revisions="<html>

</html>",     info="<html>
<p><b>Description</b> </p>
<p>Ideal heating (no hydraulics) based on embedded heat emission, but with limited power <i>QNom</i> per zone. This model assumes a thermal inertia of each zone and computes the heat flux that would be required to heat up the zone to the set point within a time <i>t</i>. This heat flux is limited to <i>QNom</i> and imposed on the heatPort <i>heatPortEmb</i>. A COP can be passed in order to compute the electricity consumption of the heating.</p>
<p><u>Note</u>: the responsiveness of the system is influenced by the time constant <i>t </i>and of course by the inertia of the embedded system in which the heat is injected. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>No inertia in this model (but inertia supposed in the embedded system to which this model will be linked); responsiveness modelled by time constant <i>t</i> for reaching the temperature set point. </li>
<li>Limited output power according to <i>QNom[nZones]</i></li>
<li>Heat emitted through <i>heatPortEmb</i> </li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding surBou surface of a <a href=\"modelica://Buildings.Rooms.MixedAir\">MixedAir</a> room. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
</ol></p>

</html>"),     Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -100},{140,100}}), graphics),
        Icon(coordinateSystem(extent={{-120,-100},{140,100}})));
    end Ideal_FloorHeating;

    model Ideal_RadiatorHeating "Ideal heating, no DHW, with radiators"
      extends FBM.HeatingDHWsystems.Interfaces.Partial_IdealHeating;
      extends FBM.Interfaces.BaseClasses.HeatingSystem(
        final isHea = true,
        final isCoo = false,
          nConvPorts = nZones,
          nRadPorts = nZones,
          nTemSen = nZones,
          nEmbPorts=0);
    equation
       for i in 1:nZones loop
         if noEvent((TSet[i] - TSensor[i]) > 0) then
           QHeatZone[i] = Buildings.Utilities.Math.Functions.smoothMin(x1=C[i]*(TSet[i] - TSensor[i])/t, x2=QNom[i],deltaX=1);
         else
           QHeatZone[i] = 0;
         end if;
         heatPortRad[i].Q_flow = -fractionRad[i]*QHeatZone[i];
         heatPortCon[i].Q_flow = -(1 - fractionRad[i])*QHeatZone[i];
       end for;
      QHeaSys = sum(QHeatZone);
      annotation (Documentation(info="<html>
<p><b>Description</b> </p>
<p>Ideal heating (no hydraulics) but with limited power <i>QNom</i> per zone. There are no radiators. This model assumes a thermal inertia of each zone and computes the heat flux that would be required to heat up the zone to the set point within a time <i>t</i>. This heat flux is limited to <i>QNom</i> and splitted in a radiative and a convective part which are imposed on the heatPorts <i>heatPortRad</i> and <i>heatPortCon</i> respectively. A COP can be passed in order to compute the electricity consumption of the heating.</p>
<p><u>Note</u>: the responsiveness of the system is influenced by the time constant <i>t</i>.  For small values of<i> t</i>, this system is close to ideal, but for larger values, there may still be deviations between the zone temperature and it&apos;s set point. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>No inertia; responsiveness modelled by time constant <i>t</i> for reaching the temperature set point. </li>
<li>Limited output power according to <i>QNom[nZones]</i></li>
<li>Heat emitted through <i>heatPortRad</i> and <i>heatPortCon</i> </li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts of a <a href=\"modelica://Buildings.Rooms.MixedAir\">MixedAir</a> room. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
<li>Set all parameters that are required. </li>
</ol></p>

</html>"),     Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -100},{120,100}}),
                             graphics),
        Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},{
                120,100}}),
            graphics));
    end Ideal_RadiatorHeating;

    model NoHeatingSystem "No heating or cooling system"
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
        annotation (Placement(transformation(extent={{-68,10},{-88,30}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlowRad[
        nZones](each Q_flow=0) if nRadPorts >=1
        annotation (Placement(transformation(extent={{-66,-30},{-86,-10}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow prescribedHeatFlowEmb[
        nEmbPorts](each Q_flow=0) if nEmbPorts >=1
        annotation (Placement(transformation(extent={{-16,-94},{-36,-74}})));
    equation
      if nConvPorts >=1 then
          connect(prescribedHeatFlowCon.port, heatPortCon) annotation (Line(
          points={{-88,20},{-118,20}},
          color={191,0,0},
          smooth=Smooth.None));
      end if;
      if nEmbPorts >=1 then
        connect(prescribedHeatFlowEmb.port, heatPortEmb) annotation (Line(
          points={{-36,-84},{-48,-84},{-48,-100},{-60,-100}},
          color={191,0,0},
          smooth=Smooth.None));
      end if;
      if nRadPorts >=1 then
        connect(prescribedHeatFlowRad.port, heatPortRad) annotation (Line(
          points={{-86,-20},{-120,-20}},
          color={191,0,0},
          smooth=Smooth.None));
      end if;
      annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -100},{120,100}}), graphics));
    end NoHeatingSystem;

    model Solar_Radiator_TES
      "Basic hydraulic heating (with heating curve) with radiator and thermal energy storage. No DHW"
       extends FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_Test(
        final isHea=true,
        final isCoo=false,
        nConvPorts=nZones,
        nRadPorts=nZones,
        nTemSen=nZones,
        nEmbPorts=0,
        nZones=1,
        minSup=true,
        TSupMin=273.15 + 30,
        redeclare FBM.Components.SolarPannelFieldWithTes heater(
          nSer=5,
          nbrNodes=6,
          VTan=10),
        redeclare Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2
          emission[nZones](
          each T_a_nominal=TSupNom,
          each T_b_nominal=TSupNom - dTSupRetNom,
          TAir_nominal=TRoomNom,
          Q_flow_nominal=QNom,
          redeclare each package Medium = Medium),
        ctrl_Heating(dTHeaterSet=2));
    equation
      QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
      connect(emission.heatPortCon, heatPortCon) annotation (Line(
          points={{86,27.2},{86,42},{-120,42},{-120,32},{-118,32},{-118,20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(emission.heatPortRad, heatPortRad) annotation (Line(
          points={{92,27.2},{92,42},{88,42},{88,42},{-120,42},{-120,-20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(weaBus, heater.weaBus) annotation (Line(
          points={{-70,88},{-38,88},{-38,6},{-38,4},{-57.8,4},{-57.8,4.4}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
                {120,100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-120,-100},{
                120,100}})),
        Documentation(info="<html>
<p><b>Description</b> </p>
<p>Multi-zone Hydraulic heating system with <a href=\"modelica://IDEAS.Thermal.Components.Emission.EmbeddedPipe\">embedded pipe</a> emission system (TABS). Except for the emission system, this model is identical to <a href=\"modelica://FBM.HeatingDHWsystems.SimpleHeatingSystem.Boiler_FloorHeating\">Boiler_FloorHeating</a>. There is no thermal energy storage tank and no domestic hot water system. A schematic hydraulic scheme is given below:</p>
<p><img src=\"modelica://FBM/images/HydraulicScheme_Heating_Emisision_low.png\"/></p>
<p><br/>For multizone systems, the components <i>pumpRad</i>, <i>emission</i> and <i>pipeReturn</i> are arrays of size <i>nZones</i>. In this model, the <i>emission</i> is a an embedded pipe, the <i>heater</i> is a replaceable component and can be a boiler or heat pump.</p>
<p>There are two controllers in the model (not represented in the hydraulic scheme): one for the heater set temperature (<a href=\"modelica://FBM.Controls.ControlHeating.Ctrl_Heating\">Ctrl_Heating</a>), and another one for the on/off signal of <i>pumpRad</i> (= thermostat). The system is controlled based on a temperature measurement in each zone, a set temperature for each zone, and a general heating curve (not per zone). The heater will produce hot water at a temperature slightly above the heating curve, and the <i>3-Way valve</i> will mix it with return water to reach the heating curve set point. Right after the <i>3-Way valve</i>, the flow is splitted in <i>nZones</i> flows and each <i>pumpRad</i> will set the flowrate in the zonal distribution circuit based on the zone temperature and set point. </p>
<p>The heat losses of the heater and all the pipes are connected to a central fix temperature. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>Controllers try to limit or avoid events for faster simulation</li>
<li>Single heating curve for all zones</li>
<li>Heat emitted through <i>heatPortEmb</i> (to the core of a building construction layer or a <a href=\"modelica://IDEAS.Thermal.Components.Emission.NakedTabs\">nakedTabs</a>)</li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts of a <a href=\"modelica://Buildings.Rooms.MixedAir\">MixedAir</a> room. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
<li>Set all parameters that are required. </li>
<li>Not all parameters of the sublevel components are ported to the uppermost level. Therefore, it might be required to modify these components deeper down the hierarchy. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>This is a system level model, no validation performed.</p>
<p><h4>Example </h4></p>
<p>An example of the use of this model can be found in<a href=\"modelica://FBM.HeatingDHWsystems.Examples.Heating_Radiator\"> FBM.HeatingDHWsystems.Examples.Heating_Radiator</a>.</p>
</html>",     revisions="<html>
<p><ul>
<li>2016 November, Wilfried Thomaré: first version</li>
</ul></p>
</html>"));
    end Solar_Radiator_TES;

    model Multi_Radiator
      "Basic hydraulic heating (with heating curve) with radiator. No TES, no DHW"
       extends FBM.HeatingDHWsystems.Interfaces.Partial_HydraulicHeating_Multi_Test(
        final isHea=true,
        final isCoo=false,
        nConvPorts=nZones,
        nRadPorts=nZones,
        nTemSen=nZones,
        nEmbPorts=0,
        nZones=1,
        minSup=true,
        TSupMin=273.15 + 30,
        redeclare Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2
          emission[nZones](
          each T_a_nominal=TSupNom,
          each T_b_nominal=TSupNom - dTSupRetNom,
          TAir_nominal=TRoomNom,
          redeclare each package Medium = Medium,
          Q_flow_nominal=QNom),
        ctrl_Heating(dTHeaterSet=2),
        heater(dp_nominal=sum(m_flow_nominal)/30));
    equation
      QHeaSys = -sum(emission.heatPortCon.Q_flow) - sum(emission.heatPortRad.Q_flow);
      connect(emission.heatPortCon, heatPortCon) annotation (Line(
          points={{86,27.2},{86,38},{86,38},{86,38},{-120,38},{-120,20},{-118,
              20}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(emission.heatPortRad, heatPortRad) annotation (Line(
          points={{92,27.2},{92,38},{92,38},{92,38},{-120,38},{-120,-20},{-120,
              -20}},
          color={191,0,0},
          smooth=Smooth.None));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-100},
                {120,100}})),
        Icon(coordinateSystem(preserveAspectRatio=true, extent={{-120,-100},{
                120,100}})),
        Documentation(info="<html>
<p><b>Description</b> </p>
<p>Multi-zone Hydraulic heating system with <a href=\"modelica://IDEAS.Thermal.Components.Emission.EmbeddedPipe\">embedded pipe</a> emission system (TABS). Except for the emission system, this model is identical to <a href=\"modelica://FBM.HeatingDHWsystems.SimpleHeatingSystem.Boiler_FloorHeating\">Boiler_FloorHeating</a>. There is no thermal energy storage tank and no domestic hot water system. A schematic hydraulic scheme is given below:</p>
<p><img src=\"modelica://FBM/images/HydraulicScheme_Heating_Emisision_low.png\"/></p>
<p><br/>For multizone systems, the components <i>pumpRad</i>, <i>emission</i> and <i>pipeReturn</i> are arrays of size <i>nZones</i>. In this model, the <i>emission</i> is a an embedded pipe, the <i>heater</i> is a replaceable component and can be a boiler or heat pump.</p>
<p>There are two controllers in the model (not represented in the hydraulic scheme): one for the heater set temperature (<a href=\"modelica://FBM.Controls.ControlHeating.Ctrl_Heating\">Ctrl_Heating</a>), and another one for the on/off signal of <i>pumpRad</i> (= thermostat). The system is controlled based on a temperature measurement in each zone, a set temperature for each zone, and a general heating curve (not per zone). The heater will produce hot water at a temperature slightly above the heating curve, and the <i>3-Way valve</i> will mix it with return water to reach the heating curve set point. Right after the <i>3-Way valve</i>, the flow is splitted in <i>nZones</i> flows and each <i>pumpRad</i> will set the flowrate in the zonal distribution circuit based on the zone temperature and set point. </p>
<p>The heat losses of the heater and all the pipes are connected to a central fix temperature. </p>
<p><h4>Assumptions and limitations </h4></p>
<p><ol>
<li>Controllers try to limit or avoid events for faster simulation</li>
<li>Single heating curve for all zones</li>
<li>Heat emitted through <i>heatPortEmb</i> (to the core of a building construction layer or a <a href=\"modelica://IDEAS.Thermal.Components.Emission.NakedTabs\">nakedTabs</a>)</li>
</ol></p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Connect the heating system to the corresponding heatPorts of a <a href=\"modelica://Buildings.Rooms.MixedAir\">MixedAir</a> room. </li>
<li>Connect <i>TSet</i> and <i>TSensor</i> </li>
<li>Set all parameters that are required. </li>
<li>Not all parameters of the sublevel components are ported to the uppermost level. Therefore, it might be required to modify these components deeper down the hierarchy. </li>
</ol></p>
<p><h4>Validation </h4></p>
<p>This is a system level model, no validation performed.</p>
<p><h4>Example </h4></p>
<p>An example of the use of this model can be found in<a href=\"modelica://FBM.HeatingDHWsystems.Examples.Heating_Radiator\"> FBM.HeatingDHWsystems.Examples.Heating_Radiator</a>.</p>
</html>",     revisions="<html>
<p><ul>
<li>2016 November, Wilfried Thomaré: first version</li>
</ul></p>
</html>"));
    end Multi_Radiator;
  end SimpleHeatingSystem;
extends Modelica.Icons.Package;
  package Examples "Models for testing heating systems "
  extends Modelica.Icons.ExamplesPackage;
    model Boiler_Embedded
      "Example and test for heating system with embedded emissionExample and test for heating system with embedded emission"
      import Buildings;
      extends Modelica.Icons.Example;
      final parameter Integer nZones=2 "Number of zones";
    package MediumA = Buildings.Media.Air
        "Medium model for air";
        package MediumW = Buildings.Media.Water "Medium model";
        parameter Integer nConExtWin = 2 "Number of constructions with a window";
      parameter Integer nConBou = 1
        "Number of surface that are connected to constructions that are modeled inside the room";
      parameter Integer nSurBou = 1
        "Number of surface that are connected to the room air volume";
      parameter
        FBM.Components.BaseClasses.RadiantSlabChar[nZones]
                                           radSlaCha_ValidationEmpa
        annotation (Placement(transformation(extent={{-134,-138},{-114,-118}})));
      FBM.HeatingDHWsystems.SimpleHeatingSystem.Boiler_FloorHeating
        heating(
        each RadSlaCha=radSlaCha_ValidationEmpa,
        CvDataSupply=Buildings.Fluid.Types.CvTypes.Kv,
        TSupMin=273.15 + 22,
        redeclare package Medium = MediumW,
        TSupNom=273.15 + 45,
        InsuHeatCondu=0.04,
        nZones=nZones,
        dTSupRetNom=5,
        corFac_val=0,
        AEmb={6*5 for i in 1:nZones},
        KvSupply=60,
        Pipelength=15,
        QNom={3000 for i in 1:nZones},
        TRoomNom={567.3,567.3})
        annotation (Placement(transformation(extent={{114,20},{152,38}})));
      Modelica.Thermal.HeatTransfer.Components.Convection[nZones] convectionTabs
        annotation (Placement(transformation(
            extent={{8,-8},{-8,8}},
            rotation=0,
            origin={76,36})));
      FBM.Components.NakedTabs[nZones] nakedTabs(radSlaCha=
           radSlaCha_ValidationEmpa, A_floor=6*5)
        annotation (Placement(transformation(extent={{104,26},{84,46}})));
      Modelica.Blocks.Sources.RealExpression[nZones] realExpression(each y=11*6*4)
        annotation (Placement(transformation(extent={{150,48},{130,68}})));
      Modelica.Blocks.Sources.Constant mFlowDH(k=0)
        "Room temperature set point at night"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={150,-10})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")
        annotation (Placement(transformation(extent={{-220,218},{-200,238}})));
      parameter
        Buildings.HeatTransfer.Data.OpaqueConstructions.Insulation100Concrete200
        matLayExt(material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
                  "Construction material for exterior walls"
        annotation (Placement(transformation(extent={{-100,-138},{-80,-118}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Brick120 matLayPar
        "Construction material for partition walls"
        annotation (Placement(transformation(extent={{-60,-138},{-40,-118}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayRoo(final
          nLay=2, material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2)})
                          "Construction material for roof"
        annotation (Placement(transformation(extent={{-20,-138},{0,-118}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayFlo(
            final nLay=3, material={Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2),
            Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.10),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.05)})
                          "Construction material for floor"
        annotation (Placement(transformation(extent={{20,-138},{40,-118}})));
      Modelica.Blocks.Sources.Constant uSha(k=0.25)
        "Control signal for the shading device"
        annotation (Placement(transformation(extent={{-136,-34},{-116,-14}})));
      Modelica.Blocks.Routing.Replicator replicator(nout=max(1, nConExtWin))
        annotation (Placement(transformation(extent={{-108,-34},{-88,-14}})));
      Buildings.ThermalZones.Detailed.MixedAir roo(
        redeclare package Medium = MediumA,
        nConPar=1,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        nSurBou=1,
        linearizeRadiation=false,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        nConExt=3,
        nPorts=4,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          each til=Buildings.Types.Tilt.Floor),
        AFlo=7*7,
        hRoo=3,
        datConExt(
          layers={matLayRoo,matLayExt,matLayExt},
          A={7*7,7*3,7*3},
          til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
        surBou(
          each A=7*7,
          each absIR=0.9,
          each absSol=0.9,
          each til=Buildings.Types.Tilt.Floor),
        nConExtWin=nConExtWin,
        T_start=273.15 + 18,
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={2,2},
          wWin={5,5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
        lat=0.73268921998722) "Room model"
        annotation (Placement(transformation(extent={{-36,70},{4,110}})));
      Buildings.ThermalZones.Detailed.MixedAir roo1(
        redeclare package Medium = MediumA,
        nConExt=2,
        nConPar=1,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        linearizeRadiation=false,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        nSurBou=2,
        nPorts=4,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          til={Buildings.Types.Tilt.Floor}),
        AFlo=7*7,
        hRoo=3,
        datConExt(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
        surBou(
          A={7*3,7*7},
          each absIR=0.9,
          each absSol=0.9,
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Ceiling}),
        nConExtWin=nConExtWin,
        T_start=273.15 + 18,
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={2,2},
          wWin={5,5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
        lat=0.73268921998722) "Room model"
        annotation (Placement(transformation(extent={{-36,20},{4,60}})));
      Buildings.HeatTransfer.Sources.FixedTemperature TSoi[nConBou](each T=280.15)
        "Boundary condition for construction"
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            origin={30,12})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nZones] TRoo
        annotation (Placement(transformation(extent={{68,90},{78,100}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaSou(redeclare package Medium = MediumA,
        azi=Buildings.Types.Azimuth.S,
        VRoo=7*7*3,
        s=1)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,150},{-118,190}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaEas(
        azi=Buildings.Types.Azimuth.E,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,128},{-118,168}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaNor(
        azi=Buildings.Types.Azimuth.N,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,110},{-118,150}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaWes(
        azi=Buildings.Types.Azimuth.W,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,90},{-118,130}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaSou1(
        azi=Buildings.Types.Azimuth.S,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,66},{-120,106}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaEas1(
        azi=Buildings.Types.Azimuth.E,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,46},{-120,86}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaNor1(
        azi=Buildings.Types.Azimuth.N,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,26},{-120,66}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaWes1(
        azi=Buildings.Types.Azimuth.W,
        s=1,
        redeclare package Medium = MediumA,
        VRoo=7*7*3)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,8},{-120,48}})));
      Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
            transformation(extent={{-196,172},{-156,212}}), iconTransformation(
              extent={{-298,194},{-278,214}})));
      Modelica.Blocks.Math.MatrixGain gai(K=3.5*[0.5; 0.3; 0.2])
        "Matrix gain to split up heat gain in radiant, convective and latent gain"
        annotation (Placement(transformation(extent={{-72,50},{-52,70}})));
      Modelica.Blocks.Sources.CombiTimeTable intGaiFra(
           extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic, table=[0,0.03;
            3600*8,0.03; 3600*9,0.7; 3600*12,0.7; 3600*12,0.7; 3600*13,0.7; 3600*13,
            0.8; 3600*17,0.8; 3600*19,0.1; 3600*24,0.03])
        "Fraction of internal heat gain"
        annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
      parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearAir13Clear
        glaSys(haveExteriorShade=true, shade=
            Buildings.HeatTransfer.Data.Shades.Gray())
        annotation (Placement(transformation(extent={{50,-138},{70,-118}})));
    equation
      connect(realExpression.y, convectionTabs.Gc) annotation (Line(
          points={{129,58},{76,58},{76,44}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(heating.mDHW60C, mFlowDH.y) annotation (Line(points={{142.5,19.82},
              {140,19.82},{140,1},{150,1}},  color={0,0,127}));
      connect(uSha.y,replicator. u) annotation (Line(
          points={{-115,-24},{-110,-24}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(TSoi.port, roo1.surf_conBou)
        annotation (Line(points={{20,12},{-10,12},{-10,24}}, color={191,0,0}));
      connect(roo.heaPorAir, TRoo[1].port) annotation (Line(points={{-17,90},{48,90},
              {48,94},{56,94},{56,95},{68,95}},
                                color={191,0,0}));
      connect(roo1.heaPorAir, TRoo[2].port) annotation (Line(points={{-17,40},{48,40},
              {48,94},{68,94},{68,95}},
                                color={191,0,0}));
      connect(roo.surf_conBou[1], roo1.surf_surBou[2]) annotation (Line(points={{-10,
              74},{-46,74},{-46,26.5},{-19.8,26.5}}, color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{114.317,
              30.8},{48,30.8},{48,90},{-17,90}},
                                           color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{114,27.2},
              {48,27.2},{48,86.2},{-17,86.2}}, color={191,0,0}));
      connect(leaSou.port_b, roo.ports[1]) annotation (Line(points={{-118,170},{-88,
              170},{-88,77},{-31,77}}, color={0,127,255}));
      connect(leaEas.port_b, roo.ports[2]) annotation (Line(points={{-118,148},{-88,
              148},{-88,79},{-31,79}}, color={0,127,255}));
      connect(leaNor.port_b, roo.ports[3]) annotation (Line(points={{-118,130},{-86,
              130},{-86,81},{-31,81}}, color={0,127,255}));
      connect(leaWes.port_b, roo.ports[4]) annotation (Line(points={{-118,110},{-86,
              110},{-86,83},{-31,83}}, color={0,127,255}));
      connect(leaSou1.port_b, roo1.ports[1]) annotation (Line(points={{-120,86},{
              -104,86},{-104,27},{-31,27}},
                                       color={0,127,255}));
      connect(leaEas1.port_b, roo1.ports[2]) annotation (Line(points={{-120,66},{
              -104,66},{-104,29},{-31,29}},
                                       color={0,127,255}));
      connect(leaNor1.port_b, roo1.ports[3]) annotation (Line(points={{-120,46},{
              -104,46},{-104,31},{-31,31}},
                                       color={0,127,255}));
      connect(leaWes1.port_b, roo1.ports[4]) annotation (Line(points={{-120,28},{
              -104,28},{-104,33},{-31,33}},
                                       color={0,127,255}));
      connect(weaDat.weaBus, weaBus) annotation (Line(
          points={{-200,228},{-176,228},{-176,192}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(weaBus, leaSou.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,170},{-154,170}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaEas.weaBus) annotation (Line(
          points={{-176,192},{-176,148},{-154,148}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaNor.weaBus) annotation (Line(
          points={{-176,192},{-176,130},{-154,130}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaWes.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,110},{-154,110}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaSou1.weaBus) annotation (Line(
          points={{-176,192},{-176,139},{-176,86},{-156,86}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaEas1.weaBus) annotation (Line(
          points={{-176,192},{-176,129},{-176,66},{-156,66}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaNor1.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,46},{-156,46}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaWes1.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,28},{-156,28}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, roo.weaBus) annotation (Line(
          points={{-176,192},{2,192},{2,107.9},{1.9,107.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, roo1.weaBus) annotation (Line(
          points={{-176,192},{1,192},{1,57.9},{1.9,57.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(heating.weaBus, weaBus) annotation (Line(
          points={{104.5,37.1},{110,37.1},{110,192},{-176,192}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(intGaiFra.y,gai. u) annotation (Line(
          points={{-79,60},{-74,60}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash));
      connect(gai.y, roo.qGai_flow) annotation (Line(points={{-51,60},{-48,60},{-48,
              98},{-37.6,98}}, color={0,0,127}));
      connect(gai.y, roo1.qGai_flow) annotation (Line(points={{-51,60},{-46,60},{-46,
              48},{-37.6,48}},     color={0,0,127}));
      connect(roo.heaPorAir, convectionTabs[1].fluid) annotation (Line(points={{-17,
              90},{26,90},{26,36},{68,36}}, color={191,0,0}));
      connect(convectionTabs[2].fluid, roo1.heaPorAir) annotation (Line(points={{68,
              36},{26,36},{26,40},{-17,40}}, color={191,0,0}));
      connect(heating.heatPortCon[2], roo1.heaPorAir) annotation (Line(points={{114.317,
              30.8},{50,30.8},{50,40},{-17,40}}, color={191,0,0}));
      connect(heating.heatPortRad[2], roo1.heaPorRad) annotation (Line(points={{114,
              27.2},{48,27.2},{48,36.2},{-17,36.2}}, color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{114.317,
              30.8},{50,30.8},{50,90},{-17,90}},
                                           color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{114,27.2},
              {48,27.2},{48,86.2},{-17,86.2}}, color={191,0,0}));
      connect(TRoo.T, heating.TSensor) annotation (Line(points={{78,95},{96,95},
              {96,23.6},{113.367,23.6}},
                                    color={0,0,127}));
      connect(nakedTabs.portCore, heating.heatPortEmb) annotation (Line(points={{104,36},
              {110,36},{110,20},{123.5,20}},       color={191,0,0}));
      connect(nakedTabs.port_a, convectionTabs.solid) annotation (Line(points={{94,46},
              {90,46},{90,36},{84,36}}, color={191,0,0}));
      connect(nakedTabs.port_b, convectionTabs.solid) annotation (Line(points={{94,26.2},
              {90,26.2},{90,36},{84,36}}, color={191,0,0}));
      connect(replicator.y, roo.uSha) annotation (Line(points={{-87,-24},{-46,-24},{
              -46,108},{-37.6,108}}, color={0,0,127}));
      connect(replicator.y, roo1.uSha) annotation (Line(points={{-87,-24},{-62,-24},
              {-62,58},{-37.6,58}}, color={0,0,127}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-220,-200},{220,
                240}})),
        experiment(StopTime=200000, Interval=900),
        __Dymola_experimentSetupOutput,
        Icon(coordinateSystem(extent={{-220,-200},{220,240}})));
    end Boiler_Embedded;

    model Solar_Radiator
      "Example and test for heating system with radiator emission and solar production."
      import Buildings;
      extends Modelica.Icons.Example;
      final parameter Integer nZones=2 "Number of zones";
    package MediumA = Buildings.Media.Air
        "Medium model for air";
        package MediumW = Buildings.Media.Water "Medium model";
        parameter Integer nConExtWin = 2 "Number of constructions with a window";
      parameter Integer nConBou = 1
        "Number of surface that are connected to constructions that are modeled inside the room";
      parameter Integer nSurBou = 1
        "Number of surface that are connected to the room air volume";
      FBM.HeatingDHWsystems.SimpleHeatingSystem.Solar_Radiator_TES
        heating(
        CvDataSupply=Buildings.Fluid.Types.CvTypes.Kv,
        TSupMin=273.15 + 22,
        redeclare package Medium = MediumW,
        TSupNom=273.15 + 45,
        InsuHeatCondu=0.04,
        nZones=nZones,
        dTSupRetNom=5,
        corFac_val=0,
        KvSupply=60,
        Pipelength=15,
        QNom={3000 for i in 1:nZones},
        TRoomNom(displayUnit="K") = {294.15,294.15})
        annotation (Placement(transformation(extent={{114,20},{152,38}})));
      Modelica.Blocks.Sources.Constant mFlowDH(k=0)
        "Room temperature set point at night"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={150,-10})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")
        annotation (Placement(transformation(extent={{-220,218},{-200,238}})));
      parameter
        Buildings.HeatTransfer.Data.OpaqueConstructions.Insulation100Concrete200
        matLayExt(material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
                  "Construction material for exterior walls"
        annotation (Placement(transformation(extent={{-100,-138},{-80,-118}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Brick120 matLayPar
        "Construction material for partition walls"
        annotation (Placement(transformation(extent={{-60,-138},{-40,-118}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayRoo(final
          nLay=2, material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2)})
                          "Construction material for roof"
        annotation (Placement(transformation(extent={{-20,-138},{0,-118}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayFlo(
            final nLay=3, material={Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2),
            Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.10),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.05)})
                          "Construction material for floor"
        annotation (Placement(transformation(extent={{20,-138},{40,-118}})));
      Modelica.Blocks.Sources.Constant uSha(k=0.25)
        "Control signal for the shading device"
        annotation (Placement(transformation(extent={{-136,-34},{-116,-14}})));
      Modelica.Blocks.Routing.Replicator replicator(nout=max(1, nConExtWin))
        annotation (Placement(transformation(extent={{-108,-34},{-88,-14}})));
      Buildings.ThermalZones.Detailed.MixedAir roo(
        redeclare package Medium = MediumA,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        linearizeRadiation=false,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        nConExt=3,
        nPorts=4,
        AFlo=7*7,
        hRoo=3,
        datConExt(
          layers={matLayRoo,matLayExt,matLayExt},
          A={7*7,7*3,7*3},
          til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
        nConExtWin=nConExtWin,
        T_start=273.15 + 18,
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={2,2},
          wWin={5,5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
        nConPar=1,
        nSurBou=0,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          each til=Buildings.Types.Tilt.Floor),
        lat=0.73268921998722) "Room model"
        annotation (Placement(transformation(extent={{-36,70},{4,110}})));
      Buildings.ThermalZones.Detailed.MixedAir roo1(
        redeclare package Medium = MediumA,
        nConExt=2,
        nConPar=1,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        linearizeRadiation=false,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        nSurBou=2,
        nPorts=4,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          til={Buildings.Types.Tilt.Floor}),
        AFlo=7*7,
        hRoo=3,
        datConExt(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
        surBou(
          A={7*3,7*7},
          each absIR=0.9,
          each absSol=0.9,
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Ceiling}),
        nConExtWin=nConExtWin,
        T_start=273.15 + 18,
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={2,2},
          wWin={5,5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
        lat=0.73268921998722) "Room model"
        annotation (Placement(transformation(extent={{-36,20},{4,60}})));
      Buildings.HeatTransfer.Sources.FixedTemperature TSoi[nConBou](each T=280.15)
        "Boundary condition for construction"
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            origin={30,12})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nZones] TRoo
        annotation (Placement(transformation(extent={{68,90},{78,100}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaSou(redeclare package Medium = MediumA,
        azi=Buildings.Types.Azimuth.S,
        VRoo=7*7*3,
        s=1)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,150},{-118,190}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaEas(
        azi=Buildings.Types.Azimuth.E,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,128},{-118,168}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaNor(
        azi=Buildings.Types.Azimuth.N,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,110},{-118,150}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaWes(
        azi=Buildings.Types.Azimuth.W,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,90},{-118,130}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaSou1(
        azi=Buildings.Types.Azimuth.S,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,66},{-120,106}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaEas1(
        azi=Buildings.Types.Azimuth.E,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,46},{-120,86}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaNor1(
        azi=Buildings.Types.Azimuth.N,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,26},{-120,66}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaWes1(
        azi=Buildings.Types.Azimuth.W,
        s=1,
        redeclare package Medium = MediumA,
        VRoo=7*7*3)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,8},{-120,48}})));
      Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
            transformation(extent={{-196,172},{-156,212}}), iconTransformation(
              extent={{-298,194},{-278,214}})));
      Modelica.Blocks.Math.MatrixGain gai(K=3.5*[0.5; 0.3; 0.2])
        "Matrix gain to split up heat gain in radiant, convective and latent gain"
        annotation (Placement(transformation(extent={{-72,50},{-52,70}})));
      Modelica.Blocks.Sources.CombiTimeTable intGaiFra(
           extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic, table=[0,0.03;
            3600*8,0.03; 3600*9,0.7; 3600*12,0.7; 3600*12,0.7; 3600*13,0.7; 3600*13,
            0.8; 3600*17,0.8; 3600*19,0.1; 3600*24,0.03])
        "Fraction of internal heat gain"
        annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
      parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearArgon13Clear
        glaSys(
        haveInteriorShade=true,
        shade=Buildings.HeatTransfer.Data.Shades.Gray(),
        UFra=2.1)
        annotation (Placement(transformation(extent={{60,-138},{80,-118}})));
      Modelica.Blocks.Interaction.Show.RealValue realValue
        annotation (Placement(transformation(extent={{146,52},{166,72}})));
    equation
      connect(heating.mDHW60C, mFlowDH.y) annotation (Line(points={{142.5,19.82},
              {140,19.82},{140,1},{150,1}},  color={0,0,127}));
      connect(uSha.y,replicator. u) annotation (Line(
          points={{-115,-24},{-110,-24}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(TSoi.port, roo1.surf_conBou)
        annotation (Line(points={{20,12},{-10,12},{-10,24}}, color={191,0,0}));
      connect(roo.heaPorAir, TRoo[1].port) annotation (Line(points={{-17,90},{48,90},
              {48,94},{56,94},{56,95},{68,95}},
                                color={191,0,0}));
      connect(roo1.heaPorAir, TRoo[2].port) annotation (Line(points={{-17,40},{48,40},
              {48,94},{68,94},{68,95}},
                                color={191,0,0}));
      connect(roo.surf_conBou[1], roo1.surf_surBou[2]) annotation (Line(points={{-10,74},
              {-16,74},{-16,26.5},{-19.8,26.5}},     color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{114.317,
              30.8},{48,30.8},{48,90},{-17,90}},
                                           color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{114,27.2},
              {48,27.2},{48,86.2},{-17,86.2}}, color={191,0,0}));
      connect(leaSou.port_b, roo.ports[1]) annotation (Line(points={{-118,170},{-88,
              170},{-88,77},{-31,77}}, color={0,127,255}));
      connect(leaEas.port_b, roo.ports[2]) annotation (Line(points={{-118,148},{-88,
              148},{-88,79},{-31,79}}, color={0,127,255}));
      connect(leaNor.port_b, roo.ports[3]) annotation (Line(points={{-118,130},{-86,
              130},{-86,81},{-31,81}}, color={0,127,255}));
      connect(leaWes.port_b, roo.ports[4]) annotation (Line(points={{-118,110},{-86,
              110},{-86,83},{-31,83}}, color={0,127,255}));
      connect(leaSou1.port_b, roo1.ports[1]) annotation (Line(points={{-120,86},{
              -104,86},{-104,27},{-31,27}},
                                       color={0,127,255}));
      connect(leaEas1.port_b, roo1.ports[2]) annotation (Line(points={{-120,66},{
              -104,66},{-104,29},{-31,29}},
                                       color={0,127,255}));
      connect(leaNor1.port_b, roo1.ports[3]) annotation (Line(points={{-120,46},{
              -104,46},{-104,31},{-31,31}},
                                       color={0,127,255}));
      connect(leaWes1.port_b, roo1.ports[4]) annotation (Line(points={{-120,28},{
              -104,28},{-104,33},{-31,33}},
                                       color={0,127,255}));
      connect(weaDat.weaBus, weaBus) annotation (Line(
          points={{-200,228},{-176,228},{-176,192}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(weaBus, leaSou.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,170},{-154,170}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaEas.weaBus) annotation (Line(
          points={{-176,192},{-176,148},{-154,148}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaNor.weaBus) annotation (Line(
          points={{-176,192},{-176,130},{-154,130}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaWes.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,110},{-154,110}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaSou1.weaBus) annotation (Line(
          points={{-176,192},{-176,139},{-176,86},{-156,86}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaEas1.weaBus) annotation (Line(
          points={{-176,192},{-176,129},{-176,66},{-156,66}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaNor1.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,46},{-156,46}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaWes1.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,28},{-156,28}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, roo.weaBus) annotation (Line(
          points={{-176,192},{2,192},{2,107.9},{1.9,107.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, roo1.weaBus) annotation (Line(
          points={{-176,192},{1,192},{1,57.9},{1.9,57.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(heating.weaBus, weaBus) annotation (Line(
          points={{104.5,37.1},{110,37.1},{110,192},{-176,192}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(intGaiFra.y,gai. u) annotation (Line(
          points={{-79,60},{-74,60}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash));
      connect(gai.y, roo.qGai_flow) annotation (Line(points={{-51,60},{-48,60},{-48,
              98},{-37.6,98}}, color={0,0,127}));
      connect(gai.y, roo1.qGai_flow) annotation (Line(points={{-51,60},{-46,60},{-46,
              48},{-37.6,48}},     color={0,0,127}));
      connect(heating.heatPortCon[2], roo1.heaPorAir) annotation (Line(points={{114.317,
              30.8},{50,30.8},{50,40},{-17,40}}, color={191,0,0}));
      connect(heating.heatPortRad[2], roo1.heaPorRad) annotation (Line(points={{114,
              27.2},{48,27.2},{48,36.2},{-17,36.2}}, color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{114.317,
              30.8},{50,30.8},{50,90},{-17,90}},
                                           color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{114,27.2},
              {48,27.2},{48,86.2},{-17,86.2}}, color={191,0,0}));
      connect(TRoo.T, heating.TSensor) annotation (Line(points={{78,95},{96,95},
              {96,23.6},{113.367,23.6}},
                                    color={0,0,127}));
      connect(replicator.y, roo.uSha) annotation (Line(points={{-87,-24},{-46,-24},{
              -46,108},{-37.6,108}}, color={0,0,127}));
      connect(replicator.y, roo1.uSha) annotation (Line(points={{-87,-24},{-62,-24},
              {-62,58},{-37.6,58}}, color={0,0,127}));
      connect(heating.TSupSolar, realValue.numberPort) annotation (Line(points=
              {{128.06,38.54},{128.06,50.27},{144.5,50.27},{144.5,62}}, color={
              0,0,127}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-220,-200},{220,
                240}})),
        experiment(StopTime=200000, Interval=900),
        __Dymola_experimentSetupOutput,
        Icon(coordinateSystem(extent={{-220,-200},{220,240}})));
    end Solar_Radiator;

    model SingleProd_radiator "Example and test for heating system with radiator emission."
      import Buildings;
      extends Modelica.Icons.Example;
      final parameter Integer nZones=2 "Number of zones";
    package MediumA = Buildings.Media.Air
        "Medium model for air";
        package MediumW = Buildings.Media.Water "Medium model";
        parameter Integer nConExtWin = 2 "Number of constructions with a window";
      parameter Integer nConBou = 1
        "Number of surface that are connected to constructions that are modeled inside the room";
      parameter Integer nSurBou = 1
        "Number of surface that are connected to the room air volume";
      Modelica.Blocks.Sources.Constant mFlowDH(k=0)
        "Room temperature set point at night"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={162,-76})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam="modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")
        annotation (Placement(transformation(extent={{-220,218},{-200,238}})));
      parameter
        Buildings.HeatTransfer.Data.OpaqueConstructions.Insulation100Concrete200
        matLayExt(material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
                  "Construction material for exterior walls"
        annotation (Placement(transformation(extent={{-100,-138},{-80,-118}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Brick120 matLayPar
        "Construction material for partition walls"
        annotation (Placement(transformation(extent={{-60,-138},{-40,-118}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayRoo(final
          nLay=2, material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.1),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2)})
                          "Construction material for roof"
        annotation (Placement(transformation(extent={{-20,-138},{0,-118}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayFlo(
            final nLay=3, material={Buildings.HeatTransfer.Data.Solids.Concrete(x=0.2),
            Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.10),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.05)})
                          "Construction material for floor"
        annotation (Placement(transformation(extent={{20,-138},{40,-118}})));
      Modelica.Blocks.Sources.Constant uSha(k=0.25)
        "Control signal for the shading device"
        annotation (Placement(transformation(extent={{-136,-34},{-116,-14}})));
      Modelica.Blocks.Routing.Replicator replicator(nout=max(1, nConExtWin))
        annotation (Placement(transformation(extent={{-108,-34},{-88,-14}})));
      Buildings.ThermalZones.Detailed.MixedAir roo(
        redeclare package Medium = MediumA,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        linearizeRadiation=false,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        nConExt=3,
        nPorts=4,
        AFlo=7*7,
        hRoo=3,
        datConExt(
          layers={matLayRoo,matLayExt,matLayExt},
          A={7*7,7*3,7*3},
          til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
        nConExtWin=nConExtWin,
        T_start=273.15 + 18,
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={2,2},
          wWin={5,5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
        nConPar=1,
        nSurBou=0,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          each til=Buildings.Types.Tilt.Floor),
        lat=0.73268921998722) "Room model"
        annotation (Placement(transformation(extent={{-36,70},{4,110}})));
      Buildings.ThermalZones.Detailed.MixedAir roo1(
        redeclare package Medium = MediumA,
        nConExt=2,
        nConPar=1,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        linearizeRadiation=false,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        nSurBou=2,
        nPorts=4,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          til={Buildings.Types.Tilt.Floor}),
        AFlo=7*7,
        hRoo=3,
        datConExt(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
        surBou(
          A={7*3,7*7},
          each absIR=0.9,
          each absSol=0.9,
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Ceiling}),
        nConExtWin=nConExtWin,
        T_start=273.15 + 18,
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={2,2},
          wWin={5,5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
        lat=0.73268921998722) "Room model"
        annotation (Placement(transformation(extent={{-36,20},{4,60}})));
      Buildings.HeatTransfer.Sources.FixedTemperature TSoi[nConBou](each T=280.15)
        "Boundary condition for construction"
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            origin={30,12})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nZones] TRoo
        annotation (Placement(transformation(extent={{68,90},{78,100}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaSou(redeclare package Medium = MediumA,
        azi=Buildings.Types.Azimuth.S,
        VRoo=7*7*3,
        s=1)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,150},{-118,190}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaEas(
        azi=Buildings.Types.Azimuth.E,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,128},{-118,168}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaNor(
        azi=Buildings.Types.Azimuth.N,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,110},{-118,150}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaWes(
        azi=Buildings.Types.Azimuth.W,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-154,90},{-118,130}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaSou1(
        azi=Buildings.Types.Azimuth.S,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,66},{-120,106}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaEas1(
        azi=Buildings.Types.Azimuth.E,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,46},{-120,86}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaNor1(
        azi=Buildings.Types.Azimuth.N,
        VRoo=7*7*3,
        s=1,
        redeclare package Medium = MediumA)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,26},{-120,66}})));
      Buildings.Examples.VAVReheat.ThermalZones.RoomLeakage
                  leaWes1(
        azi=Buildings.Types.Azimuth.W,
        s=1,
        redeclare package Medium = MediumA,
        VRoo=7*7*3)
        "Model for air infiltration through the envelope"
        annotation (Placement(transformation(extent={{-156,8},{-120,48}})));
      Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
            transformation(extent={{-196,172},{-156,212}}), iconTransformation(
              extent={{-298,194},{-278,214}})));
      Modelica.Blocks.Math.MatrixGain gai(K=3.5*[0.5; 0.3; 0.2])
        "Matrix gain to split up heat gain in radiant, convective and latent gain"
        annotation (Placement(transformation(extent={{-72,50},{-52,70}})));
      Modelica.Blocks.Sources.CombiTimeTable intGaiFra(
           extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic, table=[0,0.03;
            3600*8,0.03; 3600*9,0.7; 3600*12,0.7; 3600*12,0.7; 3600*13,0.7; 3600*13,
            0.8; 3600*17,0.8; 3600*19,0.1; 3600*24,0.03])
        "Fraction of internal heat gain"
        annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
      parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearArgon13Clear
        glaSys(
        haveInteriorShade=true,
        shade=Buildings.HeatTransfer.Data.Shades.Gray(),
        UFra=2.1)
        annotation (Placement(transformation(extent={{60,-138},{80,-118}})));
      ModularProductionSystem.SingleProduction_Radiator heating(
        nZones=nZones,
        QNom=ones(nZones)*3000,
        CvDataSupply=Buildings.Fluid.Types.CvTypes.Kv,
        KvSupply=60,
        source=FBM.HeatingDHWsystems.Types.HeatSource.GasBoiler)
        annotation (Placement(transformation(extent={{134,18},{174,38}})));
    equation
      connect(heating.mDHW60C, mFlowDH.y) annotation (Line(points={{160,17.8},{
              160,17.8},{160,-65},{162,-65}}, color={0,0,127}));
      connect(uSha.y,replicator. u) annotation (Line(
          points={{-115,-24},{-110,-24}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(TSoi.port, roo1.surf_conBou)
        annotation (Line(points={{20,12},{-10,12},{-10,24}}, color={191,0,0}));
      connect(roo.heaPorAir, TRoo[1].port) annotation (Line(points={{-17,90},{48,90},
              {48,94},{56,94},{56,95},{68,95}},
                                color={191,0,0}));
      connect(roo1.heaPorAir, TRoo[2].port) annotation (Line(points={{-17,40},{48,40},
              {48,94},{68,94},{68,95}},
                                color={191,0,0}));
      connect(roo.surf_conBou[1], roo1.surf_surBou[2]) annotation (Line(points={{-10,74},{-16,74},{-16,26.5},{-19.8,26.5}},
                                                     color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{142.2,
              30},{48,30},{48,90},{-17,90}},     color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{142,26},
              {48,26},{48,86.2},{-17,86.2}},         color={191,0,0}));
      connect(leaSou.port_b, roo.ports[1]) annotation (Line(points={{-118,170},{-88,
              170},{-88,77},{-31,77}}, color={0,127,255}));
      connect(leaEas.port_b, roo.ports[2]) annotation (Line(points={{-118,148},{-88,
              148},{-88,79},{-31,79}}, color={0,127,255}));
      connect(leaNor.port_b, roo.ports[3]) annotation (Line(points={{-118,130},{-86,
              130},{-86,81},{-31,81}}, color={0,127,255}));
      connect(leaWes.port_b, roo.ports[4]) annotation (Line(points={{-118,110},{-86,
              110},{-86,83},{-31,83}}, color={0,127,255}));
      connect(leaSou1.port_b, roo1.ports[1]) annotation (Line(points={{-120,86},{
              -104,86},{-104,27},{-31,27}},
                                       color={0,127,255}));
      connect(leaEas1.port_b, roo1.ports[2]) annotation (Line(points={{-120,66},{
              -104,66},{-104,29},{-31,29}},
                                       color={0,127,255}));
      connect(leaNor1.port_b, roo1.ports[3]) annotation (Line(points={{-120,46},{
              -104,46},{-104,31},{-31,31}},
                                       color={0,127,255}));
      connect(leaWes1.port_b, roo1.ports[4]) annotation (Line(points={{-120,28},{
              -104,28},{-104,33},{-31,33}},
                                       color={0,127,255}));
      connect(weaDat.weaBus, weaBus) annotation (Line(
          points={{-200,228},{-176,228},{-176,192}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(weaBus, leaSou.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,170},{-154,170}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaEas.weaBus) annotation (Line(
          points={{-176,192},{-176,148},{-154,148}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaNor.weaBus) annotation (Line(
          points={{-176,192},{-176,130},{-154,130}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaWes.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,110},{-154,110}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaSou1.weaBus) annotation (Line(
          points={{-176,192},{-176,139},{-176,86},{-156,86}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaEas1.weaBus) annotation (Line(
          points={{-176,192},{-176,129},{-176,66},{-156,66}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaNor1.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,46},{-156,46}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, leaWes1.weaBus) annotation (Line(
          points={{-176,192},{-176,192},{-176,28},{-156,28}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, roo.weaBus) annotation (Line(
          points={{-176,192},{2,192},{2,107.9},{1.9,107.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, roo1.weaBus) annotation (Line(
          points={{-176,192},{1,192},{1,57.9},{1.9,57.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(heating.weaBus, weaBus) annotation (Line(
          points={{136,37},{110,37},{110,192},{-176,192}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(intGaiFra.y,gai. u) annotation (Line(
          points={{-79,60},{-74,60}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash));
      connect(gai.y, roo.qGai_flow) annotation (Line(points={{-51,60},{-48,60},{-48,
              98},{-37.6,98}}, color={0,0,127}));
      connect(gai.y, roo1.qGai_flow) annotation (Line(points={{-51,60},{-46,60},{-46,
              48},{-37.6,48}},     color={0,0,127}));
      connect(heating.heatPortCon[2], roo1.heaPorAir) annotation (Line(points={{142.2,
              30},{50,30},{50,40},{-17,40}},      color={191,0,0}));
      connect(heating.heatPortRad[2], roo1.heaPorRad) annotation (Line(points={{142,26},
              {48,26},{48,36.2},{-17,36.2}},          color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{142.2,
              30},{50,30},{50,90},{-17,90}},     color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{142,26},
              {48,26},{48,86.2},{-17,86.2}},         color={191,0,0}));
      connect(TRoo.T, heating.TSensor) annotation (Line(points={{78,95},{96,95},
              {96,22},{141.6,22}}, color={0,0,127}));
      connect(replicator.y, roo.uSha) annotation (Line(points={{-87,-24},{-46,-24},{
              -46,108},{-37.6,108}}, color={0,0,127}));
      connect(replicator.y, roo1.uSha) annotation (Line(points={{-87,-24},{-62,-24},
              {-62,58},{-37.6,58}}, color={0,0,127}));
      annotation (
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-220,-200},{220,
                240}})),
        experiment(StopTime=200000, Interval=900),
        __Dymola_experimentSetupOutput,
        Icon(coordinateSystem(extent={{-220,-200},{220,240}})));
    end SingleProd_radiator;

    model MultiProd_radiator
      "Example and test for Multi heating system with radiator emission."
      import Buildings;
      extends Modelica.Icons.Example;
      final parameter Integer nZones=2 "Number of zones";
    package MediumA = Buildings.Media.Air
        "Medium model for air";
        package MediumW = Buildings.Media.Water "Medium model";
        parameter Integer nConExtWin = 2 "Number of constructions with a window";
      parameter Integer nConBou = 1
        "Number of surface that are connected to constructions that are modeled inside the room";
      parameter Integer nSurBou = 1
        "Number of surface that are connected to the room air volume";
      Modelica.Blocks.Sources.Constant mFlowDH(k=0)
        "Room temperature set point at night"
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={162,-76})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")
        annotation (Placement(transformation(extent={{-126,158},{-106,178}})));
      parameter
        Buildings.HeatTransfer.Data.OpaqueConstructions.Insulation100Concrete200
        matLayExt(material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(
            x=0.20),Buildings.HeatTransfer.Data.Solids.Concrete(x=0.20)})
                  "Construction material for exterior walls"
        annotation (Placement(transformation(extent={{-134,-44},{-114,-24}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Brick120 matLayPar
        "Construction material for partition walls"
        annotation (Placement(transformation(extent={{-94,-44},{-74,-24}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayRoo(final
          nLay=2, material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(
            x=0.20),Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
                          "Construction material for roof"
        annotation (Placement(transformation(extent={{-54,-44},{-34,-24}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayFlo(
            final nLay=3, material={Buildings.HeatTransfer.Data.Solids.Concrete(
            x=0.15),Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.10),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.05)})
                          "Construction material for floor"
        annotation (Placement(transformation(extent={{-14,-44},{6,-24}})));
      Modelica.Blocks.Sources.Constant uSha(k=0.25)
        "Control signal for the shading device"
        annotation (Placement(transformation(extent={{-122,90},{-102,110}})));
      Modelica.Blocks.Routing.Replicator replicator(nout=max(1, nConExtWin))
        annotation (Placement(transformation(extent={{-94,90},{-74,110}})));
      Buildings.ThermalZones.Detailed.MixedAir roo(
        redeclare package Medium = MediumA,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        nConExt=3,
        AFlo=7*7,
        hRoo=3,
        datConExt(
          layers={matLayRoo,matLayExt,matLayExt},
          A={7*7,7*3,7*3},
          til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
        nConExtWin=nConExtWin,
        nConPar=1,
        nSurBou=0,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          each til=Buildings.Types.Tilt.Floor),
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={1.7,1.7},
          wWin={3.5,3.5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
        lat=0.65484753534827,
        T_start=273.15 + 18) "Room model"
        annotation (Placement(transformation(extent={{-36,70},{4,110}})));
      Buildings.ThermalZones.Detailed.MixedAir roo1(
        redeclare package Medium = MediumA,
        nConExt=2,
        nConPar=1,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        nSurBou=2,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          til={Buildings.Types.Tilt.Floor}),
        AFlo=7*7,
        hRoo=3,
        surBou(
          A={7*3,7*7},
          each absIR=0.9,
          each absSol=0.9,
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Ceiling}),
        nConExtWin=nConExtWin,
        datConExt(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.E,Buildings.Types.Azimuth.N}),
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={1.7,1.7},
          wWin={3.5,3.5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W}),
        lat=0.65484753534827,
        T_start=273.15 + 18) "Room model"
        annotation (Placement(transformation(extent={{-36,20},{4,60}})));
      Buildings.HeatTransfer.Sources.FixedTemperature TSoi[nConBou](each T=
            281.15)
        "Boundary condition for construction"
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            origin={30,12})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nZones] TRoo
        annotation (Placement(transformation(extent={{68,90},{78,100}})));
      Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
            transformation(extent={{-102,112},{-62,152}}),  iconTransformation(
              extent={{-298,194},{-278,214}})));
      Modelica.Blocks.Math.MatrixGain gai(K=2*[0.5; 0.3; 0.2])
        "Matrix gain to split up heat gain in radiant, convective and latent gain"
        annotation (Placement(transformation(extent={{-72,50},{-52,70}})));
      Modelica.Blocks.Sources.CombiTimeTable intGaiFra(
           extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic, table=[0,0.03;
            3600*8,0.03; 3600*9,0.7; 3600*12,0.7; 3600*12,0.7; 3600*13,0.7; 3600*13,
            0.8; 3600*17,0.8; 3600*19,0.1; 3600*24,0.03])
        "Fraction of internal heat gain"
        annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
      FBM.HeatingDHWsystems.SimpleHeatingSystem.Multi_Radiator heating(
        nZones=nZones,
        CvDataSupply=Buildings.Fluid.Types.CvTypes.Kv,
        nSource=2,
        InsuHeatCondu=0.032,
        lat(displayUnit="deg") = 0.65484753534827,
        source={FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler,FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField},
        kInsTes=0.032,
        til=0.785,
        includePipesSol=true,
        QNom=ones(nZones)*2000,
        VTanTes=12,
        redeclare package Medium = MediumW,
        nbrNodesTes=24,
        Tes_start(displayUnit="K") = 273.15 + 65,
        KvSupply=50,
        perColector=
            Buildings.Fluid.SolarCollectors.Data.GlazedFlatPlate.FP_ThermaLiteHS20(),
        mPrim_flow_nominal=0.745,
        nSer=5,
        hTanTes=4,
        dInsTes=0.3,
        TSupNom=273.15 + 55,
        QBoiler(displayUnit="kW") = 12000,
        InsuPipeThickness=0.04,
        dpHexPrim=18000,
        dpHexSecon=18000,
        T_nominal=353.15)
        annotation (Placement(transformation(extent={{134,18},{174,38}})));
      parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearAir13Clear
        glaSys(haveExteriorShade=true, shade=
            Buildings.HeatTransfer.Data.Shades.Gray())
        annotation (Placement(transformation(extent={{18,-44},{38,-24}})));
    equation
      connect(heating.mDHW60C, mFlowDH.y) annotation (Line(points={{164,17.8},{
              160,17.8},{160,-65},{162,-65}}, color={0,0,127}));
      connect(uSha.y,replicator. u) annotation (Line(
          points={{-101,100},{-96,100}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(TSoi.port, roo1.surf_conBou)
        annotation (Line(points={{20,12},{-10,12},{-10,24}}, color={191,0,0}));
      connect(roo.heaPorAir, TRoo[1].port) annotation (Line(points={{-17,90},{48,90},
              {48,94},{56,94},{56,95},{68,95}},
                                color={191,0,0}));
      connect(roo1.heaPorAir, TRoo[2].port) annotation (Line(points={{-17,40},{48,40},
              {48,94},{68,94},{68,95}},
                                color={191,0,0}));
      connect(roo.surf_conBou[1], roo1.surf_surBou[2]) annotation (Line(points={{-10,74},{-16,74},{-16,26.5},{-19.8,26.5}},
                                                     color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{134.333,
              30},{48,30},{48,90},{-17,90}},     color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{134,26},
              {48,26},{48,86.2},{-17,86.2}},         color={191,0,0}));
      connect(weaDat.weaBus, weaBus) annotation (Line(
          points={{-106,168},{-82,168},{-82,132}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(weaBus, roo.weaBus) annotation (Line(
          points={{-82,132},{2,132},{2,107.9},{1.9,107.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, roo1.weaBus) annotation (Line(
          points={{-82,132},{1,132},{1,57.9},{1.9,57.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(heating.weaBus, weaBus) annotation (Line(
          points={{124,37},{110,37},{110,132},{-82,132}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(intGaiFra.y,gai. u) annotation (Line(
          points={{-79,60},{-74,60}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash));
      connect(gai.y, roo.qGai_flow) annotation (Line(points={{-51,60},{-48,60},{-48,
              98},{-37.6,98}}, color={0,0,127}));
      connect(gai.y, roo1.qGai_flow) annotation (Line(points={{-51,60},{-46,60},{-46,
              48},{-37.6,48}},     color={0,0,127}));
      connect(heating.heatPortCon[2], roo1.heaPorAir) annotation (Line(points={{134.333,
              30},{50,30},{50,40},{-17,40}},      color={191,0,0}));
      connect(heating.heatPortRad[2], roo1.heaPorRad) annotation (Line(points={{134,26},
              {48,26},{48,36.2},{-17,36.2}},          color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{134.333,
              30},{50,30},{50,90},{-17,90}},     color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{134,26},
              {48,26},{48,86.2},{-17,86.2}},         color={191,0,0}));
      connect(TRoo.T, heating.TSensor) annotation (Line(points={{78,95},{96,95},
              {96,22},{133.333,22}},
                                   color={0,0,127}));
      connect(replicator.y, roo.uSha) annotation (Line(points={{-73,100},{-46,
              100},{-46,108},{-37.6,108}},
                                     color={0,0,127}));
      connect(replicator.y, roo1.uSha) annotation (Line(points={{-73,100},{-62,
              100},{-62,58},{-37.6,58}},
                                    color={0,0,127}));
      annotation ( preferredView="info",
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},
                {180,180}})),
        experiment(StopTime=200000, Interval=900),
        __Dymola_experimentSetupOutput,
        Icon(coordinateSystem(extent={{-140,-100},{180,180}})),
        Documentation(info="<html>
<p>
Example for the used of the <a href=\"modelica://FBM.HeatingDHWsystems.ModularProductionSystem.MultiProduction_Radiator\">Multi-source template</a> </p>
<p>The example is composed of the same building that other examples with same radiators implementation but with two heat source and a thermal heat storage tank (TES).
The Heating system is composed of <a href=\"modelica://FBM.Components.SolarPannelFieldWithoutTes\">Solar collector</a> connect to the TES for Inter-seasonal heat storage and a usual gas boiler for the winter supplement
</p>
</html>",
    revisions="<html>
<ul>
<li>
December 2016, Wilfried Thomaré :<br/>
First Implementation
</li>
</ul>
</html>"));
    end MultiProd_radiator;

    model MultiProd_radiator_DHW
      "Example and test for Multi heating system with radiator emission."
      import Buildings;
      extends Modelica.Icons.Example;
      final parameter Integer nZones=2 "Number of zones";
    package MediumA = Buildings.Media.Air
        "Medium model for air";
        package MediumW = Buildings.Media.Water "Medium model";
        parameter Integer nConExtWin = 2 "Number of constructions with a window";
      parameter Integer nConBou = 1
        "Number of surface that are connected to constructions that are modeled inside the room";
      parameter Integer nSurBou = 1
        "Number of surface that are connected to the room air volume";
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(filNam=
            "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos")
        annotation (Placement(transformation(extent={{-112,110},{-92,130}})));
      parameter
        Buildings.HeatTransfer.Data.OpaqueConstructions.Insulation100Concrete200
        matLayExt(material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.20),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.20)})
                  "Construction material for exterior walls"
        annotation (Placement(transformation(extent={{-106,-74},{-86,-54}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Brick120 matLayPar
        "Construction material for partition walls"
        annotation (Placement(transformation(extent={{-80,-74},{-60,-54}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayRoo(final
          nLay=2, material={Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.20),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15)})
                          "Construction material for roof"
        annotation (Placement(transformation(extent={{-54,-74},{-34,-54}})));
      parameter Buildings.HeatTransfer.Data.OpaqueConstructions.Generic matLayFlo(
            final nLay=3, material={Buildings.HeatTransfer.Data.Solids.Concrete(x=0.15),
            Buildings.HeatTransfer.Data.Solids.InsulationBoard(x=0.10),
            Buildings.HeatTransfer.Data.Solids.Concrete(x=0.05)})
                          "Construction material for floor"
        annotation (Placement(transformation(extent={{-14,-74},{6,-54}})));
      Modelica.Blocks.Sources.Constant uSha(k=0.25)
        "Control signal for the shading device"
        annotation (Placement(transformation(extent={{-112,88},{-92,108}})));
      Modelica.Blocks.Routing.Replicator replicator(nout=max(1, nConExtWin))
        annotation (Placement(transformation(extent={{-84,88},{-64,108}})));
      Buildings.ThermalZones.Detailed.MixedAir roo(
        redeclare package Medium = MediumA,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        nConExt=3,
        AFlo=7*7,
        hRoo=3,
        datConExt(
          layers={matLayRoo,matLayExt,matLayExt},
          A={7*7,7*3,7*3},
          til={Buildings.Types.Tilt.Ceiling,Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W,Buildings.Types.Azimuth.N}),
        nConExtWin=nConExtWin,
        nConPar=1,
        nSurBou=0,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          each til=Buildings.Types.Tilt.Floor),
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={1.7,1.7},
          wWin={3.5,3.5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.E}),
        T_start=273.15 + 17,
        lat=0.65484753534827) "Room model"
        annotation (Placement(transformation(extent={{-36,70},{4,110}})));
      Buildings.ThermalZones.Detailed.MixedAir roo1(
        redeclare package Medium = MediumA,
        nConExt=2,
        nConPar=1,
        datConPar(
          layers={matLayPar},
          each A=10,
          each til=Buildings.Types.Tilt.Wall),
        nConBou=1,
        nSurBou=2,
        datConBou(
          layers={matLayFlo},
          each A=7*7,
          til={Buildings.Types.Tilt.Floor}),
        AFlo=7*7,
        hRoo=3,
        surBou(
          A={7*3,7*7},
          each absIR=0.9,
          each absSol=0.9,
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Ceiling}),
        nConExtWin=nConExtWin,
        datConExt(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.E,Buildings.Types.Azimuth.N}),
        datConExtWin(
          layers={matLayExt,matLayExt},
          A={7*3,7*3},
          glaSys={glaSys,glaSys},
          hWin={1.7,1.7},
          wWin={3.5,3.5},
          ove(
            wR={0,0},
            wL={0,0},
            gap={0.1,0.1},
            dep={1,1}),
          fFra={0.1,0.1},
          til={Buildings.Types.Tilt.Wall,Buildings.Types.Tilt.Wall},
          azi={Buildings.Types.Azimuth.S,Buildings.Types.Azimuth.W}),
        T_start=273.15 + 17,
        lat=0.65484753534827) "Room model"
        annotation (Placement(transformation(extent={{-36,20},{4,60}})));
      Buildings.HeatTransfer.Sources.FixedTemperature TSoi[nConBou](each T=
            281.15)
        "Boundary condition for construction"
        annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            origin={30,12})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nZones] TRoo
        annotation (Placement(transformation(extent={{70,88},{80,98}})));
      Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
            transformation(extent={{-84,100},{-44,140}}),   iconTransformation(
              extent={{-298,194},{-278,214}})));
      Modelica.Blocks.Math.MatrixGain gai(K=2*[0.5; 0.3; 0.2])
        "Matrix gain to split up heat gain in radiant, convective and latent gain"
        annotation (Placement(transformation(extent={{-72,50},{-52,70}})));
      Modelica.Blocks.Sources.CombiTimeTable intGaiFra(
           extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic, table=[0,0.03;
            3600*8,0.03; 3600*9,0.7; 3600*12,0.7; 3600*12,0.7; 3600*13,0.7; 3600*13,
            0.8; 3600*17,0.8; 3600*19,0.1; 3600*24,0.03])
        "Fraction of internal heat gain"
        annotation (Placement(transformation(extent={{-100,50},{-80,70}})));
      parameter Buildings.HeatTransfer.Data.GlazingSystems.DoubleClearArgon13Clear
        glaSys(
        haveInteriorShade=true,
        shade=Buildings.HeatTransfer.Data.Shades.Gray(),
        UFra=2.1)
        annotation (Placement(transformation(extent={{26,-74},{46,-54}})));
      FBM.HeatingDHWsystems.ModularProductionSystem.MultiProduction_Radiator_DHW heating(
        nZones=nZones,
        CvDataSupply=Buildings.Fluid.Types.CvTypes.Kv,
        nSource=2,
        InsuHeatCondu=0.032,
        lat(displayUnit="deg") = 0.65484753534827,
        source={FBM.HeatingDHWsystems.Types.HeatSource.CondensingBoiler,FBM.HeatingDHWsystems.Types.HeatSource.SolarPannelField},
        kInsTes=0.032,
        til=0.785,
        includePipesSol=true,
        VTanTes=12,
        redeclare package Medium = MediumW,
        mPrim_flow_nominal=0.745,
        nSer=5,
        dInsTes=0.3,
        TSupNom=273.15 + 55,
        volumeTank=2,
        nPar=3,
        hTanTes=3,
        KvSupply=60,
        QNom=ones(nZones)*4000,
        nbrNodesTes=10,
        dpHexPrim=12000,
        dpHexSecon=12000,
        QBoiler(displayUnit="kW") = 16000,
        T_nominal=343.15,
        Tes_start(displayUnit="K") = 273.15 + 70,
        InsuPipeThickness=0.03)
        annotation (Placement(transformation(extent={{116,20},{156,40}})));
      Modelica.Blocks.Sources.Pulse mDHW60C(
        period=10000,
        startTime=5000,
        offset=0,
        amplitude=0.1,
        width=4)        annotation (Placement(transformation(extent={{-10,-10},
                {10,10}},
            rotation=90,
            origin={142,-58})));
    equation
      connect(uSha.y,replicator. u) annotation (Line(
          points={{-91,98},{-86,98}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(TSoi.port, roo1.surf_conBou)
        annotation (Line(points={{20,12},{-10,12},{-10,24}}, color={191,0,0}));
      connect(roo.heaPorAir, TRoo[1].port) annotation (Line(points={{-17,90},{
              48,90},{48,94},{56,94},{56,93},{70,93}},
                                color={191,0,0}));
      connect(roo1.heaPorAir, TRoo[2].port) annotation (Line(points={{-17,40},{
              48,40},{48,94},{70,94},{70,93}},
                                color={191,0,0}));
      connect(roo.surf_conBou[1], roo1.surf_surBou[2]) annotation (Line(points={{-10,74},{-16,74},{-16,26.5},{-19.8,26.5}},
                                                     color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{124.2,
              32},{48,32},{48,90},{-17,90}},     color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{124,28},
              {48,28},{48,86.2},{-17,86.2}},         color={191,0,0}));
      connect(weaDat.weaBus, weaBus) annotation (Line(
          points={{-92,120},{-64,120}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(weaBus, roo.weaBus) annotation (Line(
          points={{-64,120},{20,120},{20,107.9},{1.9,107.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(weaBus, roo1.weaBus) annotation (Line(
          points={{-64,120},{19,120},{19,57.9},{1.9,57.9}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(heating.weaBus, weaBus) annotation (Line(
          points={{118,39},{118,39},{118,120},{-64,120}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%second",
          index=1,
          extent={{6,3},{6,3}}));
      connect(intGaiFra.y,gai. u) annotation (Line(
          points={{-79,60},{-74,60}},
          color={0,0,127},
          smooth=Smooth.None,
          pattern=LinePattern.Dash));
      connect(gai.y, roo.qGai_flow) annotation (Line(points={{-51,60},{-48,60},{-48,
              98},{-37.6,98}}, color={0,0,127}));
      connect(gai.y, roo1.qGai_flow) annotation (Line(points={{-51,60},{-46,60},{-46,
              48},{-37.6,48}},     color={0,0,127}));
      connect(heating.heatPortCon[2], roo1.heaPorAir) annotation (Line(points={{124.2,
              32},{50,32},{50,40},{-17,40}},      color={191,0,0}));
      connect(heating.heatPortRad[2], roo1.heaPorRad) annotation (Line(points={{124,28},
              {48,28},{48,36.2},{-17,36.2}},          color={191,0,0}));
      connect(heating.heatPortCon[1], roo.heaPorAir) annotation (Line(points={{124.2,
              32},{50,32},{50,90},{-17,90}},     color={191,0,0}));
      connect(heating.heatPortRad[1], roo.heaPorRad) annotation (Line(points={{124,28},
              {48,28},{48,86.2},{-17,86.2}},         color={191,0,0}));
      connect(TRoo.T, heating.TSensor) annotation (Line(points={{80,93},{96,93},
              {96,24},{123.6,24}}, color={0,0,127}));
      connect(replicator.y, roo.uSha) annotation (Line(points={{-63,98},{-42,98},
              {-42,108},{-37.6,108}},color={0,0,127}));
      connect(replicator.y, roo1.uSha) annotation (Line(points={{-63,98},{-48,
              98},{-48,58},{-37.6,58}},
                                    color={0,0,127}));
      connect(mDHW60C.y, heating.mDHW60C) annotation (Line(points={{142,-47},{
              142,-47},{142,19.8}}, color={0,0,127}));
      annotation ( preferredView="info",
        Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,-80},
                {160,140}})),
        experiment(StopTime=200000, Interval=900),
        __Dymola_experimentSetupOutput,
        Icon(coordinateSystem(extent={{-120,-80},{160,140}})),
        Documentation(info="<html>
<p>
Example for the used of the <a href=\"modelica://FBM.HeatingDHWsystems.ModularProductionSystem.MultiProduction_Radiator\">Multi-source template</a> </p>
<p>The example is composed of the same building that other examples with same radiators implementation but with two heat source and a thermal heat storage tank (TES).
The Heating system is composed of <a href=\"modelica://FBM.Components.SolarPannelFieldWithoutTes\">Solar collector</a> connect to the TES for Inter-seasonal heat storage and a usual gas boiler for the winter supplement
</p>
</html>",
    revisions="<html>
<ul>
<li>
December 2016, Wilfried Thomaré :<br/>
First Implementation
</li>
</ul>
</html>"));
    end MultiProd_radiator_DHW;
  end Examples;

  package Interfaces "Partial models for heating systems"
  extends Modelica.Icons.InterfacesPackage;
    partial model Partial_IdealHeating
      "Ideal, non-hydraulic heating with limited power"
      parameter Integer nZones(min=1)
        "Number of conditioned thermal zones in the building";
      parameter Real fractionRad[nZones]=0.3*ones(nZones)
        "Fraction of radiative to total power";
      parameter Real COP=3 "virtual COP to get a PEl as output";
      parameter Modelica.SIunits.Time t=10
        "Time needed to reach temperature setpoint";
      parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
        "Nominal power, can be seen as the max power of the emission system per zone";
      parameter Real[nZones] VZones = 50*ones(nZones)
        "Conditioned volumes of the zones";
      final parameter Modelica.SIunits.HeatCapacity[nZones] C=1012*1.204*VZones*5
        "Heat capacity of the conditioned zones (air capacity with a correction factor of 5)";
      Modelica.SIunits.Power[nZones] QHeatZone;
    end Partial_IdealHeating;

    partial model Partial_HydraulicHeating "Hydraulic multi-zone heating "
      replaceable package Medium = Buildings.Media.Water;
      extends FBM.Interfaces.BaseClasses.HeatingSystem(
        isHea = true,
        isCoo = false,
        nConvPorts = nZones,
        nRadPorts = nZones,
        nTemSen = nZones,
        nEmbPorts=0,
        nZones=1);
      // --- Parameter: General parameters for the design (nominal) conditions and heat curve
      parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
        "Nominal power, can be seen as the max power of the emission system per zone";
      parameter Boolean minSup=true
        "true to limit the supply temperature on the lower side";
        parameter Modelica.SIunits.Temperature TSupMin=273.15 + 30
        "Minimum supply temperature if enabled";
      parameter Modelica.SIunits.Temperature TSupNom=273.15 + 45
        "Nominal supply temperature";
      parameter Modelica.SIunits.TemperatureDifference dTSupRetNom=10
        "Nominal DT in the heating system";
      parameter Modelica.SIunits.Temperature[nZones] TRoomNom={294.15 for i in 1:
          nZones} "Nominal room temperature";
      parameter Modelica.SIunits.TemperatureDifference corFac_val = 0
        "correction term for TSet of the heating curve";
      parameter Modelica.SIunits.Time timeFilter=43200
        "Time constant for the filter of ambient temperature for computation of heating curve";
      final parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal = QNom/(4180.6*dTSupRetNom)
        "Nominal mass flow rates";
        // -- Pipes and valve parameters
      parameter Modelica.SIunits.Thickness InsuPipeThickness=0.02
                                                                 "Thickness of the pipe insulation";
      parameter Modelica.SIunits.Length Pipelength=5
                                                    "pipe length of the supply OR return branch";
      parameter Modelica.SIunits.Pressure dp=3000 "Pressure drop over a single pipe"
        annotation(Dialog(group = "Pipes",
                         enable = includePipes));
     parameter Modelica.SIunits.ThermalConductivity InsuHeatCondu=0.04;
     parameter Real fraKSupply(min=0, max=1) = 0.7
        "Fraction Kv(port_3->port_2)/Kv(port_1->port_2)";
     parameter Real[2] lSupply(each min=0, each max=1) = {0.01, 0.01}
        "Valve leakage, l=Kv(y=0)/Kv(y=1)";
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
                         enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Av)));
      parameter Real deltaMSupply = 0.02
        "Fraction of nominal flow rate where linearization starts, if y=1"
        annotation(Dialog(group="Pressure-flow linearization"));
      parameter Modelica.SIunits.Pressure dpValve_nominalSupply(displayUnit="Pa",
                                                          min=0,
                                                          fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.OpPoint then true else false)
        "Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint"
        annotation(Dialog(group="Nominal condition",
                   enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      parameter Modelica.SIunits.Density rhoStdSupply=Medium.density_pTX(101325, 273.15+4, Medium.X_default)
        "Inlet density for which valve coefficients are defined"
      annotation(Dialog(group="Nominal condition", tab="Advanced"));
    protected
      parameter Real Kv_SISupply(
        min=0,
        fixed= false)
        "Flow coefficient for fully open valve in SI units, Kv=m_flow/sqrt(dp) [kg/s/(Pa)^(1/2)]"
      annotation(Dialog(group = "Flow Coefficient Supply Valve",
                        enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      // --- production components of hydraulic circuit
      replaceable Buildings.Fluid.Boilers.BoilerPolynomial heater(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        Q_flow_nominal=sum(QNom),
        dp_nominal=0,
        fue=Buildings.Fluid.Data.Fuels.NaturalGasHigherHeatingValue()) "Heater (boiler, heat pump, ...)"
        annotation (Placement(transformation(extent={{10,10},{-10,-10}},
            rotation=270,
            origin={-124,22})));
      // --- distribution components of hydraulic circuit
      Buildings.Fluid.Movers.FlowControlled_m_flow[nZones] pumpRad(
        m_flow_nominal=m_flow_nominal,
        redeclare each replaceable package Medium = Medium,
        each tau=30,
        each filteredSpeed=false,
        each nominalValuesDefineDefaultPressureCurve=true)
                  annotation (Placement(transformation(extent={{98,72},{122,48}})));
      Buildings.Fluid.Actuators.Valves.ThreeWayLinear idealCtrlMixer(
        m_flow_nominal=sum(m_flow_nominal),
        redeclare replaceable package Medium = Medium,
        CvData=Buildings.Fluid.Types.CvTypes.Kv,
        Kv=KvSupply,
        fraK=fraKSupply,
        l=lSupply,
        deltaM=deltaMSupply)                           annotation (Placement(transformation(extent={{34,46},{56,70}})));
      Buildings.Fluid.FixedResistances.Pipe pipeReturn(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        nSeg=3,
        thicknessIns=InsuPipeThickness,
        lambdaIns=InsuHeatCondu,
        length=Pipelength)
               annotation (Placement(transformation(extent={{2,-88},{-18,-96}})));
      Buildings.Fluid.FixedResistances.Pipe pipeSupply(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        nSeg=3,
        thicknessIns=InsuPipeThickness,
        lambdaIns=InsuHeatCondu,
        length=Pipelength)
               annotation (Placement(transformation(extent={{-16,54},{4,62}})));
      Buildings.Fluid.FixedResistances.Pipe[nZones] pipeReturnEmission(
        redeclare each replaceable package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        each nSeg=3,
        each thicknessIns=InsuPipeThickness,
        each lambdaIns=InsuHeatCondu,
        each length=Pipelength)
        annotation (Placement(transformation(extent={{148,-88},{128,-96}})));
      // --- emission components of hydraulic circuit
      replaceable Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2[
                                                    nZones] emission(
        redeclare each replaceable package Medium = Medium)
        annotation (Placement(transformation(extent={{140,50},{170,70}})));
      // --- boudaries
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=293.15)
        "fixed temperature to simulate heat losses of hydraulic components"
        annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=0,
            origin={-149,45})));
      Buildings.Fluid.Sources.FixedBoundary absolutePressure(redeclare
          replaceable package
                  Medium =
            Medium, use_T=false,
        nPorts=1)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-106,-114})));
      // --- controllers
      replaceable FBM.Controls.ControlHeating.Ctrl_Heating ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom,
        TSupMin=TSupMin,
        minSup=minSup,
        corFac_val=corFac_val,
        THeaterSet(start=293.15)) constrainedby
        FBM.Controls.ControlHeating.Interfaces.Partial_Ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom)
        "Controller for the heater and the emission set point "
        annotation (Placement(transformation(extent={{-176,56},{-156,76}})));
      Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-0.5,
          each uHigh=0.5) "onoff controller for the pumps of the emission circuits"
        annotation (Placement(transformation(extent={{-140,-86},{-120,-66}})));
      Modelica.Blocks.Sources.RealExpression TSet_max(y=max(TSet))
        "maximum value of set point temperature" annotation (Placement(
            transformation(
            extent={{-21,-10},{21,10}},
            rotation=90,
            origin={-184,41})));
      Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
        annotation (Placement(transformation(extent={{-174,-82},{-160,-68}})));
      // --- Interface
      // --- Sensors
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemEm_in(redeclare
          replaceable package
                  Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
        "Inlet temperature of the emission system"
        annotation (Placement(transformation(extent={{66,48},{86,68}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemHea_out(redeclare
          replaceable package Medium =
                           Medium, m_flow_nominal=sum(m_flow_nominal))
        "Outlet temperature of the heater"
        annotation (Placement(transformation(extent={{-62,48},{-42,68}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemEm_out(redeclare
          replaceable package Medium =
                           Medium, m_flow_nominal=sum(m_flow_nominal))
        "Outlet temperature of the emission system" annotation (Placement(
            transformation(
            extent={{8,-8},{-8,8}},
            rotation=0,
            origin={90,-92})));
      Buildings.Fluid.FixedResistances.Junction spl(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal={sum(m_flow_nominal),sum(m_flow_nominal),-sum(
            m_flow_nominal)},
        dp_nominal={0,0,0})
        annotation (Placement(transformation(extent={{76,-88},{68,-96}})));
      Buildings.Fluid.MixingVolumes.MixingVolume vol(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        V=sum(m_flow_nominal)*30/1000,
        nPorts=1 + nZones)
        annotation (Placement(transformation(extent={{104,-92},{124,-72}})));
      Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
            m_flow_nominal)
        annotation (Placement(transformation(extent={{-104,-86},{-84,-66}})));
    protected
      Buildings.Controls.Continuous.LimPID conVal2(k=0.5, controllerType=Modelica.Blocks.Types.SimpleController.P)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={22,90})));
    public
      Controls.ControlHeating.OnOff_Heater onOff_Heater
        annotation (Placement(transformation(extent={{-134,76},{-114,96}})));
    initial equation
      if  CvDataSupply == Buildings.Fluid.Types.CvTypes.OpPoint then
        Kv_SISupply =           sum(m_flow_nominal)/sqrt(dpValve_nominalSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Kv then
        Kv_SISupply =           KvSupply*rhoStdSupply/3600/sqrt(1E5)
          "Unit conversion m3/(h*sqrt(bar)) to kg/(s*sqrt(Pa))";
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Cv then
        Kv_SISupply =           CvSupply*rhoStdSupply*0.0631/1000/sqrt(6895)
          "Unit conversion USG/(min*sqrt(psi)) to kg/(s*sqrt(Pa))";
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      else
        assert(CvDataSupply == Buildings.Fluid.Types.CvTypes.Av, "Invalid value for CvData.
Obtained CvData = "     + String(CvDataSupply) + ".");
        Kv_SISupply =           AvSupply*sqrt(rhoStdSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      end if;
    equation
        // connections that are function of the number of circuits
      for i in 1:nZones loop
        connect(pipeReturnEmission[i].heatPort, fixedTemperature.port) annotation (
            Line(
            points={{138,-94},{138,44},{-142,44},{-142,45}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(senTemEm_in.port_b, pumpRad[i].port_a) annotation (Line(
            points={{86,58},{86,60},{98,60}},
            color={0,127,255},
            smooth=Smooth.None));
      end for;
      // general connections for any configuration
      connect(fixedTemperature.port, heater.heatPort) annotation (Line(
          points={{-142,45},{-142,22},{-131.2,22}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(pipeSupply.heatPort, fixedTemperature.port) annotation (Line(
          points={{-6,60},{-6,80},{-18,80},{-18,44},{-142,44},{-142,45}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(pipeReturn.port_b, heater.port_a) annotation (Line(
          points={{-18,-92},{-124,-92},{-124,12}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeSupply.port_b, idealCtrlMixer.port_1) annotation (Line(
          points={{4,58},{34,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(absolutePressure.ports[1], heater.port_a) annotation (Line(
          points={{-106,-104},{-106,-92},{-124,-92},{-124,12}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(emission.port_b, pipeReturnEmission.port_a) annotation (Line(
          points={{170,60},{188,60},{188,-92},{148,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pumpRad.port_b, emission.port_a) annotation (Line(
          points={{122,60},{140,60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(TSet_max.y, ctrl_Heating.TRoo_in1) annotation (Line(
          points={{-184,64.1},{-184,70},{-176,70}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(add.y, heatingControl.u) annotation (Line(
          points={{-159.3,-75},{-146,-75},{-146,-76},{-142,-76}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(idealCtrlMixer.port_2, senTemEm_in.port_a) annotation (Line(
          points={{56,58},{66,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(heater.port_b, senTemHea_out.port_a) annotation (Line(
          points={{-124,32},{-124,58},{-62,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemHea_out.port_b, pipeSupply.port_a) annotation (Line(
          points={{-42,58},{-16,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(TSensor, add.u1) annotation (Line(
          points={{-124,-60},{-190,-60},{-190,-70.8},{-175.4,-70.8}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(senTemEm_out.port_b, spl.port_1) annotation (Line(
          points={{82,-92},{76,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(spl.port_2, pipeReturn.port_a) annotation (Line(
          points={{68,-92},{2,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(idealCtrlMixer.port_3, spl.port_3) annotation (Line(
          points={{45,46},{44,46},{44,34},{92,34},{92,-78},{72,-78},{72,-88}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeReturnEmission.port_b, vol.ports[1:nZones]) annotation (Line(
          points={{128,-92},{114,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(vol.ports[end], senTemEm_out.port_a) annotation (Line(
          points={{114,-92},{98,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(add.u2, TSet) annotation (Line(
          points={{-175.4,-79.2},{-194,-79.2},{-194,104},{-40,104}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(heatingControl.y, booleanToReal.u) annotation (Line(points={{-119,-76},
              {-106,-76}},              color={255,0,255}));
      connect(booleanToReal.y, pumpRad.m_flow_in) annotation (Line(points={{-83,-76},
              {6,-76},{109.76,-76},{109.76,45.6}},
                                                 color={0,0,127}));
      connect(fixedTemperature.port, pipeReturn.heatPort) annotation (Line(points={{-142,45},
              {-142,45},{-142,-94},{-8,-94}},              color={191,0,0}));
      connect(weaBus, ctrl_Heating.weaBus) annotation (Line(
          points={{-70,88},{-182,88},{-182,72.2},{-183.6,72.2}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(conVal2.y, idealCtrlMixer.y) annotation (Line(points={{33,90},{46,90},
              {46,72.4},{45,72.4}}, color={0,0,127}));
      connect(senTemEm_in.T, conVal2.u_m) annotation (Line(points={{76,69},{74,69},{
              74,70},{22,70},{22,78}}, color={0,0,127}));
      connect(ctrl_Heating.THeaCur, conVal2.u_s) annotation (Line(points={{-156,70},
              {-28,70},{-28,90},{10,90}}, color={0,0,127}));
      connect(ctrl_Heating.THeaterSet, onOff_Heater.reference) annotation (Line(
            points={{-157,67},{-157,93.5},{-135,93.5},{-135,93}}, color={0,0,127}));
      connect(heater.T, onOff_Heater.u) annotation (Line(points={{-132,33},{
              -140,33},{-140,81},{-135,81}},
                                    color={0,0,127}));
      connect(heater.y, onOff_Heater.u_m) annotation (Line(points={{-132,10},{
              -130,10},{-130,75},{-123,75}},
                                    color={0,0,127}));
      connect(onOff_Heater.y, heater.y) annotation (Line(points={{-113.2,86},{
              -108,86},{-108,10},{-132,10}},
                                    color={0,0,127}));
        annotation(Dialog(group = "Nominal condition"),
                  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -100},{200,100}}), graphics={Rectangle(
              extent={{-98,30},{88,-64}},
              lineColor={135,135,135},
              lineThickness=1), Text(
              extent={{36,30},{86,20}},
              lineColor={135,135,135},
              lineThickness=1,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="Thermal Energy Storage")}), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-120,-100},{200,100}}), graphics));
    end Partial_HydraulicHeating;

    partial model Partial_HydraulicHeating_SingleSource
      "Hydraulic multi-zone heating "
      replaceable package Medium = Buildings.Media.Water;
       //---Liste
      import sou = FBM.HeatingDHWsystems.Types.HeatSource;
      parameter sou source "kind of heat source"
                                                annotation(Dialog(group= "Settings"));
      extends FBM.Components.BaseClasses.SolarParameter;
      extends FBM.Components.BaseClasses.BoilerParameter;
      extends FBM.Interfaces.BaseClasses.HeatingSystem(
        isHea = true,
        isCoo = false,
        nConvPorts = nZones,
        nRadPorts = nZones,
        nTemSen = nZones,
        nEmbPorts=0,
        nZones=1);
      // --- Parameter: General parameters for the design (nominal) conditions and heat curve
      parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
        "Nominal power, can be seen as the max power of the emission system per zone";
      parameter Boolean minSup=true
        "true to limit the supply temperature on the lower side";
        parameter Modelica.SIunits.Temperature TSupMin=273.15 + 30
        "Minimum supply temperature if enabled";
      parameter Modelica.SIunits.Temperature TSupNom=273.15 + 45
        "Nominal supply temperature";
      parameter Modelica.SIunits.TemperatureDifference dTSupRetNom=10
        "Nominal DT in the heating system";
      parameter Modelica.SIunits.Temperature[nZones] TRoomNom={294.15 for i in 1:
          nZones} "Nominal room temperature";
      parameter Modelica.SIunits.TemperatureDifference corFac_val = 0
        "correction term for TSet of the heating curve";
      parameter Modelica.SIunits.Time timeFilter=43200
        "Time constant for the filter of ambient temperature for computation of heating curve";
      final parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal = QNom/(4180.6*dTSupRetNom)
        "Nominal mass flow rates";
        // -- Pipes and valve parameters
      parameter Modelica.SIunits.Thickness InsuPipeThickness=0.02
                                                                 "Thickness of the pipe insulation";
      parameter Modelica.SIunits.Length Pipelength=5
                                                    "pipe length of the supply OR return branch";
      parameter Modelica.SIunits.Pressure dp=3000 "Pressure drop over a single pipe"
        annotation(Dialog(group = "Pipes",
                         enable = includePipes));
     parameter Modelica.SIunits.ThermalConductivity InsuHeatCondu=0.04;
     parameter Real fraKSupply(min=0, max=1) = 0.7
        "Fraction Kv(port_3->port_2)/Kv(port_1->port_2)";
     parameter Real[2] lSupply(each min=0, each max=1) = {0.01, 0.01}
        "Valve leakage, l=Kv(y=0)/Kv(y=1)";
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
                         enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Av)));
      parameter Real deltaMSupply = 0.02
        "Fraction of nominal flow rate where linearization starts, if y=1"
        annotation(Dialog(group="Pressure-flow linearization"));
      parameter Modelica.SIunits.Pressure dpValve_nominalSupply(displayUnit="Pa",
                                                          min=0,
                                                          fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.OpPoint then true else false)
        "Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint"
        annotation(Dialog(group="Nominal condition",
                   enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      parameter Modelica.SIunits.Density rhoStdSupply=Medium.density_pTX(101325, 273.15+4, Medium.X_default)
        "Inlet density for which valve coefficients are defined"
      annotation(Dialog(group="Nominal condition", tab="Advanced"));
    protected
      parameter Real Kv_SISupply(
        min=0,
        fixed= false)
        "Flow coefficient for fully open valve in SI units, Kv=m_flow/sqrt(dp) [kg/s/(Pa)^(1/2)]"
      annotation(Dialog(group = "Flow Coefficient Supply Valve",
                        enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      // --- production components of hydraulic circuit
      replaceable FBM.HeatingDHWsystems.Interfaces.SourceTemplate heater(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        Q_Solar_nominal=sum(QNom),
        QBoiler=sum(QNom),
        source=source,
        mPrim_flow_nominal=mPrim_flow_nominal,
        dp_nominal=dp_nominal)
        annotation (Placement(transformation(extent={{-134,32},{-114,12}})));
      // --- distribution components of hydraulic circuit
      Buildings.Fluid.Movers.FlowControlled_m_flow[nZones] pumpRad(
        m_flow_nominal=m_flow_nominal,
        redeclare each replaceable package Medium = Medium,
        each tau=30,
        each filteredSpeed=false,
        each nominalValuesDefineDefaultPressureCurve=true)
                  annotation (Placement(transformation(extent={{98,72},{122,48}})));
      Buildings.Fluid.Actuators.Valves.ThreeWayLinear idealCtrlMixer(
        m_flow_nominal=sum(m_flow_nominal),
        redeclare replaceable package Medium = Medium,
        CvData=Buildings.Fluid.Types.CvTypes.Kv,
        Kv=KvSupply,
        fraK=fraKSupply,
        l=lSupply,
        deltaM=deltaMSupply)                           annotation (Placement(transformation(extent={{34,46},{56,70}})));
      Buildings.Fluid.FixedResistances.Pipe pipeReturn(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        nSeg=3,
        thicknessIns=InsuPipeThickness,
        lambdaIns=InsuHeatCondu,
        length=Pipelength)
               annotation (Placement(transformation(extent={{2,-88},{-18,-96}})));
      Buildings.Fluid.FixedResistances.Pipe pipeSupply(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        nSeg=3,
        thicknessIns=InsuPipeThickness,
        lambdaIns=InsuHeatCondu,
        length=Pipelength)
               annotation (Placement(transformation(extent={{-16,62},{4,54}})));
      Buildings.Fluid.FixedResistances.Pipe[nZones] pipeReturnEmission(
        redeclare each replaceable package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        each nSeg=3,
        each thicknessIns=InsuPipeThickness,
        each lambdaIns=InsuHeatCondu,
        each length=Pipelength)
        annotation (Placement(transformation(extent={{148,-96},{128,-88}})));
      // --- emission components of hydraulic circuit
      replaceable Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2[
                                                    nZones] emission(
        redeclare each replaceable package Medium = Medium)
        annotation (Placement(transformation(extent={{140,50},{170,70}})));
      // --- boudaries
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=293.15)
        "fixed temperature to simulate heat losses of hydraulic components"
        annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=0,
            origin={-149,45})));
      Buildings.Fluid.Sources.FixedBoundary absolutePressure(redeclare
          replaceable package Medium =
            Medium, use_T=false,
        nPorts=1)
        annotation (Placement(transformation(extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-106,-114})));
      // --- controllers
      replaceable FBM.Controls.ControlHeating.Ctrl_Heating ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom,
        TSupMin=TSupMin,
        minSup=minSup,
        corFac_val=corFac_val,
        THeaterSet(start=293.15)) constrainedby
        FBM.Controls.ControlHeating.Interfaces.Partial_Ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom)
        "Controller for the heater and the emission set point "
        annotation (Placement(transformation(extent={{-176,56},{-156,76}})));
      Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-0.5,
          each uHigh=0.5) "onoff controller for the pumps of the emission circuits"
        annotation (Placement(transformation(extent={{-140,-86},{-120,-66}})));
      Modelica.Blocks.Sources.RealExpression TSet_max(y=max(TSet))
        "maximum value of set point temperature" annotation (Placement(
            transformation(
            extent={{-21,-10},{21,10}},
            rotation=90,
            origin={-184,41})));
      Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
        annotation (Placement(transformation(extent={{-174,-82},{-160,-68}})));
      // --- Interface
      // --- Sensors
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemEm_in(redeclare
          replaceable package Medium =
                           Medium, m_flow_nominal=sum(m_flow_nominal))
        "Inlet temperature of the emission system"
        annotation (Placement(transformation(extent={{66,48},{86,68}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemHea_out(redeclare
          replaceable package Medium =
                           Medium, m_flow_nominal=sum(m_flow_nominal))
        "Outlet temperature of the heater"
        annotation (Placement(transformation(extent={{-62,48},{-42,68}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemEm_out(redeclare
          replaceable package Medium =
                           Medium, m_flow_nominal=sum(m_flow_nominal))
        "Outlet temperature of the emission system" annotation (Placement(
            transformation(
            extent={{8,-8},{-8,8}},
            rotation=0,
            origin={90,-92})));
      Buildings.Fluid.FixedResistances.Junction spl(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal={sum(m_flow_nominal),sum(m_flow_nominal),-sum(m_flow_nominal)},
        dp_nominal={0,0,0})
        annotation (Placement(transformation(extent={{76,-88},{68,-96}})));
      Buildings.Fluid.MixingVolumes.MixingVolume vol(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        V=sum(m_flow_nominal)*30/1000,
        nPorts=1 + nZones)
        annotation (Placement(transformation(extent={{104,-92},{124,-72}})));
      Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
            m_flow_nominal)
        annotation (Placement(transformation(extent={{-104,-86},{-84,-66}})));
      Buildings.Controls.Continuous.LimPID conVal(
        controllerType=Modelica.Blocks.Types.SimpleController.PI,
        k=0.1,
        Ti=120)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={14,86})));
    public
      Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=5)
        annotation (Placement(transformation(extent={{-144,78},{-124,98}})));
      Modelica.Blocks.Math.BooleanToReal booleanToReal1
        annotation (Placement(transformation(extent={{-114,76},{-94,96}})));
    protected
      Buildings.Controls.Continuous.LimPID conHeatSource(
        Ti=120,
        controllerType=Modelica.Blocks.Types.SimpleController.P,
        k=1) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={-74,86})));
    initial equation
      if  CvDataSupply == Buildings.Fluid.Types.CvTypes.OpPoint then
        Kv_SISupply =           sum(m_flow_nominal)/sqrt(dpValve_nominalSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Kv then
        Kv_SISupply =           KvSupply*rhoStdSupply/3600/sqrt(1E5)
          "Unit conversion m3/(h*sqrt(bar)) to kg/(s*sqrt(Pa))";
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Cv then
        Kv_SISupply =           CvSupply*rhoStdSupply*0.0631/1000/sqrt(6895)
          "Unit conversion USG/(min*sqrt(psi)) to kg/(s*sqrt(Pa))";
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      else
        assert(CvDataSupply == Buildings.Fluid.Types.CvTypes.Av, "Invalid value for CvData.
Obtained CvData = "     + String(CvDataSupply) + ".");
        Kv_SISupply =           AvSupply*sqrt(rhoStdSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      end if;
    equation
        // connections that are function of the number of circuits
      for i in 1:nZones loop
        connect(pipeReturnEmission[i].heatPort, fixedTemperature.port) annotation (
            Line(
            points={{138,-90},{138,44},{-142,44},{-142,45}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(senTemEm_in.port_b, pumpRad[i].port_a) annotation (Line(
            points={{86,58},{86,60},{98,60}},
            color={0,127,255},
            smooth=Smooth.None));
              connect(fixedTemperature.port, pumpRad[i].heatPort) annotation (Line(points={{
              -142,45},{98,45},{98,68.16},{110,68.16}}, color={191,0,0}));
      end for;
      // general connections for any configuration
      connect(pipeSupply.heatPort, fixedTemperature.port) annotation (Line(
          points={{-6,56},{-6,44},{-142,44},{-142,45}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(pipeReturn.port_b, heater.port_a) annotation (Line(
          points={{-18,-92},{-134,-92},{-134,18.25}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeSupply.port_b, idealCtrlMixer.port_1) annotation (Line(
          points={{4,58},{34,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(absolutePressure.ports[1], heater.port_a) annotation (Line(
          points={{-106,-104},{-106,-92},{-134,-92},{-134,18.25}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(emission.port_b, pipeReturnEmission.port_a) annotation (Line(
          points={{170,60},{188,60},{188,-92},{148,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pumpRad.port_b, emission.port_a) annotation (Line(
          points={{122,60},{140,60}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(TSet_max.y, ctrl_Heating.TRoo_in1) annotation (Line(
          points={{-184,64.1},{-184,70},{-176,70}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(add.y, heatingControl.u) annotation (Line(
          points={{-159.3,-75},{-146,-75},{-146,-76},{-142,-76}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(idealCtrlMixer.port_2, senTemEm_in.port_a) annotation (Line(
          points={{56,58},{66,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(heater.port_b, senTemHea_out.port_a) annotation (Line(
          points={{-114,18.25},{-112,18.25},{-112,58},{-62,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemHea_out.port_b, pipeSupply.port_a) annotation (Line(
          points={{-42,58},{-16,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(TSensor, add.u1) annotation (Line(
          points={{-124,-60},{-190,-60},{-190,-70.8},{-175.4,-70.8}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(senTemEm_out.port_b, spl.port_1) annotation (Line(
          points={{82,-92},{76,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(spl.port_2, pipeReturn.port_a) annotation (Line(
          points={{68,-92},{2,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(idealCtrlMixer.port_3, spl.port_3) annotation (Line(
          points={{45,46},{44,46},{44,34},{92,34},{92,-78},{72,-78},{72,-88}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeReturnEmission.port_b, vol.ports[1:nZones]) annotation (Line(
          points={{128,-92},{114,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(vol.ports[end], senTemEm_out.port_a) annotation (Line(
          points={{114,-92},{98,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(add.u2, TSet) annotation (Line(
          points={{-175.4,-79.2},{-192,-79.2},{-192,104},{-40,104}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(heatingControl.y, booleanToReal.u) annotation (Line(points={{-119,-76},
              {-106,-76}},              color={255,0,255}));
      connect(booleanToReal.y, pumpRad.m_flow_in) annotation (Line(points={{-83,-76},
              {6,-76},{109.76,-76},{109.76,45.6}},
                                                 color={0,0,127}));
      connect(fixedTemperature.port, pipeReturn.heatPort) annotation (Line(points={{-142,45},
              {-142,45},{-142,-94},{-8,-94}},              color={191,0,0}));
      connect(ctrl_Heating.THeaCur, conVal.u_s) annotation (Line(points={{-156,70},{
              -26,70},{-26,86},{2,86}}, color={0,0,127}));
      connect(conVal.y, idealCtrlMixer.y) annotation (Line(points={{25,86},{46,86},{
              46,72.4},{45,72.4}}, color={0,0,127}));
      connect(senTemEm_in.T, conVal.u_m)
        annotation (Line(points={{76,69},{14,69},{14,74}}, color={0,0,127}));
      connect(weaBus, ctrl_Heating.weaBus) annotation (Line(
          points={{-70,88},{-182,88},{-182,72.2},{-183.6,72.2}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(onOffController.y, booleanToReal1.u)
        annotation (Line(points={{-123,88},{-122,88},{-122,86},{-116,86}},
                                                       color={255,0,255}));
      connect(onOffController.reference, ctrl_Heating.THeaterSet)
        annotation (Line(points={{-146,94},{-157,94},{-157,67}}, color={0,0,127}));
      connect(booleanToReal1.y, conHeatSource.u_s)
        annotation (Line(points={{-93,86},{-86,86}}, color={0,0,127}));
      connect(conHeatSource.y, heater.U) annotation (Line(points={{-63,86},{-58,
              86},{-58,52},{-162,52},{-162,13.875},{-134.6,13.875}},
                                                        color={0,0,127}));
      connect(heater.T, conHeatSource.u_m) annotation (Line(points={{-113,
              13.875},{-110,13.875},{-110,16},{-102,16},{-102,46},{-74,46},{-74,
              74}},                                                 color={0,0,127}));
      connect(heater.T, onOffController.u) annotation (Line(points={{-113,
              13.875},{-113,6.5},{-146,6.5},{-146,82}},
                                          color={0,0,127}));
      connect(fixedTemperature.port, heater.Ambiant) annotation (Line(points={{-142,45},
              {-142,45},{-142,12.125},{-124,12.125}},      color={191,0,0}));
     if   (source == sou.SolarPannelField) then
        connect(weaBus, heater.weaBus) annotation (Line(
          points={{-70,88},{-178,88},{-178,15.125},{-141.2,15.125}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
     end if;
            annotation(Dialog(group = "Nominal condition"),
                  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}), graphics={Rectangle(
              extent={{-98,30},{88,-64}},
              lineColor={135,135,135},
              lineThickness=1), Text(
              extent={{36,30},{86,20}},
              lineColor={135,135,135},
              lineThickness=1,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="Thermal Energy Storage")}), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-200,-100},{200,100}}), graphics));
    end Partial_HydraulicHeating_SingleSource;

    partial model Partial_HydraulicHeating_MultiSource "Hydraulic multi-zone heating "
      replaceable package Medium = Buildings.Media.Water;
      extends FBM.Components.BaseClasses.SolarParameter;
      extends FBM.Components.BaseClasses.BoilerParameter;
      extends FBM.HeatingDHWsystems.BaseClasses.TES_parameters;
      extends FBM.HeatingDHWsystems.BaseClasses.AHU_Heating_parameters;
      extends FBM.Interfaces.BaseClasses.HeatingSystem(
        isHea = true,
        isCoo = false,
        nConvPorts = nZones,
        nRadPorts = nZones,
        nTemSen = nZones,
        nEmbPorts=0,
        nZones=1);
      //---MultiSource parameters
       parameter Integer nSource = 1 annotation(Dialog(group= "Settings"));
      import sou = FBM.HeatingDHWsystems.Types.HeatSource;
      parameter sou source[nSource] "type of heat source"
                                                         annotation(Dialog(group= "Settings"));
      // --- Parameter: General parameters for the design (nominal) conditions and heat curve
      parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
        "Nominal power, can be seen as the max power of the emission system per zone";
      parameter Boolean minSup=true
        "true to limit the supply temperature on the lower side";
        parameter Modelica.SIunits.Temperature TSupMin=273.15 + 30
        "Minimum supply temperature if enabled";
      parameter Modelica.SIunits.Temperature TSupNom=273.15 + 45
        "Nominal supply temperature";
      parameter Modelica.SIunits.TemperatureDifference dTSupRetNom=10
        "Nominal DT in the heating system";
      parameter Modelica.SIunits.Temperature[nZones] TRoomNom={294.15 for i in 1:
          nZones} "Nominal room temperature";
      parameter Modelica.SIunits.TemperatureDifference corFac_val = 0
        "correction term for TSet of the heating curve";
      parameter Modelica.SIunits.Time timeFilter=43200
        "Time constant for the filter of ambient temperature for computation of heating curve";
      final parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal = QNom/(4180.6*dTSupRetNom)
        "Nominal mass flow rates";
        // -- Pipes and valve parameters
      parameter Modelica.SIunits.Thickness InsuPipeThickness=0.02
                                                                 "Thickness of the pipe insulation";
      parameter Modelica.SIunits.Length Pipelength=5
                                                    "pipe length of the supply OR return branch";
      parameter Modelica.SIunits.Pressure dp=3000 "Pressure drop over a single pipe"
        annotation(Dialog(group = "Pipes",
                         enable = includePipes));
     parameter Modelica.SIunits.ThermalConductivity InsuHeatCondu=0.04;
     parameter Real fraKSupply(min=0, max=1) = 0.7
        "Fraction Kv(port_3->port_2)/Kv(port_1->port_2)";
     parameter Real[2] lSupply(each min=0, each max=1) = {0.01, 0.01}
        "Valve leakage, l=Kv(y=0)/Kv(y=1)";
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
                         enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Av)));
      parameter Real deltaMSupply = 0.02
        "Fraction of nominal flow rate where linearization starts, if y=1"
        annotation(Dialog(group="Pressure-flow linearization"));
      parameter Modelica.SIunits.Pressure dpValve_nominalSupply(displayUnit="Pa",
                                                          min=0,
                                                          fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.OpPoint then true else false)
        "Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint"
        annotation(Dialog(group="Nominal condition",
                   enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      parameter Modelica.SIunits.Density rhoStdSupply=Medium.density_pTX(101325, 273.15+4, Medium.X_default)
        "Inlet density for which valve coefficients are defined"
      annotation(Dialog(group="Nominal condition", tab="Advanced"));
    protected
      parameter Real Kv_SISupply(
        min=0,
        fixed= false)
        "Flow coefficient for fully open valve in SI units, Kv=m_flow/sqrt(dp) [kg/s/(Pa)^(1/2)]"
      annotation(Dialog(group = "Flow Coefficient Supply Valve",
                        enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      // --- production components of hydraulic circuit
      replaceable FBM.HeatingDHWsystems.Interfaces.MultiSourceTemplate[nSource] heater(
        redeclare replaceable package Medium = Medium,
        each m_flow_nominal=sum(m_flow_nominal),
        each Q_Solar_nominal=sum(QNom),
        source = {source[i] for i in 1:nSource},
        mPrim_flow_nominal=mPrim_flow_nominal,
        includePipesSol=includePipesSol,
        dpHexPrim=dpHexPrim,
        dpHexSecon=dpHexSecon,
        lat=lat,
        T_nominal=T_nominal,
        effCur=effCur,
        UA=UA,
        dTSupRetNom=dTSupRetNom,
        dp_nominal=dp_nominal,
        TBoiRet_min=TBoiRet_min,
        azi=azi,
        til=til,
        rho=rho,
        nPar=nPar,
        nSer=nSer,
        InsuPipeThicknessSol=InsuPipeThicknessSol,
        PipelengthSol=PipelengthSol,
        InsuHeatConduSol=InsuHeatConduSol,
        T_a1_nominal=T_a1_nominal,
        T_a2_nominal=T_a2_nominal,
        HOn=HOn,
        HOff=HOff,
        nbrNodes=nbrNodes,
        VTan=VTan,
        hTan=hTan,
        dInsTan=dInsTan,
        dp_nominalSol=dp_nominalSol,
        each QBoiler=QBoiler)
        annotation (Placement(transformation(extent={{-136,10},{-116,-10}})));
      // --- distribution components of hydraulic circuit
      Buildings.Fluid.FixedResistances.Junction spl(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal={sum(m_flow_nominal),sum(m_flow_nominal),-sum(m_flow_nominal)},
        dp_nominal={0,0,0})
        annotation (Placement(transformation(extent={{76,-88},{68,-96}})));
      Buildings.Fluid.Movers.FlowControlled_m_flow[nZones] pumpRad(
        m_flow_nominal=m_flow_nominal,
        redeclare each replaceable package Medium = Medium,
        each tau=30,
        each filteredSpeed=false,
        each nominalValuesDefineDefaultPressureCurve=true,
        p_start=300000)
                  annotation (Placement(transformation(extent={{72,70},{96,46}})));
      Buildings.Fluid.Actuators.Valves.ThreeWayLinear idealCtrlMixer(
        m_flow_nominal=sum(m_flow_nominal),
        redeclare replaceable package Medium = Medium,
        CvData=Buildings.Fluid.Types.CvTypes.Kv,
        Kv=KvSupply,
        fraK=fraKSupply,
        l=lSupply,
        deltaM=deltaMSupply,
        p_start=300000)                                annotation (Placement(transformation(extent={{14,46},
                {36,70}})));
      Buildings.Fluid.FixedResistances.Pipe pipeReturn(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        thicknessIns=InsuPipeThickness,
        lambdaIns=InsuHeatCondu,
        length=Pipelength,
        nSeg=5,
        p_start=300000)
               annotation (Placement(transformation(extent={{22,-88},{2,-96}})));
      Buildings.Fluid.FixedResistances.Pipe pipeSupply(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        thicknessIns=InsuPipeThickness,
        lambdaIns=InsuHeatCondu,
        length=Pipelength,
        nSeg=5,
        p_start=300000)
               annotation (Placement(transformation(extent={{-16,62},{4,54}})));
      Buildings.Fluid.FixedResistances.Pipe[nZones] pipeReturnEmission(
        redeclare each replaceable package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        each thicknessIns=InsuPipeThickness,
        each lambdaIns=InsuHeatCondu,
        each length=Pipelength,
        each nSeg=5)
        annotation (Placement(transformation(extent={{148,-96},{128,-88}})));
      // --- emission components of hydraulic circuit
      replaceable Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2[
                                                    nZones] emission(
        redeclare each replaceable package Medium = Medium, p_start=300000)
        annotation (Placement(transformation(extent={{108,48},{138,68}})));
      // --- boudaries
        Buildings.Fluid.MixingVolumes.MixingVolume vol(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        V=sum(m_flow_nominal)*30/1000,
        nPorts=1 + nZones,
        p_start=300000)
        annotation (Placement(transformation(extent={{104,-92},{124,-72}})));
      Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=293.15)
        "fixed temperature to simulate heat losses of hydraulic components"
        annotation (Placement(transformation(
            extent={{-7,-7},{7,7}},
            rotation=0,
            origin={-165,45})));
      // --- controllers
      replaceable FBM.Controls.ControlHeating.Ctrl_Heating_TES ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom,
        TSupMin=TSupMin,
        minSup=minSup,
        corFac_val=corFac_val,
        DHW=false,
        dTHeaterSet=3)            constrainedby
        FBM.Controls.ControlHeating.Interfaces.Partial_Ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom)
        "Controller for the heater and the emission set point "
        annotation (Placement(transformation(extent={{-124,72},{-104,92}})));
      Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-0.5,
          each uHigh=0.5) "onoff controller for the pumps of the emission circuits"
        annotation (Placement(transformation(extent={{-140,-86},{-120,-66}})));
      Modelica.Blocks.Sources.RealExpression TSet_max(y=max(TSet))
        "maximum value of set point temperature" annotation (Placement(
            transformation(
            extent={{-21,-10},{21,10}},
            rotation=0,
            origin={-164,85})));
      Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
        annotation (Placement(transformation(extent={{-174,-82},{-160,-68}})));
      // --- Interface
      // --- Sensors
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemEm_in(redeclare
          replaceable package Medium =
                           Medium, m_flow_nominal=sum(m_flow_nominal))
        "Inlet temperature of the emission system"
        annotation (Placement(transformation(extent={{44,48},{64,68}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemHea_out(redeclare
          replaceable package Medium =
                           Medium, m_flow_nominal=sum(m_flow_nominal))
        "Outlet temperature of the heater"
        annotation (Placement(transformation(extent={{-48,48},{-28,68}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemEm_out(redeclare
          replaceable package Medium =
                           Medium, m_flow_nominal=sum(m_flow_nominal))
        "Outlet temperature of the emission system" annotation (Placement(
            transformation(
            extent={{8,-8},{-8,8}},
            rotation=0,
            origin={90,-92})));
      Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
            m_flow_nominal)
        annotation (Placement(transformation(extent={{-104,-86},{-84,-66}})));
      Buildings.Controls.Continuous.LimPID conVal(
        Ti=600,
        controllerType=Modelica.Blocks.Types.SimpleController.PI,
        k=1)
        annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={16,88})));
    public
      Buildings.Fluid.Storage.StratifiedEnhanced TESTank(
        redeclare package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        nSeg=nbrNodesTes,
        T_start=Tes_start,
        VTan=VTanTes,
        hTan=hTanTes,
        dIns=dInsTes,
        kIns=kInsTes)                                    annotation (Placement(transformation(extent={{-54,-32},
                {-22,0}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nbrNodesTes] TTank
        annotation (Placement(transformation(extent={{-76,18},{-86,28}})));
    public
      Modelica.Blocks.Logical.OnOffController[nSource] onOffController(bandwidth=2)
        annotation (Placement(transformation(extent={{-94,74},{-74,94}})));
      Modelica.Blocks.Math.BooleanToReal[nSource] booleanToReal1
        annotation (Placement(transformation(extent={{-64,74},{-44,94}})));
      Buildings.Fluid.Storage.ExpansionVessel exp(redeclare package Medium =
            Medium, V_start=2)
        annotation (Placement(transformation(extent={{-18,-16},{2,4}})));
          // --- AHU components
      Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear valAHU(
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        l={0.01,0.01},
        dpValve_nominal=6000,
        m_flow_nominal=mAHU_flow_nominal,
        redeclare package Medium = Medium) if
                                             isAHU == true
                              "Three-way valve for boiler"
                            annotation (Placement(transformation(
            extent={{10,10},{-10,-10}},
            rotation=180,
            origin={132,20})));
      Buildings.Fluid.FixedResistances.Junction spl_AHU(
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        dp_nominal={0,0,-200},
        m_flow_nominal={mAHU_flow_nominal,-mAHU_flow_nominal,-mAHU_flow_nominal},
        redeclare package Medium = Medium) if                                        isAHU == true
        "Splitter" annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=0,
            origin={132,-36})));
      Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
            Medium) if
          isAHU == true annotation (Placement(transformation(extent={{182,90},{202,110}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
            Medium) if
          isAHU == true annotation (Placement(transformation(extent={{152,90},{172,110}})));
      Modelica.Blocks.Sources.Constant TSetAHUOut(k=TAHUSet) if  isAHU == true
        "Temperature setpoint for AHU outlet"
       annotation (Placement(transformation(extent={{56,78},{76,98}})));
      Buildings.Controls.Continuous.LimPID conPIDAHU(
        Td=1,
        controllerType=Modelica.Blocks.Types.SimpleController.PI,
        k=1,
        Ti=600) if                                                   isAHU == true
        "Controller for valve in AHU loop"
       annotation (Placement(transformation(extent={{92,78},{112,98}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTemAHU_out(redeclare
          replaceable package Medium = Medium, m_flow_nominal=mAHU_flow_nominal) if   isAHU == true
        "Outlet temperature of the AHU loop"
        annotation (Placement(transformation(extent={{180,10},{200,30}})));
    initial equation
      if  CvDataSupply == Buildings.Fluid.Types.CvTypes.OpPoint then
        Kv_SISupply =           sum(m_flow_nominal)/sqrt(dpValve_nominalSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Kv then
        Kv_SISupply =           KvSupply*rhoStdSupply/3600/sqrt(1E5)
          "Unit conversion m3/(h*sqrt(bar)) to kg/(s*sqrt(Pa))";
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Cv then
        Kv_SISupply =           CvSupply*rhoStdSupply*0.0631/1000/sqrt(6895)
          "Unit conversion USG/(min*sqrt(psi)) to kg/(s*sqrt(Pa))";
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      else
        assert(CvDataSupply == Buildings.Fluid.Types.CvTypes.Av, "Invalid value for CvData.
Obtained CvData = "     + String(CvDataSupply) + ".");
        Kv_SISupply =           AvSupply*sqrt(rhoStdSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      end if;
    equation
        // connections that are function of the number of circuits
      for i in 1:nZones loop
        connect(pipeReturnEmission[i].heatPort, fixedTemperature.port) annotation (
            Line(
            points={{138,-90},{138,44},{-158,44},{-158,45}},
            color={191,0,0},
            smooth=Smooth.None));
        connect(senTemEm_in.port_b, pumpRad[i].port_a) annotation (Line(
            points={{64,58},{72,58}},
            color={0,127,255},
            smooth=Smooth.None));
              connect(fixedTemperature.port, pumpRad[i].heatPort) annotation (Line(points={{-158,45},
                {76,45},{76,66.16},{84,66.16}},         color={191,0,0}));
      end for;
      // general connections for any configuration
      connect(pipeSupply.heatPort, fixedTemperature.port) annotation (Line(
          points={{-6,56},{-6,44},{-158,44},{-158,45}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(pipeSupply.port_b, idealCtrlMixer.port_1) annotation (Line(
          points={{4,58},{14,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(emission.port_b, pipeReturnEmission.port_a) annotation (Line(
          points={{138,58},{152,58},{152,-92},{148,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(TSet_max.y, ctrl_Heating.TRoo_in1) annotation (Line(
          points={{-140.9,85},{-140.9,86},{-124,86}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(add.y, heatingControl.u) annotation (Line(
          points={{-159.3,-75},{-146,-75},{-146,-76},{-142,-76}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(idealCtrlMixer.port_2, senTemEm_in.port_a) annotation (Line(
          points={{36,58},{44,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(senTemHea_out.port_b, pipeSupply.port_a) annotation (Line(
          points={{-28,58},{-16,58}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(TSensor, add.u1) annotation (Line(
          points={{-124,-60},{-190,-60},{-190,-70.8},{-175.4,-70.8}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(senTemEm_out.port_b, spl.port_1) annotation (Line(
          points={{82,-92},{76,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(spl.port_2, pipeReturn.port_a) annotation (Line(
          points={{68,-92},{22,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(idealCtrlMixer.port_3, spl.port_3) annotation (Line(
          points={{25,46},{26,46},{26,34},{94,34},{94,-88},{72,-88}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(pipeReturnEmission.port_b, vol.ports[1:nZones]) annotation (Line(
          points={{128,-92},{114,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(vol.ports[end], senTemEm_out.port_a) annotation (Line(
          points={{114,-92},{98,-92}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(add.u2, TSet) annotation (Line(
          points={{-175.4,-79.2},{-194,-79.2},{-194,104},{-40,104}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(heatingControl.y, booleanToReal.u) annotation (Line(points={{-119,-76},
              {-106,-76}},              color={255,0,255}));
      connect(booleanToReal.y, pumpRad.m_flow_in) annotation (Line(points={{-83,-76},
              {96,-76},{98,-76},{98,36},{83.76,36},{83.76,43.6}},
                                                 color={0,0,127}));
      connect(fixedTemperature.port, pipeReturn.heatPort) annotation (Line(points={{-158,45},
              {-144,45},{-144,-94},{12,-94}},              color={191,0,0}));
      connect(ctrl_Heating.THeaCur, conVal.u_s) annotation (Line(points={{-104,86},
              {-104,90},{-100,90},{-100,100},{-2,100},{-2,88},{4,88}},
                                        color={0,0,127}));
      connect(conVal.y, idealCtrlMixer.y) annotation (Line(points={{27,88},{26,88},{
              26,72.4},{25,72.4}}, color={0,0,127}));
      connect(senTemEm_in.T, conVal.u_m)
        annotation (Line(points={{54,69},{54,69},{54,72},{16,72},{16,76}},
                                                           color={0,0,127}));
      connect(weaBus, ctrl_Heating.weaBus) annotation (Line(
          points={{-70,88},{-178,88},{-178,88.2},{-131.6,88.2}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(fixedTemperature.port, TESTank.heaPorTop) annotation (Line(points={{-158,45},
              {-22,45},{-22,-4.16},{-34.8,-4.16}},                                                                              color={191,0,0}));
      connect(fixedTemperature.port, TESTank.heaPorSid) annotation (Line(points={{-158,45},
              {-22,45},{-22,-16},{-29.04,-16}},                                                                          color={191,0,0}));
      connect(fixedTemperature.port, TESTank.heaPorBot) annotation (Line(points={{-158,45},
              {-22,45},{-22,-27.84},{-34.8,-27.84}},                                                                            color={191,0,0}));
      for i in 1:nSource loop
        if (source[i] == sou.SolarPannelField) then
      connect(weaBus, heater[i].weaBus) annotation (Line(
          points={{-70,88},{-178,88},{-178,-6.875},{-143.2,-6.875}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(heater[i].TBoTank, TTank[posTesBot].T) annotation (Line(points={{-116,
                  8.375},{-86,8.375},{-86,23}},
                                         color={0,0,127}));
        end if;
      connect(fixedTemperature.port, heater[i].Ambiant) annotation (Line(points={{-158,45},
                {-144,45},{-144,-9.875},{-126,-9.875}},    color={191,0,0}));
      connect(TTank[posTesBot].T, onOffController[i].u) annotation (Line(points={{-86,23},
              {-100,23},{-100,78},{-96,78}},
                                  color={0,0,127}));
      connect(ctrl_Heating.THeaCur, onOffController[i].reference) annotation (Line(
            points={{-104,86},{-102,86},{-102,90},{-96,90}}, color={0,0,127}));
      end for;
      if isAHU == true then
      connect(valAHU.port_2, senTemAHU_out.port_a)
        annotation (Line(points={{142,20},{161,20},{180,20}}, color={0,127,255}));
      connect(valAHU.port_1, TESTank.port_a)
        annotation (Line(points={{122,20},{-54,20},{-54,-16}}, color={0,127,255}));
      connect(spl_AHU.port_3, valAHU.port_3) annotation (Line(
          points={{132,-26},{132,10}},
          color={0,127,255},
          smooth=Smooth.None));
      connect(TSetAHUOut.y,conPIDAHU. u_s)
        annotation (Line(points={{77,88},{77,88},{90,88}},
                                                         color={0,0,127}));
      connect(spl_AHU.port_2, port_b) annotation (Line(points={{142,-36},{162,-36},{
                162,100}},
                         color={0,127,255}));
      connect(conPIDAHU.y,valAHU. y)
        annotation (Line(points={{113,88},{132,88},{132,32}}, color={0,0,127}));
      connect(senTemAHU_out.port_b, port_a)
        annotation (Line(points={{200,20},{192,20},{192,100}},
                                                     color={0,127,255}));
      connect(senTemAHU_out.T, conPIDAHU.u_m) annotation (Line(points={{190,31},{160,
                31},{160,76},{102,76}},
                                      color={0,0,127}));
      connect(spl_AHU.port_1, TESTank.port_b) annotation (Line(points={{122,-36},{94,
              -36},{94,-86},{-22,-86},{-22,-16}}, color={0,127,255}));
      end if;
      connect(TESTank.heaPorVol, TTank.port)
        annotation (Line(points={{-38,-16},{-38,23},{-76,23}},
                                                           color={191,0,0}));
      connect(ctrl_Heating.TTankTop, TTank[posTesTop].T)
        annotation (Line(points={{-124,82},{-124,23},{-86,23}},
                                                              color={0,0,127}));
      connect(TTank[posTesBot].T, ctrl_Heating.TTankBot) annotation (Line(points={{-86,23},
              {-124,23},{-124,78}},       color={0,0,127}));
      connect(pumpRad.port_b, emission.port_a)
        annotation (Line(points={{96,58},{108,58}}, color={0,127,255}));
      connect(onOffController.y, booleanToReal1.u) annotation (Line(points={{-73,84},
              {-66,84}},            color={255,0,255}));
      connect(booleanToReal1.y, heater.U) annotation (Line(points={{-43,84},{
              -26,84},{-26,-8},{-82,-8},{-82,-8.125},{-136.6,-8.125}},
                                                color={0,0,127}));
      connect(TESTank.port_a, senTemHea_out.port_a) annotation (Line(points={{-54,-16},
              {-54,-16},{-54,58},{-48,58}},          color={0,127,255}));
      connect(pipeReturn.port_b, TESTank.port_b) annotation (Line(points={{2,-92},{-22,
              -92},{-22,-16}},                     color={0,127,255}));
      connect(exp.port_a, TESTank.port_b) annotation (Line(points={{-8,-16},{-8,-16},
              {-22,-16}},                color={0,127,255}));
             annotation(Dialog(group = "Nominal condition"),
                  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
                -100},{200,100}}), graphics={Rectangle(
              extent={{-100,30},{86,-64}},
              lineColor={135,135,135},
              lineThickness=1), Text(
              extent={{-74,28},{-24,18}},
              lineColor={135,135,135},
              lineThickness=1,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="Thermal Energy Storage"),
            Line(
              points={{0,30},{0,-64}},
              color={135,135,135},
              thickness=1),     Text(
              extent={{16,28},{66,18}},
              lineColor={135,135,135},
              lineThickness=1,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="DHW",
              fontSize=12)}),                         Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-200,-100},{200,100}}), graphics));
    end Partial_HydraulicHeating_MultiSource;

    model SourceTemplate
      "Template for any kind of heat source into the heating system template"
      extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;
      extends FBM.Components.BaseClasses.SolarParameter;
      extends FBM.Components.BaseClasses.BoilerParameter;
      import sou = FBM.HeatingDHWsystems.Types.HeatSource;
      parameter sou source = sou.SolarPannelField "kind of heat source";
      replaceable package MediumPrim =
          FBM.Media.Glycol20Water80 "Medium 1 in the component"
          annotation (choicesAllMatching = true,Dialog(enable = source == sou.SolarPannelField));
      Modelica.Blocks.Interfaces.RealInput U "Control signal for heat source"
        annotation (Placement(transformation(extent={{-126,50},{-86,90}})));
      Modelica.Blocks.Interfaces.RealOutput T "Output for control command"
        annotation (Placement(transformation(extent={{100,60},{120,80}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Ambiant
        "Connect the heat source to Ambient for heat losses computation"
        annotation (Placement(transformation(extent={{-10,88},{10,108}})));
      Buildings.Fluid.Boilers.BoilerPolynomial Condesing(
        fue=Buildings.Fluid.Data.Fuels.NaturalGasHigherHeatingValue(),
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        effCur=effCur,
        a=a,
        UA=UA,
        VWat=VWat,
        mDry=mDry,
        Q_flow_nominal=QBoiler,
        dp_nominal=dp_nominal,
        T_nominal=363.15) if                                              source == sou.CondensingBoiler
        annotation (Placement(transformation(extent={{-8,52},{12,72}})));
      Buildings.Fluid.Boilers.BoilerPolynomial Gas(                     fue=
            Buildings.Fluid.Data.Fuels.NaturalGasLowerHeatingValue(),
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        T_nominal=T_nominal,
        effCur=effCur,
        a=a,
        UA=UA,
        VWat=VWat,
        mDry=mDry,
        Q_flow_nominal=QBoiler,
        dp_nominal=dp_nominal) if                                        source == sou.GasBoiler
        annotation (Placement(transformation(extent={{-8,22},{12,42}})));
      Buildings.Fluid.Boilers.BoilerPolynomial Wood(                     fue=
            Buildings.Fluid.Data.Fuels.WoodAirDriedLowerHeatingValue(),
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        T_nominal=T_nominal,
        effCur=effCur,
        a=a,
        UA=UA,
        VWat=VWat,
        mDry=mDry,
        Q_flow_nominal=QBoiler,
        dp_nominal=dp_nominal) if                                         source == sou.WoodBoiler
        annotation (Placement(transformation(extent={{0,-46},{20,-26}})));
      Components.SolarPannelFieldWithTes solarPannelField(
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        perColector=perColector,
        mPrim_flow_nominal=mPrim_flow_nominal,
        redeclare package MediumPrim = MediumPrim,
        dpHexPrim=dpHexPrim,
        dpHexSecon=dpHexSecon,
        lat=lat,
        azi=azi,
        til=til,
        rho=rho,
        nPar=nPar,
        T_a1_nominal=T_a1_nominal,
        T_a2_nominal=T_a2_nominal,
        HOn=HOn,
        HOff=HOff,
        posTBot=posTBot,
        nbrNodes=nbrNodes,
        VTan=VTan,
        hTan=hTan,
        dInsTan=dInsTan,
        fue=fue,
        Q_Solar_nominal=Q_Solar_nominal,
        InsuPipeThicknessSol=InsuPipeThicknessSol,
        PipelengthSol=PipelengthSol,
        InsuHeatConduSol=InsuHeatConduSol,
        includePipesSol=includePipesSol) if           source == sou.SolarPannelField
        annotation (Placement(transformation(extent={{-22,-204},{22,-146}})));
      Buildings.BoundaryConditions.WeatherData.Bus weaBus if source == sou.SolarPannelField annotation (Placement(
            transformation(extent={{-88,-172},{-48,-132}}),
                                                         iconTransformation(extent={
                {-182,40},{-162,60}})));
      Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear valBoi(
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        l={0.01,0.01},
        dpValve_nominal=6000,
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal) if                                         source == sou.WoodBoiler
                              "Three-way valve for boiler"
                            annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=0,
            origin={-40,-36})));
      Buildings.Fluid.FixedResistances.Junction spl1(
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        dp_nominal={0,0,-200},
        redeclare package Medium = Medium,
        m_flow_nominal={m_flow_nominal,-m_flow_nominal,-m_flow_nominal}) if                                         source == sou.WoodBoiler
        "Splitter" annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=0,
            origin={68,-36})));
      Modelica.Blocks.Sources.Constant TSetBoiRet(k=TBoiRet_min)
        "Temperature setpoint for boiler return"
        annotation (Placement(transformation(extent={{-100,-68},{-80,-48}})));
      Buildings.Controls.Continuous.LimPID conPIDBoi(
        Td=1,
        controllerType=Modelica.Blocks.Types.SimpleController.PI,
        k=0.1,
        Ti=120,
        reverseAction=true) if                                         source == sou.WoodBoiler "Controller for valve in boiler loop"
        annotation (Placement(transformation(extent={{-70,-68},{-50,-48}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort temRet(redeclare package Medium = Medium, m_flow_nominal=m_flow_nominal) if                                        source == sou.WoodBoiler
                                                       "Return water temperature"
                                              annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=0,
            origin={-14,-36})));
      Buildings.Fluid.Movers.FlowControlled_m_flow pumpWood(
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        nominalValuesDefineDefaultPressureCurve=true) if                                        source == sou.WoodBoiler
        annotation (Placement(transformation(extent={{30,-46},{50,-26}})));
      Modelica.Blocks.Math.Gain gain(k=m_flow_nominal) if                                        source == sou.WoodBoiler annotation (Placement(
            transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={40,-10})));
    equation
      if (source == sou.CondensingBoiler) then
      connect(port_a, Condesing.port_a) annotation (Line(points={{-100,0},{-56,0},{-56,
                62},{-8,62}},color={0,127,255}));
      connect(Condesing.port_b, port_b) annotation (Line(points={{12,62},{54,62},{54,
                0},{100,0}},
                           color={0,127,255}));
      connect(Condesing.T,T)  annotation (Line(points={{13,70},{56,70},{110,70}},
                    color={0,0,127}));
       connect(U, Condesing.y) annotation (Line(points={{-106,70},{-60,70},{-10,70}},
                        color={0,0,127}));
     connect(Ambiant, Condesing.heatPort)
        annotation (Line(points={{0,98},{0,82},{0,69.2},{2,69.2}},
                                                            color={191,0,0}));
      elseif (source == sou.GasBoiler) then
      connect(port_a, Gas.port_a)
        annotation (Line(points={{-100,0},{-56,0},{-56,32},{-8,32}},
                                                            color={0,127,255}));
          connect(Gas.port_b, port_b)
        annotation (Line(points={{12,32},{12,32},{54,32},{54,0},{100,0}},
                                                         color={0,127,255}));
          connect(Gas.T,T)  annotation (Line(points={{13,40},{26,40},{26,70},{
                110,70}},
            color={0,0,127}));
          connect(U, Gas.y) annotation (Line(points={{-106,70},{-18,70},{-18,40},
                {-10,40}},
            color={0,0,127}));
      connect(Ambiant, Gas.heatPort)
        annotation (Line(points={{0,98},{0,66},{0,39.2},{2,39.2}},
                                                          color={191,0,0}));
      elseif (source == sou.WoodBoiler) then
       connect(Ambiant, Wood.heatPort)
        annotation (Line(points={{0,98},{0,98},{0,-28.8},{10,-28.8}},
                                                              color={191,0,0}));
      connect(U, Wood.y) annotation (Line(points={{-106,70},{-106,70},{-18,70},{-18,-28},{-2,-28}},
                                         color={0,0,127}));
      connect(Wood.T,T)  annotation (Line(points={{21,-28},{26,-28},{26,70},{
                110,70}},
            color={0,0,127}));
      connect(port_a, valBoi.port_1) annotation (Line(points={{-100,0},{-57,0},{-57,-36},{-50,-36}}, color={0,127,255}));
      connect(spl1.port_2, port_b) annotation (Line(points={{78,-36},{84,-36},{84,0},{100,0}}, color={0,127,255}));
      connect(valBoi.port_3, spl1.port_3) annotation (Line(points={{-40,-26},{-40,-14},{68,-14},{68,-26}}, color={0,127,255}));
      connect(TSetBoiRet.y, conPIDBoi.u_s) annotation (Line(points={{-79,-58},{
                -72,-58}},                                                                          color={0,0,127}));
      connect(conPIDBoi.y, valBoi.y) annotation (Line(points={{-49,-58},{-40,
                -58},{-40,-48}},                                                              color={0,0,127}));
      connect(valBoi.port_2, temRet.port_a) annotation (Line(points={{-30,-36},{-24,-36}}, color={0,127,255}));
      connect(temRet.port_b, Wood.port_a) annotation (Line(points={{-4,-36},{0,-36}}, color={0,127,255}));
      connect(conPIDBoi.u_m, temRet.T) annotation (Line(points={{-60,-70},{-14,
                -70},{-14,-47}},                                                                color={0,0,127}));
      connect(Wood.port_b, pumpWood.port_a)
      annotation (Line(points={{20,-36},{26,-36},{30,-36}}, color={0,127,255}));
      connect(spl1.port_1, pumpWood.port_b)
        annotation (Line(points={{58,-36},{54,-36},{50,-36}}, color={0,127,255}));
      connect(pumpWood.m_flow_in, gain.y) annotation (Line(points={{39.8,-24},{40,-24},
              {40,-14.4}}, color={0,0,127}));
      connect(U, gain.u)
        annotation (Line(points={{-106,70},{40,70},{40,-5.2}}, color={0,0,127}));
      elseif (source == sou.SolarPannelField) then
      connect(port_a, solarPannelField.port_a) annotation (Line(points={{-100,0},{-56,0},{-56,-175},{-22,-175}},
                                       color={0,127,255}));
      connect(solarPannelField.port_b, port_b) annotation (Line(points={{22,-175},{55,-175},{55,0},{100,0}},
                                    color={0,127,255}));
      connect(weaBus, solarPannelField.weaBus) annotation (Line(
          points={{-68,-152},{-46,-152},{-46,-151.22},{-16.72,-151.22}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(solarPannelField.T,T)  annotation (Line(points={{-23.76,-192.4},{
                -23.76,-84},{26,-84},{26,70},{106,70},{110,70}},
                                        color={0,0,127}));
      connect(solarPannelField.y, U) annotation (Line(points={{22.44,-194.72},{
                22.44,-84},{-18,-84},{-18,68},{-18,70},{-106,70}},            color=
             {0,0,127}));
               connect(Ambiant, solarPannelField.heatPort)
        annotation (Line(points={{0,98},{0,-146.58}},       color={191,0,0}));
      else
      end if;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-220},{100,100}}),
                                                                    graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{6,-8},{6,8},{-6,0},{6,-8}},
              lineColor={191,0,0},
              fillColor={191,0,0},
              fillPattern=FillPattern.Solid,
              origin={-10,18},
              rotation=90),
            Line(
              points={{10,62},{0,72}},
              color={191,0,0},
              thickness=0.5),
            Line(
              points={{22,6.12322e-15},{6,1.10218e-15}},
              color={191,0,0},
              thickness=0.5,
              origin={0,66},
              rotation=90),
            Polygon(
              points={{6,-8},{6,8},{-6,0},{6,-8}},
              lineColor={191,0,0},
              fillColor={191,0,0},
              fillPattern=FillPattern.Solid,
              origin={10,18},
              rotation=90),
            Rectangle(
              extent={{-100,12},{100,-12}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={192,192,192}),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={217,236,256}),
            Line(
              points={{-10,20},{-10,60},{-10,62}},
              color={191,0,0},
              smooth=Smooth.Bezier,
              thickness=0.5),
            Line(
              points={{10,20},{10,60},{10,62}},
              color={191,0,0},
              smooth=Smooth.Bezier,
              thickness=0.5),
            Line(
              points={{5,-5},{-5,5}},
              color={191,0,0},
              thickness=0.5,
              origin={-5,67},
              rotation=90)}),                                        Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-100,-220},{100,100}})));
    end SourceTemplate;

    model MultiSourceTemplate
      "Template for any kind of heat source into the heating system template"
      extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;
      extends FBM.Components.BaseClasses.SolarParameter;
      extends FBM.Components.BaseClasses.BoilerParameter;
      import sou = FBM.HeatingDHWsystems.Types.HeatSource;
      parameter sou source = sou.SolarPannelField "kind of heat source";
      replaceable package MediumPrim =
          FBM.Media.Glycol20Water80 "Medium 1 in the component"
          annotation (choicesAllMatching = true,Dialog(enable = source == sou.SolarPannelField));
      Modelica.Blocks.Interfaces.RealInput U "Control signal for heat source"
        annotation (Placement(transformation(extent={{-126,50},{-86,90}})));
      Modelica.Blocks.Interfaces.RealOutput T "Output for control command"
        annotation (Placement(transformation(extent={{100,60},{120,80}})));
      Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a Ambiant
        "Connect the heat source to Ambient for heat losses computation"
        annotation (Placement(transformation(extent={{-10,88},{10,108}})));
      Buildings.Fluid.Boilers.BoilerPolynomial Condesing(
        fue=Buildings.Fluid.Data.Fuels.NaturalGasHigherHeatingValue(),
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        effCur=effCur,
        a=a,
        UA=UA,
        VWat=VWat,
        mDry=mDry,
        Q_flow_nominal=QBoiler,
        dp_nominal=dp_nominal,
        T_nominal=363.15) if                                              source == sou.CondensingBoiler
        annotation (Placement(transformation(extent={{-8,40},{12,60}})));
      Buildings.Fluid.Boilers.BoilerPolynomial Gas(                     fue=
            Buildings.Fluid.Data.Fuels.NaturalGasLowerHeatingValue(),
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        T_nominal=T_nominal,
        effCur=effCur,
        a=a,
        UA=UA,
        VWat=VWat,
        mDry=mDry,
        Q_flow_nominal=QBoiler,
        dp_nominal=dp_nominal) if                                        source == sou.GasBoiler
        annotation (Placement(transformation(extent={{-8,10},{12,30}})));
      Buildings.Fluid.Boilers.BoilerPolynomial Wood(                     fue=
            Buildings.Fluid.Data.Fuels.WoodAirDriedLowerHeatingValue(),
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        T_nominal=T_nominal,
        effCur=effCur,
        a=a,
        UA=UA,
        VWat=VWat,
        mDry=mDry,
        Q_flow_nominal=QBoiler,
        dp_nominal=dp_nominal) if                                         source == sou.WoodBoiler
        annotation (Placement(transformation(extent={{0,-46},{20,-26}})));
      Components.SolarPannelFieldWithoutTes solarPannelField(
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal,
        perColector=perColector,
        mPrim_flow_nominal=mPrim_flow_nominal,
        redeclare package MediumPrim = MediumPrim,
        dpHexPrim=dpHexPrim,
        dpHexSecon=dpHexSecon,
        lat=lat,
        azi=azi,
        til=til,
        rho=rho,
        nPar=nPar,
        T_a1_nominal=T_a1_nominal,
        T_a2_nominal=T_a2_nominal,
        HOn=HOn,
        HOff=HOff,
        fue=fue,
        Q_Solar_nominal=Q_Solar_nominal,
        InsuPipeThicknessSol=InsuPipeThicknessSol,
        PipelengthSol=PipelengthSol,
        InsuHeatConduSol=InsuHeatConduSol,
        includePipesSol=includePipesSol) if           source == sou.SolarPannelField
        annotation (Placement(transformation(extent={{-22,-204},{22,-146}})));
      Buildings.BoundaryConditions.WeatherData.Bus weaBus if source == sou.SolarPannelField annotation (Placement(
            transformation(extent={{-88,-172},{-48,-132}}),
                                                         iconTransformation(extent={
                {-182,40},{-162,60}})));
      Buildings.Fluid.Actuators.Valves.ThreeWayEqualPercentageLinear valBoi(
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        l={0.01,0.01},
        dpValve_nominal=6000,
        redeclare package Medium = Medium,
        m_flow_nominal=m_flow_nominal) if                                         source == sou.WoodBoiler
                              "Three-way valve for boiler"
                            annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=0,
            origin={-40,-36})));
      Buildings.Fluid.FixedResistances.Junction spl1(
        energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
        dp_nominal={0,0,-200},
        redeclare package Medium = Medium,
        m_flow_nominal={m_flow_nominal,-m_flow_nominal,-m_flow_nominal}) if                                         source == sou.WoodBoiler
        "Splitter" annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=0,
            origin={68,-36})));
      Buildings.Fluid.Movers.FlowControlled_m_flow pumWood(redeclare package Medium = Medium,
          m_flow_nominal=m_flow_nominal) if                                                                                          source == sou.WoodBoiler annotation (Placement(transformation(extent={{26,-46},{46,-26}})));
      Modelica.Blocks.Sources.Constant TSetBoiRet(k=TBoiRet_min)
        "Temperature setpoint for boiler return"
        annotation (Placement(transformation(extent={{-100,-68},{-80,-48}})));
      Buildings.Controls.Continuous.LimPID conPIDBoi(
        Td=1,
        controllerType=Modelica.Blocks.Types.SimpleController.PI,
        k=0.1,
        Ti=120,
        reverseAction=true) if                                         source == sou.WoodBoiler "Controller for valve in boiler loop"
        annotation (Placement(transformation(extent={{-70,-68},{-50,-48}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort temRet(redeclare package
          Medium =                                                                 Medium, m_flow_nominal=m_flow_nominal) if                                        source == sou.WoodBoiler
                                                       "Return water temperature"
                                              annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=0,
            origin={-14,-36})));
      Modelica.Blocks.Interfaces.RealInput TBoTank if           source == sou.SolarPannelField
        "Temperature at the bottom of the tank"
        annotation (Placement(transformation(extent={{120,-214},{80,-174}})));
      Modelica.Blocks.Math.Gain gainWood(k=m_flow_nominal) if                                    source == sou.WoodBoiler
        annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={36,-10})));
      Buildings.Fluid.Movers.FlowControlled_m_flow pumGas(redeclare package Medium =
            Medium, m_flow_nominal=m_flow_nominal) if         source == sou.GasBoiler
        annotation (Placement(transformation(extent={{-48,12},{-34,26}})));
      Modelica.Blocks.Math.Gain gainGas(k=m_flow_nominal) if                                        source == sou.GasBoiler
        annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-42,38})));
      Buildings.Fluid.Movers.FlowControlled_m_flow pumCond(
                                                      redeclare package Medium =
            Medium, m_flow_nominal=m_flow_nominal) if         source == sou.CondensingBoiler
        annotation (Placement(transformation(extent={{-36,42},{-22,56}})));
      Modelica.Blocks.Math.Gain gainCond(
                                        k=m_flow_nominal) if                                        source == sou.CondensingBoiler
        annotation (Placement(transformation(
            extent={{-4,-4},{4,4}},
            rotation=270,
            origin={-30,64})));
    equation
      if (source == sou.CondensingBoiler) then
        connect(pumCond.port_b, Condesing.port_a) annotation (Line(points={{-22,49},{-16,
              49},{-16,50},{-8,50}}, color={0,127,255}));
      connect(port_a, pumCond.port_a) annotation (Line(points={{-100,0},{-56,0},{-56,
              49},{-36,49}}, color={0,127,255}));
      connect(pumCond.m_flow_in, gainCond.y) annotation (Line(points={{-29.14,57.4},{-30,57.4},
              {-30,59.6}}, color={0,0,127}));
      connect(U, gainCond.u)
        annotation (Line(points={{-106,70},{-30,70},{-30,68.8}}, color={0,0,127}));
      connect(Condesing.port_b, port_b) annotation (Line(points={{12,50},{54,50},{54,
                0},{100,0}},
                           color={0,127,255}));
      connect(Condesing.T,T)  annotation (Line(points={{13,58},{26,58},{26,70},{110,
                70}},
                    color={0,0,127}));
       connect(U, Condesing.y) annotation (Line(points={{-106,70},{-10,70},{-10,58}},
                        color={0,0,127}));
     connect(Ambiant, Condesing.heatPort)
        annotation (Line(points={{0,98},{0,82},{0,57.2},{2,57.2}},
                                                            color={191,0,0}));
      elseif (source == sou.GasBoiler) then
          connect(port_a, pumGas.port_a) annotation (Line(points={{-100,0},{-56,0},{
                -56,19},{-48,19}},
                             color={0,127,255}));
      connect(pumGas.port_b, Gas.port_a)
        annotation (Line(points={{-34,19},{-18,19},{-18,20},{-8,20}},
                                                    color={0,127,255}));
      connect(pumGas.m_flow_in, gainGas.y) annotation (Line(points={{-41.14,27.4},{-42,27.4},
                {-42,33.6}},
                      color={0,0,127}));
      connect(U, gainGas.u)
        annotation (Line(points={{-106,70},{-42,70},{-42,42.8}}, color={0,0,127}));
          connect(Gas.port_b, port_b)
        annotation (Line(points={{12,20},{12,32},{54,32},{54,0},{100,0}},
                                                         color={0,127,255}));
          connect(Gas.T,T)  annotation (Line(points={{13,28},{26,28},{26,70},{110,70}},
            color={0,0,127}));
          connect(U, Gas.y) annotation (Line(points={{-106,70},{-18,70},{-18,28},{-10,
                28}},
            color={0,0,127}));
      connect(Ambiant, Gas.heatPort)
        annotation (Line(points={{0,98},{0,66},{0,27.2},{2,27.2}},
                                                          color={191,0,0}));
      elseif (source == sou.WoodBoiler) then
       connect(Ambiant, Wood.heatPort)
        annotation (Line(points={{0,98},{0,98},{0,-28.8},{10,-28.8}},
                                                              color={191,0,0}));
      connect(U, Wood.y) annotation (Line(points={{-106,70},{-106,70},{-18,70},{-18,-28},{-2,-28}},
                                         color={0,0,127}));
      connect(Wood.T,T)  annotation (Line(points={{21,-28},{26,-28},{26,70},{
                110,70}},
            color={0,0,127}));
      connect(port_a, valBoi.port_1) annotation (Line(points={{-100,0},{-57,0},{-57,-36},{-50,-36}}, color={0,127,255}));
      connect(spl1.port_2, port_b) annotation (Line(points={{78,-36},{84,-36},{84,0},{100,0}}, color={0,127,255}));
      connect(valBoi.port_3, spl1.port_3) annotation (Line(points={{-40,-26},{-40,-14},{68,-14},{68,-26}}, color={0,127,255}));
      connect(Wood.port_b, pumWood.port_a) annotation (Line(points={{20,-36},{20,-36},{26,-36}}, color={0,127,255}));
      connect(spl1.port_1, pumWood.port_b) annotation (Line(points={{58,-36},{46,-36}}, color={0,127,255}));
      connect(TSetBoiRet.y, conPIDBoi.u_s) annotation (Line(points={{-79,-58},{
                -72,-58}},                                                                          color={0,0,127}));
      connect(conPIDBoi.y, valBoi.y) annotation (Line(points={{-49,-58},{-40,
                -58},{-40,-48}},                                                              color={0,0,127}));
      connect(valBoi.port_2, temRet.port_a) annotation (Line(points={{-30,-36},{-24,-36}}, color={0,127,255}));
      connect(temRet.port_b, Wood.port_a) annotation (Line(points={{-4,-36},{0,-36}}, color={0,127,255}));
      connect(conPIDBoi.u_m, temRet.T) annotation (Line(points={{-60,-70},{-14,
                -70},{-14,-47}},                                                                color={0,0,127}));
        connect(U, gainWood.u)
          annotation (Line(points={{-106,70},{36,70},{36,-5.2}}, color={0,0,127}));
        connect(gainWood.y, pumWood.m_flow_in) annotation (Line(points={{36,
                -14.4},{36,-24},{35.8,-24}},
                             color={0,0,127}));
      elseif (source == sou.SolarPannelField) then
      connect(solarPannelField.TBoTank, TBoTank) annotation (Line(points={{22.44,-194.72},
              {54.22,-194.72},{54.22,-194},{100,-194}}, color={0,0,127}));
      connect(port_a, solarPannelField.port_a) annotation (Line(points={{-100,0},{-56,0},{-56,-175},{-22,-175}},
                                       color={0,127,255}));
      connect(solarPannelField.port_b, port_b) annotation (Line(points={{22,-175},{55,-175},{55,0},{100,0}},
                                    color={0,127,255}));
      connect(weaBus, solarPannelField.weaBus) annotation (Line(
          points={{-68,-152},{-46,-152},{-46,-151.22},{-16.72,-151.22}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(solarPannelField.T,T)  annotation (Line(points={{-23.76,-192.4},{-23.76,
                -84},{32,-84},{32,70},{110,70}},
                                        color={0,0,127}));
      connect(Ambiant, solarPannelField.heatPort)
        annotation (Line(points={{0,98},{0,-146.58}},       color={191,0,0}));
      connect(U, solarPannelField.u) annotation (Line(points={{-106,70},{-106,69},{-8.8,
              69},{-8.8,-146}}, color={0,0,127}));
      else
      end if;
       annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-220},{100,100}}),
                                                                    graphics={
            Rectangle(
              extent={{-100,100},{100,-100}},
              lineColor={28,108,200},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{6,-8},{6,8},{-6,0},{6,-8}},
              lineColor={191,0,0},
              fillColor={191,0,0},
              fillPattern=FillPattern.Solid,
              origin={-10,18},
              rotation=90),
            Line(
              points={{10,62},{0,72}},
              color={191,0,0},
              thickness=0.5),
            Line(
              points={{22,6.12322e-15},{6,1.10218e-15}},
              color={191,0,0},
              thickness=0.5,
              origin={0,66},
              rotation=90),
            Polygon(
              points={{6,-8},{6,8},{-6,0},{6,-8}},
              lineColor={191,0,0},
              fillColor={191,0,0},
              fillPattern=FillPattern.Solid,
              origin={10,18},
              rotation=90),
            Rectangle(
              extent={{-100,12},{100,-12}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={192,192,192}),
            Rectangle(
              extent={{-100,10},{100,-10}},
              lineColor={0,0,0},
              fillPattern=FillPattern.HorizontalCylinder,
              fillColor={217,236,256}),
            Line(
              points={{-10,20},{-10,60},{-10,62}},
              color={191,0,0},
              smooth=Smooth.Bezier,
              thickness=0.5),
            Line(
              points={{10,20},{10,60},{10,62}},
              color={191,0,0},
              smooth=Smooth.Bezier,
              thickness=0.5),
            Line(
              points={{5,-5},{-5,5}},
              color={191,0,0},
              thickness=0.5,
              origin={-5,67},
              rotation=90)}),                                        Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-100,-220},{100,100}})));
    end MultiSourceTemplate;

    model EmitTemplate
     extends Buildings.Fluid.Interfaces.PartialTwoPortInterface;
      Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
            transformation(extent={{-106,-102},{-66,-62}}), iconTransformation(
              extent={{-86,-82},{-66,-62}})));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end EmitTemplate;

    partial model Partial_HydraulicHeating_Test
      "Hydraulic multi-zone heating "
      replaceable package Medium = Buildings.Media.Water;
      extends FBM.HeatingDHWsystems.BaseClasses.AHU_Heating_parameters;
      extends FBM.Controls.ControlHeating.Interfaces.ControlPara;
      extends FBM.Interfaces.BaseClasses.HeatingSystem(
        includePipes = true,
        isHea = true,
        isCoo = false,
        nConvPorts = nZones,
        nRadPorts = nZones,
        nTemSen = nZones,
        nEmbPorts=0,
        nZones=1);

        // --- Parameter : SetPoint Heating

       parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_Day= {294.15 for i in 1:
          nZones} "Set point temperature for room by day";
       parameter Modelica.SIunits.Temperature[nZones] T_SetPoint_night= {291.15 for i in 1:
          nZones}
                 "Set point temperature for room by night";



      // --- Parameter: General parameters for the design (nominal) conditions and heat curve
      parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
        "Nominal power, can be seen as the max power of the emission system per zone";
      parameter Modelica.SIunits.Temperature[nZones] TRoomNom={294.15 for i in 1:
          nZones} "Nominal room temperature";
      final parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal = QNom/(4180.6*dTSupRetNom)
        "Nominal mass flow rates";
        // -- Pipes and valve parameters
      parameter Modelica.SIunits.Thickness InsuPipeThickness=0.02
                                                                 "Thickness of the pipe insulation";
      parameter Modelica.SIunits.Length Pipelength=5
                                                    "pipe length of the supply OR return branch";
      parameter Modelica.SIunits.Pressure dp=3000 "Pressure drop over a single pipe"
        annotation(Dialog(group = "Pipes",
                         enable = includePipes));
     parameter Modelica.SIunits.ThermalConductivity InsuHeatCondu=0.04;
     parameter Real fraKSupply(min=0, max=1) = 0.7
        "Fraction Kv(port_3->port_2)/Kv(port_1->port_2)";
     parameter Real[2] lSupply(each min=0, each max=1) = {0.01, 0.01}
        "Valve leakage, l=Kv(y=0)/Kv(y=1)";
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
                         enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Av)));
      parameter Real deltaMSupply = 0.02
        "Fraction of nominal flow rate where linearization starts, if y=1"
        annotation(Dialog(group="Pressure-flow linearization"));
      parameter Modelica.SIunits.Pressure dpValve_nominalSupply(displayUnit="Pa",
                                                          min=0,
                                                          fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.OpPoint then true else false)
        "Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint"
        annotation(Dialog(group="Nominal condition",
                   enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      parameter Modelica.SIunits.Density rhoStdSupply=Medium.density_pTX(101325, 273.15+4, Medium.X_default)
        "Inlet density for which valve coefficients are defined"
      annotation(Dialog(group="Nominal condition", tab="Advanced"));
      parameter Real Kv_SISupply(
        min=0,
        fixed= false)
        "Flow coefficient for fully open valve in SI units, Kv=m_flow/sqrt(dp) [kg/s/(Pa)^(1/2)]"
      annotation(Dialog(group = "Flow Coefficient Supply Valve",
                        enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      // --- production components of hydraulic circuit
      replaceable Buildings.Fluid.Boilers.BoilerPolynomial heater(
        redeclare replaceable package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        fue=Buildings.Fluid.Data.Fuels.NaturalGasHigherHeatingValue()) "Heater (boiler, heat pump, ...)"
        annotation (Placement(transformation(extent={{-10,10},{10,-10}},
            rotation=90,
            origin={-66,12})));
      // --- distribution components of hydraulic circuit
      ElementaryBlocs.CollectorUnit collectorUnit(redeclare package Medium = Medium,
          m_flow_nominal=sum(m_flow_nominal)) if
                                                isAHU
        annotation (Placement(transformation(extent={{-20,6},{0,26}})));
      Buildings.HeatTransfer.Sources.PrescribedTemperature
        prescribedTemperature annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={12,-14})));
      Buildings.Utilities.Math.Average ave(nin=nTemSen)
        "Compute average of room temperatures"
        annotation (Placement(transformation(extent={{-34,-28},{-20,-14}})));
      ElementaryBlocs.MixingCircuit_Tset mixingCircuit_AHU(
        redeclare package Medium = Medium,
        InsuPipeThickness=InsuPipeThickness,
        Pipelength=Pipelength,
        InsuHeatCondu=InsuHeatCondu,
        m_flow_nominal=mAHU_flow_nominal,
        KvReturn=40,
        measureSupplyT=true) if         isAHU == true annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-12,-54})));
      Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
            Medium) if
          isAHU == true annotation (Placement(transformation(extent={{6,-110},{26,-90}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
            Medium) if
          isAHU == true annotation (Placement(transformation(extent={{-24,-110},{-4,
                -90}})));
      Modelica.Blocks.Sources.Constant TSetAHUOut(k=TAHUSet) if  isAHU == true
        "Temperature setpoint for AHU outlet"
       annotation (Placement(transformation(extent={{34,-60},{20,-46}})));
         ElementaryBlocs.MixingCircuit_Tset[nZones] mixingCircuit_Emit(
        redeclare package Medium = Medium,
        each InsuPipeThickness=InsuPipeThickness,
        each Pipelength=Pipelength,
        each InsuHeatCondu=InsuHeatCondu,
        m_flow_nominal=m_flow_nominal,
        each KvReturn=40,
        each measureSupplyT=true)
        annotation (Placement(transformation(extent={{10,2},{30,26}})));
      ElementaryBlocs.PumpSupply_m_flow[nZones] pumpSupply_m_flow(
        redeclare package Medium = Medium,
        each InsuPipeThickness=InsuPipeThickness,
        each Pipelength=Pipelength,
        each InsuHeatCondu=InsuHeatCondu,
        m_flow_nominal=m_flow_nominal,
        each includePipes=includePipes,
        each addPowerToMedium=true,
        each KvReturn=40,
        each dp=dp)
        annotation (Placement(transformation(extent={{54,0},{76,24}})));
      // --- emission components of hydraulic circuit
      replaceable Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2[
                                                    nZones] emission(
        redeclare each replaceable package Medium = Medium)
        annotation (Placement(transformation(extent={{82,10},{112,30}})));
      // --- boudaries
      // --- controllers
      replaceable FBM.Controls.ControlHeating.Ctrl_Heating ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom,
        TSupMin=TSupMin,
        minSup=minSup,
        corFac_val=corFac_val,
        THeaterSet(start=293.15)) constrainedby
        FBM.Controls.ControlHeating.Interfaces.Partial_Ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom)
        "Controller for the heater and the emission set point "
        annotation (Placement(transformation(extent={{-110,62},{-90,82}})));
      Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-0.5, each
          uHigh=1)        "onoff controller for the pumps of the emission circuits"
        annotation (Placement(transformation(extent={{18,36},{38,56}})));
      Modelica.Blocks.Sources.RealExpression TSet_max(y=max(swi.y))
        "maximum value of set point temperature" annotation (Placement(
            transformation(
            extent={{-21,-10},{21,10}},
            rotation=0,
            origin={-110,91})));
      Modelica.Blocks.Math.Add add[nZones](            each k2=+1, each k1=-1)
        annotation (Placement(transformation(extent={{-2,54},{12,40}})));

      Modelica.Blocks.Sources.Constant[nZones] TRooNig(k={T_SetPoint_night[i] for i in
                1:nZones})
        "Room temperature set point at night"
        annotation (Placement(transformation(extent={{-38,50},{-28,60}})));
     Modelica.Blocks.Sources.Constant[nZones] TRooSet(k={T_SetPoint_Day[i] for i in
                1:nZones})
        annotation (Placement(transformation(extent={{-38,82},{-28,92}})));
     Modelica.Blocks.Logical.Switch[nZones] swi "Switch to select set point"
        annotation (Placement(transformation(extent={{-18,60},{-6,72}})));

      // --- Interface

      // --- Sensors
      Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
            m_flow_nominal)
        annotation (Placement(transformation(extent={{46,36},{66,56}})));
          Buildings.Controls.SetPoints.OccupancySchedule[ nZones]
                                           occSch "Occupancy schedule"
        annotation (Placement(transformation(extent={{-38,68},{-30,76}})));

    public
      Controls.ControlHeating.OnOff_Heater onOff_Heater(bandwidth=5)
        annotation (Placement(transformation(extent={{-70,36},{-50,56}})));
      Buildings.Fluid.Storage.ExpansionVessel exp(redeclare package Medium = Medium,
          V_start=0.5)
        annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
    initial equation
      if  CvDataSupply == Buildings.Fluid.Types.CvTypes.OpPoint then
        Kv_SISupply =           sum(m_flow_nominal)/sqrt(dpValve_nominalSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Kv then
        Kv_SISupply =           KvSupply*rhoStdSupply/3600/sqrt(1E5)
          "Unit conversion m3/(h*sqrt(bar)) to kg/(s*sqrt(Pa))";
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Cv then
        Kv_SISupply =           CvSupply*rhoStdSupply*0.0631/1000/sqrt(6895)
          "Unit conversion USG/(min*sqrt(psi)) to kg/(s*sqrt(Pa))";
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      else
        assert(CvDataSupply == Buildings.Fluid.Types.CvTypes.Av, "Invalid value for CvData.
Obtained CvData = "     + String(CvDataSupply) + ".");
        Kv_SISupply =           AvSupply*sqrt(rhoStdSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      end if;
    equation
        // connections that are function of the number of circuits
      for i in 1:nZones loop
          connect(ctrl_Heating.THeaCur, mixingCircuit_Emit[i].TMixedSet) annotation (
          Line(points={{-90,76},{-46,76},{-46,32},{20,32},{20,26}},
                                                            color={0,0,127}));
                if includePipes == true then
               connect(prescribedTemperature.port, pumpSupply_m_flow[i].heatPort)
        annotation (Line(points={{12,-8},{52,-8},{52,0},{65,0}},
                                                               color={191,0,0}));
                end if;
      end for;
      // general connections for any configuration
      connect(TSet_max.y, ctrl_Heating.TRoo_in1) annotation (Line(
          points={{-86.9,91},{-98,91},{-98,76},{-110,76}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(add.y, heatingControl.u) annotation (Line(
          points={{12.7,47},{12,47},{12,46},{16,46}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(TSensor, add.u1) annotation (Line(
          points={{-124,-60},{-40,-60},{-40,42.8},{-3.4,42.8}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(weaBus, ctrl_Heating.weaBus) annotation (Line(
          points={{-70,88},{-118,88},{-118,78.2},{-117.6,78.2}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(pumpSupply_m_flow.port_b1, emission.port_a)
        annotation (Line(points={{76,19.2},{76,20},{82,20}}, color={0,127,255}));
      connect(pumpSupply_m_flow.port_a2, emission.port_b) annotation (Line(points={{76,4.8},
              {112,4.8},{112,20}},                color={0,127,255}));
      connect(booleanToReal.y, pumpSupply_m_flow.u) annotation (Line(points={{67,46},
              {65,46},{65,24.96}},         color={0,0,127}));
      connect(ctrl_Heating.THeaterSet, onOff_Heater.reference) annotation (Line(
            points={{-91,73},{-86,73},{-86,46},{-70.8,46}},       color={0,0,
              127}));
        connect(prescribedTemperature.port, heater.heatPort) annotation (Line(
            points={{12,-8},{12,-8},{-40,-8},{-40,2},{-52,2},{-52,12},{-58.8,12}},
                                                             color={191,0,0}));
       connect(prescribedTemperature.T, ave.y) annotation (Line(points={{12,-21.2},{
              12,-21.2},{12,-22},{4,-22},{4,-21},{-19.3,-21}},  color={0,0,127}));
      connect(TSensor, ave.u) annotation (Line(points={{-124,-60},{-41,-60},{-41,-21},
              {-35.4,-21}}, color={0,0,127}));
      if isAHU == true then
     for i in 1:nZones loop
         connect(collectorUnit.port_b1, mixingCircuit_Emit[i].port_a1)
        annotation (Line(points={{0,22},{10,22},{10,21.2}}, color={0,127,255}));
          connect(collectorUnit.port_a2, mixingCircuit_Emit[i].port_b2) annotation (
          Line(points={{0,10},{6,10},{6,6.8},{10,6.8}}, color={0,127,255}));
     end for;
      connect(collectorUnit.port_b3, mixingCircuit_AHU.port_a1)
        annotation (Line(points={{-16,26},{-6,26},{-6,-44}},
                                                           color={0,127,255}));
      connect(collectorUnit.port_a3, mixingCircuit_AHU.port_b2) annotation (Line(
            points={{-4,5.6},{-18,5.6},{-18,-44}},        color={0,127,255}));
      connect(mixingCircuit_AHU.port_b1, port_a) annotation (Line(points={{-6,-64},{
                4,-64},{4,-100},{16,-100}},
                                        color={0,127,255}));
      connect(mixingCircuit_AHU.port_a2, port_b) annotation (Line(points={{-18,-64},
                {-8,-64},{-8,-100},{-14,-100}},
                                             color={0,127,255}));
      connect(mixingCircuit_AHU.TMixedSet, TSetAHUOut.y) annotation (Line(points={{-2,-54},
                {19.3,-54},{19.3,-53}},                                                                           color={0,0,127}));
      else
        for i in 1:nZones loop
        end for;
         end if;
      if includePipes == true then
      end if;
      connect(exp.port_a, heater.port_a)
        annotation (Line(points={{-76,-20},{-66,-20},{-66,2}}, color={0,127,255}));
      connect(mixingCircuit_Emit.port_b1, pumpSupply_m_flow.port_a1) annotation (
          Line(points={{30,21.2},{36,21.2},{36,19.2},{54,19.2}}, color={0,127,255}));
      connect(mixingCircuit_Emit.port_a2, pumpSupply_m_flow.port_b2) annotation (
          Line(points={{30,6.8},{36,6.8},{36,4.8},{54,4.8}}, color={0,127,255}));
      connect(onOff_Heater.y, heater.y) annotation (Line(points={{-49.2,46},{-49.2,46},
              {-44,46},{-44,-4},{-58,-4},{-58,0}},         color={0,0,127}));
      connect(onOff_Heater.u_m, heater.T) annotation (Line(points={{-59,35},{-59,34.5},
              {-58,34.5},{-58,23}},           color={0,0,127}));
      connect(heater.port_b, collectorUnit.port_a1) annotation (Line(points={{
              -66,22},{-43,22},{-20,22}}, color={0,127,255}));
      connect(heater.port_a, collectorUnit.port_b2) annotation (Line(points={{
              -66,2},{-66,2},{-66,10},{-20,10}}, color={0,127,255}));
      connect(TRooNig.y, swi.u3) annotation (Line(points={{-27.5,55},{-19.2,55},{-19.2,
              61.2}}, color={0,0,127}));
      connect(heatingControl.y, booleanToReal.u)
        annotation (Line(points={{39,46},{40,46},{44,46}}, color={255,0,255}));
      connect(occSch.occupied, swi.u2) annotation (Line(points={{-29.6,69.6},{-24.8,
              69.6},{-24.8,66},{-19.2,66}}, color={255,0,255}));
      connect(swi.y, add.u2) annotation (Line(points={{-5.4,66},{-3.4,66},{-3.4,51.2}},
            color={0,0,127}));
      connect(TRooSet.y, swi.u1) annotation (Line(points={{-27.5,87},{-27.5,88.5},{-19.2,
              88.5},{-19.2,70.8}}, color={0,0,127}));
                      annotation(Dialog(group = "Nominal condition"),
                  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -100},{120,100}})),                   Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-120,-100},{120,100}}), graphics));
    end Partial_HydraulicHeating_Test;

    partial model Partial_HydraulicHeating_Multi_Test
      "Hydraulic multi-zone heating "
      replaceable package Medium = Buildings.Media.Water;
      extends FBM.HeatingDHWsystems.BaseClasses.AHU_Heating_parameters;
        extends FBM.Components.BaseClasses.SolarParameter;
      extends FBM.Components.BaseClasses.BoilerParameter;
      extends FBM.HeatingDHWsystems.BaseClasses.TES_parameters;
      extends FBM.Interfaces.BaseClasses.HeatingSystem(
        includePipes = true,
        isHea = true,
        isCoo = false,
        nConvPorts = nZones,
        nRadPorts = nZones,
        nTemSen = nZones,
        nEmbPorts=0,
        nZones=1);
     // --- Parameter : Heatsource design
      parameter Integer nSource = 1 annotation(Dialog(group= "Settings"));
      import sou = FBM.HeatingDHWsystems.Types.HeatSource;
      parameter sou source[nSource] "type of heat source"
                                                         annotation(Dialog(group= "Settings"));
      // --- Parameter: General parameters for the design (nominal) conditions and heat curve
      // --- Parameter: General parameters for the design (nominal) conditions and heat curve
      parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
        "Nominal power, can be seen as the max power of the emission system per zone";
      parameter Boolean minSup=true
        "true to limit the supply temperature on the lower side";
        parameter Modelica.SIunits.Temperature TSupMin=273.15 + 30
        "Minimum supply temperature if enabled";
      parameter Modelica.SIunits.Temperature TSupNom=273.15 + 45
        "Nominal supply temperature";
      parameter Modelica.SIunits.TemperatureDifference dTSupRetNom=10
        "Nominal DT in the heating system";
      parameter Modelica.SIunits.Temperature[nZones] TRoomNom={294.15 for i in 1:
          nZones} "Nominal room temperature";
      parameter Modelica.SIunits.TemperatureDifference corFac_val = 0
        "correction term for TSet of the heating curve";
      parameter Modelica.SIunits.Time timeFilter=43200
        "Time constant for the filter of ambient temperature for computation of heating curve";
      final parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal = QNom/(4180.6*dTSupRetNom)
        "Nominal mass flow rates";
        // -- Pipes and valve parameters
      parameter Modelica.SIunits.Thickness InsuPipeThickness=0.02
                                                                 "Thickness of the pipe insulation";
      parameter Modelica.SIunits.Length Pipelength=5
                                                    "pipe length of the supply OR return branch";
      parameter Modelica.SIunits.Pressure dp=3000 "Pressure drop over a single pipe"
        annotation(Dialog(group = "Pipes",
                         enable = includePipes));
     parameter Modelica.SIunits.ThermalConductivity InsuHeatCondu=0.04;
     parameter Real fraKSupply(min=0, max=1) = 0.7
        "Fraction Kv(port_3->port_2)/Kv(port_1->port_2)";
     parameter Real[2] lSupply(each min=0, each max=1) = {0.01, 0.01}
        "Valve leakage, l=Kv(y=0)/Kv(y=1)";
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
                         enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Av)));
      parameter Real deltaMSupply = 0.02
        "Fraction of nominal flow rate where linearization starts, if y=1"
        annotation(Dialog(group="Pressure-flow linearization"));
      parameter Modelica.SIunits.Pressure dpValve_nominalSupply(displayUnit="Pa",
                                                          min=0,
                                                          fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.OpPoint then true else false)
        "Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint"
        annotation(Dialog(group="Nominal condition",
                   enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      parameter Modelica.SIunits.Density rhoStdSupply=Medium.density_pTX(101325, 273.15+4, Medium.X_default)
        "Inlet density for which valve coefficients are defined"
      annotation(Dialog(group="Nominal condition", tab="Advanced"));
    protected
      parameter Real Kv_SISupply(
        min=0,
        fixed= false)
        "Flow coefficient for fully open valve in SI units, Kv=m_flow/sqrt(dp) [kg/s/(Pa)^(1/2)]"
      annotation(Dialog(group = "Flow Coefficient Supply Valve",
                        enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
      // --- production components of hydraulic circuit
      replaceable FBM.HeatingDHWsystems.Interfaces.MultiSourceTemplate[nSource] heater(
        redeclare replaceable package Medium = Medium,
        each m_flow_nominal=sum(m_flow_nominal),
        each Q_Solar_nominal=sum(QNom),
        source = {source[i] for i in 1:nSource},
        mPrim_flow_nominal=mPrim_flow_nominal,
        includePipesSol=includePipesSol,
        dpHexPrim=dpHexPrim,
        dpHexSecon=dpHexSecon,
        lat=lat,
        T_nominal=T_nominal,
        effCur=effCur,
        UA=UA,
        dTSupRetNom=dTSupRetNom,
        dp_nominal=dp_nominal,
        TBoiRet_min=TBoiRet_min,
        azi=azi,
        til=til,
        rho=rho,
        nPar=nPar,
        nSer=nSer,
        InsuPipeThicknessSol=InsuPipeThicknessSol,
        PipelengthSol=PipelengthSol,
        InsuHeatConduSol=InsuHeatConduSol,
        T_a1_nominal=T_a1_nominal,
        T_a2_nominal=T_a2_nominal,
        HOn=HOn,
        HOff=HOff,
        nbrNodes=nbrNodes,
        VTan=VTan,
        hTan=hTan,
        dInsTan=dInsTan,
        dp_nominalSol=dp_nominalSol,
        each QBoiler=QBoiler)
        annotation (Placement(transformation(extent={{10,10},{-10,-10}},
            rotation=270,
            origin={-94,16})));
      // --- distribution components of hydraulic circuit
      ElementaryBlocs.CollectorUnit collectorUnit(redeclare package Medium = Medium,
          m_flow_nominal=sum(m_flow_nominal)) if
                                                isAHU
        annotation (Placement(transformation(extent={{-20,6},{0,26}})));
      Buildings.HeatTransfer.Sources.PrescribedTemperature
        prescribedTemperature annotation (Placement(transformation(
            extent={{-6,-6},{6,6}},
            rotation=90,
            origin={12,-14})));
      Buildings.Utilities.Math.Average ave(nin=nTemSen)
        "Compute average of room temperatures"
        annotation (Placement(transformation(extent={{-34,-28},{-20,-14}})));
      ElementaryBlocs.MixingCircuit_Tset mixingCircuit_AHU(
        redeclare package Medium = Medium,
        InsuPipeThickness=InsuPipeThickness,
        Pipelength=Pipelength,
        InsuHeatCondu=InsuHeatCondu,
        m_flow_nominal=mAHU_flow_nominal,
        KvReturn=40,
        measureSupplyT=true) if         isAHU == true annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-2,-54})));
      Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
            Medium) if
          isAHU == true annotation (Placement(transformation(extent={{6,-110},{26,-90}})));
      Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
            Medium) if
          isAHU == true annotation (Placement(transformation(extent={{-24,-110},{-4,
                -90}})));
      Modelica.Blocks.Sources.Constant TSetAHUOut(k=TAHUSet) if  isAHU == true
        "Temperature setpoint for AHU outlet"
       annotation (Placement(transformation(extent={{36,-62},{22,-48}})));
         ElementaryBlocs.MixingCircuit_Tset[nZones] mixingCircuit_Emit(
        redeclare package Medium = Medium,
        each InsuPipeThickness=InsuPipeThickness,
        each Pipelength=Pipelength,
        each InsuHeatCondu=InsuHeatCondu,
        m_flow_nominal=m_flow_nominal,
        each KvReturn=40,
        each measureSupplyT=true)
        annotation (Placement(transformation(extent={{10,2},{30,26}})));
      ElementaryBlocs.BalancingValve balancingValve(
        redeclare package Medium = Medium,
        InsuPipeThickness=InsuPipeThickness,
        Pipelength=Pipelength,
        InsuHeatCondu=InsuHeatCondu,
        includePipes=includePipes,
        KvReturn=40,
        dp=dp,
        m_flow_nominal=sum(m_flow_nominal))
        annotation (Placement(transformation(extent={{-42,2},{-22,26}})));
      ElementaryBlocs.PumpSupply_m_flow[nZones] pumpSupply_m_flow(
        redeclare package Medium = Medium,
        each InsuPipeThickness=InsuPipeThickness,
        each Pipelength=Pipelength,
        each InsuHeatCondu=InsuHeatCondu,
        m_flow_nominal=m_flow_nominal,
        each includePipes=includePipes,
        each addPowerToMedium=true,
        each KvReturn=40,
        each dp=dp)
        annotation (Placement(transformation(extent={{40,0},{62,24}})));
      // --- emission components of hydraulic circuit
      replaceable Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2[
                                                    nZones] emission(
        redeclare each replaceable package Medium = Medium)
        annotation (Placement(transformation(extent={{74,10},{104,30}})));
      // --- boudaries
      // --- controllers
    replaceable FBM.Controls.ControlHeating.Ctrl_Heating_TES ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom,
        TSupMin=TSupMin,
        minSup=minSup,
        corFac_val=corFac_val,
        DHW=false,
        dTHeaterSet=3)            constrainedby
        FBM.Controls.ControlHeating.Interfaces.Partial_Ctrl_Heating(
        heatingCurve(timeFilter=timeFilter),
        TSupNom=TSupNom,
        dTSupRetNom=dTSupRetNom)
        "Controller for the heater and the emission set point "
        annotation (Placement(transformation(extent={{-114,72},{-94,92}})));
      Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-0.5,
          each uHigh=0.5) "onoff controller for the pumps of the emission circuits"
        annotation (Placement(transformation(extent={{-2,68},{18,88}})));
      Modelica.Blocks.Sources.RealExpression TSet_max(y=max(TSet))
        "maximum value of set point temperature" annotation (Placement(
            transformation(
            extent={{-21,-10},{21,10}},
            rotation=0,
            origin={-98,95})));
      Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
        annotation (Placement(transformation(extent={{-24,84},{-10,70}})));
      // --- Interface
      // --- Sensors
      Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
            m_flow_nominal)
        annotation (Placement(transformation(extent={{24,68},{44,88}})));
    public
      FBM.Controls.ControlHeating.OnOff_Heater_Multi[nSource] onOff_Heater(bandwidth=
           5)
        annotation (Placement(transformation(extent={{-82,48},{-102,68}})));
    public
      Buildings.Fluid.Storage.StratifiedEnhanced TESTank(
        redeclare package Medium = Medium,
        m_flow_nominal=sum(m_flow_nominal),
        nSeg=nbrNodesTes,
        T_start=Tes_start,
        VTan=VTanTes,
        hTan=hTanTes,
        dIns=dInsTes,
        kIns=kInsTes)                                    annotation (Placement(transformation(extent={{-80,0},
                {-48,32}})));
      Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor[nbrNodesTes] TTank
        annotation (Placement(transformation(extent={{-68,32},{-78,42}})));
      Buildings.Fluid.Storage.ExpansionVessel exp1(
                                                  redeclare package Medium =
            Medium, V_start=2)
        annotation (Placement(transformation(extent={{-36,-56},{-16,-36}})));
    initial equation
      if  CvDataSupply == Buildings.Fluid.Types.CvTypes.OpPoint then
        Kv_SISupply =           sum(m_flow_nominal)/sqrt(dpValve_nominalSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Kv then
        Kv_SISupply =           KvSupply*rhoStdSupply/3600/sqrt(1E5)
          "Unit conversion m3/(h*sqrt(bar)) to kg/(s*sqrt(Pa))";
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Cv then
        Kv_SISupply =           CvSupply*rhoStdSupply*0.0631/1000/sqrt(6895)
          "Unit conversion USG/(min*sqrt(psi)) to kg/(s*sqrt(Pa))";
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      else
        assert(CvDataSupply == Buildings.Fluid.Types.CvTypes.Av, "Invalid value for CvData.
Obtained CvData = "     + String(CvDataSupply) + ".");
        Kv_SISupply =           AvSupply*sqrt(rhoStdSupply);
        KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
        CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
        dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
      end if;
    equation
        // connections that are function of the number of circuits
      for i in 1:nZones loop
          connect(ctrl_Heating.THeaCur, mixingCircuit_Emit[i].TMixedSet) annotation (
          Line(points={{-94,86},{-36,86},{-36,26},{20,26}}, color={0,0,127}));
                if includePipes == true then
               connect(prescribedTemperature.port, pumpSupply_m_flow[i].heatPort)
        annotation (Line(points={{12,-8},{52,-8},{52,0},{51,0}},
                                                               color={191,0,0}));
                end if;
      end for;
      // general connections for any configuration
      connect(TSet_max.y, ctrl_Heating.TRoo_in1) annotation (Line(
          points={{-74.9,95},{-74.9,94},{-112,94},{-112,86},{-114,86}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(add.y, heatingControl.u) annotation (Line(
          points={{-9.3,77},{-8,77},{-8,78},{-4,78}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(TSensor, add.u1) annotation (Line(
          points={{-124,-60},{-76,-60},{-76,72.8},{-25.4,72.8}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(add.u2, TSet) annotation (Line(
          points={{-25.4,81.2},{-40,81.2},{-40,104}},
          color={0,0,127},
          smooth=Smooth.None));
      connect(heatingControl.y, booleanToReal.u) annotation (Line(points={{19,78},{22,
              78}},                     color={255,0,255}));
      connect(weaBus, ctrl_Heating.weaBus) annotation (Line(
          points={{-70,88},{-118,88},{-118,88.2},{-121.6,88.2}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(pumpSupply_m_flow.port_b1, emission.port_a)
        annotation (Line(points={{62,19.2},{62,20},{74,20}}, color={0,127,255}));
      connect(pumpSupply_m_flow.port_a2, emission.port_b) annotation (Line(points={{62,4.8},
              {104,4.8},{104,20}},                color={0,127,255}));
      connect(booleanToReal.y, pumpSupply_m_flow.u) annotation (Line(points={{45,78},
              {51,78},{51,24.96}},         color={0,0,127}));
       connect(prescribedTemperature.T, ave.y) annotation (Line(points={{12,-21.2},{
              12,-21.2},{12,-22},{4,-22},{4,-21},{-19.3,-21}},  color={0,0,127}));
      connect(TSensor, ave.u) annotation (Line(points={{-124,-60},{-77,-60},{-77,-21},
              {-35.4,-21}}, color={0,0,127}));
      if isAHU == true then
         connect(balancingValve.port_b1, collectorUnit.port_a1) annotation (Line(
            points={{-22,21.2},{-22,21.2},{-22,22},{-20,22}}, color={0,127,255}));
      connect(collectorUnit.port_b1, mixingCircuit_Emit[1].port_a1)
        annotation (Line(points={{0,22},{10,22},{10,21.2}}, color={0,127,255}));
      connect(balancingValve.port_a2, collectorUnit.port_b2) annotation (Line(
            points={{-22,6.8},{-20,6.8},{-20,10},{-20,10}}, color={0,127,255}));
      connect(collectorUnit.port_a2, mixingCircuit_Emit[1].port_b2) annotation (
          Line(points={{0,10},{6,10},{6,6.8},{10,6.8}}, color={0,127,255}));
      connect(collectorUnit.port_b3, mixingCircuit_AHU.port_a1)
        annotation (Line(points={{-16,26},{4,26},{4,-44}}, color={0,127,255}));
      connect(collectorUnit.port_a3, mixingCircuit_AHU.port_b2) annotation (Line(
            points={{-4,5.6},{-6,5.6},{-6,-44},{-8,-44}}, color={0,127,255}));
      connect(mixingCircuit_AHU.port_b1, port_a) annotation (Line(points={{4,-64},{4,
              -64},{4,-100},{16,-100}}, color={0,127,255}));
      connect(mixingCircuit_AHU.port_a2, port_b) annotation (Line(points={{-8,-64},{
              -8,-64},{-8,-100},{-14,-100}}, color={0,127,255}));
      connect(mixingCircuit_AHU.TMixedSet, TSetAHUOut.y) annotation (Line(points={{8,-54},{21.3,-54},{21.3,-55}}, color={0,0,127}));
      else
        for i in 1:nZones loop
         connect(balancingValve.port_b1, mixingCircuit_Emit[i].port_a1) annotation (
          Line(points={{-22,21.2},{-22,21.2},{10,21.2}},       color={0,127,255}));
      connect(balancingValve.port_a2,mixingCircuit_Emit [i].port_b2)
        annotation (Line(points={{-22,6.8},{0,6.8},{0,6},{2,6},{10,6},{10,6.8}},
                                                                 color={0,127,
              255}));
        end for;
         end if;
      if includePipes == true then
        connect(prescribedTemperature.port, balancingValve.heatPort)
        annotation (Line(points={{12,-8},{12,-8},{-32,-8},{-32,2}},
                                                             color={191,0,0}));
      end if;
      connect(mixingCircuit_Emit.port_b1, pumpSupply_m_flow.port_a1) annotation (
          Line(points={{30,21.2},{36,21.2},{36,19.2},{40,19.2}}, color={0,127,255}));
      connect(mixingCircuit_Emit.port_a2, pumpSupply_m_flow.port_b2) annotation (
          Line(points={{30,6.8},{36,6.8},{36,4.8},{40,4.8}}, color={0,127,255}));
      connect(TESTank.heaPorVol,TTank. port)
        annotation (Line(points={{-64,16},{-64,37},{-68,37}},
                                                           color={191,0,0}));
      connect(exp1.port_a, TESTank.port_b) annotation (Line(points={{-26,-56},{-26,16},
              {-48,16}},  color={0,127,255}));
        connect(TESTank.port_a, balancingValve.port_a1) annotation (Line(points={{-80,
              16},{-80,16},{-80,21.2},{-42,21.2}}, color={0,127,255}));
      connect(TESTank.port_b, balancingValve.port_b2) annotation (Line(points={{-48,
              16},{-48,16},{-48,6.8},{-42,6.8}}, color={0,127,255}));
      connect(TTank[posTesTop].T, ctrl_Heating.TTankTop) annotation (Line(points={{-78,37},{
              -118,37},{-118,82},{-114,82}}, color={0,0,127}));
      connect(TTank[posTesBot].T, ctrl_Heating.TTankBot) annotation (Line(points={{-78,37},{
              -118,37},{-118,78},{-114,78}}, color={0,0,127}));
      connect(prescribedTemperature.port, TESTank.heaPorBot) annotation (Line(
            points={{12,-8},{-60,-8},{-60,4.16},{-60.8,4.16}}, color={191,0,0}));
      connect(prescribedTemperature.port, TESTank.heaPorSid) annotation (Line(
            points={{12,-8},{-56,-8},{-56,16},{-55.04,16}}, color={191,0,0}));
      connect(prescribedTemperature.port, TESTank.heaPorTop) annotation (Line(
            points={{12,-8},{-60,-8},{-60,27.84},{-60.8,27.84}}, color={191,0,0}));
      for i in 1:nSource loop
        if (source[i] == sou.SolarPannelField) then
      connect(weaBus, heater[i].weaBus) annotation (Line(
          points={{-70,88},{-106,88},{-106,-1.2},{-100.875,-1.2}},
          color={255,204,51},
          thickness=0.5), Text(
          string="%first",
          index=-1,
          extent={{-6,3},{-6,3}}));
      connect(heater[i].TBoTank, TTank[posTesBot].T) annotation (Line(points={{-85.625,
                  26},{-86,26},{-86,36},{-78,36},{-78,37}},
                                         color={0,0,127}));
        end if;
        connect(heater[i].port_b, TESTank.port_a) annotation (Line(points={{-97.75,26},
              {-80,26},{-80,16}}, color={0,127,255}));
      connect(TESTank.port_b, heater[i].port_a) annotation (Line(points={{-48,16},{-48,
              16},{-48,6},{-97.75,6}}, color={0,127,255}));
                connect(onOff_Heater[i].y, heater[i].U) annotation (Line(points={{-102.8,58},{-110,
              58},{-110,5.4},{-102.125,5.4}}, color={0,0,127}));
        connect(onOff_Heater[i].u, TTank[posTesBot].T) annotation (Line(points=
                {{-81,53},{-82,53},{-82,52},{-80,52},{-78,52},{-78,37}}, color=
                {0,0,127}));
        connect(ctrl_Heating.THeaterSet, onOff_Heater[i].reference) annotation (
           Line(points={{-95,83},{-95,83.5},{-81,83.5},{-81,65}}, color={0,0,
                127}));
      end for;
                       annotation(Dialog(group = "Nominal condition"),
                  Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-120,
                -100},{120,100}}), graphics={
                                Text(
              extent={{-92,-12},{-42,-22}},
              lineColor={135,135,135},
              lineThickness=1,
              fillColor={0,0,255},
              fillPattern=FillPattern.Solid,
              textString="Thermal Energy Storage")}), Icon(coordinateSystem(
              preserveAspectRatio=false, extent={{-120,-100},{120,100}}), graphics));
    end Partial_HydraulicHeating_Multi_Test;
  end Interfaces;

  package Types
      extends Modelica.Icons.TypesPackage;
    type HeatSource = enumeration(
        SolarPannelField
                        "Solar pannel with primary circuit, heat exchanger and secondary circuit",
        CondensingBoiler
                        "Gas boiler working on High heating value",
        GasBoiler
              "Gas boiler working on low heating value",
        WoodBoiler
                  "Wood boiler working on pellet wood characteristics with DIN norms",
        HeatPump           "Reversible heat pump with performance curve adjusted based on Carnot efficiency")
      "Enumeration for heat source construction";
    type HeatTransmitter = enumeration(
        EmbeddePiped
                "Embedded pipe model from IDEAS library",
        Radiator     "RadiatorEn442_2 model from Building library")
      "Enumeration for heat transmitter construction";
    type HeatPump = enumeration(
        HP_AW "Air/water Heat pump",
        HP_WW "Water/Water Heat pump") "Type of the heater pump";
  end Types;

  package BaseClasses
     extends Modelica.Icons.BasesPackage;
    record DHWPara
        // --- Domestic Hot Water (DHW) Parameters
      parameter Integer nOcc = 1 "Number of occupants";
          parameter Modelica.SIunits.MassFlowRate m_flow_nominal_DHW = nOcc*nOcc*0.045*983/(3600*24)*10
        "nominal mass flow rate of DHW" annotation(Dialog(group= "DHW parameters"));
      parameter Modelica.SIunits.Temperature TDHWSet(max=273.15 + 60) = 273.15 + 45
        "DHW temperature setpoint" annotation(Dialog(group= "DHW parameters"));
      parameter Modelica.SIunits.Temperature TColdWaterNom=273.15 + 10
        "Nominal tap (cold) water temperature" annotation(Dialog(group= "DHW parameters"));
      // --- Storage Tank Parameters
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal_stoHX = m_flow_nominal_DHW * (TDHWSet - TColdWaterNom)/dTHPTankSet
        "nominal mass flow rate of HX of storage tank" annotation(Dialog(group= "DHW parameters"));
      parameter Modelica.SIunits.TemperatureDifference dTHPTankSet(min=1)=2
        "Difference between tank setpoint and heat pump setpoint";
      parameter Modelica.SIunits.Volume volumeTank=0.25 annotation(Dialog(group= "DHW parameters"));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end DHWPara;

    record TES_parameters
      parameter Modelica.SIunits.Volume VTanTes( min=0) = 5 "Tank volume" annotation (Dialog(group = "TES parameter"));
      parameter Modelica.SIunits.Length hTanTes( min=0) = 1 "Height of tank (without insulation)" annotation (Dialog(group = "TES parameter"));
      parameter Modelica.SIunits.Length dInsTes( min=0) = 0.2 "Thickness of insulation" annotation (Dialog(group = "TES parameter"));
      parameter Modelica.SIunits.ThermalConductivity kInsTes = 0.04
        "Specific heat conductivity of insulation" annotation (Dialog(group = "TES parameter"));
      parameter Integer nbrNodesTes(min=4) = 4 "Number of volume segments, minimum of 4 segments" annotation (Dialog(group = "TES parameter"));
      parameter Modelica.SIunits.Temperature Tes_start= 273.15+20
        "Start value of temperature in the tank"
        annotation(Dialog(group = "TES parameter"));
      parameter Integer posTesTop(max=nbrNodesTes) = 1
        "Position of the top temperature sensor"
                                                annotation (Dialog(group = "TES parameter"));
      parameter Integer posTesBot(max=nbrNodesTes) = nbrNodesTes - 1
        "Position of the bottom temperature sensor"
                                                   annotation (Dialog(group = "TES parameter"));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end TES_parameters;

    record AHU_Heating_parameters
    // --- AHU Parameters
    parameter Modelica.SIunits.Temperature TAHUSet=273.15+55  "Nominal AHU oulet temperature" annotation(Dialog(group= "AHU parameters", enable = isAHU == true));
    parameter Modelica.SIunits.MassFlowRate mAHU_flow_nominal = 0.5 "Mass flow rate of the AHU loop" annotation(Dialog(group= "AHU parameters", enable = isAHU == true));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end AHU_Heating_parameters;
  end BaseClasses;
end HeatingDHWsystems;
