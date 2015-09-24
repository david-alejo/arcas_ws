  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 1;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (quadrotor_controller_P)
    ;%
      section.nData     = 14;
      section.data(14)  = dumData; %prealloc
      
	  ;% quadrotor_controller_P.Constant_Value
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadrotor_controller_P.FORCEZ_Value
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 2;
	
	  ;% quadrotor_controller_P.TorqueXYZ_Value
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 3;
	
	  ;% quadrotor_controller_P.gp_6_Gain
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 6;
	
	  ;% quadrotor_controller_P.gp_7_Gain
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 7;
	
	  ;% quadrotor_controller_P.gp_9_Gain
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 8;
	
	  ;% quadrotor_controller_P.gp_10_Gain
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 9;
	
	  ;% quadrotor_controller_P.gp_11_Gain
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 10;
	
	  ;% quadrotor_controller_P.gp_Gain
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 11;
	
	  ;% quadrotor_controller_P.gp_1_Gain
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 12;
	
	  ;% quadrotor_controller_P.gp_2_Gain
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 13;
	
	  ;% quadrotor_controller_P.gp_3_Gain
	  section.data(12).logicalSrcIdx = 11;
	  section.data(12).dtTransOffset = 14;
	
	  ;% quadrotor_controller_P.gp_8_Gain
	  section.data(13).logicalSrcIdx = 12;
	  section.data(13).dtTransOffset = 15;
	
	  ;% quadrotor_controller_P.gp_13_Gain
	  section.data(14).logicalSrcIdx = 13;
	  section.data(14).dtTransOffset = 16;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 1;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
      sigMap.sections(nTotSects) = dumSection; %prealloc
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (quadrotor_controller_B)
    ;%
      section.nData     = 11;
      section.data(11)  = dumData; %prealloc
      
	  ;% quadrotor_controller_B.gp_6
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadrotor_controller_B.gp_7
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 8;
	
	  ;% quadrotor_controller_B.gp_9
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 9;
	
	  ;% quadrotor_controller_B.gp_10
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 12;
	
	  ;% quadrotor_controller_B.gp_11
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 13;
	
	  ;% quadrotor_controller_B.gp_
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 14;
	
	  ;% quadrotor_controller_B.gp_1
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 17;
	
	  ;% quadrotor_controller_B.gp_2
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 20;
	
	  ;% quadrotor_controller_B.gp_3
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 23;
	
	  ;% quadrotor_controller_B.gp_8
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 26;
	
	  ;% quadrotor_controller_B.gp_13
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 34;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 1;
    sectIdxOffset = 1;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (quadrotor_controller_DWork)
    ;%
      section.nData     = 11;
      section.data(11)  = dumData; %prealloc
      
	  ;% quadrotor_controller_DWork.ScopeARMAnglesIN_PWORK.LoggedData
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% quadrotor_controller_DWork.ScopeARMExtendedIN_PWORK.LoggedData
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
	  ;% quadrotor_controller_DWork.ScopePositionCommand_PWORK.LoggedData
	  section.data(3).logicalSrcIdx = 2;
	  section.data(3).dtTransOffset = 2;
	
	  ;% quadrotor_controller_DWork.ScopeYawCommand_PWORK.LoggedData
	  section.data(4).logicalSrcIdx = 3;
	  section.data(4).dtTransOffset = 3;
	
	  ;% quadrotor_controller_DWork.ScopeVelocityCommand_PWORK.LoggedData
	  section.data(5).logicalSrcIdx = 4;
	  section.data(5).dtTransOffset = 4;
	
	  ;% quadrotor_controller_DWork.ScopePosition_PWORK.LoggedData
	  section.data(6).logicalSrcIdx = 5;
	  section.data(6).dtTransOffset = 5;
	
	  ;% quadrotor_controller_DWork.ScopeAttitude_PWORK.LoggedData
	  section.data(7).logicalSrcIdx = 6;
	  section.data(7).dtTransOffset = 6;
	
	  ;% quadrotor_controller_DWork.ScopeLinealVelocity_PWORK.LoggedData
	  section.data(8).logicalSrcIdx = 7;
	  section.data(8).dtTransOffset = 7;
	
	  ;% quadrotor_controller_DWork.ScopeAngularVelocity_PWORK.LoggedData
	  section.data(9).logicalSrcIdx = 8;
	  section.data(9).dtTransOffset = 8;
	
	  ;% quadrotor_controller_DWork.ScopeARMAnglesStateIN_PWORK.LoggedData
	  section.data(10).logicalSrcIdx = 9;
	  section.data(10).dtTransOffset = 9;
	
	  ;% quadrotor_controller_DWork.ScopeJoystick_PWORK.LoggedData
	  section.data(11).logicalSrcIdx = 10;
	  section.data(11).dtTransOffset = 10;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 1732151426;
  targMap.checksum1 = 2449297721;
  targMap.checksum2 = 3212352586;
  targMap.checksum3 = 1308915177;

