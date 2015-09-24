/*
 * TransmisionDataType.h
 *
 *  Created on: 15/10/2013
 *      Author: CATEC
 */

#ifndef TRANSMISIONDATATYPE_H_
#define TRANSMISIONDATATYPE_H_

#include<ual/AutopilotGlobal_types.h>

enum ControlDataType
{
	QuadControlReferences = 1,
	ArmControlReferences =2
};

enum StateDataType
{
	QuadState = 3,
	ArmState =4
};


#pragma pack(1)

typedef struct{
	ControlDataType type;
	union{
		IControlRefArcas quad_control_references;
		IArmControlReferences arm_control_references;
	};
}ArcasUDPControlReferences;

typedef struct{
	StateDataType type;
	union{
		IUavStateArcas uav_state;
		IArmControlReferences arm_state;
	};
}ArcasUDPState;

#endif /* TRANSMISIONDATATYPE_H_ */
