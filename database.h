#ifndef _DATABASE_H_
#define _DATABASE_H_


class body_direction 
{
	struct Position
	{
		double x,y,z;
	};

	

public:
	//enum support{ RFOOT, LFOOT,  STAND};

	
	Position R_Supprots_LeftForward;
	Position R_Supprots_Forward;
	Position R_Supprots_RightForward;
	Position R_Supprots_Left;
	Position R_Supprots_Place;
	Position R_Supprots_Right;
	Position R_Supprots_LeftBackward;
	Position R_Supprots_Backward;
	Position R_Supprots_RightBackward;

	Position L_Supprots_LeftForward;
	Position L_Supprots_Forward;
	Position L_Supprots_RightForward;
	Position L_Supprots_Left;
	Position L_Supprots_Place;
	Position L_Supprots_Right;
	Position L_Supprots_LeftBackward;
	Position L_Supprots_Backward;
	Position L_Supprots_RightBackward;

	Position R_LegGesture00_Place;
	Position R_LegGesture30_Place;
	Position R_LegGesture90_Place;

	Position L_LegGesture00_Place;
	Position L_LegGesture30_Place;
	Position L_LegGesture90_Place;

public:
	body_direction()
	{
		R_Supprots_LeftForward.x=0.05;
		R_Supprots_LeftForward.y=-0.0076;
		R_Supprots_LeftForward.z=0;

		R_Supprots_Forward.x=0.05;
		R_Supprots_Forward.y=-0.0196;
		R_Supprots_Forward.z=0;

		R_Supprots_RightForward.x=0.05;
		R_Supprots_RightForward.y=-0.0316;
		R_Supprots_RightForward.z=0;

		//R_Supprots_Left.x=0.029;
		//R_Supprots_Left.y=-0.0076;
		//R_Supprots_Left.z=0;

		R_Supprots_Place.x=0.029;
		R_Supprots_Place.y=-0.0196;
		R_Supprots_Place.z=0;

		R_Supprots_Right.x=0.029;
		R_Supprots_Right.y=-0.0316;
		R_Supprots_Right.z=0;

		R_Supprots_LeftBackward.x=0.008;
		R_Supprots_LeftBackward.y=-0.0076;
		R_Supprots_LeftBackward.z=0;

		R_Supprots_Backward.x=0.008;
		R_Supprots_Backward.y=-0.0196;
		R_Supprots_Backward.z=0;

		R_Supprots_RightBackward.x=0.008;
		R_Supprots_RightBackward.y=-0.0316;
		R_Supprots_RightBackward.z=0;

//========= Left Supprots ==============
		L_Supprots_LeftForward.x=0.05;
		L_Supprots_LeftForward.y=0.0316;
		L_Supprots_LeftForward.z=0;

		L_Supprots_Forward.x=0.05;
		L_Supprots_Forward.y=0.0196;
		L_Supprots_Forward.z=0;

		L_Supprots_RightForward.x=0.05;
		L_Supprots_RightForward.y=0.0076;
		L_Supprots_RightForward.z=0;

		L_Supprots_Left.x=0.029;
		L_Supprots_Left.y=0.0316;
		L_Supprots_Left.z=0;

		L_Supprots_Place.x=0.029;
		L_Supprots_Place.y=0.0196;
		L_Supprots_Place.z=0;

		//L_Supprots_Right.x=0.029;
		//L_Supprots_Right.y=-0.0316;
		//L_Supprots_Right.z=0;

		L_Supprots_LeftBackward.x=0.008;
		L_Supprots_LeftBackward.y=00.0316;
		L_Supprots_LeftBackward.z=0;

		L_Supprots_Backward.x=0.008;
		L_Supprots_Backward.y=0.0196;
		L_Supprots_Backward.z=0;

		L_Supprots_RightBackward.x=0.008;
		L_Supprots_RightBackward.y=0.0076;
		L_Supprots_RightBackward.z=0;

		//========== Right- LegGesture - Low Level- Flexible===

		R_LegGesture00_Place.x=0.029;
		R_LegGesture00_Place.y=-0.0196;
		R_LegGesture00_Place.z=0.021;

		R_LegGesture30_Place.x=0.029;
		R_LegGesture30_Place.y=-0.0196;
		R_LegGesture30_Place.z=0.031;

		R_LegGesture90_Place.x=0.029;
		R_LegGesture90_Place.y=-0.0196;
		R_LegGesture90_Place.z=0.051;

		//========== Left- LegGesture - Low Level- Flexible===

		L_LegGesture00_Place.x=0.029;
		L_LegGesture00_Place.y=0.0196;
		L_LegGesture00_Place.z=0.021;

		L_LegGesture30_Place.x=0.029;
		L_LegGesture30_Place.y=0.0196;
		L_LegGesture30_Place.z=0.031;

		L_LegGesture90_Place.x=0.029;
		L_LegGesture90_Place.y=0.0196;
		L_LegGesture90_Place.z=0.051;


	}



};


#endif
//_DATABASE_H_
//EOF

