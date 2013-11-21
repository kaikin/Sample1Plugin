#include "SampleView.h"
#include "database.h"
#include <boost/lexical_cast.hpp>
using namespace std;
using namespace boost;
using namespace cnoid;


//================High Level=================================
void SampleWidget::KeyPoseButton1_Pushed(){
	
	MessageView::mainInstance()->putln("Pose-1 !"); 

	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle1 = bodyItems[0]->body()->joint(k)->q();
	    MessageView::mainInstance()->putln("jointAngle1[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle1));
		Vector3 JointPosition1 = bodyItems[0]->body()->joint(k)->p();
		MessageView::mainInstance()->putln("JointPosition1[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition1(0)));
		MessageView::mainInstance()->putln("JointPosition1[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition1(1)));
		MessageView::mainInstance()->putln("JointPosition1[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition1(2)));

		Vector3 landing_position = JointPosition1; 
			
	}

};




void SampleWidget::KeyPoseButton2_Pushed(){
	
	MessageView::mainInstance()->putln("Pose-2 !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle2 = bodyItems[0]->body()->joint(k)->q();
	    MessageView::mainInstance()->putln("jointAngle[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle2));
		Vector3 JointPosition2 = bodyItems[0]->body()->joint(k)->p();
		MessageView::mainInstance()->putln("JointPosition2[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition2(0)));
		MessageView::mainInstance()->putln("JointPosition2[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition2(1)));
		MessageView::mainInstance()->putln("JointPosition2[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition2(2)));

		Vector3 landing_position = JointPosition2; 
	}

};

void SampleWidget::KeyPoseButton3_Pushed(){
	
	MessageView::mainInstance()->putln("Pose-3 !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle3 = bodyItems[0]->body()->joint(k)->q();
	    MessageView::mainInstance()->putln("jointAngle3[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle3));
		Vector3 JointPosition3 = bodyItems[0]->body()->joint(k)->p();
		MessageView::mainInstance()->putln("JointPosition3[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition3(0)));
		MessageView::mainInstance()->putln("JointPosition3[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition3(1)));
		MessageView::mainInstance()->putln("JointPosition3[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition3(2)));

		Vector3 landing_position = JointPosition3; 
	
	}
};

void SampleWidget::KeyPoseButton4_Pushed(){
	
	MessageView::mainInstance()->putln("Pose-4 !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle4 = bodyItems[0]->body()->joint(k)->q();
	    MessageView::mainInstance()->putln("jointAngle4[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle4));
		Vector3 JointPosition4 = bodyItems[0]->body()->joint(k)->p();
		MessageView::mainInstance()->putln("JointPosition4[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition4(0)));
		MessageView::mainInstance()->putln("JointPosition4[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition4(1)));
		MessageView::mainInstance()->putln("JointPosition4[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition4(2)));

		Vector3 landing_position = JointPosition4; 
	}
};

void SampleWidget::KeyPoseButton5_Pushed(){
	
	MessageView::mainInstance()->putln("Pose-5 !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle5 = bodyItems[0]->body()->joint(k)->q();
	    MessageView::mainInstance()->putln("jointAngle5[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle5));
		Vector3 JointPosition5 = bodyItems[0]->body()->joint(k)->p();
		MessageView::mainInstance()->putln("JointPosition5[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition5(0)));
		MessageView::mainInstance()->putln("JointPosition5[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition5(1)));
		MessageView::mainInstance()->putln("JointPosition5[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition5(2)));

		Vector3 landing_position = JointPosition5; 
	}
};

void SampleWidget::KeyPoseButton6_Pushed(){
	
	MessageView::mainInstance()->putln("Pose-6 !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle6 = bodyItems[0]->body()->joint(k)->q();
	    MessageView::mainInstance()->putln("jointAngle6[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle6));
		Vector3 JointPosition6 = bodyItems[0]->body()->joint(k)->p();
		MessageView::mainInstance()->putln("JointPosition6[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition6(0)));
		MessageView::mainInstance()->putln("JointPosition6[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition6(1)));
		MessageView::mainInstance()->putln("JointPosition6[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition6(2)));

		Vector3 landing_position = JointPosition6; 
	}
};

void SampleWidget::KeyPoseButton7_Pushed(){
	
	MessageView::mainInstance()->putln("Pose-7 !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle7 = bodyItems[0]->body()->joint(k)->q();
	    MessageView::mainInstance()->putln("jointAngle7[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle7));
		Vector3 JointPosition7 = bodyItems[0]->body()->joint(k)->p();
		MessageView::mainInstance()->putln("JointPosition7[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition7(0)));
		MessageView::mainInstance()->putln("JointPosition7[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition7(1)));
		MessageView::mainInstance()->putln("JointPosition7[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition7(2)));

		Vector3 landing_position = JointPosition7; 
	}
};

void SampleWidget::KeyPoseButton8_Pushed(){
	
	MessageView::mainInstance()->putln("Pose-8 !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle8 = bodyItems[0]->body()->joint(k)->q();
	    MessageView::mainInstance()->putln("jointAngle8[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle8));
		Vector3 JointPosition8 = bodyItems[0]->body()->joint(k)->p();
		MessageView::mainInstance()->putln("JointPosition8[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition8(0)));
		MessageView::mainInstance()->putln("JointPosition8[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition8(1)));
		MessageView::mainInstance()->putln("JointPosition8[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition8(2)));

		Vector3 landing_position = JointPosition8; 
	}
};

void SampleWidget::KeyPoseButton9_Pushed(){
	
	MessageView::mainInstance()->putln("Pose-9 !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle9 = bodyItems[0]->body()->joint(k)->q();
	    MessageView::mainInstance()->putln("jointAngle9[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle9));
		Vector3 JointPosition9 = bodyItems[0]->body()->joint(k)->p();
		MessageView::mainInstance()->putln("JointPosition9[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition9(0)));
		MessageView::mainInstance()->putln("JointPosition9[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition9(1)));
		MessageView::mainInstance()->putln("JointPosition9[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(JointPosition9(2)));

		Vector3 landing_position = JointPosition9; 
	}
};

#include <cnoid/JointPath>
//================R_Support_Middle Level=================================
void SampleWidget::R_Support_MiddleButton1_Pushed(){
	
	MessageView::mainInstance()->putln("R_Support_Middle-LeftForward_1 !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	
	for(size_t k = 0; k < bodyItems[0]->body()->numJoints(); k++){
		double jointAngle = bodyItems[0]->body()->joint(k)->q();
		MessageView::mainInstance()->putln("jointAngle[" + lexical_cast<string>(k) + "] = " + lexical_cast<string>(jointAngle));
	}


	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	//bodyItems[0]->moveToOrigin();

	/*double default_foot_z = 0.021;
	double default_waist_z = 0.1542;*/

	/*Vector3 WaistPosition = waist->p();
	WaistPosition(2) = default_waist_z;
	body->rootLink()->T().translation() = WaistPosition;
    body->calcForwardKinematics();

	Vector3 LfootPosition = lfoot->p();
	LfootPosition(2) = default_foot_z;
	lfoot->T().translation() = LfootPosition;*/

	bodyItems[0]->notifyKinematicStateChange(false);
	bodyItems[0]->acceptKinematicStateEdit();

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);
	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_Supprots_LeftForward.x, bodyDirection.R_Supprots_LeftForward.y, originalPosition(2));
	
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	lfoot_waist_rfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);

};

void SampleWidget::R_Support_MiddleButton2_Pushed(){
	
	MessageView::mainInstance()->putln("R_Support_Middle-Forward_2 !");
	MessageView::mainInstance()->putln("changing !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");
	
	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);
	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);

	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_Supprots_Forward.x, bodyDirection.R_Supprots_Forward.y, originalPosition(2));
	
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	lfoot_waist_rfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);


};

void SampleWidget::R_Support_MiddleButton3_Pushed(){
	
	MessageView::mainInstance()->putln("R_Support_Middle-RightForward_3 !");
	
	MessageView::mainInstance()->putln("changing !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");
	
	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);

	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);

	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_Supprots_RightForward.x, bodyDirection.R_Supprots_RightForward.y, originalPosition(2));
	
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	lfoot_waist_rfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);

};

void SampleWidget::R_Support_MiddleButton4_Pushed(){
//	
//	MessageView::mainInstance()->putln("R_Support_Middle-Left_4 !");
//	MessageView::mainInstance()->putln("changing !");
//	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
//	BodyPtr body = bodyItems[0]->body();
//
//	Link* rfoot = body->link("R_ANKLE_R");
//	Link* lfoot = body->link("L_ANKLE_R");
//	Link* waist = body->link("WAIST");
//	
//	bodyItems[0]->setCurrentBaseLink(lfoot);
//	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,lfoot,rfoot);
//
//	body_direction bodyDirection;
//	Vector3 originalPosition = rfoot->p();
//	Vector3 landing_position = Vector3(bodyDirection.R_Supprots_Left.x, bodyDirection.R_Supprots_Left.y, originalPosition(2));
//	
//	lfoot_waist_rfoot->setGoal(lfoot->p(), lfoot->R(), landing_position, rfoot->R());
//	
//	lfoot_waist_rfoot->calcInverseKinematics();
//
//	bodyItems[0]->notifyKinematicStateChange(true);
};


void SampleWidget::R_Support_MiddleButton5_Pushed(){
	
	MessageView::mainInstance()->putln("R_Support_Middle-Place_5 !");
	MessageView::mainInstance()->putln("changing !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");
	
	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);
	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);

	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_Supprots_Place.x, bodyDirection.R_Supprots_Place.y, originalPosition(2));
	
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	lfoot_waist_rfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::R_Support_MiddleButton6_Pushed(){
	
	MessageView::mainInstance()->putln("R_Support_Middle-Right_6 !");
	MessageView::mainInstance()->putln("changing !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");
	
	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);
	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);

	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_Supprots_Right.x, bodyDirection.R_Supprots_Right.y, originalPosition(2));
	
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	lfoot_waist_rfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::R_Support_MiddleButton7_Pushed(){
	
	MessageView::mainInstance()->putln("R_Support_Middle-LeftBackward_7 !");
	MessageView::mainInstance()->putln("changing !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");
	
	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);
	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);

	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_Supprots_LeftBackward.x, bodyDirection.R_Supprots_LeftBackward.y, originalPosition(2));
	
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	lfoot_waist_rfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::R_Support_MiddleButton8_Pushed(){
	
	MessageView::mainInstance()->putln("R_Support_Middle-Backward_8 !");
	MessageView::mainInstance()->putln("changing !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");
	
	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);
	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);

	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_Supprots_Backward.x, bodyDirection.R_Supprots_Backward.y, originalPosition(2));
	
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	lfoot_waist_rfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::R_Support_MiddleButton9_Pushed(){
	
	MessageView::mainInstance()->putln("R_Support_Middle-RightBackward_9 !");
	MessageView::mainInstance()->putln("changing !");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");
	
	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);
	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);

	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_Supprots_RightBackward.x, bodyDirection.R_Supprots_RightBackward.y, originalPosition(2));
	
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	lfoot_waist_rfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};


//================L_Support_Middle Level=================================
void SampleWidget::L_Support_MiddleButton1_Pushed(){
	
	MessageView::mainInstance()->putln("L_Support_Middle-LeftForward_1 !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	//bodyItems[0]->moveToOrigin();

	/*double default_foot_z = 0.021;
	double default_waist_z = 0.1542;*/

	/*Vector3 WaistPosition = waist->p();
	WaistPosition(2) = default_waist_z;
	body->rootLink()->T().translation() = WaistPosition;
    body->calcForwardKinematics();

	Vector3 LfootPosition = lfoot->p();
	LfootPosition(2) = default_foot_z;
	lfoot->T().translation() = LfootPosition;*/

	bodyItems[0]->notifyKinematicStateChange(false);
	bodyItems[0]->acceptKinematicStateEdit();

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);
	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_Supprots_LeftForward.x, bodyDirection.L_Supprots_LeftForward.y, originalPosition(2));
	
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	rfoot_waist_lfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::L_Support_MiddleButton2_Pushed(){
	
	MessageView::mainInstance()->putln("L_Support_Middle-Forward_2 !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);
	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_Supprots_Forward.x, bodyDirection.L_Supprots_Forward.y, originalPosition(2));
	
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	rfoot_waist_lfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::L_Support_MiddleButton3_Pushed(){
	
	MessageView::mainInstance()->putln("L_Support_Middle-RightForward_1 !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);
	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_Supprots_RightForward.x, bodyDirection.L_Supprots_RightForward.y, originalPosition(2));
	
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	rfoot_waist_lfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::L_Support_MiddleButton4_Pushed(){
	
	MessageView::mainInstance()->putln("L_Support_Middle-Left_4 !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);
	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_Supprots_Left.x, bodyDirection.L_Supprots_Left.y, originalPosition(2));
	
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	rfoot_waist_lfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};
void SampleWidget::L_Support_MiddleButton5_Pushed(){
	
	MessageView::mainInstance()->putln("L_Support_Middle-Place_5 !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);
	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_Supprots_Place.x, bodyDirection.L_Supprots_Place.y, originalPosition(2));
	
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	rfoot_waist_lfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::L_Support_MiddleButton6_Pushed(){
	
	//MessageView::mainInstance()->putln("L_Support_Middle-Right_6 !");
	
};

void SampleWidget::L_Support_MiddleButton7_Pushed(){
	
	MessageView::mainInstance()->putln("L_Support_Middle-LeftBackward_7 !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);
	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_Supprots_LeftBackward.x, bodyDirection.L_Supprots_LeftBackward.y, originalPosition(2));
	
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	rfoot_waist_lfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::L_Support_MiddleButton8_Pushed(){
	
	MessageView::mainInstance()->putln("L_Support_Middle-Backward_8 !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);
	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_Supprots_Backward.x, bodyDirection.L_Supprots_Backward.y, originalPosition(2));
	
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	rfoot_waist_lfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

void SampleWidget::L_Support_MiddleButton9_Pushed(){
	
	MessageView::mainInstance()->putln("L_Support_Middle-RightBackward_9 !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);
	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_Supprots_RightBackward.x, bodyDirection.L_Supprots_RightBackward.y, originalPosition(2));
	
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	rfoot_waist_lfoot->calcInverseKinematics();

	bodyItems[0]->notifyKinematicStateChange(true);
};

//===============Right LegGesture Low-level Flexible=======
void SampleWidget::R_LegGesture00_LowButton0_Pushed(){
	
	MessageView::mainInstance()->putln("R_LegGesture00_LowButton0_Pushed !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);

	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_LegGesture00_Place.x, bodyDirection.R_LegGesture00_Place.y, bodyDirection.R_LegGesture00_Place.z);
	
	//debug
	MessageView::mainInstance()->putln("moving to (" + lexical_cast<string>(landing_position(0)) + ", " + lexical_cast<string>(landing_position(1)) + ", "+ lexical_cast<string>(landing_position(2)) + ")");
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	bool calclationIsSolved = lfoot_waist_rfoot->calcInverseKinematics();
	if(calclationIsSolved){
		MessageView::mainInstance()->putln("calc solved");
	}else{
		MessageView::mainInstance()->putln("calc failed");
	}

	bodyItems[0]->notifyKinematicStateChange(true);

};


void SampleWidget::R_LegGesture30_LowButton1_Pushed(){
	
	MessageView::mainInstance()->putln("R_LegGesture30_LowButton5_Pushed !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);

	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_LegGesture30_Place.x, bodyDirection.R_LegGesture30_Place.y, bodyDirection.R_LegGesture30_Place.z);
	
	//debug
	MessageView::mainInstance()->putln("moving to (" + lexical_cast<string>(landing_position(0)) + ", " + lexical_cast<string>(landing_position(1)) + ", "+ lexical_cast<string>(landing_position(2)) + ")");
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	bool calclationIsSolved = lfoot_waist_rfoot->calcInverseKinematics();
	if(calclationIsSolved){
		MessageView::mainInstance()->putln("calc solved");
	}else{
		MessageView::mainInstance()->putln("calc failed");
	}

	bodyItems[0]->notifyKinematicStateChange(true);

};

void SampleWidget::R_LegGesture90_LowButton3_Pushed(){
	
	MessageView::mainInstance()->putln("R_LegGesture90_LowButton3_Pushed !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(lfoot);

	JointPathPtr lfoot_waist_rfoot = getCustomJointPath(body,waist,rfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = rfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.R_LegGesture90_Place.x, bodyDirection.R_LegGesture90_Place.y, bodyDirection.R_LegGesture90_Place.z);
	
	//debug
	MessageView::mainInstance()->putln("moving to (" + lexical_cast<string>(landing_position(0)) + ", " + lexical_cast<string>(landing_position(1)) + ", "+ lexical_cast<string>(landing_position(2)) + ")");
	lfoot_waist_rfoot->setGoal(waist->p(), waist->R(), landing_position, rfoot->R());
	
	bool calclationIsSolved = lfoot_waist_rfoot->calcInverseKinematics();
	if(calclationIsSolved){
		MessageView::mainInstance()->putln("calc solved");
	}else{
		MessageView::mainInstance()->putln("calc failed");
	}

	bodyItems[0]->notifyKinematicStateChange(true);

};


//===============Left LegGesture Low-level Flexible=======
void SampleWidget::L_LegGesture00_LowButton0_Pushed(){
	
	MessageView::mainInstance()->putln("L_LegGesture00_LowButton0_Pushed !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);

	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_LegGesture00_Place.x, bodyDirection.L_LegGesture00_Place.y, bodyDirection.L_LegGesture00_Place.z);
	
	//debug
	MessageView::mainInstance()->putln("moving to (" + lexical_cast<string>(landing_position(0)) + ", " + lexical_cast<string>(landing_position(1)) + ", "+ lexical_cast<string>(landing_position(2)) + ")");
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	bool calclationIsSolved = rfoot_waist_lfoot->calcInverseKinematics();
	if(calclationIsSolved){
		MessageView::mainInstance()->putln("calc solved");
	}else{
		MessageView::mainInstance()->putln("calc failed");
	}

	bodyItems[0]->notifyKinematicStateChange(true);

};


void SampleWidget::L_LegGesture30_LowButton1_Pushed(){
	
	MessageView::mainInstance()->putln("L_LegGesture30_LowButton5_Pushed !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);

	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_LegGesture30_Place.x, bodyDirection.L_LegGesture30_Place.y, bodyDirection.L_LegGesture30_Place.z);
	
	//debug
	MessageView::mainInstance()->putln("moving to (" + lexical_cast<string>(landing_position(0)) + ", " + lexical_cast<string>(landing_position(1)) + ", "+ lexical_cast<string>(landing_position(2)) + ")");
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	bool calclationIsSolved = rfoot_waist_lfoot->calcInverseKinematics();
	if(calclationIsSolved){
		MessageView::mainInstance()->putln("calc solved");
	}else{
		MessageView::mainInstance()->putln("calc failed");
	}

	bodyItems[0]->notifyKinematicStateChange(true);

};

void SampleWidget::L_LegGesture90_LowButton3_Pushed(){
	
	MessageView::mainInstance()->putln("L_LegGesture90_LowButton3_Pushed !");
	MessageView::mainInstance()->putln("Changing Body positions");
	ItemList<BodyItem> bodyItems = ItemTreeView::mainInstance()->selectedItems<BodyItem>();
	

	BodyPtr body = bodyItems[0]->body();

	Link* rfoot = body->link("R_ANKLE_R");
	Link* lfoot = body->link("L_ANKLE_R");
	Link* waist = body->link("WAIST");

	bodyItems[0]->setCurrentBaseLink(waist);
	bodyItems[0]->setCurrentBaseLink(rfoot);

	JointPathPtr rfoot_waist_lfoot = getCustomJointPath(body,waist,lfoot);
	
	body_direction bodyDirection;
	Vector3 originalPosition = lfoot->p();
	Vector3 landing_position = Vector3(bodyDirection.L_LegGesture90_Place.x, bodyDirection.L_LegGesture90_Place.y, bodyDirection.L_LegGesture90_Place.z);
	
	//debug
	MessageView::mainInstance()->putln("moving to (" + lexical_cast<string>(landing_position(0)) + ", " + lexical_cast<string>(landing_position(1)) + ", "+ lexical_cast<string>(landing_position(2)) + ")");
	rfoot_waist_lfoot->setGoal(waist->p(), waist->R(), landing_position, lfoot->R());
	
	bool calclationIsSolved = rfoot_waist_lfoot->calcInverseKinematics();
	if(calclationIsSolved){
		MessageView::mainInstance()->putln("calc solved");
	}else{
		MessageView::mainInstance()->putln("calc failed");
	}

	bodyItems[0]->notifyKinematicStateChange(true);

};


SampleWidget::SampleWidget()
{

    QVBoxLayout* TopLayout = new QVBoxLayout();

//=====================KeyPose group========================
	KeyPoseButton1.setText("Pose_1");
	KeyPoseButton2.setText("Pose_2");
	KeyPoseButton3.setText("Pose_3");
	KeyPoseButton4.setText("Pose_4");
	KeyPoseButton5.setText("Pose_5");
	KeyPoseButton6.setText("Pose_6");
	KeyPoseButton7.setText("Pose_7");
	KeyPoseButton8.setText("Pose_8");
	KeyPoseButton9.setText("Pose_9");

	KeyPoseButton1.sigClicked().connect(bind(&SampleWidget::KeyPoseButton1_Pushed, this));
	KeyPoseButton2.sigClicked().connect(bind(&SampleWidget::KeyPoseButton2_Pushed, this));
	KeyPoseButton3.sigClicked().connect(bind(&SampleWidget::KeyPoseButton3_Pushed, this));
	KeyPoseButton4.sigClicked().connect(bind(&SampleWidget::KeyPoseButton4_Pushed, this));
	KeyPoseButton5.sigClicked().connect(bind(&SampleWidget::KeyPoseButton5_Pushed, this));
	KeyPoseButton6.sigClicked().connect(bind(&SampleWidget::KeyPoseButton6_Pushed, this));
	KeyPoseButton7.sigClicked().connect(bind(&SampleWidget::KeyPoseButton7_Pushed, this));
	KeyPoseButton8.sigClicked().connect(bind(&SampleWidget::KeyPoseButton8_Pushed, this));
	KeyPoseButton9.sigClicked().connect(bind(&SampleWidget::KeyPoseButton9_Pushed, this));




	QGroupBox* group1 = new QGroupBox();
	group1->setTitle("Setting Pose ");
	group1->setAlignment(Qt::AlignCenter);
	QVBoxLayout *Hvl1 = new QVBoxLayout();
	QHBoxLayout *Hhl1 = new QHBoxLayout();
	QHBoxLayout *Hhl2 = new QHBoxLayout();
	QHBoxLayout *Hhl3 = new QHBoxLayout();

	Hhl1->addWidget(&KeyPoseButton1,false,Qt::AlignCenter);
	Hhl1->addWidget(&KeyPoseButton2,false,Qt::AlignCenter);
	Hhl1->addWidget(&KeyPoseButton3,false,Qt::AlignCenter);
	Hvl1->addLayout(Hhl1);

	Hhl2->addWidget(&KeyPoseButton4,false,Qt::AlignCenter);
	Hhl2->addWidget(&KeyPoseButton5,false,Qt::AlignCenter);
	Hhl2->addWidget(&KeyPoseButton6,false,Qt::AlignCenter);
	Hvl1->addLayout(Hhl2);

	Hhl3->addWidget(&KeyPoseButton7,false,Qt::AlignCenter);
	Hhl3->addWidget(&KeyPoseButton8,false,Qt::AlignCenter);
	Hhl3->addWidget(&KeyPoseButton9,false,Qt::AlignCenter);
	Hvl1->addLayout(Hhl3);

	group1->setLayout(Hvl1);
	TopLayout->addWidget(group1);




//=====================R_Support_Middle level group========================

	R_Support_MiddleButton1.setText("LeftForward_1");
	R_Support_MiddleButton2.setText("Forward_2");
	R_Support_MiddleButton3.setText("RightForward_3");
	R_Support_MiddleButton4.setText("  ");
	R_Support_MiddleButton5.setText("Place_5");
	R_Support_MiddleButton6.setText("Right_6");
	R_Support_MiddleButton7.setText("LeftBackward_7");
	R_Support_MiddleButton8.setText("Backward_8");
	R_Support_MiddleButton9.setText("RightBackward_9");

	R_Support_MiddleButton1.sigClicked().connect(bind(&SampleWidget::R_Support_MiddleButton1_Pushed, this));
	R_Support_MiddleButton2.sigClicked().connect(bind(&SampleWidget::R_Support_MiddleButton2_Pushed, this));
	R_Support_MiddleButton3.sigClicked().connect(bind(&SampleWidget::R_Support_MiddleButton3_Pushed, this));
	R_Support_MiddleButton4.sigClicked().connect(bind(&SampleWidget::R_Support_MiddleButton4_Pushed, this));
	R_Support_MiddleButton5.sigClicked().connect(bind(&SampleWidget::R_Support_MiddleButton5_Pushed, this));
	R_Support_MiddleButton6.sigClicked().connect(bind(&SampleWidget::R_Support_MiddleButton6_Pushed, this));
	R_Support_MiddleButton7.sigClicked().connect(bind(&SampleWidget::R_Support_MiddleButton7_Pushed, this));
	R_Support_MiddleButton8.sigClicked().connect(bind(&SampleWidget::R_Support_MiddleButton8_Pushed, this));
	R_Support_MiddleButton9.sigClicked().connect(bind(&SampleWidget::R_Support_MiddleButton9_Pushed, this));


	QGroupBox* group2 = new QGroupBox();
	group2->setTitle("R_Support_Middle_Level_Directions ");
	group2->setAlignment(Qt::AlignCenter);
	QVBoxLayout *Mvl1 = new QVBoxLayout();
	QHBoxLayout *Mhl1 = new QHBoxLayout();
	QHBoxLayout *Mhl2 = new QHBoxLayout();
	QHBoxLayout *Mhl3 = new QHBoxLayout();

	Mhl1->addWidget(&R_Support_MiddleButton1,false,Qt::AlignCenter);
	Mhl1->addWidget(&R_Support_MiddleButton2,false,Qt::AlignCenter);
	Mhl1->addWidget(&R_Support_MiddleButton3,false,Qt::AlignCenter);
	Mvl1->addLayout(Mhl1);

	Mhl2->addWidget(&R_Support_MiddleButton4,false,Qt::AlignCenter);
	Mhl2->addWidget(&R_Support_MiddleButton5,false,Qt::AlignCenter);
	Mhl2->addWidget(&R_Support_MiddleButton6,false,Qt::AlignCenter);
	Mvl1->addLayout(Mhl2);

	Mhl3->addWidget(&R_Support_MiddleButton7,false,Qt::AlignCenter);
	Mhl3->addWidget(&R_Support_MiddleButton8,false,Qt::AlignCenter);
	Mhl3->addWidget(&R_Support_MiddleButton9,false,Qt::AlignCenter);
	Mvl1->addLayout(Mhl3);

	group2->setLayout(Mvl1);
	TopLayout->addWidget(group2);


	TopLayout->addStretch();
	this->setLayout(TopLayout);

//=====================L_Support_Middle Level group========================
	L_Support_MiddleButton1.setText("LeftForward_1");
	L_Support_MiddleButton2.setText("Forward_2");
	L_Support_MiddleButton3.setText("RightForward_3");
	L_Support_MiddleButton4.setText("Left_4");
	L_Support_MiddleButton5.setText("Place_5");
	L_Support_MiddleButton6.setText("  ");
	L_Support_MiddleButton7.setText("LeftBackward_7");
	L_Support_MiddleButton8.setText("Backward_8");
	L_Support_MiddleButton9.setText("RightBackward_9");

	L_Support_MiddleButton1.sigClicked().connect(bind(&SampleWidget::L_Support_MiddleButton1_Pushed, this));
	L_Support_MiddleButton2.sigClicked().connect(bind(&SampleWidget::L_Support_MiddleButton2_Pushed, this));
	L_Support_MiddleButton3.sigClicked().connect(bind(&SampleWidget::L_Support_MiddleButton3_Pushed, this));
	L_Support_MiddleButton4.sigClicked().connect(bind(&SampleWidget::L_Support_MiddleButton4_Pushed, this));
	L_Support_MiddleButton5.sigClicked().connect(bind(&SampleWidget::L_Support_MiddleButton5_Pushed, this));
	L_Support_MiddleButton6.sigClicked().connect(bind(&SampleWidget::L_Support_MiddleButton6_Pushed, this));
	L_Support_MiddleButton7.sigClicked().connect(bind(&SampleWidget::L_Support_MiddleButton7_Pushed, this));
	L_Support_MiddleButton8.sigClicked().connect(bind(&SampleWidget::L_Support_MiddleButton8_Pushed, this));
	L_Support_MiddleButton9.sigClicked().connect(bind(&SampleWidget::L_Support_MiddleButton9_Pushed, this));

	QGroupBox* group3 = new QGroupBox();
	group3->setTitle("L_Support_Middle_Level_Directions ");
	group3->setAlignment(Qt::AlignCenter);
	QVBoxLayout *Lvl1 = new QVBoxLayout();
	QHBoxLayout *Lhl1 = new QHBoxLayout();
	QHBoxLayout *Lhl2 = new QHBoxLayout();
	QHBoxLayout *Lhl3 = new QHBoxLayout();

	Lhl1->addWidget(&L_Support_MiddleButton1,false,Qt::AlignCenter);
	Lhl1->addWidget(&L_Support_MiddleButton2,false,Qt::AlignCenter);
	Lhl1->addWidget(&L_Support_MiddleButton3,false,Qt::AlignCenter);
	Lvl1->addLayout(Lhl1);

	Lhl2->addWidget(&L_Support_MiddleButton4,false,Qt::AlignCenter);
	Lhl2->addWidget(&L_Support_MiddleButton5,false,Qt::AlignCenter);
	Lhl2->addWidget(&L_Support_MiddleButton6,false,Qt::AlignCenter);
	Lvl1->addLayout(Lhl2);

	Lhl3->addWidget(&L_Support_MiddleButton7,false,Qt::AlignCenter);
	Lhl3->addWidget(&L_Support_MiddleButton8,false,Qt::AlignCenter);
	Lhl3->addWidget(&L_Support_MiddleButton9,false,Qt::AlignCenter);
	Lvl1->addLayout(Lhl3);

	group3->setLayout(Lvl1);
	TopLayout->addWidget(group3);

	//=================Right-LegGesture Low-level Flexible=======
	
	R_LegGesture00_LowButton0.setText("Flex 0");
	R_LegGesture30_LowButton1.setText("Flex 30");
	R_LegGesture90_LowButton3.setText("Flex 90");

	R_LegGesture00_LowButton0.sigClicked().connect(bind(&SampleWidget::R_LegGesture00_LowButton0_Pushed, this));
	R_LegGesture30_LowButton1.sigClicked().connect(bind(&SampleWidget::R_LegGesture30_LowButton1_Pushed, this));
	R_LegGesture90_LowButton3.sigClicked().connect(bind(&SampleWidget::R_LegGesture90_LowButton3_Pushed, this));
	

	QGroupBox* group4 = new QGroupBox();
	group4->setTitle("R_LegGesture LowLevel Flexible");
	group4->setAlignment(Qt::AlignCenter);
	QVBoxLayout *FRLvl1 = new QVBoxLayout();
	QHBoxLayout *FRLhl1 = new QHBoxLayout();
	/*QHBoxLayout *Mhl2 = new QHBoxLayout();
	QHBoxLayout *Mhl3 = new QHBoxLayout();*/

	FRLhl1->addWidget(&R_LegGesture00_LowButton0,false,Qt::AlignCenter);
	FRLhl1->addWidget(&R_LegGesture30_LowButton1,false,Qt::AlignCenter);
	FRLhl1->addWidget(&R_LegGesture90_LowButton3,false,Qt::AlignCenter);
	//Mhl1->addWidget(&R_Support_MiddleButton3,false,Qt::AlignCenter);
	FRLvl1->addLayout(FRLhl1);

	/*Mhl2->addWidget(&R_Support_MiddleButton4,false,Qt::AlignCenter);
	Mhl2->addWidget(&R_Support_MiddleButton5,false,Qt::AlignCenter);
	Mhl2->addWidget(&R_Support_MiddleButton6,false,Qt::AlignCenter);
	Mvl1->addLayout(Mhl2);

	Mhl3->addWidget(&R_Support_MiddleButton7,false,Qt::AlignCenter);
	Mhl3->addWidget(&R_Support_MiddleButton8,false,Qt::AlignCenter);
	Mhl3->addWidget(&R_Support_MiddleButton9,false,Qt::AlignCenter);
	Mvl1->addLayout(Mhl3);*/

	group4->setLayout(FRLvl1);
	TopLayout->addWidget(group4);


	TopLayout->addStretch();
	this->setLayout(TopLayout);

 //=================Left-LegGesture Low-level Flexible=======
	L_LegGesture00_LowButton0.setText("Flex 0");
	L_LegGesture30_LowButton1.setText("Flex 30");
	L_LegGesture90_LowButton3.setText("Flex 90");

	L_LegGesture00_LowButton0.sigClicked().connect(bind(&SampleWidget::L_LegGesture00_LowButton0_Pushed, this));
	L_LegGesture30_LowButton1.sigClicked().connect(bind(&SampleWidget::L_LegGesture30_LowButton1_Pushed, this));
	L_LegGesture90_LowButton3.sigClicked().connect(bind(&SampleWidget::L_LegGesture90_LowButton3_Pushed, this));
	

	QGroupBox* group5 = new QGroupBox();
	group5->setTitle("L_LegGesture LowLevel Flexible");
	group5->setAlignment(Qt::AlignCenter);
	QVBoxLayout *FLLvl1 = new QVBoxLayout();
	QHBoxLayout *FLLhl1 = new QHBoxLayout();
	/*QHBoxLayout *Mhl2 = new QHBoxLayout();
	QHBoxLayout *Mhl3 = new QHBoxLayout();*/

	FLLhl1->addWidget(&L_LegGesture00_LowButton0,false,Qt::AlignCenter);
	FLLhl1->addWidget(&L_LegGesture30_LowButton1,false,Qt::AlignCenter);
	FLLhl1->addWidget(&L_LegGesture90_LowButton3,false,Qt::AlignCenter);
	//Mhl1->addWidget(&R_Support_MiddleButton3,false,Qt::AlignCenter);
	FLLvl1->addLayout(FLLhl1);

	/*Mhl2->addWidget(&R_Support_MiddleButton4,false,Qt::AlignCenter);
	Mhl2->addWidget(&R_Support_MiddleButton5,false,Qt::AlignCenter);
	Mhl2->addWidget(&R_Support_MiddleButton6,false,Qt::AlignCenter);
	Mvl1->addLayout(Mhl2);

	Mhl3->addWidget(&R_Support_MiddleButton7,false,Qt::AlignCenter);
	Mhl3->addWidget(&R_Support_MiddleButton8,false,Qt::AlignCenter);
	Mhl3->addWidget(&R_Support_MiddleButton9,false,Qt::AlignCenter);
	Mvl1->addLayout(Mhl3);*/

	group5->setLayout(FLLvl1);
	TopLayout->addWidget(group5);


	TopLayout->addStretch();
	this->setLayout(TopLayout);
 

}


SampleView::SampleView()
{
    setName("Laban_Symbols");
	setDefaultLayoutArea(View::CENTER);
	sampleWidget = new SampleWidget();
	QVBoxLayout* TopQVBox = new QVBoxLayout();

    scrollArea.setFrameShape(QFrame::NoFrame);
	scrollArea.setWidget(sampleWidget);
	scrollArea.setWidgetResizable(true);

	TopQVBox->addWidget(&scrollArea);
	setLayout(TopQVBox);

}
