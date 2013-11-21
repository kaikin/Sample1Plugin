#ifndef _SAMPLEVIEW_H_
#define _SAMPLEVIEW_H_
#include <cnoid/MessageView>
#include <vector>
#include <cnoid/View>
#include <cnoid/MainWindow>
#include <cnoid/SpinBox>
#include <cnoid/ComboBox>
#include <cnoid/Button>
#include <cnoid/TimeBar>
#include <cnoid/SceneGraph>
#include <cnoid/ScenePieces>
#include <cnoid/SceneView>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include <cnoid/MessageView>
#include <QBoxLayout>
#include <QScrollArea>
#include <QGroupBox>
#include <QFileDialog>
#include <boost/signals.hpp>
#include <boost/bind.hpp>
using namespace std;
using namespace boost;
using namespace cnoid;



class SampleWidget :  public QWidget 
{
public:
	SampleWidget();
	void KeyPoseButton1_Pushed();
	void KeyPoseButton2_Pushed();
	void KeyPoseButton3_Pushed();
	void KeyPoseButton4_Pushed();
	void KeyPoseButton5_Pushed();
	void KeyPoseButton6_Pushed();
	void KeyPoseButton7_Pushed();
	void KeyPoseButton8_Pushed();
	void KeyPoseButton9_Pushed();

	void KeyPoseButton1_DoublePushed();

	void R_Support_MiddleButton1_Pushed();
	void R_Support_MiddleButton2_Pushed();
	void R_Support_MiddleButton3_Pushed();
	void R_Support_MiddleButton4_Pushed();
	void R_Support_MiddleButton5_Pushed();
	void R_Support_MiddleButton6_Pushed();
	void R_Support_MiddleButton7_Pushed();
	void R_Support_MiddleButton8_Pushed();
	void R_Support_MiddleButton9_Pushed();

	void L_Support_MiddleButton1_Pushed();
	void L_Support_MiddleButton2_Pushed();
	void L_Support_MiddleButton3_Pushed();
	void L_Support_MiddleButton4_Pushed();
	void L_Support_MiddleButton5_Pushed();
	void L_Support_MiddleButton6_Pushed();
	void L_Support_MiddleButton7_Pushed();
	void L_Support_MiddleButton8_Pushed();
	void L_Support_MiddleButton9_Pushed();

	void R_LegGesture00_LowButton0_Pushed();
	void R_LegGesture30_LowButton1_Pushed();
	void R_LegGesture90_LowButton3_Pushed();
	
	void L_LegGesture00_LowButton0_Pushed();
	void L_LegGesture30_LowButton1_Pushed();
	void L_LegGesture90_LowButton3_Pushed();

///*
//	bool changeBaseLink(){
//	     ItemList<BodyItem> bodyItems = 
//            ItemTreeView::mainInstance()->selectedItems<BodyItem>(); 
//            BodyPtr body = bodyItems[0]->body(); 
//			Link*  rfoot =  body->link("R_ANKLE_R");
//			Link*  lfoot =  body->link("L_ANKLE_R");
//
//			double defaultFootHeight = 0.1527;
//
//			double heightofRFOOT = rfoot->p().z();
//			double heightofLFOOT = lfoot->p().z();
//
//			if(heightofRFOOT){
//
//				lfoot->p().z() = defaultFootHeight;
//
//			}else{
//
//
//			}
//
//
//
//
//	}*/

	bool changeBaseLink_toRfoot(){
        ItemList<BodyItem> bodyItems = 
            ItemTreeView::mainInstance()->selectedItems<BodyItem>(); 
  
            BodyPtr body = bodyItems[0]->body(); 
			Link*  rfoot =  body->link("R_ANKLE_R");
			bodyItems[0]->setCurrentBaseLink(rfoot);
			 bodyItems[0]->notifyKinematicStateChange(true);
		return true;
	}

	bool changeBaseLink_toLfoot(){
        ItemList<BodyItem> bodyItems = 
            ItemTreeView::mainInstance()->selectedItems<BodyItem>(); 
  
            BodyPtr body = bodyItems[0]->body(); 
			Link*  lfoot =  body->link("L_ANKLE_R");
			bodyItems[0]->setCurrentBaseLink(lfoot);
			bodyItems[0]->notifyKinematicStateChange(true);
		return true;
	}

	bool ButtonPushed(int poseNumber){

		if(_poseSeq.size() < poseNumber) return false;
		//if(poseNumber != 1 || poseNumber != 2 || poseNumber != 5 || poseNumber != 6 ){
		//	changeBaseLink_toLfoot();
		//}else{
		//	changeBaseLink_toRfoot();
		//}

        ItemList<BodyItem> bodyItems = 
            ItemTreeView::mainInstance()->selectedItems<BodyItem>(); 
  
            BodyPtr body = bodyItems[0]->body(); 
            for(int j=0; j < body->numJoints(); ++j){
                body->joint(j)->q() = _poseSeq[poseNumber][j]; 
            }
            bodyItems[0]->notifyKinematicStateChange(true);
     
		return true;	
	
	}
	//bool loadPoses(){

	//	_poseSeq.clear();

	//
	//	double pose0[20] = {-3.7744917e-008, 0.0361667472, 0.155604296, -0.67986773, -0.52426354, 0.0361667582, 6.20740952e-009, -0.0565271089, -0.567735357, 
 //       0.335194938, -0.2325404, -0.0565271144, 0, 0, 0, -0.523598776, 0, 0, 
 //       0.523598776, 0}; 

	//	vector<double> pose0v;
	//	for(size_t k  = 0; k < 20; k++)
	//		pose0v.push_back(pose0[k]);
	//	_poseSeq.push_back(pose0v);


	//	double pose1[20] ={-0.192660186, 0.261169384, 0.491534014, -1.45728135, -0.965747456, 0.261169409, -3.18412527e-008, 0.233962073, -0.445698707, 
 //       0.492604819, 0.0469061588, 0.233962065, 0, 0, 0, -0.523598776, 0, 0, 
 //       0.523598776, 0}; 

	//	vector<double> pose1v;
	//	for(size_t k  = 0; k < 20; k++)
	//		pose1v.push_back(pose1[k]);
	//	_poseSeq.push_back(pose1v);

	//	double pose2[20] ={ -0.25975638, -0.0955523858, 0.714136321, -0.712824932, 0.00131127022, -0.0955523678, 6.44733168e-009, -0.0814893505, -0.432632761, 
 //       0.939680644, 0.507047907, -0.0814893566, 0, 0, 0, -0.523598776, 0, 0, 
 //       0.523598776, 0}; 

	//	vector<double> pose2v;
	//	for(size_t k  = 0; k < 20; k++)
	//		pose2v.push_back(pose2[k]);
	//	_poseSeq.push_back(pose2v);

	//	double pose3[20] = { -0.259756351, -0.0891860207, 0.421587511, -0.757635642, -0.336048283, -0.0891859992, -0.156369525, -0.0530658208, -1.01456374, 
 //       1.1448666, 0.130302925, -0.0530658416, 0, 0, 0, -0.523598776, 0, 0, 
 //       0.523598776, 0}; 

	//	vector<double> pose3v;
	//	for(size_t k  = 0; k < 20; k++)
	//		pose3v.push_back(pose3[k]);
	//	_poseSeq.push_back(pose3v);


	//	double pose4[20] = {-0.259756367, -0.122004652, 0.458523265, -1.08148357, -0.622960434, -0.122004633, -0.156381454, -0.070498502, -0.692636731, 
 //       0.269408601, -0.423228038, -0.0704985233, 0, 0, 0, -0.523598776, 0, 0, 
 //       0.523598776, 0}; 

	//	vector<double> pose4v;
	//	for(size_t k  = 0; k < 20; k++)
	//		pose4v.push_back(pose4[k]);
	//	_poseSeq.push_back(pose4v);


	//	double pose5[20] = {-0.150410565, 0.128914435, 0.817119324, -1.43372203, -0.616602864, 0.128914461, -0.156381478, 0.133390565, -0.723802323, 
 //       0.959578723, 0.2357765, 0.133390543, 0, 0, 0, -0.523598776, 0, 0, 
 //       0.523598776, 0 }; 

	//	vector<double> pose5v;
	//	for(size_t k  = 0; k < 20; k++)
	//		pose5v.push_back(pose5[k]);
	//	_poseSeq.push_back(pose5v);

	//	double pose6[20] ={-0.0719613582, 0.020201763, 0.512896065, -1.22324199, -0.710346118, 0.0202017957, -0.15638146, -0.0457439213, -0.953370212, 
 //       0.796101463, -0.157268643, -0.0457439456, 0, 0, 0, -0.523598776, 0, 0, 
 //       0.523598776, 0 }; 

	//	vector<double> pose6v;
	//	for(size_t k  = 0; k < 20; k++)
	//		pose6v.push_back(pose6[k]);
	//	_poseSeq.push_back(pose6v);

	//	double pose7[20] ={-0.0719612909, -0.275522507, 0.524798287, -0.962790591, -0.437992511, -0.275522474, 4.16679427e-008, -0.341542812, -0.952026152, 
 //       1.37661417, 0.424588144, -0.341542842, 0, 0, 0, -0.523598776, 0, 0, 
 //       0.523598776, 0}; 

	//	vector<double> pose7v;
	//	for(size_t k  = 0; k < 20; k++)
	//		pose7v.push_back(pose7[k]);
	//	_poseSeq.push_back(pose7v);

	//	double pose8[20] ={-0.0719613389, -0.0565057514, 0.74439034, -0.893191808, -0.148801666, -0.0565057188, 3.95430048e-008, -0.125554992, -0.391624848, 
 //       0.97559267, 0.583967974, -0.125555026, 0, 0, 0, -0.523598776, 0, 0, 
 //       0.523598776, 0}; 

	//	vector<double> pose8v;
	//	for(size_t k  = 0; k < 20; k++)
	//		pose8v.push_back(pose8[k]);
	//	_poseSeq.push_back(pose8v);

	//	return true;

	//}
	
private:
	vector< vector<double> > _poseSeq;

	PushButton KeyPoseButton1;
	PushButton KeyPoseButton2;
	PushButton KeyPoseButton3;
	PushButton KeyPoseButton4;
	PushButton KeyPoseButton5;
	PushButton KeyPoseButton6;
	PushButton KeyPoseButton7;
	PushButton KeyPoseButton8;
	PushButton KeyPoseButton9;

	PushButton R_Support_MiddleButton1;
	PushButton R_Support_MiddleButton2;
	PushButton R_Support_MiddleButton3;
	PushButton R_Support_MiddleButton4;
	PushButton R_Support_MiddleButton5;
	PushButton R_Support_MiddleButton6;
	PushButton R_Support_MiddleButton7;
	PushButton R_Support_MiddleButton8;
	PushButton R_Support_MiddleButton9;

	PushButton L_Support_MiddleButton1;
	PushButton L_Support_MiddleButton2;
	PushButton L_Support_MiddleButton3;
	PushButton L_Support_MiddleButton4;
	PushButton L_Support_MiddleButton5;
	PushButton L_Support_MiddleButton6;
	PushButton L_Support_MiddleButton7;
	PushButton L_Support_MiddleButton8;
	PushButton L_Support_MiddleButton9;

	PushButton R_LegGesture00_LowButton0;
	PushButton R_LegGesture30_LowButton1;
	PushButton R_LegGesture90_LowButton3;

	PushButton L_LegGesture00_LowButton0;
	PushButton L_LegGesture30_LowButton1;
	PushButton L_LegGesture90_LowButton3;
};


class SampleView : public cnoid::View
{
public:
    SampleView();

private:
	QScrollArea scrollArea;
	SampleWidget* sampleWidget;


};


#endif
//_SAMPLEVIEW_H_
//EOF