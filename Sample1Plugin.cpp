/**
  @author Shin'ichiro Nakaoka
*/

#include <cnoid/Plugin>
#include <cnoid/ItemTreeView>
#include <cnoid/BodyItem>
#include <cnoid/ToolBar>
#include <boost/bind.hpp>
#include <cnoid/MessageView>
#include  "SampleView.h"
//#include <iostream>
//#include <cnoid/MessageView>

using namespace boost;
using namespace cnoid;
//using namespace std;

class Sample1Plugin : public Plugin
{
public:
    
    Sample1Plugin() : Plugin("Sample1")
    {
        require("Body");
    }
    
    virtual bool initialize()
    {
        ToolBar* bar = new ToolBar("Sample1");
		/*bar->addButton("LeftForward_1")
            ->sigClicked().connect(bind(&Sample1Plugin::LeftForward_1Function, this));
        bar->addButton("Forward_2")
            ->sigClicked().connect(bind(&Sample1Plugin::Forward_2Function, this));
		bar->addButton("RightForward_3")
            ->sigClicked().connect(bind(&Sample1Plugin::RightForward_3Function, this));
        bar->addButton("Left_4")
            ->sigClicked().connect(bind(&Sample1Plugin::Left_4Function, this));
        bar->addButton("Place_5")
            ->sigClicked().connect(bind(&Sample1Plugin::Place_5Function, this));
		bar->addButton("Right_6")
            ->sigClicked().connect(bind(&Sample1Plugin::Right_6Function, this));
		bar->addButton("LeftBackward_7")
            ->sigClicked().connect(bind(&Sample1Plugin::LeftBackward_7Function, this));
        bar->addButton("Backward_8")
            ->sigClicked().connect(bind(&Sample1Plugin::Backward_8Function, this));
		bar->addButton("RightBackward_9")
            ->sigClicked().connect(bind(&Sample1Plugin::RightBackward_9Function, this));*/
		addToolBar(bar);
		addView(new SampleView());

        return true;
    }
	/*void LeftForward_1Function()
	{
			 MessageView::instance()->putln("LeftForward_1 !");
	}

		void Forward_2Function()
	{
			 MessageView::instance()->putln("Forward_2 !");
	}

	void RightForward_3Function()
	{
			 MessageView::instance()->putln("RightForward_3 !");
	}

		void Left_4Function()
	{
			 MessageView::instance()->putln("Left_4 !");
	}

		void Place_5Function()
	{
			 MessageView::instance()->putln("Place_5 !");
	}

		void Right_6Function()
	{
			 MessageView::instance()->putln("Right_6 !");
	}
		void LeftBackward_7Function()
	{
			 MessageView::instance()->putln("LeftBackward_7 !");
	}

		void Backward_8Function()
	{
			 MessageView::instance()->putln("Backward_8 !");
	}

		void RightBackward_9Function()
	{
			 MessageView::instance()->putln("RightBackward_9 !");
	}*/

  //  void onButtonClicked(double dq)
  //  {
		//double pose0[20] ={-3.78656729e-008, 0.0361667252, 0.155612182, -0.679881892, -0.524269817, 0.0361667362, 6.28337145e-009, -0.0565271089, -0.567737262, 
  //      0.33519838, -0.232538862, -0.0565271143, 0, 0, 0, -0.523598776, 0, 0, 
		//0.523598776, 0}; 

		//// pose0[1] = 11111;

		//double pose1[20] ={ -0.192660187, 0.261166232, 0.491534074, -1.45728277, -0.965748824, 0.261166257, -3.18410416e-008, 0.233959652, -0.445701432, 
		//0.492609417, 0.0469080314, 0.233959644, 0, 0, 0, -0.523598776, 0, 0, 
  //      0.523598776, 0}; 

		//double pose2[20] ={-0.25975638, -0.0955523858, 0.714136321, -0.712824932, 0.00131127022, -0.0955523678, 6.44733168e-009, -0.0814893505, -0.432632761, 
		//0.939680644, 0.507047907, -0.0814893566, 0, 0, 0, -0.523598776, 0, 0, 
		//0.523598776, 0}; 

		//double pose3[20] = {-0.25975635, -0.0550980264, 0.407850759, -0.769932861, -0.362082273, -0.0550980026, -0.156368078, -0.0156084574, -1.00848365, 
		//1.17967128, 0.171187705, -0.0156084814, 0, 0, 0, -0.523598776, 0, 0, 
		//0.523598776, 0}; 

		//double pose4[20] = { -0.259756367, -0.122005044, 0.458530972, -1.08153598, -0.623005141, -0.122005024, -0.156381454, -0.070499939, -0.692815638, 
		//0.269744141, -0.423071405, -0.0704999603, 0, 0, 0, -0.523598776, 0, 0, 
		//0.523598776, 0}; 

		//double pose5[20] = {-0.150410565, 0.128914435, 0.817119324, -1.43372203, -0.616602864, 0.128914461, -0.156381478, 0.133390565, -0.723802323, 
		//0.959578723, 0.2357765, 0.133390543, 0, 0, 0, -0.523598776, 0, 0, 
		//0.523598776, 0 }; 

		//double pose6[20] ={-0.0719613612, 0.0350357321, 0.483544233, -1.21038839, -0.726844347, 0.0350357636, -0.156381462, -0.0290500342, -0.962415007, 
		//0.841060094, -0.121354812, -0.0290500574, 0, 0, 0, -0.523598776, 0, 0, 
		//0.523598776, 0 }; 

		//double pose7[20] ={-0.071961294, -0.27552246, 0.524790847, -0.962777371, -0.437986729, -0.275522428, 3.98484517e-008, -0.341542875, -0.952023107, 
		//1.37660888, 0.424585902, -0.341542904, 0, 0, 0, -0.523598776, 0, 0, 
		//0.523598776, 0}; 

		//double pose8[20] ={-0.0719613386, -0.0565057717, 0.744393665, -0.893197758, -0.148804293, -0.056505739, 3.98974173e-008, -0.125554972, -0.391627434, 
		//0.975597255, 0.583969974, -0.125555006, 0, 0, 0, -0.523598776, 0, 0, 
		//0.523598776, 0}; 
		//

  //      ItemList<BodyItem> bodyItems = // container,  
  //          ItemTreeView::mainInstance()->selectedItems<BodyItem>();  //ItemTreeView and mainInstance can be 
  //  
  //      for(size_t i=0; i < bodyItems.size(); ++i){  // bodyItems only one
  //          BodyPtr body = bodyItems[i]->body(); 
  //          for(int j=0; j < body->numJoints(); ++j){
  //              body->joint(j)->q() = pose0[j]; 

		//

		//		MessageView::instance()->putln("body->joint(j)->q()  !");
		//		//cout << "body -> joint(j)->q()" << body->joint(j)->q() << endl;
  //          }
  //          bodyItems[i]->notifyKinematicStateChange(true);
  //      }
  //  }
};


CNOID_IMPLEMENT_PLUGIN_ENTRY(Sample1Plugin)
