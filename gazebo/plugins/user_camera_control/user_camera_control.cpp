/*
 * Gazebo GUI plugin for control over the user GUI camera
 * see user_camera_control.h for examples of how to run
 */
 
#include "user_camera_control.h"


using namespace gazebo;

	
// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(UserCameraControl)



UserCameraControl::UserCameraControl() : GUIPlugin()
{
	mRenderRate = 20.0;
	mUpdateRenderRate = true;
	
	this->setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

	// Create layouts
	QHBoxLayout* mainLayout = new QHBoxLayout;
	QHBoxLayout* frameLayout = new QHBoxLayout();
	
	QFrame* mainFrame = new QFrame();

	// Create FPS spinbox
	mSpinBox = new QSpinBox();
	
	mSpinBox->setSingleStep(1);
	mSpinBox->setRange(0, 99);
	mSpinBox->setValue(mRenderRate);
	mSpinBox->setSuffix(" FPS");
	mSpinBox->setMinimumWidth(50);

	connect(mSpinBox, SIGNAL(valueChanged(int)), this, SLOT(OnSpinBox(int)));

	frameLayout->addWidget(new QLabel(QString("Render Rate")));
	frameLayout->addWidget(mSpinBox);

	mainFrame->setLayout(frameLayout);
	mainLayout->addWidget(mainFrame);

	// Remove margins to reduce space
	frameLayout->setContentsMargins(5, 0, 5, 0);
	mainLayout->setContentsMargins(0, 0, 0, 0);

	this->setLayout(mainLayout);
	
	// Position and resize this widget
	this->move(5, 5);
	this->resize(165, 30);
}

UserCameraControl::~UserCameraControl()
{
	mConnections.clear();
}

void UserCameraControl::Load(sdf::ElementPtr sdf)
{		
	printf("UserCameraControl plugin -- loading plugin instance\n");

	// try to get render_rate setting from SDF (if plugin included in world file),
	// otherwise check the ~/.gazebo/gui.ini file for it
	if( sdf->Get("render_rate", mRenderRate, mRenderRate) )
	{
		printf("UserCameraControl plugin -- default render rate from SDF:  %f\n", mRenderRate);
	}
	else
	{
		mRenderRate = gazebo::gui::getINIProperty<double>("user_camera.render_rate", mRenderRate);
		printf("UserCameraControl plugin -- default render rate from ~/.gazebo/gui.ini: %f\n", mRenderRate);
	}
	
	mSpinBox->setValue(mRenderRate);
	
	mConnections.push_back(
		event::Events::ConnectPreRender(
		std::bind(&UserCameraControl::Update, this)));	   
}

void UserCameraControl::Update()
{
	if( !mUpdateRenderRate )
		return;
	
	// get the active GUI camera
	rendering::UserCameraPtr userCam = gui::get_active_camera();
	
	if( !userCam )
		return;
	
	// update the rendering rate
	//printf("UserCameraControl plugin -- GUI camera default render rate:  %f\n", userCam->RenderRate());
	userCam->SetRenderRate(mRenderRate);
	//printf("UserCameraControl plugin -- GUI camera modified render rate: %f\n", userCam->RenderRate());

	// get the GLWidget renderer
	auto glWidget = gazebo::gui::get_main_window()->findChild<gazebo::gui::GLWidget*>("GLWidget");

	if( !glWidget )
		return;
	
	// trigger https://github.com/osrf/gazebo/blob/e4400ae7e04e4859eec392ac67b813402703bab0/gazebo/gui/GLWidget.cc#L912
	glWidget->ViewScene(glWidget->Scene());

	// confirm the changes
	userCam = gui::get_active_camera();
	printf("UserCameraControl plugin -- set GUI camera active render rate: %f\n", userCam->RenderRate());
	
	mUpdateRenderRate = false;
}

void UserCameraControl::OnSpinBox(int value)
{
	mRenderRate = value;
	mUpdateRenderRate = true;
}

