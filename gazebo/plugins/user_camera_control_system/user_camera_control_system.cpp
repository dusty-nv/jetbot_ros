 
#include <functional>

#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/MainWindow.hh>
#include <gazebo/gui/GLWidget.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>


namespace gazebo
{
	/**
	 * Gazebo system plugin for control over the user GUI camera
	 *
	 * Example commands:
	 *
	 *   export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/workspace/src/jetbot_ros/gazebo/plugins/build/
	 *   gazebo -g libuser_camera_control_system.so worlds/maze_obstacles_simple_diff.world
	 *
	 *   or add to /root/.gazebo/gui.ini 
	 *
	 *     [geometry]
      *     x=0
      *     y=0
	 *	  
	 *	  [user_camera]
	 *	  render_rate=20
	 *
	 */
	class UserCameraControlSystem : public SystemPlugin
	{
	public:
		UserCameraControlSystem() : SystemPlugin()
		{
			mUserCam = NULL;
			mRenderRate = 20.0;
		}

		virtual ~UserCameraControlSystem()
		{
			mConnections.clear();
			
			if( mUserCam != NULL )
				mUserCam.reset();
		}

		void Load(int /*_argc*/, char ** /*_argv*/)
		{
			printf("UserCameraControlSystem plugin -- loading plugin instance\n");
			
			mConnections.push_back(
				event::Events::ConnectPreRender(
				std::bind(&UserCameraControlSystem::Update, this)));	   
		}

		void Update()
		{
			if( mUserCam != NULL )
				return;
			
			// get the active GUI camera
			mUserCam = gui::get_active_camera();
			
			if( !mUserCam )
				return;

			// now that gui.ini should be loaded, get the desired setting
			mRenderRate = gazebo::gui::getINIProperty<double>("user_camera.render_rate", mRenderRate);
			printf("UserCameraControlSystem plugin -- desired render rate from ~/.gazebo/gui.ini: %f\n", mRenderRate);
		
			// update the rendering rate
			printf("UserCameraControlSystem plugin -- GUI camera default render rate:   %f\n", mUserCam->RenderRate());
			mUserCam->SetRenderRate(mRenderRate);
			printf("UserCameraControlSystem plugin -- GUI camera modified render rate:  %f\n", mUserCam->RenderRate());

			// get the GLWidget renderer
			auto glWidget = gazebo::gui::get_main_window()->findChild<gazebo::gui::GLWidget*>("GLWidget");

			if( !glWidget )
			{
				mUserCam = NULL;
				return;
			}
			
			// trigger https://github.com/osrf/gazebo/blob/e4400ae7e04e4859eec392ac67b813402703bab0/gazebo/gui/GLWidget.cc#L912
			glWidget->ViewScene(glWidget->Scene());

			// confirm the changes
			mUserCam = gui::get_active_camera();
			printf("UserCameraControlSystem plugin -- GUI camera active render rate:    %f\n", mUserCam->RenderRate());
		}

	private:
		double mRenderRate;
		rendering::UserCameraPtr mUserCam;
		std::vector<event::ConnectionPtr> mConnections;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_SYSTEM_PLUGIN(UserCameraControlSystem)
}
