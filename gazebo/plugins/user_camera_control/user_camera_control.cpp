#include <functional>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/MainWindow.hh>
#include <gazebo/gui/GLWidget.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

// export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/workspace/src/jetbot_ros/gazebo/plugins/user_camera_control/build/
// gazebo -g libuser_camera_control.so worlds/maze_obstacles_simple_diff.world

namespace gazebo
{
  class UserCameraControl : public SystemPlugin
  {
    public: UserCameraControl() : SystemPlugin()
    {
	    this->userCam = NULL;
    }
    
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~UserCameraControl()
    {
        this->connections.clear();
        if (this->userCam)
            this->userCam.reset();
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
       printf("UserCameraControl::Load()\n");

       this->connections.push_back(
          event::Events::ConnectPreRender(
            std::bind(&UserCameraControl::Update, this)));	   
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
        printf("UserCameraControl::Init()\n");
    }

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
    private: void Update()
    {
      if (!this->userCam)
      {
	    // get the active GUI camera
         this->userCam = gui::get_active_camera();
         printf("UserCameraControl plugin -- GUI camera default render rate:  %f\n", this->userCam->RenderRate());
	    
	    // update the rendering rate
	    this->userCam->SetRenderRate(20.0f);
	    printf("UserCameraControl plugin -- GUI camera modified render rate:  %f\n", this->userCam->RenderRate());
	    
	    // get the GLWidget renderer
	    auto glWidget = gazebo::gui::get_main_window()->findChild<gazebo::gui::GLWidget *>("GLWidget");
	    
	    // trigger https://github.com/osrf/gazebo/blob/e4400ae7e04e4859eec392ac67b813402703bab0/gazebo/gui/GLWidget.cc#L912
	    glWidget->ViewScene(glWidget->Scene());
	    
	    // confirm the changes
	    this->userCam = gui::get_active_camera();
         printf("UserCameraControl plugin -- GUI camera active render rate:  %f\n", this->userCam->RenderRate());
      }
    }

    /// Pointer the user camera.
    private: rendering::UserCameraPtr userCam;

    /// All the event connections.
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(UserCameraControl)
}