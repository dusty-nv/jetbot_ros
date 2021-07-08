 
#include <functional>

#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/MainWindow.hh>
#include <gazebo/gui/GLWidget.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>


namespace gazebo
{
	/**
	 * Gazebo GUI plugin for control over the user GUI camera
	 *
	 * Example commands:
	 *
	 *   export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/workspace/src/jetbot_ros/gazebo/plugins/build/
	 *
	 *   add to world SDF:
	 *
	 *      <gui>
	 *		<plugin name="user_camera_control" filename="libuser_camera_control.so">
	 *			<render_rate>40.0</render_rate>
	 *		</plugin>
	 *	   </gui>
	 *
	 *   or add to /root/.gazebo/gui.ini 
	 *
	 *     [geometry]
      *     x=0
      *     y=0
	 *	  
	 *     [overlay_plugins]
	 *     filenames=libuser_camera_control.so
	 *
	 *	  [user_camera]
	 *	  render_rate=20 
	 */
	class UserCameraControl : public GUIPlugin
	{
		Q_OBJECT
		
	public:
		UserCameraControl();

		virtual ~UserCameraControl();

		void Load(sdf::ElementPtr sdf);

		void Update();

	public slots:
		void OnSpinBox(int value);
		
	private:
		double mRenderRate;
		bool mUpdateRenderRate;
		
		QSpinBox* mSpinBox;
		std::vector<event::ConnectionPtr> mConnections;
	};
}
