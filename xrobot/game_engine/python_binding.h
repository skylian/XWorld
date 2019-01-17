#ifndef PLAYGROUND_PY_H_
#define PLAYGROUND_PY_H_

#include <memory>
#include <vector>
#include <functional>
#include <unordered_map>

#include <boost/python.hpp>

#include "render_engine/render.h"
#include "map_grid.h"
#include "map_suncg.h"
#include "lidar.h"
#include "task.h"
#include "navigation.h"

using namespace xrobot;

typedef render_engine::GLContext CTX;

void list2vec(const boost::python::list& ns, std::vector<float>& v) {
	int L = len(ns);
	v.resize(L);
	for (int i=0; i<L; ++i) {
		v[i] = boost::python::extract<float>(ns[i]);
	}
}

inline boost::python::tuple vec2tuple(const glm::vec3& v) {
	return boost::python::make_tuple(v.x, v.y, v.z);
}

inline boost::python::tuple vec2tuple(const glm::vec4& v) {
	return boost::python::make_tuple(v.x, v.y, v.z, v.w);
}

inline glm::vec3 list2vec3(const boost::python::list& ns) {
	std::vector<float> ns_v;
	list2vec(ns, ns_v);

	assert(ns_v.size() == 3);
	return glm::vec3(ns_v[0], ns_v[1], ns_v[2]);
}

inline glm::vec4 list2vec4(const boost::python::list& ns) {
	std::vector<float> ns_v;
	list2vec(ns, ns_v);

	assert(ns_v.size() == 4);
	return glm::vec4(ns_v[0], ns_v[1], ns_v[2], ns_v[3]);
}

inline glm::quat list2quat(const boost::python::list& ns) {
	std::vector<float> ns_v;
	list2vec(ns, ns_v);

	assert(ns_v.size() == 4);
	return glm::angleAxis(ns_v[3], glm::vec3(ns_v[0], ns_v[1], ns_v[2]));
}

// Range hold AABB struct
struct Range {
	glm::vec3 min, max;

	Range() {
		min = glm::vec3(0);
		max = glm::vec3(0);
	}

	Range(boost::python::list min_py, boost::python::list max_py) {
		min = list2vec3(min_py);
		max = list2vec3(max_py);
	}

	// Two AABBs are equal
	bool __eq__(const Range& other) { 
		return min == other.min && max == other.max;
	}

	// Two AABBs are intersect
	bool __ne__(const Range& other) { 
		return glm::all(glm::lessThanEqual(min, other.max)) &&
			   glm::all(glm::greaterThanEqual(max, other.min));
	}

	// Contains
	bool __gt__(const Range& other) {
		return glm::all(glm::lessThan(min, other.min)) &&
			   glm::all(glm::greaterThan(max, other.max));
	}

	// Contains
	bool __lt__(const Range& other) {
		return glm::all(glm::lessThan(other.min, min)) &&
			   glm::all(glm::greaterThan(other.max, max));
	}

	// To String
	static std::string __str__(const Range& self) {
		return std::string( 
			"(" +
				std::to_string(self.min.x) + ", " + 
				std::to_string(self.min.y) + ", " +
				std::to_string(self.min.z) + ", " +
				std::to_string(self.max.x) + ", " +
				std::to_string(self.max.y) + ", " +
				std::to_string(self.max.z) +
			")"
		);
	}

	boost::python::tuple GetMin() const { return vec2tuple(min); }
	boost::python::tuple GetMax() const { return vec2tuple(max); }
};

// Thing hold object or robot in playground
class Thing {
public:
	Thing();

	// Returns current position in (x y z)
	boost::python::tuple GetPosition();

	// Return current orientation in (x y z w). It uses 
	// normalized quaternion for orientation representation!
	boost::python::tuple GetOrientation();

	// Return the label
	std::string GetLabel();

	// Return properities
	std::string GetProPerties() =delete;


	long __hash__() { return (long) (uintptr_t) robot_.lock().get(); }
	bool __eq__(const Thing& other) { 
		return robot_.lock().get() == other.robot_.lock().get();
	}
	static std::string __str__(const Thing& self) { return self.label_; }

	std::weak_ptr<RobotBase> GetPtr() const { return robot_; }
	void SetPtr(const std::weak_ptr<RobotBase>& robot) {
        robot_ = robot;
        Sync();
    }

private:
	//  Update all the status
	void Sync();
	std::string label_;
	boost::python::tuple position_;
	boost::python::tuple orientation_;
	std::weak_ptr<RobotBase> robot_;
};

// class Agent : public Thing {
// public:
// 	Agent();

// 	float GetYaw() const;
// 	float GetPitch() const;

// private:
// 	std::shared_ptr<Playground> scene_;
// };

// NavAgent hold a object with navigation functionality
class NavAgent {
public:
	explicit NavAgent(const int uid, const std::string& label);

	std::string GetLabel() const { return label_; }
	int GetUid() const { return uid_; }
	bool __eq__(const NavAgent& other) { return uid_ == other.uid_; }
	static std::string __str__(const NavAgent& self) { return self.label_; }

private:
	// Used to validate and access the agent in crowd. -1 means 
	// this agent is invalid
	int uid_;
	std::string label_;
};

// Playground defines the scene and renderer. Used to control the robot,
// generate a scene, enable certain feature and query the object.
//
// If you are going to seek some special functionalities, 
// such as IK, constraints, carving while path-finding, multi-rays lidar 
// and high quality rendering this wrapper is not supported, use C++ instead.
class Playground {
public:

	// Create a empty playground with basic rendering parameters.
	// 
	// If you are going to flyover visualization, use DEBUG_VISUALIZATION
	// 
	// Quality also can be adjust by switch RENDER_QUALITY_LOW and 
	// RENDER_QUALITY_NORMAL. However, low quality rendering 
	// does not have anti-aliasing which means images could have jagged edges.
	Playground(const int w, const int h,
			   const int headless = 0, 
			   const int quality = 0,
			   const int device = 0); 

	~Playground();


	void AssignTag(const std::string& path, const std::string& tag);
	void LoadTag(const std::string& path);
	void MakeObjectPickable(const std::string& tag);
	boost::python::list GetGoals() const;

	// Adjust directional light setting. Use python dict to update
	// the configurations
	//
	// Example in Python:
	// 	light_conf = dict()
	//	light_conf["ambient"] = 0.1
	// 	lifht_conf["exposure"] = 1.0
	//  env.SetLighting(light_conf)
	//
	// Use key "direction_x", "direction_y", "direction_z" to update direction
	// Use key "ambient" to update ambient factor
	// Use key "exposure" to update exposure factor
	void SetLighting(const boost::python::dict& lighting);

	// Create a camera and attach it to a certain object in the scene
	void AttachCameraTo(
            const Thing& object, const boost::python::list& offset_py);
	void UpdateAttachCamera(const float pitch);

	// Create a free camera
	void FreeCamera(
            const boost::python::list& position,
            const float yaw,
            const float pitch);
	void UpdateFreeCamera(
            const boost::python::list& position,
            const float yaw,
            const float pitch);

	// Enable single ray lidar in the scene
	// 
	// A desired number of rays is less than 720
	void EnableLidar(const int num_rays, const float max_distance);

	// Update lidar and return the results. -1 means no-hit
	//
	// Front, up vectors and attached position are necessary
	boost::python::list UpdateLidar(
            const boost::python::list& front_py, 
            const boost::python::list& up_py,
            const boost::python::list& position_py);

	// Enable inventory for robot temporary storage. 
	//
	// This member function is nessecary for use "Pickup" and "Putdown" actions
	void EnableInventory(const int max_capacity = 1);

	void ClearInventory();

	// Enable navigations to use path-finding for a object or robot
	//
	// The range of the baking area is defined by two 'vectors'. 
	// The minimum y must smaller than the ground, and the maximum y cannot
	// greater than the ceiling.
	//
	// Path-finding relies on grid map which is generated by a special
	// depth camera on top of the scene. The range of the baking area
	// needs to be passed into this member function.
	void EnableNavigation(
            const boost::python::list& min_corner, 
            const boost::python::list& max_corner,
            const bool kill_after_arrived);

	// Bake the grid map for path-finding.
	void BakeNavigationMesh();

	// Assign a threshould for determine ground when baking the grid map
	// 
	// The threshould is a log-based value between 0 to 1
	//
	// The surface threshould will be automatically calculated base on
	// the major depth samples in the grid map. However, you may need to
	// assign it by yourself in some scenario
	void AssignSurfaceLevel(const float level);

	// Assign a scale up value to dilate the grid map
	//
	// In some cases, a large size agent cannot pass a small gap. You need
	// dilate the grid map to fill the gaps
	//
	// A negative value for erode the grid map
	void AssignAgentRadius(const float radius);


	// Assign a target to an agent
	//
	// If an agent is no longer available, it will be neglected automatically
	void AssignNavigationAgentTarget(
            const NavAgent& agent,
            const boost::python::list& position);

	// Spawn an agent with path-finding feature
	NavAgent SpawnNavigationAgent(
            const std::string& path,
            const std::string& label,
            const boost::python::list& position,
            const boost::python::list& orientation);

	// Clear everything in the scene, including the camera
	void Clear();
	void ClearSpawnObjectsExceptAgent();


	// Generate a empty scene with checkerboard style floors and walls
	//
	// Default size is 5x5 (25m x 25m)
	void CreateArena(const int width = 5, const int length = 5);

	// Generate a empty SUNCG scene
	void CreateSceneFromSUNCG();

	// Generate a empty scene
	void CreateEmptyScene(
            const float min_x = -5,
            const float max_x = 5,
            const float min_z = -5,
            const float max_z = 5);

	// Generate a random size room
	void CreateRandomGenerateScene();


	void LoadXWorldScene(const std::string& filename);
	boost::python::dict GetXWorldScene() const { return json_scene_; }

	// Load a profile to generate random size room
	//
	// Example in Python:
	//	conf = dict()
	//	conf["room"] = [floor_path, wall_apth, door_path,...]
	//	conf["on_floor"] = [crate_path, "crate_label",...]
	//  conf["on_object"] = [crate_path, "crate_label",...]
	//  env.LoadRandomSceneConfigure(conf)
	//

	boost::python::list LocateObjectInGrid(Thing& object);
	boost::python::list LocatePositionInGrid(const float x, const float z);
	boost::python::list GetRoomVisitSequence();
	boost::python::list GetRoomGroups();
	boost::python::list GetSpaceNearPosition(
            const boost::python::list& position,
            const float radius);
	boost::python::list LoadSceneConfigure(
            const int w,
            const int l, 
            const int n,
            const int d);
	void LoadBasicObjects(const boost::python::list& doors,
						  const boost::python::list& keys,
						  const boost::python::list& key_tags,
						  const std::string& unlocked_door,
						  const std::string& wall,
						  const boost::python::list& tiles);
	void LoadModels(const boost::python::list& models,
					const boost::python::list& tags);
	void SpawnModelsConf(const boost::python::dict& conf);
	void SpawnModels();

	void ResolvePath();

	// Load SUNCG
	//
	// "filter" means remove a certain type or some types object while 
	// loading the scene
	void LoadSUNCG(const std::string& house,
				   const std::string& metadata,
				   const std::string& suncg_data_dir,
				   const int filter = -1);

	// Generate object at position with orientation. It uses 
	// normalized quaternion for orientation representation!
	// 
	// Only uniform scale is supported in Python
	Thing SpawnAnObject(const std::string& file, 
					    const boost::python::list& position_py,
					    const boost::python::list& orentation_py,
					    const float scale,
					    const std::string& label,
					    const bool fixed = true,
					    const bool occupy = true);

	void RemoveAnObject(Thing& object);

	// Initialize camera
	//
	// Make sure call this member function before rendering loop
	void Initialize();

	void HoldActions(const bool hold);

	// Update simulation and renderer
	void Update();

	// Update renderer 
	//
	// This also swap buffer in back
	void UpdateRenderer();

	// Update simulation Only
	//
	// This member function will not return any of available actions for 
	// robot. To "UpdateSimulationWithAction" for actions return
	boost::python::dict UpdateSimulation();

	// Update simulation with action applied
	//
	// This member function will return the actions which available to robot
	// in Python dict
	//
	// The number of actions are limited to 10! It cannot execute any action
	// id are larger than 10
	boost::python::dict UpdateSimulationWithAction(const int action);

	// Get observation
	//
	// Velocities and Accelerations are not supported
	boost::python::dict GetObservationSpace();

	// Get actions
	//
	// Action ids are the continuous order numbers in the list
	//
	// Attach/Detach and Pickup/Putdown cannot use simultaneously.
	boost::python::dict GetActionSpace();

	// Get camera position and orientation
	boost::python::tuple GetCameraPosition() const;
	boost::python::tuple GetCameraFront() const;
	boost::python::tuple GetCameraRight() const;
	boost::python::tuple GetCameraUp() const;
	float GetCameraYaw() const;

	// Capture Size
	int GetWidth() const { return w_; }
	int GetHeight() const { return h_; }

	// Get camera near clipping plane's distance
	//
	// Could be useful for calculating real depth
	float GetNearClippingDistance();

	// Get camera far clipping plane's distance
	//
	// Could be useful for calculating real depth
	float GetFarClippingDistance();

	// Get raw renderered images
	//
	// 8-bit unsigned char RGBA raw 
	boost::python::object GetCameraRGBDRaw();

	Thing GetAgent() const { return agent_; }

	// Move the object or robot 
	//
	// The actual distance offset per step is 0.005 * speed
	void MoveForward(const float speed = 1);
	void MoveBackward(const float speed = 1);
	
	// Turn the object or robot
	//
	// The actual radius offset per step is 0.005 * speed
	void TurnLeft(const float speed = 1);
	void TurnRight(const float speed = 1);
	
	// Pitch up or down in 0.5 deg angle
	//
	// The angle will be clamped down in -45 to 45 deg 
	void LookUp();
	void LookDown();

	// Pick up the object in the center of camera (also with in 3 unit length)
	std::string Grasp();

	// Put down the object in the center of camera (also with in 3 unit length)
	std::string PutDown();

	// Attach or stick the object root base at the center of camera (also with in 3 unit length)
	void Attach();

	// Detach the object which already been attached
	void Detach();

	// Rotate an object a certain degree angles
	std::string Rotate(const boost::python::list& angle_py);

	// Teleport an object or robot to a certain position in scene
	void Teleport(
            const Thing& object,
            const boost::python::list& position_py,
            const boost::python::list& orientation_py);

	// Move or rotate the joint in certain position with maximum forces	
	//
	// "joint_position" should follow { joint_id : joint_position }
	void ControlJointPositions(
            const Thing& object, 
            const boost::python::dict& joint_positions,
            const float max_force);

	// "joint_velocity" should follow { joint_id : joint_velocity }
	void ControlJointVelocities(
            const Thing& object, 
            const boost::python::dict& joint_velocities,
            const float max_force);

	// Enable or disable the interaction for further actions
	//
	// Actions only can be executed after enable interaction
	boost::python::list EnableInteraction();
	void DisableInteraction();

	// Take an action to a object with action id
	//
	// action id are limited from 0 to 3
	bool TakeAction(const int action_id);

	// Open / Close Inventory
	// TODO
	boost::python::list OpenInventory();
	void CloseInventory();
	void Use(const int object_id);

	bool UseObject(const int inventory_id);

	boost::python::list QueryLastEvent();

	// Check Contact
	bool QueryContact(const Thing& object);

	// Query two objects' (Bouding Box) are intersect
	bool QueryObjectAABBIntersect(const Thing& object_a, const Thing& object_b);

	// Query a certain object is at camera center
	//
	// This will use ray-cast to test ray intersection with object 
	// concave or convex hull
	bool QueryObjectWithLabelAtCameraCenter(const std::string& label);

	// Query a certain object is at forward
	//
	// Only use distance and angle
	bool QueryObjectWithLabelAtForward(const std::string& label);

	// Query a certain object is near the robot
	bool QueryObjectWithLabelNearMe(const std::string& label);

	// Return object at camera center
	Thing QueryObjectAtCameraCenter();

	// Return a list contains all object with certain label
	boost::python::list QueryObjectByLabel(const std::string& label);

	boost::python::list QueryObjectNearObject(
            Thing& object, 
            const bool exlude = true,
            const float dist = 2.0f);

	// Get the framerate, rendered frames count and cache information
	boost::python::dict GetStatus() const;

	void HighlightCenter(const bool highlight);

	void DisplayInventory(const bool display);


	// Debug and Unfinished Member Functions
	//
	// You should not use any of this member functions
	bool GetKeyPressUp() { return ctx_->GetKeyPressUp(); }
	bool GetKeyPressDown() { return ctx_->GetKeyPressDown(); }
	bool GetKeyPressRight() { return ctx_->GetKeyPressRight(); }
	bool GetKeyPressLeft() { return ctx_->GetKeyPressLeft(); }
	bool GetKeyPress1() { return ctx_->GetKeyPress1(); }
	bool GetKeyPress2() { return ctx_->GetKeyPress2(); }
	bool GetKeyPress3() { return ctx_->GetKeyPress3(); }
	bool GetKeyPress4() { return ctx_->GetKeyPress4(); }
	bool GetKeyPressKP9() { return ctx_->GetKeyPressKP9(); }
	bool GetKeyPressKP6() { return ctx_->GetKeyPressKP6(); }
	bool GameOver() const { return gameover_; }

private:
	int w_, h_;
	int iterations_;
	float camera_aspect_;
	float camera_pitch_;
	float camera_yaw_;
	
	bool hold_actions_;
	bool highlight_objects_;
	bool kill_after_arrived_;
	bool gameover_;
	bool interact_;
	bool inventory_opened_;
	Thing agent_;

	boost::python::dict json_scene_;
	boost::python::list current_actions_;
	boost::python::list current_objects_;
	boost::python::list current_event_;

	std::vector<Thing> objects_;
	std::shared_ptr<Map> scene_;
	std::shared_ptr<Inventory> inventory_;
	std::shared_ptr<Navigation> crowd_;
	std::shared_ptr<Lidar> lidar_;
	std::shared_ptr<render_engine::Render> renderer_;
	render_engine::GLContext * ctx_;
	render_engine::Camera * main_camera_;
};

// Python Binding

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(EnableInventory_member_overloads, 
									   Playground::EnableInventory, 0, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateEmptyScene_member_overloads, 
									   Playground::CreateEmptyScene, 0, 4)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(LoadSUNCG_member_overloads, 
									   Playground::LoadSUNCG, 3, 4)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SpawnAnObject_member_overloads, 
									   Playground::SpawnAnObject, 5, 7)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(MoveForward_member_overloads, 
									   Playground::MoveForward, 0, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(MoveBackward_member_overloads, 
									   Playground::MoveBackward, 0, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TurnLeft_member_overloads, 
									   Playground::TurnLeft, 0, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(TurnRight_member_overloads, 
									   Playground::TurnRight, 0, 1)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(QueryObjectNearObject_member_overloads, 
									   Playground::QueryObjectNearObject, 1, 3)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateArena_member_overloads, 
									   Playground::CreateArena, 0, 2)

BOOST_PYTHON_MODULE(libxrobot)
{
	using namespace boost::python;

	class_<Range>("Range", init<boost::python::list, boost::python::list>())
	.add_property("GetMin", &Range::GetMin)
	.add_property("GetMax", &Range::GetMax)
	.def("__eq__", &Range::__eq__)
	.def("__ne__", &Range::__ne__)
	.def("__lt__", &Range::__lt__)
	.def("__gt__", &Range::__gt__)
	.def("__str__", &Range::__str__)
	;

	class_<Thing>("Thing", no_init)
	.def("GetPosition", &Thing::GetPosition)
	.def("GetOrientation", &Thing::GetOrientation)
	.def("GetLabel", &Thing::GetLabel)
	.def("__hash__", &Thing::__hash__)
	.def("__eq__", &Thing::__eq__)
	.def("__str__", &Thing::__str__)
	;

	class_<NavAgent>("NavAgent", no_init)
	.def("GetLabel", &NavAgent::GetLabel)
	.def("__eq__", &NavAgent::__eq__)
	.def("__str__", &NavAgent::__str__)
	;

	class_<Playground>("Playground", init<int,int,optional<int,int,int>>())

	.def("CreateArena", &Playground::CreateArena,
		CreateArena_member_overloads(
			args("width", "length"), "dimension"
		)
	)

	.def("EnableInventory", &Playground::EnableInventory,
		EnableInventory_member_overloads(
			args("max_capacity"), "capacity"
		)
	)
	.def("CreateEmptyScene", &Playground::CreateEmptyScene, 
		CreateEmptyScene_member_overloads(
			args("min_x", "max_x", "min_z", "max_z"), "range"
		)
	)
	.def("LoadSUNCG", &Playground::LoadSUNCG,
		LoadSUNCG_member_overloads(
			args("house", "metadata", "suncg_data_dir", "filter"), "suncg"
		)
	)
	.def("SpawnAnObject", &Playground::SpawnAnObject,
		SpawnAnObject_member_overloads(
			args("file", "position_py", "orentation_py", "scale", "label", "fixed", "occupy"),
			"spawn"
		)
	)
	.def("MoveForward", &Playground::MoveForward,
		MoveForward_member_overloads(
			args("speed"), "speed"
		)
	)
	.def("MoveBackward", &Playground::MoveBackward,
		MoveBackward_member_overloads(
			args("speed"), "speed"
		)
	)
	.def("TurnLeft", &Playground::TurnLeft,
		TurnLeft_member_overloads(
			args("speed"), "speed"
		)
	)
	.def("TurnRight", &Playground::TurnRight,
		TurnRight_member_overloads(
			args("speed"), "speed"
		)
	)

	.def("QueryObjectNearObject", &Playground::QueryObjectNearObject,
		QueryObjectNearObject_member_overloads(
			args("near"), "dist"
		)
	)

	.def("ClearSpawnObjectsExceptAgent", &Playground::ClearSpawnObjectsExceptAgent)
	.def("HoldActions", &Playground::HoldActions)
	.def("UpdateAttachCamera", &Playground::UpdateAttachCamera)
	.def("ClearInventory", &Playground::ClearInventory)
	.def("RemoveAnObject", &Playground::RemoveAnObject)
	.def("GetAgent", &Playground::GetAgent)
	.def("GetXWorldScene", &Playground::GetXWorldScene)
	.def("DisplayInventory", &Playground::DisplayInventory)
	.def("LoadXWorldScene", &Playground::LoadXWorldScene)
	.def("HighlightCenter", &Playground::HighlightCenter)
	.def("QueryLastEvent", &Playground::QueryLastEvent)
	.def("GetCameraYaw", &Playground::GetCameraYaw)
	.def("GetSpaceNearPosition", &Playground::GetSpaceNearPosition)
	.def("ResolvePath", &Playground::ResolvePath)
	.def("GetGoals", &Playground::GetGoals)
	.def("LocatePositionInGrid", &Playground::LocatePositionInGrid)
	.def("LocateObjectInGrid", &Playground::LocateObjectInGrid)
	.def("GetRoomVisitSequence", &Playground::GetRoomVisitSequence)
	.def("QueryContact", &Playground::QueryContact)
	.def("GetRoomGroups", &Playground::GetRoomGroups)
	.def("LoadBasicObjects", &Playground::LoadBasicObjects)
	.def("OpenInventory", &Playground::OpenInventory)
	.def("CloseInventory", &Playground::CloseInventory)
	.def("Use", &Playground::Use)
	.def("AssignTag", &Playground::AssignTag)
	.def("LoadTag", &Playground::LoadTag)
	.def("MakeObjectPickable", &Playground::MakeObjectPickable)
	.def("GetWidth", &Playground::GetWidth)
	.def("GetHeight", &Playground::GetHeight)
	.def("SetLighting", &Playground::SetLighting)
	.def("EnableLidar", &Playground::EnableLidar)
	.def("UpdateLidar", &Playground::UpdateLidar)
	.def("EnableNavigation", &Playground::EnableNavigation)
	.def("AssignSurfaceLevel", &Playground::AssignSurfaceLevel)
	.def("AssignAgentRadius", &Playground::AssignAgentRadius)
	.def("BakeNavigationMesh", &Playground::BakeNavigationMesh)
	.def("AssignNavigationAgentTarget", &Playground::AssignNavigationAgentTarget)
	.def("SpawnNavigationAgent", &Playground::SpawnNavigationAgent)
	.def("Clear", &Playground::Clear)
	.def("CreateSceneFromSUNCG", &Playground::CreateSceneFromSUNCG)
	.def("CreateRandomGenerateScene", &Playground::CreateRandomGenerateScene)
	.def("LoadSceneConfigure", &Playground::LoadSceneConfigure)
	.def("SpawnModels", &Playground::SpawnModels)
	.def("SpawnModelsConf", &Playground::SpawnModelsConf)
	.def("LoadModels", &Playground::LoadModels)
	.def("AttachCameraTo", &Playground::AttachCameraTo)
	.def("FreeCamera", &Playground::FreeCamera)
	.def("UpdateFreeCamera", &Playground::UpdateFreeCamera)
	.def("Initialize", &Playground::Initialize)
	.def("UpdateSimulation", &Playground::UpdateSimulation)
	.def("UpdateRenderer", &Playground::UpdateRenderer)
	.def("Update", &Playground::Update)
	.def("LookUp", &Playground::LookUp)
	.def("LookDown", &Playground::LookDown)
	.def("Grasp", &Playground::Grasp)
	.def("PutDown", &Playground::PutDown)
	.def("Attach", &Playground::Attach)
	.def("Detach", &Playground::Detach)
	.def("Rotate", &Playground::Rotate)
	.def("TakeAction", &Playground::TakeAction)
	.def("ControlJointPositions", &Playground::ControlJointPositions)
	.def("ControlJointVelocities", &Playground::ControlJointVelocities)
	.def("Teleport", &Playground::Teleport)
	.def("GetCameraRGBDRaw", &Playground::GetCameraRGBDRaw)
	.def("QueryObjectAABBIntersect", &Playground::QueryObjectAABBIntersect)
	.def("QueryObjectWithLabelAtCameraCenter", &Playground::QueryObjectWithLabelAtCameraCenter)
	.def("QueryObjectWithLabelAtForward", &Playground::QueryObjectWithLabelAtForward)
	.def("QueryObjectWithLabelNearMe", &Playground::QueryObjectWithLabelNearMe)
	.def("QueryObjectAtCameraCenter", &Playground::QueryObjectAtCameraCenter)
	.def("QueryObjectByLabel", &Playground::QueryObjectByLabel)
	.def("UpdateSimulationWithAction", &Playground::UpdateSimulationWithAction)
	.def("GetObservationSpace", &Playground::GetObservationSpace)
	.def("GetActionSpace", &Playground::GetActionSpace)
	.def("GetCameraPosition", &Playground::GetCameraPosition)
	.def("GetCameraRight", &Playground::GetCameraRight)
	.def("GetCameraFront", &Playground::GetCameraFront)
	.def("GetCameraUp", &Playground::GetCameraUp)
	.def("GetStatus", &Playground::GetStatus)
	;

	scope().attr("ENABLE_INTERACTION")  = 11;
	scope().attr("DISABLE_INTERACTION") = 12;
	scope().attr("NO_ACTION")           = 14;
	scope().attr("HEADLESS")            = 1;
	scope().attr("VISUALIZATION")       = 0;
	scope().attr("DEBUG_VISUALIZATION") = 0;
	scope().attr("GRID")                = 0;
	scope().attr("SUNCG")               = 1;
	scope().attr("REMOVE_NONE")         = -1;
	scope().attr("REMOVE_STAIR")        = 1;
	scope().attr("REMOVE_DOOR")         = 2;
	scope().attr("WORLD_UP")            = boost::python::make_tuple(0, 1, 0);
	scope().attr("METACLASS_WALL")      = std::string("Wall");
	scope().attr("METACLASS_FLOOR")     = std::string("Floor");
	scope().attr("METACLASS_CEILING")   = std::string("Ceiling");
	scope().attr("VERY_LOW")                 = 0;
	scope().attr("LOW")                      = 1;
	scope().attr("NORMAL_NO_SHADOW")         = 2;
	scope().attr("NORMAL")                   = 3;
	scope().attr("HIGH")                     = 4;
	scope().attr("FLAT")                     = 0;
	scope().attr("BLINN")                    = 1;
	scope().attr("BLINN_AO_AA")              = 2;
	scope().attr("BLINN_SHADOW_AO_R_AA")     = 3;
	scope().attr("VCT_SHADOW_AO_R_AA")       = 4;
	scope().attr("GPU0")                     = 0;
	scope().attr("GPU1")                     = 1;
	scope().attr("GPU2")                     = 2;
	scope().attr("GPU3")                     = 3;
}
#endif // PLAYGROUND_PY_H_
