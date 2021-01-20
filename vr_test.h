#pragma once
#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/render/frame_buffer.h>
#include <cgv/defines/quote.h>
#include <cgv_gl/point_renderer.h>

#include <math.h>
#include "orbittools/coreLib.h"
#include "orbittools/orbitLib.h"

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>


class vr_test :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider {
protected:
	static std::string get_input_directory() {
		return QUOTE_SYMBOL_VALUE(INPUT_DIR);
	};
	// different interaction states for the controllers
	enum InteractionState {
		IS_NONE,
		IS_OVER,
		IS_GRAB
	};

	// store the scene as colored boxes
	/*std::vector<box3> boxes;
	std::vector<rgb> box_colors;*/

	//Mesh renderer for earth
	cgv::render::mesh_render_info earth_info;
	cgv::media::mesh::simple_mesh<> earth_sphere;

	//Line renderer for one orbit data
	cgv::render::rounded_cone_render_style orbit_style;
	cgv::render::sphere_render_style ptx_style;
	std::vector<cTle> tles;
	std::vector<pair<cSatellite, bool>> sats;
	std::vector<pair<string, bool>> actives;
	int nb_active;
	std::map<string, std::vector<pair<cSatellite, bool>>> satellites;
	std::map<string, cgv::render::rounded_cone_render_style> orbit_styles;
	std::map<string, cgv::render::sphere_render_style> sat_styles;
	std::vector<std::vector<vec3>> pos;
	std::map<string, std::vector<vec3>> sat_pos;
	std::map<string, std::vector<vec3>> sat_orbit_pos;
	std::vector<vec3> all_pos_sat;
	std::vector<std::pair<string, vec3>> names_plus_pos;
	std::vector<vec3> all_pos_orbit;
	std::vector<vec3> all_colors_sat;
	std::vector<vec3> all_colors_orbit;

	bool is_active;
	time_t visual_now;
	time_t old_time;
	time_t v_min_2, v_plus_2;

	// rendering styles
	cgv::render::box_render_style style;
	cgv::render::rounded_cone_render_style cone_style;

	// sample for rendering a mesh
	double mesh_scale;
	dvec3 mesh_location;
	dquat mesh_orientation;

	// Ground From Space Earth shader parameters
	cgv::render::shader_program ground_from_space;
	float Kr = 0.0030f;
	float Km = 0.0015f;
	float ESun = 16.0f;
	float g = -0.75f;
	float InnerRadius = 10.0f * 25.0f;
	float OuterRadius = 10.25f * 25.0f;
	float Scale = 1.0f / (OuterRadius - InnerRadius);
	float ScaleDepth = 0.25f;
	float ScaleOverScaleDepth = Scale / ScaleDepth;

	// render information for mesh
	cgv::render::mesh_render_info MI;

	// sample for rendering text labels
	std::string label_text;
	int label_font_idx;
	bool label_upright;
	float label_size;
	rgb label_color;

private:
	bool label_outofdate; // whether label texture is out of date

protected:
	unsigned label_resolution; // resolution of label texture
	cgv::render::texture label_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer label_fbo; // fbo used for offline rendering of label

	// general font information
	std::vector<const char*> font_names;
	std::string font_enum_decl;

	// current font face used
	cgv::media::font::font_face_ptr label_font_face;
	cgv::media::font::FontFaceAttributes label_face_type;

	// manage controller input configuration for left controller
	std::vector<vr::controller_input_config> left_inp_cfg;

	// store handle to vr kit of which left deadzone and precision is configured
	void* last_kit_handle;

	// length of to be rendered rays
	float ray_length;

	// keep reference to vr_view_interactor
	vr_view_interactor* vr_view_ptr;

	// store the movable boxes
	/*std::vector<box3> movable_boxes;
	std::vector<rgb> movable_box_colors;
	std::vector<vec3> movable_box_translations;
	std::vector<quat> movable_box_rotations;*/

	// intersection points
	/*std::vector<vec3> intersection_points;
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;*/

	// state of current interaction with boxes for all controllers
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::box_render_style movable_style;

	int nr_cameras;
	int frame_width, frame_height;
	int frame_split;
	float seethrough_gamma;
	mat4 camera_to_head_matrix[2];
	cgv::math::fmat<float, 4, 4> camera_projection_matrix[4];
	vec2 focal_lengths[4];
	vec2 camera_centers[4];
	cgv::render::texture camera_tex;
	cgv::render::shader_program seethrough;
	GLuint camera_tex_id;
	bool undistorted;
	bool shared_texture;
	bool max_rectangle;
	float camera_aspect;
	bool show_seethrough;
	bool use_matrix;
	float background_distance;
	float background_extent;
	vec2 extent_texcrd;
	vec2 center_left;
	vec2 center_right;

public:

	void init_cameras(vr::vr_kit* kit_ptr);

	void start_camera();

	void stop_camera();

	/// compute intersection points of controller ray with movable boxes
	//void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	/// keep track of status changes
	void on_status_change(void* kit_handle, int ci, vr::VRStatus old_status, vr::VRStatus new_status);
	/// register on device change events
	void on_device_change(void* kit_handle, bool attach);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	//void construct_table(float tw, float td, float th, float tW);
	/// construct boxes that represent a room of dimensions w,d,h and wall width W
	//void construct_room(float w, float d, float h, float W, bool walls, bool ceiling);
	/// construct boxes for environment
	//void construct_environment(float s, float ew, float ed, float w, float d, float h);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	//void construct_movable_boxes(float tw, float td, float th, float tW, size_t nr);
	/// construct a scene with a table
	void build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW);

	
public:
	vr_test();

	std::string get_type_name() { return "vr_test"; }

	void stream_help(std::ostream& os);

	void on_set(void* member_ptr);

	bool handle(cgv::gui::event& e);
	
	bool init(cgv::render::context& ctx);
	
	void calculate_positions_and_orbits();

	string intersection(vec3 origin, vec3 direction);

	void clear(cgv::render::context& ctx);

	void init_frame(cgv::render::context& ctx);

	void draw(cgv::render::context& ctx);

	void finish_draw(cgv::render::context& ctx);

	void create_gui();

};

///@}
