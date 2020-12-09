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

#include <math.h>

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
	cgv::render::rounded_cone_renderer orbit_one;
	std::string orbit_name;
	std::vector<double> line_1;
	std::vector<double> line_2;

	/**
	*SGP4 data
	* */
	//constants
	const double ke = pow(5.9722*pow(10,24)* 6.674*pow(10,-11.0), 0.5);
	const double earth_radius_at_equator = 6378;
	const double km_per_earth_radii = 6378.135;
	const double rev_per_day_to_rad_per_sec = 2 * M_PI / (24 * 3600);
	const double deg_to_rad = M_PI / 180;
		//gravitational zonal harmonic of Earth
	const double j2 = 2 * 5.413080 * pow(10, -4) / pow(earth_radius_at_equator, 2);
	const double j3 = -0.253881 * pow(10, -1);
	const double j4 = 0.62098875 * pow(10, -6) * -8 / 3 / pow(earth_radius_at_equator, 4);
	
	const double k2 = 5.413080 * pow(10,-4);
	const double k4 = 0.62098875 * pow(10,-6);
	const double A30 = -1 * j3 * pow(earth_radius_at_equator, 3);
	const double s_density_param = 78.0;
	const double q0_density_param = 120.0;

	const double astronomical_unit = 149597870.700; // 1 AU in Kms
	//initial mean elements
	tm epoch;
	time_t epoch_time;
	double orbit_incl, raan, eccentricity, arg_perigee, mean_anom, mean_motion;
	double bstar;
	double orig_mean_motion, orig_semimaj_axis;
	double s_param;
	double q0_min_s_four;

	//following constants
	double theta;
	double xi;
	double beta0;
	double eta;
	double C2, C1, C3, C4, C5;
	double D2, D3, D4;
	//secular effects
	double t_min_t0;
	double secul_anomaly, secul_arg_perigee, secul_raan;
	double delta_arg_perig,delta_anom;
	double anom_p,arg_perigee_fixed,raan_fixed,eccentricity_fixed, semimaj_axis_fixed;
	double L, beta, mean_motion_fixed;
	//long-period periodic terms
	double a_x_N, L_L, a_y_N_L, L_T, a_y_N;
	//kepler's equation's terms
	double U, temp2, temp3, temp4, temp5, temp6;
	double sinEpw, cosEpw, Epw;
	//prelim for short-period periodics
	double ecosE, esinE, e_L, p_L;
	double r, r_dot, r_f_dot;
	double cos_u, sin_u, u;
	double delt_r, delt_u, delt_raan, delt_incl, delt_r_dot, delt_r_f_dot;

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

	void clear(cgv::render::context& ctx);

	void init_frame(cgv::render::context& ctx);

	void draw(cgv::render::context& ctx);

	void finish_draw(cgv::render::context& ctx);

	void create_gui();

};

///@}
