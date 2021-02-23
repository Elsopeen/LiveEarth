#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <vr_view_interactor.h>
#include "label_manager.h"
#include <cgv/defines/quote.h>
#include <cgv/gui/trigger.h>
#include <chrono>
#include <thread>

#include "lib_begin.h"
#include "orbittools/coreLib.h"
#include "orbittools/orbitLib.h"

namespace vr {

	/// class manages static and dynamic parts of scene
	class vr_scene :
		public cgv::base::node,
		public cgv::render::drawable,
		public cgv::gui::event_handler,
		public cgv::gui::provider
	{
	private:
		// keep reference to vr view (initialized in init function)
		vr_view_interactor* vr_view_ptr;
	protected:
		//@name boxes and table
		//@{	

		// store the static part of the scene as colored boxes with the table in the last 5 boxes
		std::vector<box3> boxes;
		std::vector<rgb> box_colors;

		// ui parameters for table construction
		float table_width, table_depth, table_height, leg_width, leg_offset;
		rgb table_color, leg_color;

		// rendering style for rendering of boxes
		cgv::render::box_render_style style;

		/// construct boxes for environment
		void construct_environment(float s, float ew, float ed, float w, float d, float h);
		/// construct a scene with a table
		void build_scene(float w, float d, float h, float W);
		//@}

		/// Return the directory INPUT_DIR
		static std::string get_input_directory() {
			return QUOTE_SYMBOL_VALUE(INPUT_DIR);
		};

		//@name Satellites
		//@{
		/// Mesh renderer for earth
		cgv::render::mesh_render_info earth_info;
		cgv::media::mesh::simple_mesh<> earth_sphere;

		/// Orbit rendering variables
		cgv::render::rounded_cone_render_style orbit_style;
		std::map<string, cgv::render::rounded_cone_render_style> orbit_styles;

		/// Satellites rendering variables
		std::map<string, cgv::render::sphere_render_style> sat_styles;
		cgv::render::sphere_render_style ptx_style;
		std::map<string, vec3> all_pos_sat_interp;


		class satellite_state
		{
		public:
			time_t timestamp;
			map<string, vec3> satellite_positions;
			vector<vec3> orbits;
			vector<vec3> all_colors_orbit;
			vector<vec3> all_colors_sat;
		};

		class state_queue {
		public:
			deque<satellite_state> states;

			void calculate_positions_at(vr_scene* obj, time_t timestamp);
			map<string, vec3> interpolate_at(vr_scene* obj, time_t temp_now);
			void remove_oldest_state();
			const vector<vec3> orbits() { return states[0].orbits; };
			const vector<vec3> col_orbits() { return states[0].all_colors_orbit; };
			const vector<vec3> col_sats() { return states[0].all_colors_sat; };
			void clear() { states.clear(); };
		};

		state_queue sat_queue;

		/// Satellites storing variables
		std::map<string, bool> actives;
		std::map<string, std::vector<pair<cSatellite, bool>>> satellites;

		/// labels for the selected satellites
		std::map<string, uint32_t> li_sat;
		uint32_t listing_datasets_label;
		uint32_t listing_orbits_label;

		//Animation control variables
		enum InteractionState {
			IS_NONE,
			IS_OVER,
			IS_GRAB
		};
		InteractionState state[4];

		int grabber_throttle_1;
		int grabber_throttle_2;

		bool pause; //false play true pause
		cgv::gui::trigger trig;
		int incr;
		bool forback; //false forward true backward
		bool is_active;
		thread anim_thread;
		thread launch_new_thread(time_t tmp) {
			return thread(
				[&] (state_queue* queue, vr_scene* scene, time_t tmp_t) { 
					queue->calculate_positions_at(scene, tmp_t); 
				}, 
				&sat_queue, this, tmp);
		};

		time_t visual_now;
		time_t v_min_2, v_plus_2;

		//@}

		//@name labels
		//@{	
		/// use label manager to organize labels in texture
		label_manager lm;

		/// store label placements for rectangle renderer
		std::vector<vec3> label_positions;
		std::vector<quat> label_orientations;
		std::vector<vec2> label_extents;
		std::vector<vec4> label_texture_ranges;

		// label visibility
		std::vector<bool> label_visibilities;

		/// for rectangle renderer a plane_render_style is needed
		cgv::render::plane_render_style prs;
	public:
		/// different coordinate systems used to place labels
		enum CoordinateSystem
		{
			CS_LAB,
			CS_TABLE,
			CS_HEAD,
			CS_LEFT_CONTROLLER,
			CS_RIGHT_CONTROLLER
		};
		/// different alignments
		enum LabelAlignment
		{
			LA_CENTER,
			LA_LEFT,
			LA_RIGHT,
			LA_BOTTOM,
			LA_TOP
		};
		/// for each label coordinate system
		std::vector<CoordinateSystem> label_coord_systems;
		/// size of pixel in meters
		float pixel_scale;
		/// add a new label without placement information and return its index
		uint32_t add_label(const std::string& text, const rgba& bg_clr, int _border_x = 4, int _border_y = 4, int _width = -1, int _height = -1) {
			uint32_t li = lm.add_label(text, bg_clr, _border_x, _border_y, _width, _height);
			label_positions.push_back(vec3(0.0f));
			label_orientations.push_back(quat());
			label_extents.push_back(vec2(1.0f));
			label_texture_ranges.push_back(vec4(0.0f));
			label_coord_systems.push_back(CS_LAB);
			label_visibilities.push_back(true);
			return li;
		}
		/// update label text
		void update_label_text(uint32_t li, const std::string& text) { lm.update_label_text(li, text); }
		/// fix the label size based on the font metrics even when text is changed later on
		void fix_label_size(uint32_t li) { lm.fix_label_size(li); }
		/// place a label relative to given coordinate system
		void place_label(uint32_t li, const vec3& pos, const quat& ori = quat(),
			CoordinateSystem coord_system = CS_LAB, LabelAlignment align = LA_CENTER, float scale = 1.0f) {
			label_extents[li] = vec2(scale * pixel_scale * lm.get_label(li).get_width(), scale * pixel_scale * lm.get_label(li).get_height());
			static vec2 offsets[5] = { vec2(0.0f,0.0f), vec2(0.5f,0.0f), vec2(-0.5f,0.0f), vec2(0.0f,0.5f), vec2(0.0f,-0.5f) };
			label_positions[li] = pos + ori.get_rotated(vec3(offsets[align] * label_extents[li], 0.0f));
			label_orientations[li] = ori;
			label_coord_systems[li] = coord_system;
		}
		/// hide a label
		void hide_label(uint32_t li) { label_visibilities[li] = false; }
		/// show a label
		void show_label(uint32_t li) { label_visibilities[li] = true; }
		//@}

	public:
		/// standard constructor for scene
		vr_scene();
		/// return type name
		std::string get_type_name() const { return "vr_scene"; }
		/// reflect member variables
		bool self_reflect(cgv::reflect::reflection_handler& rh);
		/// callback on member updates to keep data structure consistent
		void on_set(void* member_ptr);
		//@name cgv::gui::event_handler interface
		//@{
		/// provide interaction help to stream
		void stream_help(std::ostream& os);
		/// handle events
		bool handle(cgv::gui::event& e);
		/// Calculate the name of the closest satellite to the cursor
		string intersection(vec3 origin, vec3 direction);
		/// calculate labels
		void fill_labels();
		/// Changes the time to allow the animation
		void change_time_queue(double, double dt);
		/// starts the animation
		void start_anim(cgv::gui::button&);
		/// stops the animation
		void stop_anim(cgv::gui::button&);
		/// Changes the direction of the anim
		void forback_anim(cgv::gui::button&);

		//@}

		//@name cgv::render::drawable interface
		//@{
		/// initialization called once per context creation
		bool init(cgv::render::context& ctx);
		/// initialization called once per frame
		void init_frame(cgv::render::context& ctx);
		/// called before context destruction to clean up GPU objects
		void clear(cgv::render::context& ctx);
		/// draw scene here
		void draw(cgv::render::context& ctx);
		//@}

		// invert boolean value of the toggle button control
		void activate_dataset_or_orbit(cgv::gui::control<bool>& in);

		/// cgv::gui::provider function to create classic UI
		void create_gui();
	};

}

#include <cgv/config/lib_end.h>