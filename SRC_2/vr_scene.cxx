#include "vr_scene.h"
#include <cgv/base/register.h>
#include <cgv/math/ftransform.h>
#include <cgv/math/pose.h>
#include <cg_vr/vr_events.h>
#include <random>

namespace vr {

	void vr_scene::construct_table(float tw, float td, float th, float tW, float tO, rgb table_clr, rgb leg_clr)
	{
		float x0 = -0.5f * tw;
		float x1 = -0.5f * tw + tO;
		float x2 = -0.5f * tw + tO + tW;
		float x3 = 0.5f * tw - tO - tW;
		float x4 = 0.5f * tw - tO;
		float x5 = 0.5f * tw;
		float y0 = 0;
		float y1 = th - tW;
		float y2 = th;
		float z0 = -0.5f * td;
		float z1 = -0.5f * td + tO;
		float z2 = -0.5f * td + tO + tW;
		float z3 = 0.5f * td - tO - tW;
		float z4 = 0.5f * td - tO;
		float z5 = 0.5f * td;
		boxes.push_back(box3(vec3(x0, y1, z0), vec3(x5, y2, z5))); box_colors.push_back(table_clr);

		boxes.push_back(box3(vec3(x1, y0, z1), vec3(x2, y1, z2))); box_colors.push_back(leg_clr);
		boxes.push_back(box3(vec3(x3, y0, z1), vec3(x4, y1, z2))); box_colors.push_back(leg_clr);
		boxes.push_back(box3(vec3(x3, y0, z3), vec3(x4, y1, z4))); box_colors.push_back(leg_clr);
		boxes.push_back(box3(vec3(x1, y0, z3), vec3(x2, y1, z4))); box_colors.push_back(leg_clr);
	}

	void vr_scene::construct_room(float w, float d, float h, float W, bool walls, bool ceiling) {
		// construct floor
		boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d), vec3(0.5f * w, 0, 0.5f * d)));
		box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

		if (walls) {
			// construct walls
			boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w, h, -0.5f * d)));
			box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
			boxes.push_back(box3(vec3(-0.5f * w, -W, 0.5f * d), vec3(0.5f * w, h, 0.5f * d + W)));
			box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

			boxes.push_back(box3(vec3(0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w + W, h, 0.5f * d + W)));
			box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
		}
		if (ceiling) {
			// construct ceiling
			boxes.push_back(box3(vec3(-0.5f * w - W, h, -0.5f * d - W), vec3(0.5f * w + W, h + W, 0.5f * d + W)));
			box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
		}
	}

	void vr_scene::construct_environment(float s, float ew, float ed, float w, float d, float h) {
		std::default_random_engine generator;
		std::uniform_real_distribution<float> distribution(0, 1);
		unsigned n = unsigned(ew / s);
		unsigned m = unsigned(ed / s);
		float ox = 0.5f * float(n) * s;
		float oz = 0.5f * float(m) * s;
		for (unsigned i = 0; i < n; ++i) {
			float x = i * s - ox;
			for (unsigned j = 0; j < m; ++j) {
				float z = j * s - oz;
				if (fabsf(x) < 0.5f * w && fabsf(x + s) < 0.5f * w && fabsf(z) < 0.5f * d && fabsf(z + s) < 0.5f * d)
					continue;
				float h = 0.2f * (std::max(abs(x) - 0.5f * w, 0.0f) + std::max(abs(z) - 0.5f * d, 0.0f)) * distribution(generator) + 0.1f;
				boxes.push_back(box3(vec3(x, 0.0f, z), vec3(x + s, h, z + s)));
				rgb color = cgv::media::color<float, cgv::media::HLS>(distribution(generator), 0.1f * distribution(generator) + 0.15f, 0.3f);
				box_colors.push_back(color);
			}
		}
	}

	void vr_scene::build_scene(float w, float d, float h, float W)
	{
		//construct_room(w, d, h, W, false, false);
		construct_environment(0.3f, 3 * w, 3 * d, w, d, h);
		//construct_table(table_width, table_depth, table_height, leg_width, leg_offset, table_color, leg_color);
	}

	vr_scene::vr_scene()
	{
		set_name("vr_scene");
		vr_view_ptr = 0;

		table_color = rgb(0.3f, 0.2f, 0.0f);
		table_width = 1.6f;
		table_depth = 0.8f;
		table_height = 0.7f;
		leg_color = rgb(0.2f, 0.1f, 0.1f);
		leg_width = 0.04f;
		leg_offset = 0.0f;

		build_scene(10, 10, 3, 0.2f);

		pixel_scale = 0.001f;

		on_set(&table_width);

		state[0] = state[1] = state[2] = state[3] = IS_NONE;

		trig = cgv::gui::trigger();
		incr = 60;
		cgv::signal::connect(trig.shoot, this, &vr_scene::change_time);
	}

	bool vr_scene::self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("table_color", table_color) &&
			rh.reflect_member("table_width", table_width) &&
			rh.reflect_member("table_depth", table_depth) &&
			rh.reflect_member("table_height", table_height) &&
			rh.reflect_member("table_leg color", leg_color) &&
			rh.reflect_member("table_legs", leg_width) &&
			rh.reflect_member("table_offset", leg_offset);

	}

	void vr_scene::on_set(void* member_ptr)
	{
		if (member_ptr >= &table_width && member_ptr < &leg_color + 1) {
			boxes.resize(boxes.size() - 5);
			box_colors.resize(box_colors.size() - 5);
			//construct_table(table_width, table_depth, table_height, leg_width, leg_offset, table_color, leg_color);
		}
		update_member(member_ptr);
		post_redraw();
	}

	vector<string> get_all_files_names_within_folder(string folder)
	{
		vector<string> names = vector<string>();
		string search_path = folder + "/*.*";
		std::wstring wc(search_path.size(), L'#');
		mbstowcs(&wc[0], search_path.c_str(), search_path.size());
		WIN32_FIND_DATA fd;
		HANDLE hFind = ::FindFirstFile(wc.c_str(), &fd);
		if (hFind != INVALID_HANDLE_VALUE) {
			do {
				// read all (real) files in current folder
				// , delete '!' read other 2 default folder . and ..
				if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
					wstring ws(fd.cFileName);
					string filename(ws.begin(), ws.end());
					names.push_back(filename);
				}
			} while (::FindNextFile(hFind, &fd));
			::FindClose(hFind);
		}
		else {
			std::cerr << GetLastError() << std::endl;
		}
		return names;
	}

	bool vr_scene::init(cgv::render::context& ctx)
	{
		cgv::render::ref_sphere_renderer(ctx, 1);
		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_rounded_cone_renderer(ctx, 1);
		cgv::gui::connect_vr_server(true);
		lm.init(ctx);
		cgv::media::font::font_ptr f = cgv::media::font::find_font("Courier New");
		lm.set_font_face(f->get_font_face(cgv::media::font::FFA_BOLD));
		lm.set_font_size(36.0f);
		lm.set_text_color(rgba(0, 0, 0, 1));

		auto view_ptr = find_view_as_node();
		if (view_ptr) {
			// if the view points to a vr_view_interactor
			vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
			if (vr_view_ptr) {
				// configure vr event processing
				vr_view_ptr->set_event_type_flags(
					cgv::gui::VREventTypeFlags(
						cgv::gui::VRE_KEY +
						cgv::gui::VRE_ONE_AXIS +
						cgv::gui::VRE_ONE_AXIS_GENERATES_KEY +
						cgv::gui::VRE_TWO_AXES +
						cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
						cgv::gui::VRE_POSE
					));
				vr_view_ptr->enable_vr_event_debugging(false);
				//configure vr rendering
				vr_view_ptr->draw_action_zone(false);
				vr_view_ptr->draw_vr_kits(true);
				vr_view_ptr->enable_blit_vr_views(true);
				vr_view_ptr->set_blit_vr_view_width(200);
			}
		}
		grabber_throttle_1 = 0;
		grabber_throttle_2 = 0;
		pause = true;
		back = false;
		ticker = time(0);

		/**
		* Time interval
		*/
		visual_now = time(0);
		tm v_min_2_tm = *gmtime(&visual_now);
		v_min_2_tm.tm_year -= 2;
		v_min_2 = mktime(&v_min_2_tm);
		tm v_plus_2_tm = *gmtime(&visual_now);
		v_plus_2_tm.tm_year += 2;
		v_plus_2 = mktime(&v_plus_2_tm);

		old_time = visual_now - 1000000;



		/**
		* Construction of Earth
		*/
		if (earth_sphere.read(get_input_directory() + "/../models/sphere_36_cuts.obj")) {
			earth_info.construct(ctx, earth_sphere);
			earth_info.bind(ctx, ctx.ref_surface_shader_program(true), true);
			auto& mats = earth_info.get_materials();
			if (mats.size() > 0) {
				//setup texture paths
				int di = mats[0]->add_image_file(get_input_directory() + "/../images/earthmap.jpg");

				//ensure the textures are loaded
				mats[0]->ensure_textures(ctx);

				//set texture indices
				mats[0]->set_diffuse_index(di);
			}
		}
		/**
		* Reading all files from directory
		*/
		srand(time(0));
		actives = std::vector<pair<string, bool>>();
		satellites = std::map<string, std::vector<pair<cSatellite, bool>>>();
		orbit_styles = std::map<string, cgv::render::rounded_cone_render_style>();
		sat_styles = std::map<string, cgv::render::sphere_render_style>();
		string path = get_input_directory() + "/../sat_data";
		auto result = get_all_files_names_within_folder(path);
		for (const auto& entry : result) {
			std::string line, line1, line2, line3;
			std::ifstream file_reader(path + "/" + entry);
			int cpt = 0;
			std::vector<pair<cSatellite, bool>> satels = std::vector<pair<cSatellite, bool>>();
			if (file_reader.is_open())
			{
				while (std::getline(file_reader, line))
				{
					if (cpt % 3 == 0) {
						line1 = line;
					}
					else if (cpt % 3 == 1) {
						line2 = line;
					}
					else if (cpt % 3 == 2) {
						line3 = line;
						satels.push_back(pair<cSatellite, bool>(cSatellite(cTle(line1, line2, line3)), false));
					}
					cpt++;
				}
				file_reader.close();
			}
			cpt = 0;
			actives.push_back(pair<string, bool>(entry, false));
			satellites.insert(pair<string, std::vector<pair<cSatellite, bool>>>(entry, satels));
			cgv::render::rounded_cone_render_style rend = cgv::render::rounded_cone_render_style();
			rend.surface_color = rgba(static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 1)),
				static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 1)), static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / 1)), 1);
			rend.radius = 0.01f;
			orbit_styles.insert(pair<string, cgv::render::rounded_cone_render_style>(entry, rend));
			cgv::render::sphere_render_style rend_ptx = cgv::render::sphere_render_style();
			rend_ptx.surface_color = rend.surface_color;
			rend_ptx.radius = 0.0125F;
			sat_styles.insert(pair<string, cgv::render::sphere_render_style>(entry, rend_ptx));
		}
		ptx_style = cgv::render::sphere_render_style();
		ptx_style.radius = 0.00625F;
		orbit_style = cgv::render::rounded_cone_render_style();
		orbit_style.radius = ptx_style.radius / 2;
		//sat_pos = std::map<cSatellite, std::vector<vec3>>();
		//sat_orbit_pos = std::map<cSatellite, std::vector<vec3>>();


		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_sphere_renderer(ctx, 1);
		cgv::render::ref_rounded_cone_renderer(ctx, 1);
		//cgv::render::ref_point_renderer(ctx, 1);

		return true;
	}

	void vr_scene::init_frame(cgv::render::context& ctx)
	{
		bool repack = lm.is_packing_outofdate();
		lm.ensure_texture_uptodate(ctx);
		if (repack) {
			for (uint32_t li = 0; li < label_texture_ranges.size(); ++li)
				label_texture_ranges[li] = lm.get_texcoord_range(li);
		}
	}

	void vr_scene::clear(cgv::render::context& ctx)
	{
		cgv::render::ref_box_renderer(ctx, -1);
		lm.destruct(ctx);
	}

	void vr_scene::draw(cgv::render::context& ctx)
	{
		// activate render styles
		auto& br = cgv::render::ref_box_renderer(ctx);
		br.set_render_style(style);
		auto& rr = cgv::render::ref_rectangle_renderer(ctx);
		rr.set_render_style(prs);

		// draw static part
		br.set_box_array(ctx, boxes);
		br.set_color_array(ctx, box_colors);
		br.render(ctx, 0, boxes.size());

		// compute label poses in lab coordinate system
		std::vector<vec3> P;
		std::vector<quat> Q;
		std::vector<vec2> E;
		std::vector<vec4> T;
		mat34 pose[5];
		bool valid[5];
		valid[CS_LAB] = valid[CS_TABLE] = true;
		pose[CS_LAB].identity();
		pose[CS_TABLE].identity();
		cgv::math::pose_position(pose[CS_TABLE]) = vec3(0.0f, table_height, 0.0f);
		valid[CS_HEAD] = vr_view_ptr && vr_view_ptr->get_current_vr_state() && vr_view_ptr->get_current_vr_state()->hmd.status == vr::VRS_TRACKED;
		if (valid[CS_HEAD])
			pose[CS_HEAD] = reinterpret_cast<const mat34&>(vr_view_ptr->get_current_vr_state()->hmd.pose[0]);
		valid[CS_LEFT_CONTROLLER] = vr_view_ptr && vr_view_ptr->get_current_vr_state() && vr_view_ptr->get_current_vr_state()->controller[0].status == vr::VRS_TRACKED;
		if (valid[CS_LEFT_CONTROLLER])
			pose[CS_LEFT_CONTROLLER] = reinterpret_cast<const mat34&>(vr_view_ptr->get_current_vr_state()->controller[0].pose[0]);
		valid[CS_RIGHT_CONTROLLER] = vr_view_ptr && vr_view_ptr->get_current_vr_state() && vr_view_ptr->get_current_vr_state()->controller[1].status == vr::VRS_TRACKED;
		if (valid[CS_RIGHT_CONTROLLER])
			pose[CS_RIGHT_CONTROLLER] = reinterpret_cast<const mat34&>(vr_view_ptr->get_current_vr_state()->controller[1].pose[0]);
		for (uint32_t li = 0; li < label_coord_systems.size(); ++li) {
			if (!label_visibilities[li] || !valid[label_coord_systems[li]])
				continue;
			mat34 label_pose = cgv::math::pose_construct(label_orientations[li], label_positions[li]);
			cgv::math::pose_transform(pose[label_coord_systems[li]], label_pose);
			P.push_back(cgv::math::pose_position(label_pose));
			Q.push_back(quat(cgv::math::pose_orientation(label_pose)));
			E.push_back(label_extents[li]);
			T.push_back(label_texture_ranges[li]);
		}
		// draw labels
		if (!P.empty()) {
			rr.set_position_array(ctx, P);
			rr.set_rotation_array(ctx, Q);
			rr.set_extent_array(ctx, E);
			rr.set_texcoord_array(ctx, T);
			lm.get_texture()->enable(ctx);
			rr.render(ctx, 0, P.size());
			lm.get_texture()->disable(ctx);
		}

		if (!earth_info.is_constructed())
			return;
		//push back a model view matrix to transform the rendering of this mesh
		ctx.push_modelview_matrix();


		// translate and scale
		double R = 0.5;
		ctx.mul_modelview_matrix(
			cgv::math::translate4<double>(vec3(0, 1.0, 0)) *
			cgv::math::scale4<double>(dvec3(vec3(1, 1, 1) * R)) *
			cgv::math::rotate4<double>(-0.0F, vec3(1, 0, 0))
		);

		// actually draw the mesh
		earth_info.draw_all(ctx);

		// restore the previous transform
		ctx.pop_modelview_matrix();


		//Draw of orbits and satellites
		int calc_actives = 0;
		for (pair<string, bool> p : actives) {
			if (p.second)
				calc_actives++;
		}
		if (calc_actives != nb_active) {
			calculate_positions_and_orbits();
			nb_active = calc_actives;
		}
		if (all_pos_orbit.size() != 0) { //orbit for selected satellites
			auto& orbit = cgv::render::ref_rounded_cone_renderer(ctx);
			orbit.set_render_style(orbit_style);
			orbit.set_position_array(ctx, all_pos_orbit);
			orbit.set_color_array(ctx, all_colors_orbit);

			orbit.validate_and_enable(ctx);
			orbit.draw(ctx, 0, all_pos_orbit.size());
			orbit.disable(ctx);
		}
		if (all_pos_sat.size() != 0) { //position only
			auto& ptx = cgv::render::ref_sphere_renderer(ctx);
			ptx.set_render_style(ptx_style);
			ptx.set_position_array(ctx, all_pos_sat);
			ptx.set_color_array(ctx, all_colors_sat);

			ptx.validate_and_enable(ctx);
			ptx.draw(ctx, 0, all_pos_sat.size());
			ptx.disable(ctx);
		}

	}

	void vr_scene::stream_help(std::ostream& os)
	{
		os << "vr_scene: navigate scenes with direction pad left and right and save with down" << std::endl;
	}

	bool vr_scene::handle(cgv::gui::event& e)
	{
		// check if vr event flag is not set and don't process events in this case
		if ((e.get_flags() & cgv::gui::EF_VR) == 0)
			return false;
		// check event id
		switch (e.get_kind()) {
		case cgv::gui::EID_KEY:
		{
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			if (vrke.get_action() != cgv::gui::KA_RELEASE) {
				switch (vrke.get_key()) {
				case vr::VR_GRIP:
					std::cout << "grip button " << (vrke.get_controller_index() == 0 ? "left" : "right") << " controller pressed" << std::endl;
					return true;
				case vr::VR_DPAD_RIGHT:
					std::cout << "touch pad of " << (vrke.get_controller_index() == 0 ? "left" : "right") << " controller pressed at right direction" << std::endl;
					return true;
				case vr::VR_A:
					back = !back;
					if (back) {
						incr = -60;
					}
					else {
						incr = 60;
					}
					return true;
				}
			}
			break;
		}
		case cgv::gui::EID_THROTTLE:
		{
			cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);
			std::cout << "throttle " << vrte.get_throttle_index() << " of controller " << vrte.get_controller_index()
				<< " adjusted from " << vrte.get_last_value() << " to " << vrte.get_value() << std::endl;
			if (vrte.get_throttle_index() == 1 && vrte.get_value() > 0.5f) {
				grabber_throttle_1 = 1;
			}
			else {
				grabber_throttle_1 = 0;
			}
			if (vrte.get_throttle_index() == 2 && vrte.get_value() > 0.5f) {
				grabber_throttle_2 = 2;
			}
			else {
				grabber_throttle_2 = 0;
			}

			return true;
		}
		case cgv::gui::EID_STICK:
		{
			cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
			switch (vrse.get_action()) {
			case cgv::gui::SA_TOUCH:
				if (state[vrse.get_controller_index()] == IS_OVER)
				state[vrse.get_controller_index()] = IS_GRAB;
				break;
			case cgv::gui::SA_RELEASE:
				if (state[vrse.get_controller_index()] == IS_GRAB)
				state[vrse.get_controller_index()] = IS_OVER;
				break;
			case cgv::gui::SA_PRESS:
				pause = !pause;
				if (pause) {
					trig.stop();
				}
				else {
					if (trig.schedule_recuring(10))
						cout << "Play" << endl;
				}
			case cgv::gui::SA_UNPRESS:
				std::cout << "stick " << vrse.get_stick_index()
					<< " of controller " << vrse.get_controller_index()
					<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
					<< " at " << vrse.get_x() << ", " << vrse.get_y() << std::endl;

				return true;
			case cgv::gui::SA_MOVE:
			case cgv::gui::SA_DRAG:
				std::cout << "stick " << vrse.get_stick_index()
					<< " of controller " << vrse.get_controller_index()
					<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
					<< " from " << vrse.get_last_x() << ", " << vrse.get_last_y()
					<< " to " << vrse.get_x() << ", " << vrse.get_y() << std::endl;
				return true;
			}
			return true;
		}
		case cgv::gui::EID_POSE:
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
			// check for controller pose events
			int ci = vrpe.get_trackable_index();
			if (ci != -1) {
				vec3 origin, direction;
				vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
				string result = intersection(origin, direction);
				vector<string> res = vector<string>();
				std::string delimiter = "*&_";

				size_t pos = 0;
				while ((pos = result.find(delimiter)) != std::string::npos) {
					res.push_back(result.substr(0, pos));
					result.erase(0, pos + delimiter.length());
				}
				res.push_back(result);
				if (res.size() == 2) {
					for (auto j = satellites.begin(); j != satellites.end(); j++) {
						if (j->first == res[0]) {
							for (int i = 0; i < j->second.size(); i++) {
								if (j->second[i].first.Name() == res[1]) {
									if (grabber_throttle_1 == 1 && !(j->second[i].second)) {
										j->second[i].second = true;
										calculate_positions_and_orbits();
									}
									else if (grabber_throttle_2 == 2 && (j->second[i].second)) {
										j->second[i].second = false;
										calculate_positions_and_orbits();
									}
								}

							}
						}
					}

				}
				post_redraw();
			}
			return true;
		}
		return false;
	}

	void vr_scene::calculate_positions_and_orbits() {
		all_pos_sat = std::vector<vec3>();
		all_pos_orbit = std::vector<vec3>();
		all_colors_sat = std::vector<vec3>();
		all_colors_orbit = std::vector<vec3>();
		names_plus_pos = std::vector<pair<string, vec3>>();
		//draw orbits and satellites
		for (auto datasets_entry : actives) { //all datasets
			if (datasets_entry.second) { //if dataset selected
				auto sats = satellites.at(datasets_entry.first); //find satellites list
				for (auto sat_entry : sats) { //for all satellites in the list
					std::vector<vec3> pos = std::vector<vec3>(); double rev_time = (1.0 / sat_entry.first.Orbit().MeanMotion()) * (2 * M_PI) * 60;
					if (sat_entry.second) { //if satellite selected for orbit drawn
						/**
						* Calculation of one revolution orbit
						*/
						tm timer = *gmtime(&visual_now);
						timer.tm_isdst = -1;

						timer.tm_sec -= rev_time; //calculate time one orbit earlier
						time_t min_one_rev = mktime(&timer);
						std::vector<vec3> col_pos = std::vector<vec3>();
						for (float t = 0; t <= (visual_now - min_one_rev); t++) {
							//std::cout << t << std::endl;
							cVector v;
							try {

								v = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(min_one_rev + t))).Position();
								if (t == 0 || t == (visual_now - min_one_rev - 1)) {
									pos.push_back(vec3(v.m_x / (2.F*6378), 1.0F + v.m_z / (2.F*6378), v.m_y / (2.F*6378)));
									col_pos.push_back(vec3(orbit_styles.at(datasets_entry.first).surface_color.R(), orbit_styles.at(datasets_entry.first).surface_color.G(),
										orbit_styles.at(datasets_entry.first).surface_color.B()));
								}
								else {
									pos.push_back(vec3(v.m_x / (2.F*6378), 1.0F + v.m_z / (2.F*6378), v.m_y / (2.F*6378)));
									pos.push_back(vec3(v.m_x / (2.F*6378), 1.0F + v.m_z / (2.F*6378), v.m_y / (2.F*6378)));
									col_pos.push_back(vec3(orbit_styles.at(datasets_entry.first).surface_color.R(), orbit_styles.at(datasets_entry.first).surface_color.G(),
										orbit_styles.at(datasets_entry.first).surface_color.B()));
									col_pos.push_back(vec3(orbit_styles.at(datasets_entry.first).surface_color.R(), orbit_styles.at(datasets_entry.first).surface_color.G(),
										orbit_styles.at(datasets_entry.first).surface_color.B()));
								}
							}
							catch (cDecayException e) {
								std::vector<vec3> vec = std::vector<vec3>();
								vec.push_back(vec3());
								//sat_pos.insert(pair<string, std::vector<vec3>>(sat_entry.first.Name(), vec)); //Insert empty value for invalid satellites
								std::cerr << e.GetSatelliteName() + " is in the ground right now..." << std::endl;
							}
							catch (cPropagationException e) {
								std::vector<vec3> vec = std::vector<vec3>();
								vec.push_back(vec3());
								//sat_pos.insert(pair<string, std::vector<vec3>>(sat_entry.first.Name(), vec)); //Insert empty value for invalid satellites
								//std::cerr << e.Message() << std::endl;
							}
						}
						//sat_orbit_pos.insert(pair<string, std::vector<vec3>>(sat_entry.first.Name(), pos));
						all_pos_orbit.insert(all_pos_orbit.end(), pos.begin(), pos.end());
						all_colors_orbit.insert(all_colors_orbit.end(), col_pos.begin(), col_pos.end());
					} //if satellite not selected for orbit
					cVector v;
					pos = std::vector<vec3>();
					std::vector<vec3> col_pos = std::vector<vec3>();
					try {
						v = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(visual_now))).Position();
						vec3 vec = vec3(v.m_x / (2.0F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378));
						pos.push_back(vec);
						//sat_pos.insert(pair<string, std::vector<vec3>>(sat_entry.first.Name(), pos));
						col_pos.push_back(vec3(sat_styles.at(datasets_entry.first).surface_color.R(), sat_styles.at(datasets_entry.first).surface_color.G(),
							sat_styles.at(datasets_entry.first).surface_color.B()));
						names_plus_pos.push_back(pair<string, vec3>(datasets_entry.first + "*&_" + sat_entry.first.Name(), vec));
					}
					catch (cDecayException e) {
						std::vector<vec3> vec = std::vector<vec3>();
						vec.push_back(vec3());
						//sat_pos.insert(pair<string, std::vector<vec3>>(sat_entry.first.Name(), vec)); //Insert empty value for invalid satellites
						std::cerr << e.GetSatelliteName() + " is in the ground right now..." << std::endl;
					}
					catch (cPropagationException e) {
						std::vector<vec3> vec = std::vector<vec3>();
						vec.push_back(vec3());
						//sat_pos.insert(pair<string, std::vector<vec3>>(sat_entry.first.Name(), vec)); //Insert empty value for invalid satellites
						//std::cerr << e.Message() << std::endl;
					}
					all_pos_sat.insert(all_pos_sat.end(), pos.begin(), pos.end());
					all_colors_sat.insert(all_colors_sat.end(), col_pos.begin(), col_pos.end());
				}
			}
		}
	}

	string vr_scene::intersection(vec3 origin, vec3 direction) {
		for (auto entry : names_plus_pos) {
			float dist = cgv::math::length(cgv::math::cross(entry.second - origin, direction))
				/ cgv::math::length(direction);
			if (dist < 2 * ptx_style.radius) {
				return entry.first;
			}
		}
		return "";
	}

	void vr_scene::change_time(double, double dt) {
		visual_now += incr;
		calculate_positions_and_orbits();
	}

	void vr_scene::create_gui()
	{
		add_decorator("vr_scene", "heading");
		if (begin_tree_node("table", table_width)) {
			align("\a");
			add_member_control(this, "color", table_color);
			add_member_control(this, "width", table_width, "value_slider", "min=0.1;max=3.0;ticks=true");
			add_member_control(this, "depth", table_depth, "value_slider", "min=0.1;max=3.0;ticks=true");
			add_member_control(this, "height", table_height, "value_slider", "min=0.1;max=3.0;ticks=true");
			add_member_control(this, "leg color", leg_color);
			add_member_control(this, "legs", leg_width, "value_slider", "min=0.0;max=0.3;ticks=true");
			add_member_control(this, "offset", leg_offset, "value_slider", "min=0.0;max=0.5;ticks=true");
			align("\b");
			end_tree_node(table_width);
		}

		align("\a");
		std::stringstream ss;
		ss << v_min_2;
		std::string ts = ss.str();
		std::stringstream ss_plus;
		ss_plus << v_plus_2;
		std::string ts_plus = ss_plus.str();
		add_member_control(this, "Time selection", visual_now, "value_slider", "min=" + ts + ";step=1;max=" + ts_plus + ";log=true;ticks=true");
		align("\b");

		if (begin_tree_node("Datasets", is_active)) {
			align("\a");
			for (int cpt = 0; cpt < actives.size(); cpt++) {
				add_member_control(this, actives[cpt].first, actives[cpt].second);
			}
			align("\b");
			end_tree_node(is_active);
		}
	}

}
#include <cgv/base/register.h>

cgv::base::object_registration<vr::vr_scene> vr_scene_reg("vr_scene");
