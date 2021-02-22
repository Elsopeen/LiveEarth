#include "vr_scene.h"
#include <cgv/base/register.h>
#include <cgv/math/ftransform.h>
#include <cgv/math/pose.h>
#include <cg_vr/vr_events.h>
#include <random>

namespace vr {

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
		construct_environment(0.3f, 3 * w, 3 * d, w, d, h);
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

		/// Creation of the trigger event and link to its signal
		trig = cgv::gui::trigger();
		incr = 60;
		cgv::signal::connect(trig.shoot, this, &vr_scene::change_time_queue);
		forback = false;

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
		* and creating corresponding satellite object
		*/
		{
			srand(time(0));
			actives = std::map<string, bool>();
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
				actives.insert(pair<string, bool>(entry, false));
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
		}

		/**
		* Labels for listing selected datasets and orbits
		*/
		{
			listing_datasets_label = add_label("Datasets: ", rgba(252.F / 256, 186.F / 256, 3.F / 256, 1), 10, 10);
			fix_label_size(listing_datasets_label);
			place_label(listing_datasets_label, vec3(0, 2, 0), quat(), CS_LAB, LA_CENTER);
			listing_orbits_label = add_label("Orbits: ", rgba(252.F / 256, 186.F / 256, 3.F / 256, 1));
			fix_label_size(listing_orbits_label);
			place_label(listing_orbits_label, vec3(0, 1.75, 0), quat(), CS_LAB, LA_CENTER);
		}

		sat_queue.clear();
		anim_thread = thread(nullptr);

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
		/// calculate the direction matrix of the headset
		mat3 look_dir;
		for (int i = 0; i < 9; i++) {
			look_dir[i] = vr_view_ptr->get_current_vr_state()->hmd.pose[i];
		}
		quat quat_total = quat(look_dir);
		//update directions of labels
		place_label(listing_datasets_label, vec3(0, 2, 0), quat_total, CS_LAB, LA_CENTER);
		place_label(listing_orbits_label, vec3(0, 1.75, 0), quat_total, CS_LAB, LA_CENTER);
		for (auto entry : li_sat) { //update for sat labels only if shown
			if (label_visibilities[entry.second]) {
				place_label(li_sat.at(entry.first), all_pos_sat_interp.at(entry.first) + vec3(0, 0.025, 0), quat_total, CS_LAB, LA_CENTER);
			}
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
		if (all_pos_orbit.size() != 0) { //orbit for selected satellites
			auto& orbit = cgv::render::ref_rounded_cone_renderer(ctx);
			orbit.set_render_style(orbit_style);
			orbit.set_position_array(ctx, sat_queue.orbits());
			orbit.set_color_array(ctx, sat_queue.col_orbits());

			orbit.validate_and_enable(ctx);
			orbit.draw(ctx, 0, sat_queue.orbits().size());
			orbit.disable(ctx);
		}
		all_pos_sat_interp = sat_queue.interpolate_at(*this, visual_now);
		if (all_pos_sat_interp.size() != 0) { //position only
			auto& ptx = cgv::render::ref_sphere_renderer(ctx);
			ptx.set_render_style(ptx_style);
			vector<vec3> pos_sat_inter = vector<vec3>();
			for (auto entry : all_pos_sat_interp) {
				pos_sat_inter.push_back(entry.second);
			}
			ptx.set_position_array(ctx, pos_sat_inter);
			ptx.set_color_array(ctx, all_colors_sat);

			ptx.validate_and_enable(ctx);
			ptx.draw(ctx, 0, all_pos_sat_interp.size());
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
					forback = !forback;
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
				/// Pauses and resumes the animation
				pause = !pause;
				if (pause) {
					trig.stop();
				}
				else {
					trig.schedule_recuring(1.0F/60.F);
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
				vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));//get direction and origin of controller
				string result = intersection(origin, direction);//find closest satellite along the axis of the controller
				vector<string> res = vector<string>();
				std::string delimiter = "*&_";

				size_t pos = 0;
				while ((pos = result.find(delimiter)) != std::string::npos) { //split the result
					res.push_back(result.substr(0, pos));
					result.erase(0, pos + delimiter.length());
				}
				res.push_back(result);
				if (res.size() == 2) {
					for (auto j = satellites.begin(); j != satellites.end(); j++) {
						if (j->first == res[0]) { //if dataset's name equals intersection's result's dataset's name
							for (int i = 0; i < j->second.size(); i++) {
								if (j->second[i].first.Orbit().SatId() == res[1]) { //if satellite's ID == intersection's result's satID
									if (grabber_throttle_1 == 1 && !(j->second[i].second)) { //trigger pressed and not already selected
										j->second[i].second = true;
										sat_queue.clear();
										sat_queue.calculate_positions_at(*this, visual_now);
										show_label(li_sat.at(res[0] + "*&_" + res[1]));
									}
									else if (grabber_throttle_2 == 2 && (j->second[i].second)) { //grip pressed and already selected
										j->second[i].second = false;
										hide_label(li_sat.at(res[0] + "*&_" + res[1]));
										sat_queue.clear();
										sat_queue.calculate_positions_at(*this, visual_now);
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

	void vr_scene::state_queue::calculate_positions_at(vr_scene& obj, time_t timestamp) {
		satellite_state state;
		state.timestamp = timestamp;
		state.all_colors_orbit.clear();
		state.all_colors_sat.clear();
		state.orbits.clear();
		state.satellite_positions.clear();

		for (auto entry : obj.li_sat) {
			obj.label_visibilities[entry.second] = false;
		}
		obj.li_sat.clear();
		string datasets_label = "";
		string orbits_label = "";
		int cptdat = 0;
		int cptor = 0;
		for (auto datasets_entry : obj.actives) {
			if (datasets_entry.second) {
				if (cptdat % 3 == 0) {
					datasets_label += datasets_entry.first + ", \n";
					cptdat++;
				}
				else {
					datasets_label += datasets_entry.first + ", ";
					cptdat++;
				}
			}
			auto sats = obj.satellites.at(datasets_entry.first);
			for (auto sat_entry : sats) {
				string full_name = datasets_entry.first + "*&_" + sat_entry.first.Orbit().SatId();
				// all satellites
				cVector v;
				std::vector<vec3> col_pos = std::vector<vec3>();
				try {
					v = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(timestamp))).Position();
					vec3 vec = vec3(v.m_x / (2.0F * 6378.F), v.m_z / (2.0F * 6378.F), v.m_y / (2.0F * 6378.F));
					col_pos.push_back(vec3(obj.sat_styles.at(datasets_entry.first).surface_color.R(), obj.sat_styles.at(datasets_entry.first).surface_color.G(),
						obj.sat_styles.at(datasets_entry.first).surface_color.B()));
					state.satellite_positions.insert(pair<string, vec3>(full_name, vec));
					obj.li_sat.insert(pair<string, uint32_t>(full_name, obj.add_label("Dataset  : " + datasets_entry.first + "\nSat name : " + sat_entry.first.Name(),
						rgba(0.8F, 0.6F, 0.8F, 1.0F)))); //create label corresponding to said satellite
					obj.fix_label_size(obj.li_sat.at(full_name));
					obj.hide_label(obj.li_sat.at(full_name));//hide it
				}
				catch (cDecayException e) {
					std::vector<vec3> vec = std::vector<vec3>();
					vec.push_back(vec3());
					std::cerr << e.GetSatelliteName() + " is in the ground right now..." << std::endl;
				}
				catch (cPropagationException e) {
					std::vector<vec3> vec = std::vector<vec3>();
					vec.push_back(vec3());
				}
				state.all_colors_sat.insert(state.all_colors_sat.end(), col_pos.begin(), col_pos.end());
				if (sat_entry.second) {
					std::vector<vec3> pos = std::vector<vec3>();
					double rev_time = (1.0 / sat_entry.first.Orbit().MeanMotion()) * (2 * M_PI) * 60;
					if (cptor % 3 == 0) {
						orbits_label += sat_entry.first.Name() + ", \n";
						cptor++;
					}
					else {
						orbits_label += sat_entry.first.Name() + ", ";
						cptor++;
					}
					/**
					* Calculation of one revolution orbit
					*/
					tm timer = *gmtime(&timestamp);
					timer.tm_isdst = -1;

					timer.tm_sec -= rev_time; //calculate time one orbit earlier
					time_t min_one_rev = mktime(&timer);
					pos = std::vector<vec3>();
					std::vector<vec3> col_pos = std::vector<vec3>();
					for (float t = 0; t <= (timestamp - min_one_rev); t++) {
						//std::cout << t << std::endl;
						cVector v;
						try {

							v = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(min_one_rev + t))).Position();
							if (t == 0 || t == (timestamp - min_one_rev - 1)) {
								pos.push_back(vec3(v.m_x / (2.F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378)));
								col_pos.push_back(vec3(obj.orbit_styles.at(datasets_entry.first).surface_color.R(), obj.orbit_styles.at(datasets_entry.first).surface_color.G(),
									obj.orbit_styles.at(datasets_entry.first).surface_color.B()));
							}
							else {
								pos.push_back(vec3(v.m_x / (2.F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378)));
								pos.push_back(vec3(v.m_x / (2.F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378)));
								col_pos.push_back(vec3(obj.orbit_styles.at(datasets_entry.first).surface_color.R(), obj.orbit_styles.at(datasets_entry.first).surface_color.G(),
									obj.orbit_styles.at(datasets_entry.first).surface_color.B()));
								col_pos.push_back(vec3(obj.orbit_styles.at(datasets_entry.first).surface_color.R(), obj.orbit_styles.at(datasets_entry.first).surface_color.G(),
									obj.orbit_styles.at(datasets_entry.first).surface_color.B()));
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
					obj.show_label(obj.li_sat.at(full_name));
					//sat_orbit_pos.insert(pair<string, std::vector<vec3>>(sat_entry.first.Name(), pos));
					state.orbits.insert(state.orbits.end(), pos.begin(), pos.end());
					state.all_colors_orbit.insert(state.all_colors_orbit.end(), col_pos.begin(), col_pos.end());
				}

			}

		}
		obj.lm.update_label_text(obj.listing_datasets_label, "Datasets: " + datasets_label);
		obj.lm.update_label_size(obj.listing_datasets_label, -1, -1); //update size as list grows
		obj.lm.update_label_text(obj.listing_orbits_label, "Orbits: " + orbits_label);
		obj.lm.update_label_size(obj.listing_orbits_label, -1, -1); //update size as list grows

		states.push_back(state);
	}

	map<string, cgv::render::render_types::vec3> vr_scene::state_queue::interpolate_at(vr_scene& obj, time_t temp_now) {
		for (int i = states.size(); i < 3; i++) {
			if (!obj.forback)
				calculate_positions_at(obj, states.back().timestamp + 450);
			else
				calculate_positions_at(obj, states.back().timestamp - 450);
		}
		map<string, render_types::vec3> return_obj;
		for (auto i = states[0].satellite_positions.begin(); i != states[0].satellite_positions.end(); i++) {
			auto lagrange_start = ((double)(temp_now - states[1].timestamp) / (states[0].timestamp - states[1].timestamp)) *
				((double)(temp_now - states[2].timestamp) / (states[0].timestamp - states[2].timestamp));
			auto lagrange_mid = ((double)(temp_now - states[0].timestamp) / (states[1].timestamp - states[0].timestamp)) *
				((double)(temp_now - states[2].timestamp) / (states[1].timestamp - states[2].timestamp));
			auto lagrange_end = ((double)(temp_now - states[0].timestamp) / (states[2].timestamp - states[0].timestamp)) *
				((double)(temp_now - states[1].timestamp) / (states[2].timestamp - states[1].timestamp));
			return_obj.insert(pair<string, vec3>(i->first, states[0].satellite_positions.at(i->first) * lagrange_start + states[1].satellite_positions.at(i->first) * lagrange_mid
				+ states[2].satellite_positions.at(i->first) * lagrange_end));
		}
		return return_obj;
	}

	void vr_scene::state_queue::remove_oldest_state() {
		states.pop_front();
	}

	/*void vr_scene::calculate_positions_and_orbits() {
		std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		all_pos_sat_start.clear();
		all_pos_sat_interp.clear();
		all_pos_sat_mid.clear();
		all_pos_sat_end.clear();
		all_pos_orbit.clear();
		all_colors_sat.clear();
		all_colors_orbit.clear();
		names_plus_pos.clear();
		for (auto entry : li_sat) {
			label_visibilities[entry.second] = false;
		}
		li_sat.clear();
		string datasets_label = "";
		string orbits_label = "";
		int cptdat = 0;
		int cptor = 0;
		if (forback) {
			start_time = visual_now;
			mid_time = visual_now - 450;
			end_time = visual_now - 900;
		}
		else {
			start_time = visual_now;
			mid_time = visual_now + 450;
			end_time = visual_now + 900;
		}
		//draw orbits and satellites
		for (auto datasets_entry : actives) { //all datasets
			if (datasets_entry.second) { //if dataset selected
				if (cptdat % 3 == 0) {
					datasets_label += datasets_entry.first + ", \n";
					cptdat++;
				}
				else {
					datasets_label += datasets_entry.first + ", ";
					cptdat++;
				}
				auto sats = satellites.at(datasets_entry.first); //find satellites list
				for (auto sat_entry : sats) { //for all satellites in the list
					std::vector<vec3> pos = std::vector<vec3>();
					double rev_time = (1.0 / sat_entry.first.Orbit().MeanMotion()) * (2 * M_PI) * 60;
					string full_name = datasets_entry.first + "*&_" + sat_entry.first.Orbit().SatId();
					// all satellites
					cVector v_s, v_m, v_e;
					std::vector<vec3> col_pos = std::vector<vec3>();
					try {
						v_s = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(start_time))).Position(); //find its position
						v_m = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(mid_time))).Position(); //find its position
						v_e = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(end_time))).Position(); //find its position one hour later
						vec3 vec = vec3(v_s.m_x / (2.0F * 6378), 1.0F + v_s.m_z / (2.F * 6378), v_s.m_y / (2.F * 6378)); //scale it to the VR world
						vec3 vec_m = vec3(v_m.m_x / (2.0F * 6378), 1.0F + v_m.m_z / (2.F * 6378), v_m.m_y / (2.F * 6378)); //scale it to the VR world
						vec3 vec_e = vec3(v_e.m_x / (2.0F * 6378), 1.0F + v_e.m_z / (2.F * 6378), v_e.m_y / (2.F * 6378)); //scale it to the VR world

						col_pos.push_back(vec3(sat_styles.at(datasets_entry.first).surface_color.R(), sat_styles.at(datasets_entry.first).surface_color.G(),
							sat_styles.at(datasets_entry.first).surface_color.B()));
						names_plus_pos.insert(pair<string, vec3>(full_name, vec));
						all_pos_sat_start.insert(pair<string,vec3>(full_name, vec));
						all_pos_sat_mid.insert(pair<string, vec3>(full_name, vec_m));
						all_pos_sat_end.insert(pair<string, vec3>(full_name, vec_e));
						all_pos_sat_interp.insert(pair<string, vec3>(full_name, vec));
						li_sat.insert(pair<string, uint32_t>(full_name, add_label("Dataset  : " + datasets_entry.first + "\nSat name : " + sat_entry.first.Name(),
							rgba(0.8F, 0.6F, 0.8F, 1.0F)))); //create label corresponding to said satellite
						fix_label_size(li_sat.at(full_name));
						vec3 view_dir = -reinterpret_cast<const vec3&>(vr_view_ptr->get_current_vr_state()->hmd.pose[6]);
						place_label(li_sat.at(full_name), names_plus_pos.at(full_name), quat(0, view_dir[0], view_dir[1], view_dir[2]), CS_LAB, LA_LEFT);
						hide_label(li_sat.at(full_name));//hide it
					}
					catch (cDecayException e) {
						std::vector<vec3> vec = std::vector<vec3>();
						vec.push_back(vec3());
						std::cerr << e.GetSatelliteName() + " is in the ground right now..." << std::endl;
					}
					catch (cPropagationException e) {
						std::vector<vec3> vec = std::vector<vec3>();
						vec.push_back(vec3());
					}
					all_colors_sat.insert(all_colors_sat.end(), col_pos.begin(), col_pos.end());
					if (sat_entry.second) { //if satellite selected for orbit drawn
						if (cptor % 3 == 0) {
							orbits_label += sat_entry.first.Name() + ", \n";
							cptor++;
						}
						else {
							orbits_label += sat_entry.first.Name() + ", ";
							cptor++;
						}
						/**
						* Calculation of one revolution orbit
						*
						tm timer = *gmtime(&visual_now);
						timer.tm_isdst = -1;

						timer.tm_sec -= rev_time; //calculate time one orbit earlier
						time_t min_one_rev = mktime(&timer);
						pos = std::vector<vec3>();
						std::vector<vec3> col_pos = std::vector<vec3>();
						for (float t = 0; t <= (visual_now - min_one_rev); t++) {
							//std::cout << t << std::endl;
							cVector v;
							try {

								v = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(min_one_rev + t))).Position();
								if (t == 0 || t == (visual_now - min_one_rev - 1)) {
									pos.push_back(vec3(v.m_x / (2.F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378)));
									col_pos.push_back(vec3(orbit_styles.at(datasets_entry.first).surface_color.R(), orbit_styles.at(datasets_entry.first).surface_color.G(),
										orbit_styles.at(datasets_entry.first).surface_color.B()));
								}
								else {
									pos.push_back(vec3(v.m_x / (2.F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378)));
									pos.push_back(vec3(v.m_x / (2.F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378)));
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
						show_label(li_sat.at(full_name));
						//sat_orbit_pos.insert(pair<string, std::vector<vec3>>(sat_entry.first.Name(), pos));
						all_pos_orbit.insert(all_pos_orbit.end(), pos.begin(), pos.end());
						all_colors_orbit.insert(all_colors_orbit.end(), col_pos.begin(), col_pos.end());
					}
				}
			}
		}
		lm.update_label_text(listing_datasets_label, "Datasets: " + datasets_label);
		lm.update_label_size(listing_datasets_label, -1, -1); //update size as list grows
		lm.update_label_text(listing_orbits_label, "Orbits: " + orbits_label);
		lm.update_label_size(listing_orbits_label, -1, -1); //update size as list grows
		std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();


		cout << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << endl;

	}

	void vr_scene::calculate_positions_and_orbits_queue() {
		all_colors_sat.clear();
		all_pos_orbit.clear();
		all_colors_orbit.clear();
		map<string, vec3> positions_to_queue;
		for (auto entry : li_sat) {
			label_visibilities[entry.second] = false;
		}
		li_sat.clear();
		string datasets_label = "";
		string orbits_label = "";
		int cptdat = 0;
		int cptor = 0;
		
		for (auto datasets_entry : actives) {
			if (datasets_entry.second) {
				if (cptdat % 3 == 0) {
					datasets_label += datasets_entry.first + ", \n";
					cptdat++;
				}
				else {
					datasets_label += datasets_entry.first + ", ";
					cptdat++;
				}
			}
			if (time_queue.size() == 0)
				time_queue.push_back(visual_now);
			else
				time_queue.push_back(time_queue.back() + 450);
			auto sats = satellites.at(datasets_entry.first);
			for (auto sat_entry : sats) {
				string full_name = datasets_entry.first + "*&_" + sat_entry.first.Orbit().SatId();
				// all satellites
				cVector v;
				std::vector<vec3> col_pos = std::vector<vec3>();
				try {
					v = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(time_queue.back()))).Position();
					vec3 vec = vec3(v.m_x / (2.0F * 6378.F), v.m_z / (2.0F * 6378.F), v.m_y / (2.0F * 6378.F));
					col_pos.push_back(vec3(sat_styles.at(datasets_entry.first).surface_color.R(), sat_styles.at(datasets_entry.first).surface_color.G(),
						sat_styles.at(datasets_entry.first).surface_color.B()));
					positions_to_queue.insert(pair<string, vec3>(full_name, vec));
					li_sat.insert(pair<string, uint32_t>(full_name, add_label("Dataset  : " + datasets_entry.first + "\nSat name : " + sat_entry.first.Name(),
						rgba(0.8F, 0.6F, 0.8F, 1.0F)))); //create label corresponding to said satellite
					fix_label_size(li_sat.at(full_name));
					hide_label(li_sat.at(full_name));//hide it
				}
				catch (cDecayException e) {
					std::vector<vec3> vec = std::vector<vec3>();
					vec.push_back(vec3());
					std::cerr << e.GetSatelliteName() + " is in the ground right now..." << std::endl;
				}
				catch (cPropagationException e) {
					std::vector<vec3> vec = std::vector<vec3>();
					vec.push_back(vec3());
				}
				all_colors_sat.insert(all_colors_sat.end(), col_pos.begin(), col_pos.end());
				if (sat_entry.second) {
					std::vector<vec3> pos = std::vector<vec3>();
					double rev_time = (1.0 / sat_entry.first.Orbit().MeanMotion()) * (2 * M_PI) * 60;
					if (cptor % 3 == 0) {
						orbits_label += sat_entry.first.Name() + ", \n";
						cptor++;
					}
					else {
						orbits_label += sat_entry.first.Name() + ", ";
						cptor++;
					}
					/**
					* Calculation of one revolution orbit
					*
					tm timer = *gmtime(&visual_now);
					timer.tm_isdst = -1;

					timer.tm_sec -= rev_time; //calculate time one orbit earlier
					time_t min_one_rev = mktime(&timer);
					pos = std::vector<vec3>();
					std::vector<vec3> col_pos = std::vector<vec3>();
					for (float t = 0; t <= (visual_now - min_one_rev); t++) {
						//std::cout << t << std::endl;
						cVector v;
						try {

							v = sat_entry.first.PositionEci(sat_entry.first.Orbit().Epoch().SpanMin(cJulian(min_one_rev + t))).Position();
							if (t == 0 || t == (visual_now - min_one_rev - 1)) {
								pos.push_back(vec3(v.m_x / (2.F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378)));
								col_pos.push_back(vec3(orbit_styles.at(datasets_entry.first).surface_color.R(), orbit_styles.at(datasets_entry.first).surface_color.G(),
									orbit_styles.at(datasets_entry.first).surface_color.B()));
							}
							else {
								pos.push_back(vec3(v.m_x / (2.F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378)));
								pos.push_back(vec3(v.m_x / (2.F * 6378), 1.0F + v.m_z / (2.F * 6378), v.m_y / (2.F * 6378)));
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
					show_label(li_sat.at(full_name));
					//sat_orbit_pos.insert(pair<string, std::vector<vec3>>(sat_entry.first.Name(), pos));
					all_pos_orbit.insert(all_pos_orbit.end(), pos.begin(), pos.end());
					all_colors_orbit.insert(all_colors_orbit.end(), col_pos.begin(), col_pos.end());
				}

			}

		}
		lm.update_label_text(listing_datasets_label, "Datasets: " + datasets_label);
		lm.update_label_size(listing_datasets_label, -1, -1); //update size as list grows
		lm.update_label_text(listing_orbits_label, "Orbits: " + orbits_label);
		lm.update_label_size(listing_orbits_label, -1, -1); //update size as list grows

		pos_queue.push_back(positions_to_queue);

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
		if (all_pos_sat_start.size() != all_pos_sat_end.size() || visual_now == end_time) {
			calculate_positions_and_orbits();
		}
		if (!forback) {
			visual_now += 10;
		}
		else {
			visual_now -= 10;
		}
		all_pos_sat_interp.clear();
		for (auto i = all_pos_sat_start.begin(); i != all_pos_sat_start.end(); i++) {
			auto lagrange_start = ((double)(visual_now - mid_time) / (start_time - mid_time)) * ((double)(visual_now - end_time) / (start_time - end_time));
			auto lagrange_mid = ((double)(visual_now - start_time) / (mid_time - start_time)) * ((double)(visual_now - end_time) / (mid_time - end_time));
			auto lagrange_end = ((double)(visual_now - start_time) / (end_time - start_time)) * ((double)(visual_now - mid_time) / (end_time - mid_time));
			all_pos_sat_interp.insert(pair<string, vec3>(i->first, all_pos_sat_start.at(i->first) * lagrange_start + all_pos_sat_mid.at(i->first) * lagrange_mid
				+ all_pos_sat_end.at(i->first) * lagrange_end));
			names_plus_pos.at(i->first) = all_pos_sat_interp.at(i->first);
		}
	}
	*/

	void vr_scene::change_time_queue(double, double dt) {
		if (!forback) {
			visual_now += 10;
		}
		else {
			visual_now -= 10;
		}
		if ((!forback && visual_now == sat_queue.states[1].timestamp - 200)
			|| (forback && visual_now == sat_queue.states[1].timestamp + 200)) {
			if (anim_thread.joinable()) {
				anim_thread.join();
			}
			if (!forback)
				anim_thread = launch_new_thread(*this, visual_now + 450);
			else
				anim_thread = launch_new_thread(*this, visual_now - 450);
			//thread calc_thread(&vr_scene::calculate_positions_and_orbits_queue, NULL);
		}
		/*all_pos_sat_interp.clear();
		for (auto i = pos_queue[0].begin(); i != pos_queue[0].end(); i++) {
			auto lagrange_start = ((double)(visual_now - time_queue[1]) / (time_queue[0] - time_queue[1])) *
				((double)(visual_now - time_queue[2]) / (time_queue[0] - time_queue[2]));
			auto lagrange_mid = ((double)(visual_now - time_queue[0]) / (time_queue[1] - time_queue[0])) *
				((double)(visual_now - time_queue[2]) / (time_queue[1] - time_queue[2]));
			auto lagrange_end = ((double)(visual_now - time_queue[0]) / (time_queue[2] - time_queue[0])) *
				((double)(visual_now - time_queue[1]) / (time_queue[2] - time_queue[1]));
			all_pos_sat_interp.insert(pair<string, vec3>(i->first, pos_queue[0].at(i->first) * lagrange_start + pos_queue[1].at(i->first) * lagrange_mid
				+ pos_queue[2].at(i->first) * lagrange_end));
		}*/
	}

	void vr_scene::start_anim(cgv::gui::button&) {
		trig.schedule_recuring(1.0F/60.F);
	}

	void vr_scene::stop_anim(cgv::gui::button&) {
		trig.stop();
	}

	void vr_scene::forback_anim(cgv::gui::button&) {
		forback = !forback;
		sat_queue.clear();
		sat_queue.calculate_positions_at(*this, visual_now);
	}

	void vr_scene::activate_dataset_or_orbit(cgv::gui::control<bool>& in) {
		in.set_new_value(!in.get_value());
		sat_queue.clear();
		sat_queue.calculate_positions_at(*this, visual_now);
	}

	void vr_scene::create_gui()
	{

		align("\a");
		std::stringstream ss;
		ss << v_min_2;
		std::string ts = ss.str();
		std::stringstream ss_plus;
		ss_plus << v_plus_2;
		std::string ts_plus = ss_plus.str();
		add_member_control(this, "Time selection", visual_now, "value_slider", "min=" + ts + ";step=1;max=" + ts_plus + ";log=true;ticks=true");
		align("\b");

		align("\a");
		add_member_control(this, "time animation jump", incr, "value_slider", "min=-3600;step=1;max=3600;log=false;ticks=true");
		align("\b");
		align("\a");
		cgv::gui::button_ptr starter = add_button("Start animation");
		cgv::signal::connect(starter->click, this, &vr_scene::start_anim);
		cgv::gui::button_ptr stopper = add_button("Stop animation");
		cgv::signal::connect(stopper->click, this, &vr_scene::stop_anim);
		cgv::gui::button_ptr changer = add_button("Change time direction of animation");
		cgv::signal::connect(changer->click, this, &vr_scene::forback_anim);
		align("\b");

		if (begin_tree_node("Datasets", is_active)) {
			align("\a");
			for (auto cpt = actives.begin(); cpt != actives.end(); cpt++) {
				//add_member_control(this, cpt->first, cpt->second);
				auto ctrl_ptr = this->add_control(cpt->first, cpt->second, "toggle");
				cgv::signal::connect(ctrl_ptr->value_change, this, &vr_scene::activate_dataset_or_orbit);
			}
			align("\b");
			end_tree_node(is_active);
		}
		bool sat_selector;
		if (begin_tree_node("Satellites", sat_selector)) {
			align("\a");
			for (auto cpt = satellites.begin(); cpt != satellites.end(); cpt++) {
				if (actives.at(cpt->first)) {
					for (auto sat_entry = cpt->second.begin(); sat_entry != cpt->second.end(); sat_entry++) {
						auto ctrl_ptr = this->add_control(sat_entry->first.Name(), sat_entry->second, "toggle");
						cgv::signal::connect(ctrl_ptr->value_change, this, &vr_scene::activate_dataset_or_orbit);
					}
				}
			}
			align("\b");
			end_tree_node(sat_selector);
		}
	}

}
#include <cgv/base/register.h>

cgv::base::object_registration<vr::vr_scene> vr_scene_reg("vr_scene");
