#include "vr_test.h"

#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/dialog.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cg_vr/vr_events.h>
#include <cgv/defines/quote.h>

#include <random>
#include <iostream>
#include <fstream>
#include <regex>
#include <iterator>

#include "intersection.h"

/*static std::string get_input_directory()
{
	return QUOTE_SYMBOL_VALUE(INPUT_DIR);
}​/**/

void vr_test::init_cameras(vr::vr_kit* kit_ptr)
{
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	nr_cameras = camera_ptr->get_nr_cameras();
	frame_split = camera_ptr->get_frame_split();
	for (int i = 0; i < nr_cameras; ++i) {
		std::cout << "camera " << i << "(" << nr_cameras << "):" << std::endl;
		camera_ptr->put_camera_intrinsics(i, false, &focal_lengths[i](0), &camera_centers[i](0));
		camera_ptr->put_camera_intrinsics(i, true, &focal_lengths[2 + i](0), &camera_centers[2 + i](0));
		std::cout << "  fx=" << focal_lengths[i][0] << ", fy=" << focal_lengths[i][1] << ", center=[" << camera_centers[i] << "]" << std::endl;
		std::cout << "  fx=" << focal_lengths[2+i][0] << ", fy=" << focal_lengths[2+i][1] << ", center=[" << camera_centers[2+i] << "]" << std::endl;
		float camera_to_head[12];
		camera_ptr->put_camera_to_head_matrix(i, camera_to_head);
		kit_ptr->put_eye_to_head_matrix(i, camera_to_head);
		camera_to_head_matrix[i] = vr::get_mat4_from_pose(camera_to_head);
		std::cout << "  C2H=" << camera_to_head_matrix[i] << std::endl;
		camera_ptr->put_projection_matrix(i, false, 0.001f, 10.0f, &camera_projection_matrix[i](0, 0));
		camera_ptr->put_projection_matrix(i, true, 0.001f, 10.0f, &camera_projection_matrix[2+i](0, 0));
		std::cout << "  dP=" << camera_projection_matrix[i] << std::endl;
		std::cout << "  uP=" << camera_projection_matrix[2+i] << std::endl;
	}
	post_recreate_gui();
}

void vr_test::start_camera()
{
	if (!vr_view_ptr)
		return;
	vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
	if (!kit_ptr)
		return;
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	if (!camera_ptr->start())
		cgv::gui::message(camera_ptr->get_last_error());
}

void vr_test::stop_camera()
{
	if (!vr_view_ptr)
		return;
	vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
	if (!kit_ptr)
		return;
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	if (!camera_ptr->stop())
		cgv::gui::message(camera_ptr->get_last_error());
}

/// compute intersection points of controller ray with movable boxes
/*void vr_test::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
{
	for (size_t i = 0; i < movable_boxes.size(); ++i) {
		vec3 origin_box_i = origin - movable_box_translations[i];
		movable_box_rotations[i].inverse_rotate(origin_box_i);
		vec3 direction_box_i = direction;
		movable_box_rotations[i].inverse_rotate(direction_box_i);
		float t_result;
		vec3  p_result;
		vec3  n_result;
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin_box_i, direction_box_i,
			movable_boxes[i],
			t_result, p_result, n_result, 0.000001f)) {

			// transform result back to world coordinates
			movable_box_rotations[i].rotate(p_result);
			p_result += movable_box_translations[i];
			movable_box_rotations[i].rotate(n_result);

			// store intersection information
			intersection_points.push_back(p_result);
			intersection_colors.push_back(color);
			intersection_box_indices.push_back((int)i);
			intersection_controller_indices.push_back(ci);
		}
	}
}
*/
/// keep track of status changes
void vr_test::on_status_change(void* kit_handle, int ci, vr::VRStatus old_status, vr::VRStatus new_status)
{
	// ignore all but left controller changes
	if (ci != 0)
		return;
	vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
	// check for attaching of controller
	if (old_status == vr::VRS_DETACHED) {
		left_inp_cfg.resize(kit_ptr->get_device_info().controller[0].nr_inputs);
		for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii)
			left_inp_cfg[ii] = kit_ptr->get_controller_input_config(0, ii);
		post_recreate_gui();
	}
	// check for attaching of controller
	if (new_status == vr::VRS_DETACHED) {
		left_inp_cfg.clear();
		post_recreate_gui();
	}
}

/// register on device change events
void vr_test::on_device_change(void* kit_handle, bool attach)
{
	if (attach) {
		if (last_kit_handle == 0) {
			vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
			init_cameras(kit_ptr);
			if (kit_ptr) {
				last_kit_handle = kit_handle;
				// copy left controller input configurations from new device in order to make it adjustable
				left_inp_cfg.resize(kit_ptr->get_device_info().controller[0].nr_inputs);
				for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii)
					left_inp_cfg[ii] = kit_ptr->get_controller_input_config(0, ii);
				post_recreate_gui();
			}
		}
	}
	else {
		if (kit_handle == last_kit_handle) {
			last_kit_handle = 0;
			post_recreate_gui();
		}
	}
}

/// construct boxes that represent a table of dimensions tw,td,th and leg width tW

/// construct a scene with a table
void vr_test::build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
{
	/*construct_room(w, d, h, W, false, false);
	construct_table(tw, td, th, tW);
	construct_environment(0.3f, 3 * w, 3 * d, w, d, h);
	//construct_environment(0.4f, 0.5f, 1u, w, d, h);
	construct_movable_boxes(tw, td, th, tW, 50);*/
}

vr_test::vr_test() 
{
	frame_split = 0;
	extent_texcrd = vec2(0.5f, 0.5f);
	center_left  = vec2(0.5f,0.25f);
	center_right = vec2(0.5f,0.25f);
	seethrough_gamma = 0.33f;
	frame_width = frame_height = 0;
	background_distance = 2;
	background_extent = 2;
	undistorted = true;
	shared_texture = true;
	max_rectangle = false;
	nr_cameras = 0;
	camera_tex_id = -1;
	camera_aspect = 1;
	use_matrix = true;
	show_seethrough = false;
	set_name("vr_test");
	build_scene(5, 7, 3, 0.2f, 0.8f, 0.8f, 0.72f, 0.03f);
	vr_view_ptr = 0;
	ray_length = 2;
	last_kit_handle = 0;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_test::on_device_change);
	connect(cgv::gui::ref_vr_server().on_status_change, this, &vr_test::on_status_change);

	mesh_scale = 0.0005f;
	mesh_location = dvec3(0, 0.85f, 0);
	mesh_orientation = dquat(1, 0, 0, 0);

	srs.radius = 0.005f;

	label_outofdate = true;
	label_text = "Info Board";
	label_font_idx = 0;
	label_upright = true;
	label_face_type = cgv::media::font::FFA_BOLD;
	label_resolution = 256;
	label_size = 20.0f;
	label_color = rgb(1, 1, 1);

	cgv::media::font::enumerate_font_names(font_names);
	font_enum_decl = "enums='";
	for (unsigned i = 0; i < font_names.size(); ++i) {
		if (i>0)
			font_enum_decl += ";";
		std::string fn(font_names[i]);
		if (cgv::utils::to_lower(fn) == "calibri") {
			label_font_face = cgv::media::font::find_font(fn)->get_font_face(label_face_type);
			label_font_idx = i;
		}
		font_enum_decl += std::string(fn);
	}
	font_enum_decl += "'";
	state[0] = state[1] = state[2] = state[3] = IS_NONE;
}
	
void vr_test::stream_help(std::ostream& os) {
	os << "vr_test: no shortcuts defined" << std::endl;
}
	
void vr_test::on_set(void* member_ptr)
{
	if (member_ptr == &label_face_type || member_ptr == &label_font_idx) {
		label_font_face = cgv::media::font::find_font(font_names[label_font_idx])->get_font_face(label_face_type);
		label_outofdate = true;
	}
	if ((member_ptr >= &label_color && member_ptr < &label_color + 1) ||
		member_ptr == &label_size || member_ptr == &label_text) {
		label_outofdate = true;
	}

	vr::vr_kit* kit_ptr = vr::get_vr_kit(last_kit_handle);
	if (kit_ptr) {
		for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii)
			if (member_ptr >= &left_inp_cfg[ii] && member_ptr < &left_inp_cfg[ii] + 1)
				kit_ptr->set_controller_input_config(0, ii, left_inp_cfg[ii]);
	}
	update_member(member_ptr);
	post_redraw();
}
	
bool vr_test::handle(cgv::gui::event& e)
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
				std::cout << "grip button " << (vrke.get_controller_index() == 0 ? "left":"right") << " controller pressed" << std::endl;
				return true;
			case vr::VR_DPAD_RIGHT:
				std::cout << "touch pad of " << (vrke.get_controller_index() == 0 ? "left" : "right") << " controller pressed at right direction" << std::endl;
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
		case cgv::gui::SA_UNPRESS:
			std::cout << "stick " << vrse.get_stick_index()
				<< " of controller " << vrse.get_controller_index()
				<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
				<< " at " << vrse.get_x() << ", " << vrse.get_y() << std::endl;
			return true;
		case cgv::gui::SA_MOVE:
		case cgv::gui::SA_DRAG:
			return true;
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
			/*if (state[ci] == IS_GRAB) {
				// in grab mode apply relative transformation to grabbed boxes

				// get previous and current controller position
				vec3 last_pos = vrpe.get_last_position();
				vec3 pos = vrpe.get_position();
				// get rotation from previous to current orientation
				// this is the current orientation matrix times the
				// inverse (or transpose) of last orientation matrix:
				// vrpe.get_orientation()*transpose(vrpe.get_last_orientation())
				mat3 rotation = vrpe.get_rotation_matrix();
				// iterate intersection points of current controller
				for (size_t i = 0; i < intersection_points.size(); ++i) {
					if (intersection_controller_indices[i] != ci)
						continue;
					// extract box index
					unsigned bi = intersection_box_indices[i];
					// update translation with position change and rotation
					movable_box_translations[bi] = 
						rotation * (movable_box_translations[bi] - last_pos) + pos;
					// update orientation with rotation, note that quaternions
					// need to be multiplied in oposite order. In case of matrices
					// one would write box_orientation_matrix *= rotation
					movable_box_rotations[bi] = quat(rotation) * movable_box_rotations[bi];
					// update intersection points
					intersection_points[i] = rotation * (intersection_points[i] - last_pos) + pos;
				}
			}
			else {// not grab
				// clear intersections of current controller 
				size_t i = 0;
				while (i < intersection_points.size()) {
					if (intersection_controller_indices[i] == ci) {
						intersection_points.erase(intersection_points.begin() + i);
						intersection_colors.erase(intersection_colors.begin() + i);
						intersection_box_indices.erase(intersection_box_indices.begin() + i);
						intersection_controller_indices.erase(intersection_controller_indices.begin() + i);
					}
					else
						++i;
				}

				// compute intersections
				vec3 origin, direction;
				vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
				//compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				label_outofdate = true;


				// update state based on whether we have found at least 
				// one intersection with controller ray
				if (intersection_points.size() == i)
					state[ci] = IS_NONE;
				else
					if (state[ci] == IS_NONE)
						state[ci] = IS_OVER;
			}*/
			post_redraw();
		}
		return true;
	}
	return false;
}

bool vr_test::init(cgv::render::context& ctx)
{
	if (!cgv::utils::has_option("NO_OPENVR"))
		ctx.set_gamma(1.0f);

	if (!seethrough.build_program(ctx, "seethrough.glpr"))
		cgv::gui::message("could not build seethrough program");
	
	cgv::media::mesh::simple_mesh<> M;
//#ifdef 1
	if (M.read("D:/data/surface/meshes/obj/Max-Planck_lowres.obj")) {
//#else
//	if (M.read("D:/data/surface/meshes/obj/Max-Planck_highres.obj")) {
//#endif
		MI.construct(ctx, M);
		MI.bind(ctx, ctx.ref_surface_shader_program(true), true);
	}

	cgv::gui::connect_vr_server(true);

	auto view_ptr = find_view_as_node();
	if (view_ptr) {
		view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_DEVICE +
					cgv::gui::VRE_STATUS +
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_ONE_AXIS_GENERATES_KEY +
					cgv::gui::VRE_ONE_AXIS +
					cgv::gui::VRE_TWO_AXES +
					cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
					cgv::gui::VRE_POSE
				));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);
		}
	}

	/*ground_from_space.create(ctx);
	ground_from_space.attach_dir(ctx, "../../../src/shaders/groundfromspace/", true);
	ground_from_space.link(ctx, true);

	ground_from_space.enable(ctx);
	ground_from_space.set_uniform(ctx, "v3LightPos", vec3(0.0f, 0.0f, 1.0f));
	ground_from_space.set_uniform(ctx, "v3InvWavelength", vec3( 1.0f / powf(0.650f, 4.0f), 1.0f / powf(0.570f, 4.0f), 1.0f / powf(0.475f, 4.0f) ));
	ground_from_space.set_uniform(ctx, "fInnerRadius", InnerRadius);
	ground_from_space.set_uniform(ctx, "fInnerRadius2", InnerRadius * InnerRadius);
	ground_from_space.set_uniform(ctx, "fOuterRadius", OuterRadius);
	ground_from_space.set_uniform(ctx, "fOuterRadius2", OuterRadius * OuterRadius);
	ground_from_space.set_uniform(ctx, "fKrESun", Kr * ESun);
	ground_from_space.set_uniform(ctx, "fKmESun", Km * ESun);
	ground_from_space.set_uniform(ctx, "fKr4PI", Kr * 4.0f * (float)M_PI);
	ground_from_space.set_uniform(ctx, "fKm4PI", Km * 4.0f * (float)M_PI);
	ground_from_space.set_uniform(ctx, "fScale", Scale);
	ground_from_space.set_uniform(ctx, "fScaleDepth", ScaleDepth);
	ground_from_space.set_uniform(ctx, "fScaleOverScaleDepth", ScaleOverScaleDepth);
	ground_from_space.set_uniform(ctx, "g", g);
	ground_from_space.set_uniform(ctx, "g2", g * g);
	ground_from_space.set_uniform(ctx, "Samples", 4);
	ground_from_space.set_uniform(ctx, "s2Tex1", 0);
	ground_from_space.set_uniform(ctx, "s2Tex2", 1);
	ground_from_space.disable(ctx);*/

	if (earth_sphere.read(get_input_directory() + "/models/sphere_36_cuts.obj")) {
		earth_info.construct(ctx, earth_sphere);
		earth_info.bind(ctx, ctx.ref_surface_shader_program(true), true);
		auto& mats = earth_info.get_materials();
		if (mats.size() > 0) {
			//setup texture paths
			int di = mats[0]->add_image_file(get_input_directory()+"/images/earthmap.jpg");
			
			//ensure the textures are loaded
			mats[0]->ensure_textures(ctx);

			//set texture indices
			mats[0]->set_diffuse_index(di);
		}
	}

	//to finish later
	std::string line, line1, line2, line3;
	std::ifstream file_reader(get_input_directory()+"/sat_data/station.txt");
	int cpt = 0;
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
			}
			cpt++;
		}
		file_reader.close();
	}
	cTle tle_one = cTle(line1, line2, line3);
	cSatellite sat = cSatellite(tle_one, &tle_one.Name());
	time_t now = time(0);
	tm timer = *gmtime(&now);
	timer.tm_isdst = -1;
	timer.tm_sec -= 24.0 * 3600.0 / tle_one.GetField(tle_one.FLD_MMOTION);
	time_t min_one_rev = mktime(&timer);
	for (float t = 0; t <= (now - min_one_rev); t++) {
		//std::cout << t << std::endl;
		auto v = sat.PositionEci(sat.Orbit().Epoch().SpanMin(cJulian(min_one_rev + t))).Position() ;
		pos.push_back(vec3(v.m_x / (6378), v.m_y / (6378), v.m_z / (6378)));
	}
	orbit_one_style.surface_color = rgba(1.0f, 0.7f, 0.3f, 0.5f);
	orbit_one_style.radius = 0.01f;


	cgv::render::ref_box_renderer(ctx, 1);
	cgv::render::ref_sphere_renderer(ctx, 1);
	cgv::render::ref_rounded_cone_renderer(ctx, 1);
	return true;
}

void vr_test::clear(cgv::render::context& ctx)
{
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);
}

void vr_test::init_frame(cgv::render::context& ctx)
{
	if (label_fbo.get_width() != label_resolution) {
		label_tex.destruct(ctx);
		label_fbo.destruct(ctx);
	}
	if (!label_fbo.is_created()) {
		label_tex.create(ctx, cgv::render::TT_2D, label_resolution, label_resolution);
		label_fbo.create(ctx, label_resolution, label_resolution);
		label_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		label_tex.set_mag_filter(cgv::render::TF_LINEAR);
		label_fbo.attach(ctx, label_tex);
		label_outofdate = true;
	}
	if (label_outofdate && label_fbo.is_complete(ctx)) {
		glPushAttrib(GL_COLOR_BUFFER_BIT);
		label_fbo.enable(ctx);
		label_fbo.push_viewport(ctx);
		ctx.push_pixel_coords();
			glClearColor(0.5f,0.5f,0.5f,1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			glColor4f(label_color[0], label_color[1], label_color[2], 1);
			ctx.set_cursor(20, (int)ceil(label_size) + 20);
			ctx.enable_font_face(label_font_face, label_size);
			ctx.output_stream() << label_text << "\n";
			ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face

			ctx.enable_font_face(label_font_face, 0.7f*label_size);
			/*for (size_t i = 0; i < intersection_points.size(); ++i) {
				ctx.output_stream()
					<< "box " << intersection_box_indices[i]
					<< " at (" << intersection_points[i]
					<< ") with controller " << intersection_controller_indices[i] << "\n";
			}*/
			ctx.output_stream().flush();

		ctx.pop_pixel_coords();
		label_fbo.pop_viewport(ctx);
		label_fbo.disable(ctx);
		glPopAttrib();
		label_outofdate = false;

		label_tex.generate_mipmaps(ctx);
	}
	if (vr_view_ptr && vr_view_ptr->get_rendered_vr_kit() != 0 && vr_view_ptr->get_rendered_eye() == 0 && vr_view_ptr->get_rendered_vr_kit() == vr_view_ptr->get_current_vr_kit()) {
		vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
		if (kit_ptr) {
			vr::vr_camera* camera_ptr = kit_ptr->get_camera();
			if (camera_ptr && camera_ptr->get_state() == vr::CS_STARTED) {
				uint32_t width = frame_width, height = frame_height, split = frame_split;
				if (shared_texture) {
					box2 tex_range;
					if (camera_ptr->get_gl_texture_id(camera_tex_id, width, height, undistorted, &tex_range.ref_min_pnt()(0))) {
						camera_aspect = (float)width / height;
						split = camera_ptr->get_frame_split();
						switch (split) {
						case vr::CFS_VERTICAL:
							camera_aspect *= 2;
							break;
						case vr::CFS_HORIZONTAL:
							camera_aspect *= 0.5f;
							break;
						}
					}
					else
						camera_tex_id = -1;
				}
				else {
					std::vector<uint8_t> frame_data;
					if (camera_ptr->get_frame(frame_data, width, height, undistorted, max_rectangle)) {
						camera_aspect = (float)width / height;
						split = camera_ptr->get_frame_split();
						switch (split) {
						case vr::CFS_VERTICAL:
							camera_aspect *= 2;
							break;
						case vr::CFS_HORIZONTAL:
							camera_aspect *= 0.5f;
							break;
						}
						cgv::data::data_format df(width, height, cgv::type::info::TI_UINT8, cgv::data::CF_RGBA);
						cgv::data::data_view dv(&df, frame_data.data());
						if (camera_tex.is_created()) {
							if (camera_tex.get_width() != width || camera_tex.get_height() != height)
								camera_tex.destruct(ctx);
							else
								camera_tex.replace(ctx, 0, 0, dv);
						}
						if (!camera_tex.is_created())
							camera_tex.create(ctx, dv);
					}
					else if (camera_ptr->has_error())
						cgv::gui::message(camera_ptr->get_last_error());
				}
				if (frame_width != width || frame_height != height) {
					frame_width = width;
					frame_height = height;

					center_left(0) = camera_centers[2](0) / frame_width;
					center_left(1) = camera_centers[2](1) / frame_height;
					center_right(0) = camera_centers[3](0) / frame_width;
					center_right(1) = camera_centers[3](1) / frame_height;

					update_member(&frame_width);
					update_member(&frame_height);
					update_member(&center_left(0));
					update_member(&center_left(1));
					update_member(&center_right(0));
					update_member(&center_right(1));
				}
				if (split != frame_split) {
					frame_split = split;
					update_member(&frame_split);
				}
			}
		}
	}
}

void vr_test::draw(cgv::render::context& ctx)
{
	if (MI.is_constructed()) {
		dmat4 R;
		mesh_orientation.put_homogeneous_matrix(R);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(
			cgv::math::translate4<double>(mesh_location)*
			cgv::math::scale4<double>(mesh_scale, mesh_scale, mesh_scale) *
			R);
		MI.draw_all(ctx);
		ctx.pop_modelview_matrix();
	}
	if (vr_view_ptr) {
		if ((!shared_texture && camera_tex.is_created()) || (shared_texture && camera_tex_id != -1)) {
			if (vr_view_ptr->get_rendered_vr_kit() != 0 && vr_view_ptr->get_rendered_vr_kit() == vr_view_ptr->get_current_vr_kit()) {
				int eye = vr_view_ptr->get_rendered_eye();

				// compute billboard
				dvec3 vd = vr_view_ptr->get_view_dir_of_kit();
				dvec3 y = vr_view_ptr->get_view_up_dir_of_kit();
				dvec3 x = normalize(cross(vd, y));
				y = normalize(cross(x, vd));
				x *= camera_aspect * background_extent * background_distance;
				y *= background_extent * background_distance;
				vd *= background_distance;
				dvec3 eye_pos = vr_view_ptr->get_eye_of_kit(eye);
				std::vector<vec3> P;
				std::vector<vec2> T;
				P.push_back(eye_pos + vd - x - y);
				P.push_back(eye_pos + vd + x - y);
				P.push_back(eye_pos + vd - x + y);
				P.push_back(eye_pos + vd + x + y);
				double v_offset = 0.5 * (1 - eye);
				T.push_back(dvec2(0.0, 0.5 + v_offset));
				T.push_back(dvec2(1.0, 0.5 + v_offset));
				T.push_back(dvec2(0.0, v_offset));
				T.push_back(dvec2(1.0, v_offset));

				cgv::render::shader_program& prog = seethrough;
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, prog.get_position_index(), P);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, prog.get_texcoord_index(), T);
				cgv::render::attribute_array_binding::enable_global_array(ctx, prog.get_position_index());
				cgv::render::attribute_array_binding::enable_global_array(ctx, prog.get_texcoord_index());

				GLint active_texture, texture_binding;
				if (shared_texture) {
					glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
					glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_binding);
					glActiveTexture(GL_TEXTURE0);
					glBindTexture(GL_TEXTURE_2D, camera_tex_id);
				}
				else
					camera_tex.enable(ctx, 0);
				prog.set_uniform(ctx, "texture", 0);
				prog.set_uniform(ctx, "seethrough_gamma", seethrough_gamma);
				prog.set_uniform(ctx, "use_matrix", use_matrix);

				// use of convenience function
				vr::configure_seethrough_shader_program(ctx, prog, frame_width, frame_height,
					vr_view_ptr->get_current_vr_kit(), *vr_view_ptr->get_current_vr_state(),
					0.01f, 2 * background_distance, eye, undistorted);

				/* equivalent detailed code relies on more knowledge on program parameters
				mat4 TM = vr::get_texture_transform(vr_view_ptr->get_current_vr_kit(), *vr_view_ptr->get_current_vr_state(), 0.01f, 2 * background_distance, eye, undistorted);
				prog.set_uniform(ctx, "texture_matrix", TM);

				prog.set_uniform(ctx, "extent_texcrd", extent_texcrd);
				prog.set_uniform(ctx, "frame_split", frame_split);
				prog.set_uniform(ctx, "center_left", center_left);
				prog.set_uniform(ctx, "center_right", center_right);
				prog.set_uniform(ctx, "eye", eye);
				*/
				prog.enable(ctx);
				ctx.set_color(rgba(1, 1, 1, 1));

				glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);


				prog.disable(ctx);

				if (shared_texture) {
					glActiveTexture(active_texture);
					glBindTexture(GL_TEXTURE_2D, texture_binding);
				}
				else
					camera_tex.disable(ctx);

				cgv::render::attribute_array_binding::disable_global_array(ctx, prog.get_position_index());
				cgv::render::attribute_array_binding::disable_global_array(ctx, prog.get_texcoord_index());
			}
		}
		if (vr_view_ptr) {
			std::vector<vec3> P;
			std::vector<float> R;
			std::vector<rgb> C;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 4; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P.push_back(ray_origin);
					R.push_back(0.002f);
					P.push_back(ray_origin + ray_length * ray_direction);
					R.push_back(0.003f);
					rgb c(float(1 - ci), 0.5f * (int)state[ci], float(ci));
					C.push_back(c);
					C.push_back(c);
				}
			}
			if (P.size() > 0) {
				auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
				cr.set_render_style(cone_style);
				//cr.set_eye_position(vr_view_ptr->get_eye_of_kit());
				cr.set_position_array(ctx, P);
				cr.set_color_array(ctx, C);
				cr.set_radius_array(ctx, R);
				if (!cr.render(ctx, 0, P.size())) {
					cgv::render::shader_program& prog = ctx.ref_default_shader_program();
					int pi = prog.get_position_index();
					int ci = prog.get_color_index();
					cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
					cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
					cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
					cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
					glLineWidth(3);
					prog.enable(ctx);
					glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
					prog.disable(ctx);
					cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
					cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
					glLineWidth(1);
				}
			}
		}
	}

	if (!earth_info.is_constructed())
		return;
	//push back a model view matrix to transform the rendering of this mesh
	ctx.push_modelview_matrix();


	// translate and scale
	double R = 1.0;
	ctx.mul_modelview_matrix(
		cgv::math::translate4<double>(vec3(0, 0, 0))*
		cgv::math::scale4<double>(dvec3(vec3(1, 1, 1)* R))*
		cgv::math::rotate4<double>(-23.4, vec3(1,0,0))
	);

	// actually draw the mesh
	earth_info.draw_all(ctx);

	// restore the previous transform
	ctx.pop_modelview_matrix();

	orbit_one = cgv::render::ref_rounded_cone_renderer(ctx);
	orbit_one.set_render_style(orbit_one_style);
	orbit_one.set_position_array(ctx, pos);

	orbit_one.validate_and_enable(ctx);
	orbit_one.draw(ctx,0,pos.size());
	orbit_one.disable(ctx);


	// draw label
	if (vr_view_ptr && label_tex.is_created()) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		int pi = prog.get_position_index();
		int ti = prog.get_texcoord_index();
		vec3 p(0, 1.5f, 0);
		vec3 y = label_upright ? vec3(0, 1.0f, 0) : normalize(vr_view_ptr->get_view_up_dir_of_kit());
		vec3 x = normalize(cross(vec3(vr_view_ptr->get_view_dir_of_kit()), y));
		float w = 0.5f, h = 0.5f;
		std::vector<vec3> P;
		std::vector<vec2> T;
		P.push_back(p - 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(0.0f, 0.0f));
		P.push_back(p + 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(1.0f, 0.0f));
		P.push_back(p - 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(0.0f, 1.0f));
		P.push_back(p + 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(1.0f, 1.0f));
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
		prog.enable(ctx);
		label_tex.enable(ctx);
		ctx.set_color(rgb(1, 1, 1));
		glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
		label_tex.disable(ctx);
		prog.disable(ctx);
		cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
	}
}

void vr_test::finish_draw(cgv::render::context& ctx)
{
	return;
	if ((!shared_texture && camera_tex.is_created()) || (shared_texture && camera_tex_id != -1)) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		GLint active_texture, texture_binding;
		if (shared_texture) {
			glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
			glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_binding);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, camera_tex_id);
		}
		else
			camera_tex.enable(ctx, 0);

		prog.set_uniform(ctx, "texture", 0);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::translate4<double>(0, 3, 0));
		prog.enable(ctx);
		ctx.set_color(rgba(1, 1, 1, 0.8f));
		ctx.tesselate_unit_square();
		prog.disable(ctx);
		if (shared_texture) {
			glActiveTexture(active_texture);
			glBindTexture(GL_TEXTURE_2D, texture_binding);
		}
		else
			camera_tex.disable(ctx);
		ctx.pop_modelview_matrix();
		glDisable(GL_BLEND);
	}
}

void vr_test::create_gui() {
	add_decorator("vr_test", "heading", "level=2");
	add_member_control(this, "mesh_scale", mesh_scale, "value_slider", "min=0.1;max=10;log=true;ticks=true");
	add_gui("mesh_location", mesh_location, "vector", "options='min=-3;max=3;ticks=true");
	add_gui("mesh_orientation", static_cast<dvec4&>(mesh_orientation), "direction", "options='min=-1;max=1;ticks=true");
	add_member_control(this, "ray_length", ray_length, "value_slider", "min=0.1;max=10;log=true;ticks=true");
	add_member_control(this, "show_seethrough", show_seethrough, "check");
	if(last_kit_handle) {
		add_decorator("cameras", "heading", "level=3");
		add_view("nr", nr_cameras);
		if(nr_cameras > 0) {
			connect_copy(add_button("start")->click, cgv::signal::rebind(this, &vr_test::start_camera));
			connect_copy(add_button("stop")->click, cgv::signal::rebind(this, &vr_test::stop_camera));
			add_view("frame_width", frame_width, "", "w=20", "  ");
			add_view("height", frame_height, "", "w=20", "  ");
			add_view("split", frame_split, "", "w=50");
			add_member_control(this, "undistorted", undistorted, "check");
			add_member_control(this, "shared_texture", shared_texture, "check");
			add_member_control(this, "max_rectangle", max_rectangle, "check");
			add_member_control(this, "use_matrix", use_matrix, "check");
			add_member_control(this, "gamma", seethrough_gamma, "value_slider", "min=0.1;max=10;log=true;ticks=true");
			add_member_control(this, "extent_x", extent_texcrd[0], "value_slider", "min=0.2;max=2;ticks=true");
			add_member_control(this, "extent_y", extent_texcrd[1], "value_slider", "min=0.2;max=2;ticks=true");
			add_member_control(this, "center_left_x", center_left[0], "value_slider", "min=0.2;max=0.8;ticks=true");
			add_member_control(this, "center_left_y", center_left[1], "value_slider", "min=0.2;max=0.8;ticks=true");
			add_member_control(this, "center_right_x", center_right[0], "value_slider", "min=0.2;max=0.8;ticks=true");
			add_member_control(this, "center_right_y", center_right[1], "value_slider", "min=0.2;max=0.8;ticks=true");
			add_member_control(this, "background_distance", background_distance, "value_slider", "min=0.1;max=10;log=true;ticks=true");
			add_member_control(this, "background_extent", background_extent, "value_slider", "min=0.01;max=10;log=true;ticks=true");
		}
		vr::vr_kit* kit_ptr = vr::get_vr_kit(last_kit_handle);
		if (kit_ptr) {
			add_decorator("controller input configs", "heading", "level=3");
			int ti = 0, si = 0, pi = 0;
			const auto& CI = kit_ptr->get_device_info().controller[0];
			for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii) {
				std::string prefix;
				switch (CI.input_type[ii]) {
				case vr::VRI_TRIGGER: prefix = std::string("trigger[") + cgv::utils::to_string(ti++) + "]"; break;
				case vr::VRI_PAD:     prefix = std::string("pad[") + cgv::utils::to_string(pi++) + "]"; break;
				case vr::VRI_STICK:   prefix = std::string("strick[") + cgv::utils::to_string(si++) + "]"; break;
				default:              prefix = std::string("unknown[") + cgv::utils::to_string(ii) + "]";
				}
				add_member_control(this, prefix + ".dead_zone", left_inp_cfg[ii].dead_zone, "value_slider", "min=0;max=1;ticks=true;log=true");
				add_member_control(this, prefix + ".precision", left_inp_cfg[ii].precision, "value_slider", "min=0;max=1;ticks=true;log=true");
				add_member_control(this, prefix + ".threshold", left_inp_cfg[ii].threshold, "value_slider", "min=0;max=1;ticks=true");
			}
		}
	}
	if (begin_tree_node("box style", style)) {
		align("\a");
		add_gui("box style", style);
		align("\b");
		end_tree_node(style);
	}
	if (begin_tree_node("cone style", cone_style)) {
		align("\a");
		add_gui("cone style", cone_style);
		align("\b");
		end_tree_node(cone_style);
	}
	if(begin_tree_node("movable box style", movable_style)) {
		align("\a");
		add_gui("movable box style", movable_style);
		align("\b");
		end_tree_node(movable_style);
	}
	if(begin_tree_node("intersections", srs)) {
		align("\a");
		add_gui("sphere style", srs);
		align("\b");
		end_tree_node(srs);
	}
	if(begin_tree_node("mesh", mesh_scale)) {
		align("\a");
		add_member_control(this, "scale", mesh_scale, "value_slider", "min=0.0001;step=0.0000001;max=100;log=true;ticks=true");
		add_gui("location", mesh_location, "", "main_label='';long_label=true;gui_type='value_slider';options='min=-2;max=2;step=0.001;ticks=true'");
		add_gui("orientation", static_cast<dvec4&>(mesh_orientation), "direction", "main_label='';long_label=true;gui_type='value_slider';options='min=-1;max=1;step=0.001;ticks=true'");
		align("\b");
		end_tree_node(mesh_scale);
	}

	if(begin_tree_node("label", label_size)) {
		align("\a");
		add_member_control(this, "text", label_text);
		add_member_control(this, "upright", label_upright);
		add_member_control(this, "font", (cgv::type::DummyEnum&)label_font_idx, "dropdown", font_enum_decl);
		add_member_control(this, "face", (cgv::type::DummyEnum&)label_face_type, "dropdown", "enums='regular,bold,italics,bold+italics'");
		add_member_control(this, "size", label_size, "value_slider", "min=8;max=64;ticks=true");
		add_member_control(this, "color", label_color);
		add_member_control(this, "resolution", (cgv::type::DummyEnum&)label_resolution, "dropdown", "enums='256=256,512=512,1024=1024,2048=2048'");
		align("\b");
		end_tree_node(label_size);
	}
}

#include <cgv/base/register.h>

cgv::base::object_registration<vr_test> vr_test_reg("vr_test");
