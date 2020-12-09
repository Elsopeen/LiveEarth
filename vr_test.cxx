﻿#include "vr_test.h"

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
/*void vr_test::construct_table(float tw, float td, float th, float tW) {
	// construct table
	rgb table_clr(0.3f, 0.2f, 0.0f);
	boxes.push_back(box3(
		vec3(-0.5f*tw, th - tW, -0.5f*td),
		vec3(0.5f*tw, th, 0.5f*td)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f*tw + 2*tW, 0, -0.5f*td+2*tW), vec3(-0.5f*tw+tW, th - tW, -0.5f*td + tW)));
	boxes.push_back(box3(vec3(-0.5f*tw + 2*tW, 0, 0.5f*td-2*tW), vec3(-0.5f*tw + tW, th - tW, 0.5f*td - tW)));
	boxes.push_back(box3(vec3(0.5f*tw-2*tW, 0, -0.5f*td+tW), vec3(0.5f*tw - tW, th - tW, -0.5f*td +2* tW)));
	boxes.push_back(box3(vec3(0.5f*tw-2*tW, 0, 0.5f*td-2*tW), vec3(0.5f*tw - tW, th - tW, 0.5f*td - tW)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
}

/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_test::construct_room(float w, float d, float h, float W, bool walls, bool ceiling) {
	// construct floor
	boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d), vec3(0.5f*w, 0, 0.5f*d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if(walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w, h, -0.5f*d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f*w, -W, 0.5f*d), vec3(0.5f*w, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w + W, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if(ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f*w - W, h, -0.5f*d - W), vec3(0.5f*w + W, h + W, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}

/// construct boxes for environment
void vr_test::construct_environment(float s, float ew, float ed, float w, float d, float h) {
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	float ox = 0.5f*float(n)*s;
	float oz = 0.5f*float(m)*s;
	for(unsigned i = 0; i < n; ++i) {
		float x = i * s - ox;
		for(unsigned j = 0; j < m; ++j) {
			float z = j * s - oz;
			if(fabsf(x) < 0.5f*w && fabsf(x + s) < 0.5f*w && fabsf(z) < 0.5f*d && fabsf(z + s) < 0.5f*d)
				continue;
			float h = 0.2f*(std::max(abs(x) - 0.5f*w, 0.0f) + std::max(abs(z) - 0.5f*d, 0.0f))*distribution(generator) + 0.1f;
			boxes.push_back(box3(vec3(x, 0.0f, z), vec3(x + s, h, z + s)));
			rgb color = cgv::media::color<float, cgv::media::HLS>(distribution(generator), 0.1f*distribution(generator) + 0.15f, 0.3f);
			box_colors.push_back(color);
			/*box_colors.push_back(
				rgb(0.3f*distribution(generator) + 0.3f,
					0.3f*distribution(generator) + 0.2f,
					0.2f*distribution(generator) + 0.1f));*\/
		}
	}
}

/// construct boxes that can be moved around
void vr_test::construct_movable_boxes(float tw, float td, float th, float tW, size_t nr) {
	
	vec3 extent(0.75f, 0.5f, 0.05f);
	movable_boxes.push_back(box3(-0.5f * extent, 0.5f * extent));
	movable_box_colors.push_back(rgb(0, 0, 0));
	movable_box_translations.push_back(vec3(0, 1.2f, 0));
	movable_box_rotations.push_back(quat(1, 0, 0, 0));
	
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);
	for(size_t i = 0; i < nr; ++i) {
		float x = distribution(generator);
		float y = distribution(generator);
		vec3 extent(distribution(generator), distribution(generator), distribution(generator));
		extent += 0.01f;
		extent *= std::min(tw, td)*0.1f;

		vec3 center(-0.5f*tw + x * tw, th + tW, -0.5f*td + y * td);
		movable_boxes.push_back(box3(-0.5f*extent, 0.5f*extent));
		movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
		movable_box_translations.push_back(center);
		quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator), signed_distribution(generator));
		rot.normalize();
		movable_box_rotations.push_back(rot);
	}
}*/

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

	if (earth_sphere.read("../../../src/models/sphere_36_cuts.obj")) {
		earth_info.construct(ctx, earth_sphere);
		earth_info.bind(ctx, ctx.ref_surface_shader_program(true), true);
		auto& mats = earth_info.get_materials();
		if (mats.size() > 0) {
			//setup texture paths
			int di = mats[0]->add_image_file("../../../src/images/earthmap.jpg");
			
			//ensure the textures are loaded
			mats[0]->ensure_textures(ctx);

			//set texture indices
			mats[0]->set_diffuse_index(di);
		}
	}

	//to finish later
	/*std::string line;
	std::ifstream file_reader("../../../src/sat_data/station.txt");
	if (file_reader.is_open())
	{
		while (std::getline(file_reader, line))
		{
			std::cout << line << std::endl;
			std::regex reg_line_one("^1 (\d{5})U (\d{2})(\d{3})([A-Z]{0,3} {0,3}) (\d{2})(\d{3})(\.\d+ *) (\.\d+ *) (\d{5})-(\d+ *) (\d{5})-(\d+ *) (\d *) (\d{4})$");
			std::regex reg_line_two("^2 (\d{5})  ([\d\.]+) ([\d\.]+) (\d+) ([\d\.]+) {1,2}([\d\.]+) (\d{2}\.\d{8})(\d{5})(\d)$");
			std::regex reg_name("^([A-Z]+.*)$");
			std::smatch matches;

			if (std::regex_search(line, matches, reg_line_one)) {
				std::cout << "Match found line 1\n";

				for (size_t i = 0; i < matches.size(); ++i) {
					std::cout << i << ": '" << matches[i].str() << "'\n";
				}
			}
			else if (std::regex_search(line, matches, reg_line_two)){
				std::cout << "Match found line 2\n";

				for (size_t i = 0; i < matches.size(); ++i) {
					std::cout << i << ": '" << matches[i].str() << "'\n";
				}
			}
			else if (std::regex_search(line, matches, reg_name)) {
				std::cout << "Match found title\n";

				for (size_t i = 0; i < matches.size(); ++i) {
					std::cout << i << ": '" << matches[i].str() << "'\n";
				}
			}
			else {
				std::cout << "No match found" << std::endl;
			}
		}
		file_reader.close();
	}*/
	orbit_name = "ISS";
	line_1 = {25544, 98,067,20,336,0.88693861,.00004720, 00000,0,0.93347,4,0,9996};
	line_2 = { 25544, 52.6470, 238.6835, .0001915, 101.4338,18.8734,15.49130315,25801,5 };
	//reading data
	int year;
	if (line_1[3]>=0 && line_1[3]<=56) {
		year = 2000 + line_1[3];
	}
	else if (line_1[3] >= 57 && line_1[3] <= 99) {
		year = 1900 + line_1[3];
	}

	epoch.tm_isdst = -1;
	epoch.tm_yday = line_1[4];
	epoch.tm_year = year - 1900;
	epoch_time = mktime(&epoch);

	orbit_incl = line_2[1];
	raan = line_2[2];
	eccentricity = line_2[3];
	arg_perigee = line_2[4];
	mean_anom = line_2[5];
	mean_motion = line_2[6];

	bstar = line_1[10] * pow(10, -1 * line_1[11]);

	//calculating orig mean motion and orig semimaj axis
	double a1 = pow(ke / (mean_motion * rev_per_day_to_rad_per_sec), 2.0 / 3.0);
	double delt1 = 3.0 / 2 * (k2 / pow(a1, 2)) *
		((3 * pow(cos(orbit_incl * deg_to_rad), 2) - 1) / pow(1 - pow(eccentricity, 2), 3.0 / 2));
	double a0 = a1 * (1 - (1.0 / 3 * delt1) - pow(delt1, 2) - (134.0 / 81 * pow(delt1, 3)));
	double delt0 = 3.0 / 2 * (k2 / pow(a0, 2)) *
		((3 * pow(cos(orbit_incl * deg_to_rad), 2) - 1) / pow(1 - pow(eccentricity, 2), 3.0 / 2));
	orig_mean_motion = mean_motion * rev_per_day_to_rad_per_sec / (1 + delt0);
	orig_semimaj_axis = a0 / (1 - delt0);

	//calculating perigee and changing the values of s and (q0-s)^4
	double perigee = (orig_semimaj_axis * pow(10,-3) - earth_radius_at_equator)* (1 - eccentricity) ;

	if (perigee > 98 && perigee < 156) {
		s_param = orig_semimaj_axis * (1 - eccentricity) - s_density_param + earth_radius_at_equator;
		q0_min_s_four = pow(pow(pow(q0_density_param - s_density_param, 4), 1.0 / 4) + s_density_param - s_param, 4);
	}
	else if (perigee < 98) {
		s_param = 20.0 / km_per_earth_radii + earth_radius_at_equator;
		q0_min_s_four = pow(pow(pow(q0_density_param - s_density_param, 4), 1.0 / 4) + s_density_param - s_param, 4);
	}
	else {
		s_param = s_density_param;
		q0_min_s_four = pow(q0_density_param - s_density_param, 4);
	}
	//computing constants
	theta = std::cos(orbit_incl * deg_to_rad);
	xi = 1 / (orig_semimaj_axis - s_param);
	beta0 = pow(1 - pow(eccentricity, 2), 0.5);
	eta = orig_semimaj_axis * eccentricity * xi;

	C2 = q0_min_s_four * pow(xi, 4) * orig_mean_motion
		* pow(1 - pow(eta, 2), -7.0 / 2.0) * (orig_semimaj_axis *
			(1 + (3.0 / 2 * pow(eta, 2)) + (4 * eccentricity * eta) + (eccentricity * pow(eta, 3)))
			+ 3.0 / 2 * ((k2 * xi) / (1 - pow(eta, 2))) * (-1.0 / 2 + 3.0 / 2 * pow(theta, 2)) 
			* (8 + 24 * pow(eta, 2) + 3 * pow(eta, 4)));
	C1 = bstar * C2;
	C3 = (q0_min_s_four * pow(xi, 5) * A30 * orig_mean_motion 
		* earth_radius_at_equator * sin(orbit_incl * deg_to_rad)) /
		(k2 * eccentricity);
	C4 = 2 * orig_mean_motion * q0_min_s_four * pow(xi, 4) * orig_semimaj_axis * pow(beta0, 2)
		* pow(1 - pow(eta, 2), -7.0 / 2) * ((2 * eta * (1 + eccentricity * eta) + 0.5 * eccentricity + 0.5 * pow(eta, 3))
			- (2 * k2 * xi) / (orig_semimaj_axis * (1 - pow(eta, 2)))
			* (3 * (1 - 3 * pow(theta, 2))
				* (1 + 3.0 / 2 * pow(eta, 2) - 2 * eccentricity * eta 
					- 0.5 * eccentricity * pow(eta, 3))
				+ 3.0 / 4 * (1 - pow(theta, 2)) 
				* (2 * pow(eta, 2) - eccentricity * eta - eccentricity * pow(eta, 3)) 
				* cos(2 * arg_perigee)));
	C5 = 2 * q0_min_s_four * pow(xi, 4) * orig_semimaj_axis * pow(beta0, 2) 
		* pow(1 - pow(eta, 2), -7.0 / 2)
		* (1 + 11.0 / 4 * eta * (eta + eccentricity) + eccentricity * pow(eta, 3));
	D2 = 4 * orig_semimaj_axis * xi * pow(C1, 2);
	D3 = 4.0 / 3 * orig_semimaj_axis * pow(xi, 2) * (17 * orig_semimaj_axis + s_param) * pow(C1, 3);
	D4 = 2.0 / 3 * orig_semimaj_axis * pow(xi, 3) * (221 * orig_semimaj_axis + 31 * s_param) * pow(C1, 4);
	//computing secular effects
	t_min_t0 = time(0) - epoch_time;

	secul_anomaly = mean_anom + (1 + (3 * k2 * (-1 + 3 * pow(theta, 2))) / (2 * pow(orig_semimaj_axis, 2) * pow(beta0, 3))
		+ (3 * pow(k2, 2) * (13 - 78 * pow(theta, 2) + 137 * pow(theta, 4)) / (16 * pow(orig_semimaj_axis, 4) * pow(beta0, 7))))
		* orig_mean_motion * (t_min_t0);
	secul_arg_perigee = arg_perigee + (-(3 * k2 * (1 - 5 * pow(theta, 2))) / (2 * pow(orig_semimaj_axis, 2) * pow(beta0, 4))
		+ (3 * pow(k2, 2) * (7 - 114 * pow(theta, 2) + 395 * pow(theta, 4))) / (16 * pow(orig_semimaj_axis, 4) * pow(beta0, 8))
		+ (5 * k4 * (3 - 36 * pow(theta, 2) + 49 * pow(theta, 4))) / (4 * pow(orig_semimaj_axis, 4) * pow(beta0, 8)))
		* orig_mean_motion * t_min_t0;
	secul_raan = raan + (-(3 * k2 * theta) / (pow(orig_semimaj_axis, 2) * pow(beta0, 4))
		+ (3 * pow(k2, 2) * (4 * theta - 19 * pow(theta, 3))) / (2 * pow(orig_semimaj_axis, 4) * pow(beta0, 8))
		+ (5 * k4 * theta * (3 - 7 * pow(theta, 2))) / (2 * pow(orig_semimaj_axis, 4) * pow(beta0, 8)))
		* orig_mean_motion * t_min_t0;
	if (perigee >= 220) {
		delta_arg_perig = bstar * C3 * cos(arg_perigee * deg_to_rad) * t_min_t0;
		delta_anom = -2.0 / 3 * q0_min_s_four * bstar * pow(xi, 4) * earth_radius_at_equator / (eccentricity * eta)
			* (pow(1 + eta * cos(secul_anomaly), 3) - pow(1 + eta * cos(mean_anom), 3));
		anom_p = secul_anomaly + delta_arg_perig + delta_anom;
		arg_perigee_fixed = secul_arg_perigee - delta_arg_perig - delta_anom;
		raan_fixed = secul_raan - 21.0 / 2
			* (orig_mean_motion * k2 * theta) / (pow(orig_semimaj_axis, 2) * pow(beta0, 2))
			* C1 * pow(t_min_t0, 2);
		eccentricity_fixed = eccentricity - bstar * C4 * t_min_t0 - bstar * C5 * (sin(anom_p) - sin(mean_anom));

		semimaj_axis_fixed = orig_semimaj_axis 
			* pow(1-C1*t_min_t0-D2*pow(t_min_t0,2)-D3*pow(t_min_t0,3)-D4*pow(t_min_t0,4), 2);
		L = anom_p + arg_perigee_fixed + raan_fixed + orig_mean_motion
			* (3.0 / 2 * C1 * pow(t_min_t0, 2) + (D2 + 2 * pow(C1, 2)) * pow(t_min_t0, 3)
				+ 1.0 / 4 * (3 * D3 + 12 * C1 * D2 + 10 * pow(C1, 3)) * pow(t_min_t0, 4)
				+ 1.0 / 5 * (3 * D4 + 12 * C1 * D3 + 6 * pow(D2, 2) 
					+ 30 * pow(C1, 2) * D2 + 15 * pow(C1, 4)) * pow(t_min_t0, 5));
		}
	else {
		anom_p = secul_anomaly;
		arg_perigee_fixed = secul_arg_perigee;
		raan_fixed = secul_raan - 21.0 / 2
			* (orig_mean_motion * k2 * theta) / (pow(orig_semimaj_axis, 2) * pow(beta0, 2))
			* C1 * pow(t_min_t0, 2);
		eccentricity_fixed = eccentricity - bstar * C4 * t_min_t0;
		semimaj_axis_fixed = orig_semimaj_axis * pow(1 - C1 * t_min_t0, 2);
		L = anom_p + arg_perigee_fixed + raan_fixed + orig_mean_motion * 3.0 / 2 * C1 * pow(t_min_t0, 2);
	}
	beta = pow(1 - pow(eccentricity_fixed, 2), 0.5);
	mean_motion_fixed = ke / pow(semimaj_axis_fixed, 2.0 / 3);

	//long-period periodic terms
	a_x_N = eccentricity_fixed * cos(arg_perigee_fixed);
	L_L = (A30 * sin(orbit_incl * deg_to_rad)) / (8 * k2 * semimaj_axis_fixed * pow(beta, 2))
		* (eccentricity_fixed * cos(arg_perigee_fixed))
		* ((3 + 5 * theta) / (1 + theta));
	a_y_N_L = (A30 * sin(orbit_incl * deg_to_rad)) / (4 * k2 * semimaj_axis_fixed * pow(beta, 2));
	L_T = L + L_L;
	a_y_N = eccentricity_fixed * sin(arg_perigee_fixed) + a_y_N_L;

	//Solve Kepler's equations for (E+lower_omeg)
	U = L_T - raan_fixed;
	temp2 = U;
	for (int i = 0; i < 30; i++) {
		sinEpw = sin(temp2);
		cosEpw = cos(temp2);
		temp3 = a_x_N * sinEpw;
		temp4 = a_y_N * cosEpw;
		temp5 = a_x_N * cosEpw;
		temp6 = a_y_N * sinEpw;
		Epw = (U - temp4 + temp3 - temp2) / (1 - temp5 - temp6) + temp2;
		if (abs(Epw - temp2) < pow(10, -6))
			break;
		temp2 = Epw;
	}
	//prelim for short-period periodics
	ecosE = a_x_N * cos(Epw) + a_y_N * sin(Epw);
	esinE = a_x_N * sin(Epw) + a_y_N * cos(Epw);
	e_L = pow(pow(a_x_N,2) + pow(a_y_N,2), 0.5);
	p_L = semimaj_axis_fixed * (1 - pow(e_L, 2));
	r = semimaj_axis_fixed * (1 - ecosE);
	r_dot = ke * pow(semimaj_axis_fixed, 0.5) / r * esinE;
	r_f_dot = ke * pow(p_L, 0.5) / r;
	cos_u = semimaj_axis_fixed / r * (cos(Epw) - a_x_N + (a_y_N * esinE) / (1 + pow(1 - pow(e_L, 2), 0.5)));
	sin_u = semimaj_axis_fixed / r * (sin(Epw) - a_y_N - (a_x_N * esinE) / (1 + pow(1 - pow(e_L, 2), 0.5)));
	u = atan(sin_u / cos_u);
	delt_r = k2 / (2 * p_L) * (1 - pow(theta, 2)) * cos(2 * u);
	delt_u = -k2 / (4 * pow(p_L, 2)) * (7 * pow(theta, 2) - 1) * sin(2 * u);
	delt_raan = (3 * k2 * theta) / (2 * pow(p_L, 2)) * sin(2 * u);
	delt_incl = (3 * k2 * theta) / (2 * pow(p_L, 2)) * sin(orbit_incl) * cos(2 * u);
	delt_r_dot = -(k2 * mean_motion_fixed) / p_L * (1 - pow(theta, 2)) * sin(2 * u);
	delt_r_f_dot = (k2 * mean_motion_fixed) / p_L 
		* ((1 - pow(theta, 2)) * cos(2 * u) - 3.0 / 2 * (1 - 3 * pow(theta, 2)));


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
	double R = 0.5;
	ctx.mul_modelview_matrix(
		cgv::math::translate4<double>(vec3(0,0,0))*
		cgv::math::scale4<double>(dvec3(vec3(1,1,1)))*R
	);

	// actually draw the mesh
	earth_info.draw_all(ctx);


	// restore the previous transform

	ctx.pop_modelview_matrix();



	/*cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
	
	// draw dynamic boxes 
	renderer.set_render_style(movable_style);
	renderer.set_box_array(ctx, movable_boxes);
	renderer.set_color_array(ctx, movable_box_colors);
	renderer.set_translation_array(ctx, movable_box_translations);
	renderer.set_rotation_array(ctx, movable_box_rotations);
	if (renderer.validate_and_enable(ctx)) {
		if (show_seethrough) {
			glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
			renderer.draw(ctx, 0, 3);
			glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
			//renderer.draw(ctx, 3, movable_boxes.size() - 3);
		}
		//else
			//renderer.draw(ctx, 0, movable_boxes.size());
	}
	renderer.disable(ctx);*/

	// draw static boxes
	/*renderer.set_render_style(style);
	renderer.set_box_array(ctx, boxes);
	renderer.set_color_array(ctx, box_colors);
	renderer.render(ctx, 0, boxes.size());*/


	// draw intersection points
	/*if (!intersection_points.empty()) {
		auto& sr = cgv::render::ref_sphere_renderer(ctx);
		sr.set_position_array(ctx, intersection_points);
		sr.set_color_array(ctx, intersection_colors);
		sr.set_render_style(srs);
		sr.render(ctx, 0, intersection_points.size());
	}*/

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
