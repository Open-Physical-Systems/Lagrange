/* core/plugin_abi/test/test_abi_layout.c
 * SPDX-License-Identifier: Apache-2.0
 *
 * The plugin C ABI 1.0, pinned (RFC 0007): every struct's size and every field's offset on the
 * LP64 platforms Lagrange targets (Ubuntu 26.04 amd64 and arm64). Within ABI major 1 these may
 * only grow at the end; a change here that is not an append breaks every plugin built for 1.x.
 * Compiled as C11 (for _Static_assert) by the build: a failed assertion stops it. */
#include "lagrange_plugin.h"

#include <stddef.h>

#define PIN(type, size) _Static_assert(sizeof(type) == (size), #type " size changed")
#define AT(type, field, offset) \
    _Static_assert(offsetof(type, field) == (offset), #type "." #field " moved")

_Static_assert(LAGRANGE_PLUGIN_ABI_MAJOR == 1u, "this file pins ABI major 1");

PIN(lagrange_unit, 24);
AT(lagrange_unit, length, 0);
AT(lagrange_unit, angle, 7);
AT(lagrange_unit, scale, 8);
AT(lagrange_unit, kind, 16);
PIN(lagrange_signal_spec, 80);
AT(lagrange_signal_spec, struct_size, 0);
AT(lagrange_signal_spec, name, 8);
AT(lagrange_signal_spec, type, 16);
AT(lagrange_signal_spec, unit, 24);
AT(lagrange_signal_spec, array_length, 48);
AT(lagrange_signal_spec, frame, 56);
AT(lagrange_signal_spec, hold, 64);
AT(lagrange_signal_spec, optional, 65);
AT(lagrange_signal_spec, feedback, 66);
AT(lagrange_signal_spec, default_value, 72);
PIN(lagrange_step_context, 40);
AT(lagrange_step_context, struct_size, 0);
AT(lagrange_step_context, t_ns, 8);
AT(lagrange_step_context, dt_ns, 16);
AT(lagrange_step_context, step_index, 24);
AT(lagrange_step_context, seed, 32);
/* 1.0 ended at 128 bytes; 1.1 appends from there (below). */
AT(lagrange_host_api, world_body_count, 128);
AT(lagrange_host_api, struct_size, 0);
AT(lagrange_host_api, declare_input, 16);
AT(lagrange_host_api, declare_output, 24);
AT(lagrange_host_api, bus_read, 32);
AT(lagrange_host_api, bus_write, 40);
AT(lagrange_host_api, bus_read_n, 48);
AT(lagrange_host_api, bus_write_n, 56);
AT(lagrange_host_api, world_joint_count, 64);
AT(lagrange_host_api, world_joint_name, 72);
AT(lagrange_host_api, world_schema_instance_count, 80);
AT(lagrange_host_api, world_schema_real, 88);
AT(lagrange_host_api, world_schema_instance_name, 96);
AT(lagrange_host_api, config_get_real, 104);
AT(lagrange_host_api, config_get_string, 112);
AT(lagrange_host_api, log, 120);
PIN(lagrange_module_api, 128);
AT(lagrange_module_api, struct_size, 0);
AT(lagrange_module_api, domain, 8);
AT(lagrange_module_api, name, 16);
AT(lagrange_module_api, version_major, 24);
AT(lagrange_module_api, version_minor, 28);
AT(lagrange_module_api, is_substitute, 32);
AT(lagrange_module_api, deterministic, 33);
AT(lagrange_module_api, minimum_tier, 34);
AT(lagrange_module_api, preferred_step_ns, 40);
AT(lagrange_module_api, create, 48);
AT(lagrange_module_api, destroy, 56);
AT(lagrange_module_api, declare, 64);
AT(lagrange_module_api, init, 72);
AT(lagrange_module_api, step, 80);
AT(lagrange_module_api, shutdown, 88);
AT(lagrange_module_api, save_state, 96);
AT(lagrange_module_api, restore_state, 104);
AT(lagrange_module_api, last_error, 112);
AT(lagrange_module_api, shift_origin, 120);
_Static_assert(sizeof(lagrange_status) == 4, "lagrange_status size changed");

int main(void) {
    return 0;
}

/* ABI 1.1 (RFC 0008): appended to lagrange_host_api, and the new structs. */
_Static_assert(LAGRANGE_PLUGIN_ABI_MINOR >= 1u, "this file pins ABI 1.1");
PIN(lagrange_host_api, 232);
AT(lagrange_host_api, scene_overlap_sphere, 224);
PIN(lagrange_body_info, 224);
AT(lagrange_body_info, name, 8);
AT(lagrange_body_info, parent, 16);
AT(lagrange_body_info, robot, 20);
AT(lagrange_body_info, mass, 24);
AT(lagrange_body_info, com, 32);
AT(lagrange_body_info, inertia, 56);
AT(lagrange_body_info, pose_in_parent, 104);
AT(lagrange_body_info, rest_pose, 160);
AT(lagrange_body_info, kinematic, 216);
PIN(lagrange_joint_info, 152);
AT(lagrange_joint_info, name, 8);
AT(lagrange_joint_info, type, 16);
AT(lagrange_joint_info, parent, 20);
AT(lagrange_joint_info, child, 24);
AT(lagrange_joint_info, axis, 32);
AT(lagrange_joint_info, lower, 56);
AT(lagrange_joint_info, upper, 64);
AT(lagrange_joint_info, effort_limit, 72);
AT(lagrange_joint_info, velocity_limit, 80);
AT(lagrange_joint_info, armature, 88);
AT(lagrange_joint_info, initial_position, 96);
AT(lagrange_joint_info, actuated, 104);
AT(lagrange_joint_info, damping, 112);
AT(lagrange_joint_info, friction, 120);
AT(lagrange_joint_info, mimic, 128);
AT(lagrange_joint_info, mimic_multiplier, 136);
AT(lagrange_joint_info, mimic_offset, 144);
PIN(lagrange_collider_info, 160);
AT(lagrange_collider_info, body, 4);
AT(lagrange_collider_info, shape, 8);
AT(lagrange_collider_info, size, 16);
AT(lagrange_collider_info, pose, 40);
AT(lagrange_collider_info, friction, 96);
AT(lagrange_collider_info, point_count, 104);
AT(lagrange_collider_info, triangle_count, 108);
AT(lagrange_collider_info, points, 112);
AT(lagrange_collider_info, triangles, 120);
AT(lagrange_collider_info, columns, 128);
AT(lagrange_collider_info, rows, 132);
AT(lagrange_collider_info, cell, 136);
AT(lagrange_collider_info, heights, 152);
PIN(lagrange_sensor_info, 96);
AT(lagrange_sensor_info, name, 8);
AT(lagrange_sensor_info, kind, 16);
AT(lagrange_sensor_info, body, 24);
AT(lagrange_sensor_info, pose_in_body, 32);
AT(lagrange_sensor_info, rate, 88);
PIN(lagrange_site_info, 56);
AT(lagrange_site_info, gravity, 8);
AT(lagrange_site_info, region_origin, 32);
PIN(lagrange_ray_hit, 24);
AT(lagrange_ray_hit, distance, 8);
AT(lagrange_ray_hit, collider, 16);
AT(lagrange_ray_hit, body, 20);
