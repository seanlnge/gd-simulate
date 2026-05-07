fn resolve_collisions(
    level: &Level,
    tick: usize,
    player: &mut PlayerState,
    which_player: u8,
) -> Option<SimulationOutcome> {
    if player.y < -1200.0 {
        return Some(SimulationOutcome::Died {
            tick,
            time: tick as f32 / 240.0,
            state: *player,
            object_id: None,
            reason: "floor".to_owned(),
            which_player,
        });
    }

    let player_half = player.player_half();
    // snap_up_threshold from gdp::collidedWithObjectInternal. Non-platformer,
    // non-flying, vehicle-size 1 uses 10; mini uses 5; flyers use 6.
    let snap_threshold: f32 = match player.mode {
        GameMode::Ship | GameMode::Ufo | GameMode::Wave | GameMode::Swing => 6.0,
        _ if player.mini => 5.0,
        _ => 10.0,
    };

    let mut grounded_this_tick = false;
    let was_on_slope = player.on_slope;
    let mut on_slope_this_tick = false;
    let mut best_slope_snap_score: Option<f32> = None;
    // Target rotation (degrees) of the winning slope this tick, fed into the
    // post-pass cube-rotation lerp. None = no slope contact this tick.
    let mut slope_target_rotation_deg: Option<f32> = None;

    // Pass 1: slopes (gdclone triangle + RobTop rotation).
    for (slope_index, object) in nearby_objects(level, player.x) {
        if object.kind != ObjectKind::Slope {
            continue;
        }
        let Some(HitboxData::Slope { half_extents }) = object.hitbox else {
            continue;
        };
        let (hx, hy) = (half_extents[0], half_extents[1]);
        let Some(rect) = object_rect(object) else {
            continue;
        };
        // Three-hitbox split (per the user-specified GD model):
        //   * Non-rotated outer 30x30 (or mini 15x15): selects which slope is
        //     active, gates entry & detach. AABB overlap with the slope's
        //     transformed bounds.
        //   * Rotated outer (circumscribed half = `player_half * SQRT_2`):
        //     interacts with the slope surface (hypotenuse probe + lift).
        // Selection gate using non-rotated outer:
        if !intersects_box_player(rect, *player, player_half) {
            continue;
        }
        // Surface contact: when the cube is rotated to match the slope
        // angle, its 30x30 outer hitbox sits flat on the slope and the
        // perpendicular reach to the slope surface is exactly `player_half`
        // (a flat side of the rotated square against a flat surface). Using
        // a single consistent `player_half` here (instead of the previous
        // flip-flop between `player_half` on entry and `SQRT_2 * player_half`
        // on continuation) eliminates the Y-jump that was the visible Pop
        // slope jitter, while still respecting the rotated-cube model: the
        // cube renders rotated, sits flat on the slope, and snaps smoothly.
        let slope_player_half = player_half;
        // GDP `collidedWithSlopeInternal`: if not already on a slope, require
        // intersection with an exit-rect (obj rect trimmed by 1px vertically)
        // before accepting slope contact.
        if !was_on_slope {
            let exit_half_y = (rect.half_extents[1] - 1.0).max(0.0);
            let exit_rect = Rect {
                center: [rect.center[0], rect.center[1] + 1.0],
                half_extents: [rect.half_extents[0], exit_half_y],
            };
            if !intersects_box_player(exit_rect, *player, slope_player_half) {
                continue;
            }
        }
        let gravity_down = player.gravity_sign < 0.0;
        // GDP slope contact is not just AABB overlap; constrain to points that
        // are near the transformed slope hypotenuse in local space.
        let probe_y = if gravity_down {
            player.y - slope_player_half
        } else {
            player.y + slope_player_half
        };
        let Some((px_l, py_l)) = slope_world_to_local_point(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            player.x,
            probe_y,
        ) else {
            continue;
        };
        if !near_slope_hypotenuse(px_l, py_l, hx, hy) {
            continue;
        }
        let wy_opt = rotated_slope_player_world_y(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            hx,
            hy,
            player.x,
            player.y,
            slope_player_half,
            gravity_down,
        );
        let Some(wy_raw) = wy_opt else {
            continue;
        };
        let (sx1, sy1) = slope_local_to_world_point(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            -hx,
            -hy,
        );
        let (sx2, sy2) = slope_local_to_world_point(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            hx,
            hy,
        );
        let slope_dx = sx2 - sx1;
        let slope_dy = sy2 - sy1;
        // World-rotated hyp length is no longer used for player_rad_on_slope
        // (we use the local intrinsic slope angle per GDP). Kept here for
        // potential future debug; suppressed-warning let _.
        let _slope_len = (slope_dx * slope_dx + slope_dy * slope_dy).sqrt().max(1e-4);
        // GDP `playerRadOnSlope = playerRadius / cos(getSlopeAngle())` uses
        // the slope's *intrinsic* angle (from its local hitbox shape:
        // atan2(hy, hx)), not the world-rotated hypotenuse cos. For a 1x2
        // (hx=30, hy=15) the intrinsic angle is 26.57deg regardless of
        // whether the slope is rotated -90deg in world space. The previous
        // world-cos formula yielded rad=33.5 for vertical 1x2 slopes,
        // pushing the snap target out of range of the snap-attach threshold
        // and silently rejecting astrosoda's first slope.
        let local_slope_angle = hy.atan2(hx);
        let local_cos = local_slope_angle.cos().abs().max(1e-4);
        let player_rad_on_slope = slope_player_half / local_cos;
        let (_tx3, ty3) = slope_local_to_world_point(
            object.x,
            object.y,
            object.rotation,
            object.scale_x,
            object.scale_y,
            hx,
            -hy,
        );
        // `slope_floor_top` = is the slope's solid floor on the *upper* Y
        // side of the rect (i.e. cube hangs from underneath, ceiling
        // ramp)? Cross-product of (sx1->sx2) vs (sx1->t3) flips sign with
        // rotation/flipY combinations and incorrectly flags rotated 1x2
        // slopes (e.g. astrosoda's vertical wall slopes) as ceiling
        // slopes. Robust test: compare the right-angle vertex Y to the
        // hypotenuse mid-Y. If t3 is *above* the hypotenuse mid, the
        // solid mass of the triangle sits above and the cube hangs.
        let hyp_mid_y = (sy1 + sy2) / 2.0;
        let slope_floor_top = ty3 > hyp_mid_y;
        let mut wy = wy_raw;
        if !was_on_slope {
            // Progressive GDP parity: apply first-contact `newPlayerY` clamp
            // by inferred slopeFloorTop polarity, while keeping existing
            // slope-to-slope carry behavior untouched.
            wy = if slope_floor_top {
                let temp = rect.min_y() - slope_player_half;
                wy.max(temp).min(rect.max_y())
            } else {
                let temp = rect.max_y() + slope_player_half;
                wy.min(temp).max(rect.min_y())
            };
        }
        // GDP new-slope transition scalar: when switching between different
        // slope contacts with opposite top/bottom polarity, use
        // `newSlopeScalar = vehicleSize * 20` in the snapped Y solve.
        let is_new_slope = was_on_slope
            && matches!(player.slope_object, Some(prev_idx) if prev_idx != slope_index)
            && player.slope_is_current_top != slope_floor_top;
        if is_new_slope {
            let y_surf = if gravity_down {
                wy_raw - player_rad_on_slope
            } else {
                wy_raw + player_rad_on_slope
            };
            let new_slope_scalar = player.vehicle_size * 20.0;
            let floor_sign = if slope_floor_top { -1.0 } else { 1.0 };
            wy = y_surf + (player_rad_on_slope - new_slope_scalar) * floor_sign;
        }
        // Prevent pathological multi-slope overlaps from teleporting the
        // player onto a faraway surface in one tick. GDP does not use the
        // block snap threshold for slope-to-slope continuity: when the player
        // was already on a slope, collidedWithSlopeInternal compares against
        // the previous slope radius (`onSlopeThreshold`) instead. This larger
        // carry distance is needed for Pop's 26.5° -> 45° -> 63.4° transition.
        let slope_snap_delta = if gravity_down {
            wy - player.y
        } else {
            player.y - wy
        };
        let slope_grade = if slope_dx.abs() > 1e-4 {
            Some(slope_dy / slope_dx)
        } else {
            None
        };
        let player_uphill = slope_grade
            .map(|grade| grade * player.vx * player.flip_mod() < 0.0)
            .unwrap_or(false);
        let float_g = if player_uphill {
            if was_on_slope { 4.0 } else { 1.0 }
        } else {
            0.0
        };
        let prev_rad_on_slope = if was_on_slope {
            player.slope_prev_radius.max(slope_player_half)
        } else {
            player_rad_on_slope
        };
        let on_slope_threshold = prev_rad_on_slope + float_g;
        // Re-attach uses the relaxed `on_slope_threshold` while still in
        // the slope-context window. `slope_contact_cooldown > 0` covers the
        // 1-2 tick gap between back-to-back slope tiles in a chain (Pop's
        // 372 -> 371 -> 372 rotated transition). This is a slope-chaining
        // mechanic, NOT an "affected by previous slope after fully leaving"
        // effect — once the cube fully detaches and the cooldown expires,
        // no further slope influence applies.
        let slope_attach_threshold =
            if was_on_slope || on_slope_this_tick || player.slope_contact_cooldown > 0 {
                on_slope_threshold
            } else {
                snap_threshold
            };
        if slope_snap_delta > slope_attach_threshold {
            continue;
        }
        // GDP-style collidedSlope gate (`bool_h` window): if already above the
        // solved slope Y, allow a narrow re-attach band while moving uphill.
        let bool_h = player_uphill && !is_new_slope && !on_slope_this_tick;
        let continuing_same_slope = was_on_slope
            && matches!(player.slope_object, Some(prev_idx) if prev_idx == slope_index);
        let crossed = if gravity_down {
            player.y <= wy || (bool_h && player.y < wy + float_g) || continuing_same_slope
        } else {
            player.y >= wy || (bool_h && player.y > wy - float_g) || continuing_same_slope
        };
        if crossed {
            // Resolve at most one slope contact per tick: choose the "best"
            // surface in gravity direction to avoid loop-order stair-stepping.
            let snap_score = if gravity_down { wy } else { -wy };
            if let Some(best) = best_slope_snap_score && snap_score <= best {
                continue;
            }
            best_slope_snap_score = Some(snap_score);
            player.slope_object = Some(slope_index);
            player.slope_is_current_top = slope_floor_top;
            player.slope_prev_radius = player_rad_on_slope;
            player.y = wy;
            player.on_ground = true;
            grounded_this_tick = true;
            on_slope_this_tick = true;
            // Target rotation = slope surface angle in world space. In the
            // existing convention `slope_floor_top=true` means the floor is
            // on the top side of the rect (cube hangs underneath = ceiling
            // ramp, requires +180); `false` is the standard ground-side
            // ramp (cube sits on top, rotation = surface angle).
            let raw_angle_deg = slope_dy.atan2(slope_dx).to_degrees();
            let target_deg = if slope_floor_top {
                raw_angle_deg + 180.0
            } else {
                raw_angle_deg
            };
            slope_target_rotation_deg = Some(target_deg);
            // For grounded slope contact, non-flying modes should not keep
            // integrating a large downward velocity while "stuck" to slope.
            // gdp's ground path runs hitGround/updateCollide for these cases.
            // `collidedWithSlopeInternal` also restores pre-hit velocity when
            // it is strongly upward along gravity-opposed direction
            // (`upsideMod * oldVelocity > upsideMod * 5.0`).
            match player.mode {
                GameMode::Cube | GameMode::Robot | GameMode::Spider | GameMode::Ball => {
                    let old_vy = player.vy;
                    let slope_upside_down = (player.gravity_sign > 0.0) != slope_floor_top;
                    if slope_upside_down {
                        // GDP upside-down-slope branch:
                        // keep only velocity moving away from the slope.
                        if old_vy * player.flip_mod() > 0.0 {
                            player.vy = 0.0;
                        } else {
                            player.vy = old_vy;
                        }
                    } else {
                        // GDP non-upside-down branch:
                        // preserve strong upward velocity only.
                        player.vy = 0.0;
                        if old_vy * player.flip_mod() > 5.0 {
                            player.vy = old_vy;
                        }
                    }
                }
                _ => {}
            }
            let slope_grade = transformed_slope_world_grade(
                object.x,
                object.y,
                object.rotation,
                object.scale_x,
                object.scale_y,
                hx,
                hy,
            );
            // Canonical GDP `m_slopeVelocity` port from
            // `PlayerObject_collidedWithSlopeInternal.cpp` line 266:
            //
            //   slopeYVelocity   = (objRect.height * playerSpeed * speedMultiplier) / objRect.width;
            //   m_slopeVelocity  = min(1.12 / slopeAngle, 1.54)
            //                    * slopeYVelocity
            //                    * flipMod
            //                    * (playerUphill ? -1 : 1);
            //   if (flying || ball) m_slopeVelocity *= 0.75;
            //
            // GDP itself only applies `m_slopeVelocity` on a *jump* from
            // (or just-after) a slope (`addToYVelocity(m_slopeVelocity*0.25, 60)`
            // capped at 1.4x base jump in `updateJump.cpp`). Our sim also
            // applies it on detach to approximate the implicit
            // `vy = grade * vx` inertia that GDP gets for free from its
            // slope-Y-tracking position update. This proves out against
            // the recorded `pop.bin` click tape that beat Pop with these
            // values.
            // GDP `m_slopeVelocity` is computed from the slope's *world*
            // bounding rect (`objRect`), not the local hitbox extents.
            // Mixing the two (local-angle multiplier x world-rect ratio)
            // is what made the rotated 1x2 "vertical" slope in Pop launch
            // the cube at ~16 (flat-clamped to 15) instead of the
            // canonical ~10.5: a 1x2 rotated 90deg has world half-extents
            // [15, 30] (steep, atan(2)~=1.107rad, mult=1.012) but local
            // half-extents [30, 15] (gentle, atan2(15,30)=0.464rad,
            // clamped mult=1.54). The world ratio always matches GDP's
            // `objRect.height / objRect.width`.
            let rect_height = rect.half_extents[1] * 2.0;
            let rect_width = (rect.half_extents[0] * 2.0).max(1e-4);
            let slope_angle = (rect_height / rect_width).atan();
            let slope_exit_mult = (1.12_f32 / slope_angle).min(1.54);
            player.slope_exit_vy = slope_grade
                .map(|grade| {
                    let flip_mod = player.flip_mod();
                    let player_uphill = grade * player.vx * flip_mod < 0.0;
                    let slope_dir = if player_uphill { -1.0 } else { 1.0 };
                    let mut slope_velocity = slope_exit_mult
                        * ((rect_height * player.player_speed * player.speed_multiplier)
                            / rect_width)
                        * flip_mod
                        * slope_dir;
                    if matches!(
                        player.mode,
                        GameMode::Ship | GameMode::Ufo | GameMode::Wave | GameMode::Ball
                    ) {
                        slope_velocity *= 0.75;
                    }
                    slope_velocity
                })
                .unwrap_or(0.0);
            // GDP stores scalar slope velocity (`m_slopeVelocity`). Horizontal
            // carry is not applied as a separate slope-exit x impulse.
            player.slope_exit_vx = 0.0;
        }
    }
    player.on_slope = on_slope_this_tick;
    if on_slope_this_tick {
        // `slope_exit_*` updated inside the loop (last matching slope wins).
        // GDP keeps special slope collision context for 0.2 seconds after
        // leaving a slope (`m_totalTime - m_slopeEndTime < 0.2`).
        player.slope_contact_cooldown = 48;
    } else if was_on_slope && (player.slope_exit_vy != 0.0 || player.slope_exit_vx != 0.0) {
        let flip_mod = -player.gravity_sign;
        if player.slope_exit_vy * flip_mod > 0.0
            && player.slope_exit_vy * flip_mod > player.vy * flip_mod
        {
            player.vy = player.slope_exit_vy;
        }
        player.slope_exit_vy = 0.0;
        player.slope_exit_vx = 0.0;
    }
    if !on_slope_this_tick {
        player.slope_object = None;
    }

    // Cube rotation update.
    if player.mode == GameMode::Cube {
        if let Some(target) = slope_target_rotation_deg {
            // Choose the equivalent angle within +/- 180 of current rotation
            // so the lerp takes the short way around (avoids 359->0 wrap).
            let mut delta = target - player.rotation;
            while delta > 180.0 {
                delta -= 360.0;
            }
            while delta < -180.0 {
                delta += 360.0;
            }
            const SLOPE_LERP: f32 = 0.4;
            player.rotation += delta * SLOPE_LERP;
        } else if !player.on_ground {
            let dx_world = player.vx.abs() * SUBSTEP_TO_FRAME * player.player_speed;
            let airborne_forward_sign = if player.gravity_sign < 0.0 { -1.0 } else { 1.0 };
            let detached_from_slope = was_on_slope && !on_slope_this_tick;
            let spin_sign = if detached_from_slope {
                -airborne_forward_sign
            } else {
                airborne_forward_sign
            };
            let use_dash_spin = player.dash_rotation_blocks_remaining > 0.0;
            let deg_per_world = if use_dash_spin {
                360.0 / (DASH_SPIN_DISTANCE_BLOCKS * WORLD_UNITS_PER_BLOCK)
            } else {
                (360.0 * CUBE_AIR_SPIN_ROTATIONS_1X)
                    / (CUBE_AIR_SPIN_JUMP_DISTANCE_1X_BLOCKS * WORLD_UNITS_PER_BLOCK)
            };
            player.rotation += spin_sign * deg_per_world * dx_world;
        } else {
            // Grounded on a flat solid: ease toward the nearest 90-degree
            // orientation so side-landings settle to their nearest cardinal
            // angle instead of always returning to 0.
            const RETURN_LERP: f32 = 0.25;
            let target = (player.rotation / 90.0).round() * 90.0;
            let mut delta = target - player.rotation;
            while delta > 180.0 {
                delta -= 360.0;
            }
            while delta < -180.0 {
                delta += 360.0;
            }
            player.rotation += delta * RETURN_LERP;
            if delta.abs() < 0.05 {
                player.rotation = target;
            }
            player.dash_rotation_blocks_remaining = 0.0;
        }
        player.rotation = normalize_rotation_deg(player.rotation);
    }

    // Pass 2: solids and hazards (and re-process slopes only as no-ops).
    let block_snap: f32 = snap_threshold;

    for (object_index, object) in nearby_objects(level, player.x) {
        let Some(rect) = object_rect(object) else {
            continue;
        };
        if !intersects_box_player(rect, *player, player_half) {
            continue;
        }

        match object.kind {
            ObjectKind::Slope => {
                // Already handled in pass 1.
                continue;
            }
            ObjectKind::Hazard => {
                // OpenGD uses the player outer/main 30x30 box for non-circular
                // hazards (per GD Creator School "Advanced Hitboxes": the main
                // AABB hitbox is what collides with spikes facing upward).
                if lethal_hazard_intersects(object, rect, *player, player_half) {
                    return Some(SimulationOutcome::Died {
                        tick,
                        time: tick as f32 / 240.0,
                        state: *player,
                        object_id: Some(object.object_id),
                        reason: "hazard".to_owned(),
                        which_player,
                    });
                }
            }
            ObjectKind::Solid => {
                if is_h_block(object) {
                    // h-block only suppresses cube ceiling death; it is not
                    // otherwise a collidable platform/wall.
                    continue;
                }
                // Minimum-translation-vector AABB resolution: pick the axis
                // (horizontal vs vertical) with the smaller penetration.
                // Vertical penetration => land/head; horizontal penetration =>
                // either stair-snap (if block top is within reach) or death.
                let dx = player.x - rect.center[0];
                let dy = player.y - rect.center[1];
                let overlap_x = (player_half + rect.half_extents[0]) - dx.abs();
                let overlap_y = (player_half + rect.half_extents[1]) - dy.abs();
                let gravity_down = player.gravity_sign < 0.0;
                let top_surface = rect.max_y();
                let bottom_surface = rect.min_y();
                let land_y = if gravity_down {
                    top_surface + player_half
                } else {
                    bottom_surface - player_half
                };
                let head_y = if gravity_down {
                    bottom_surface - player_half
                } else {
                    top_surface + player_half
                };
                let descending_toward_surface = if gravity_down {
                    player.vy <= 0.0
                } else {
                    player.vy >= 0.0
                };
                let outer_crossed_gravity_surface = if gravity_down {
                    player.y >= top_surface && player.y - player_half <= top_surface
                } else {
                    player.y <= bottom_surface && player.y + player_half >= bottom_surface
                };

                // Vertical separation is shallower (or equal) → it's a top/bottom hit.
                // Cube-specific rule: floor contacts are resolved by the outer
                // hitbox immediately (regardless of MTV axis tie-breaks), while
                // side/ceiling lethality still uses the inner cube core.
                if player.mode == GameMode::Cube
                    && descending_toward_surface
                    && outer_crossed_gravity_surface
                {
                    let flip_mod = -player.gravity_sign;
                    let suppress_floor_snap_for_slope_chain = on_slope_this_tick
                        || (was_on_slope && player.vy * flip_mod > 0.0)
                        || (player.slope_contact_cooldown > 0
                            && player.slope_exit_vy * flip_mod > 0.0);
                    if suppress_floor_snap_for_slope_chain {
                        continue;
                    }
                    player.y = land_y;
                    player.vy = 0.0;
                    player.on_ground = true;
                    grounded_this_tick = true;
                    check_snap_jump_to_object_for_level(level, player, object_index);
                    continue;
                }
                if overlap_y <= overlap_x {
                    let above_block = (gravity_down && dy > 0.0) || (!gravity_down && dy < 0.0);
                    if above_block {
                        let flip_mod = -player.gravity_sign;
                        let suppress_floor_snap_for_slope_chain = on_slope_this_tick
                            || (was_on_slope && player.vy * flip_mod > 0.0)
                            || (player.slope_contact_cooldown > 0
                                && player.slope_exit_vy * flip_mod > 0.0);
                        if suppress_floor_snap_for_slope_chain {
                            // GDP preserves upward slope exit motion through
                            // nearby block contacts; otherwise Pop snaps flat
                            // onto the first wall-stack tile and loses the ramp.
                            continue;
                        }
                        // Land on the gravity-up surface.
                        player.y = land_y;
                        player.vy = 0.0;
                        player.on_ground = true;
                        grounded_this_tick = true;
                        if player.mode == GameMode::Cube {
                            check_snap_jump_to_object_for_level(level, player, object_index);
                        }
                    } else {
                        // Head bonk on the gravity-down surface.
                        if player.mode == GameMode::Cube {
                            let blocked_by_h_square = ceiling_death_blocked_by_h_square(level, *player);
                            let inner_hit =
                                intersects_box_player(rect, *player, cube_lethal_inner_half(player));
                            if !blocked_by_h_square {
                                if inner_hit {
                                    return Some(SimulationOutcome::Died {
                                        tick,
                                        time: tick as f32 / 240.0,
                                        state: *player,
                                        object_id: Some(object.object_id),
                                        reason: "ceiling hit".to_owned(),
                                        which_player,
                                    });
                                }
                                // No h-block and no inner-core ceiling hit:
                                // ignore outer-box ceiling overlap for cubes.
                                continue;
                            }
                        }
                        player.y = head_y;
                        if (gravity_down && player.vy > 0.0) || (!gravity_down && player.vy < 0.0) {
                            player.vy = 0.0;
                        }
                    }
                } else {
                    // Horizontal penetration. gdp's checkSnapJumpToObject lets
                    // the player ride up onto a block whose top is at most
                    // snap_threshold above the current y. If the block top is
                    // out of reach, GD treats horizontal contact as death.
                    let stair_top_within_reach = if gravity_down {
                        land_y - player.y <= block_snap && land_y - player.y >= 0.0
                    } else {
                        player.y - land_y <= block_snap && player.y - land_y >= 0.0
                    };
                    let can_stair_snap = stair_top_within_reach
                        && (descending_toward_surface
                            || player.on_ground
                            || was_on_slope
                            || on_slope_this_tick
                            || player.slope_contact_cooldown > 0);
                    if can_stair_snap {
                        player.y = land_y;
                        player.vy = 0.0;
                        player.on_ground = true;
                        grounded_this_tick = true;
                        if player.mode == GameMode::Cube {
                            check_snap_jump_to_object_for_level(level, player, object_index);
                        }
                    } else if side_hit_is_lethal(
                        rect,
                        *player,
                        player_half,
                        on_slope_this_tick,
                        was_on_slope,
                        player.slope_contact_cooldown,
                    ) {
                        return Some(SimulationOutcome::Died {
                            tick,
                            time: tick as f32 / 240.0,
                            state: *player,
                            object_id: Some(object.object_id),
                            reason: "side hit".to_owned(),
                            which_player,
                        });
                    }
                }
            }
            _ => {}
        }
    }

    if apply_implicit_bounds(level, player) {
        grounded_this_tick = true;
    }
    if !grounded_this_tick {
        player.on_ground = false;
    }
    if !on_slope_this_tick && player.slope_contact_cooldown > 0 {
        player.slope_contact_cooldown -= 1;
    }
    if player.y >= IMPLICIT_CEILING_DEATH_Y
        && ball_window_bounds(level, player).is_none()
        && !(player.mode == GameMode::Cube && ceiling_death_blocked_by_h_square(level, *player))
    {
        return Some(SimulationOutcome::Died {
            tick,
            time: tick as f32 / 240.0,
            state: *player,
            object_id: None,
            reason: "ceiling".to_owned(),
            which_player,
        });
    }
    None
}

