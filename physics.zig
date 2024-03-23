const builtin = @import("builtin");
const std = @import("std");

const RenderImage = @import("RenderImage.zig");
const Rgb = @import("Rgb.zig");
const XY = @import("xy.zig").XY;
const zmath = @import("zmath");

const Dist = f64;
fn centimeters(comptime d: comptime_float) comptime_float { return d; }
fn meters     (comptime d: comptime_float) comptime_float { return d * 100; }
fn kilometers (comptime d: comptime_float) comptime_float { return d * 10000; }

const Mass = f64;
fn grams    (comptime m: comptime_float) comptime_float { return m; }
fn kilograms(comptime m: comptime_float) comptime_float { return m * 1000; }

fn distCast(v: anytype) Dist {
    const V = @TypeOf(v);
    return switch (@typeInfo(Dist)) {
        .Int => switch (@typeInfo(V)) {
            .Int => @intCast(v),
            .Float => @intFromFloat(v),
            else => @compileError("todo: distCast to " ++ @typeName(V)),
        },
        .Float => switch (@typeInfo(V)) {
            .Int => @floatFromInt(v),
            .Float => @floatCast(v),
            else => @compileError("todo: distCast to " ++ @typeName(V)),
        },
        else => @compileError("todo: handle Dist type " ++ @typeName(Dist)),
    };
}
fn floatFromDist(comptime Float: type, dist: Dist) Float {
    switch (@typeInfo(Dist)) {
        .Int => return @floatFromInt(dist),
        .Float => return @floatCast(dist),
        else => @compileError("todo: handle Dist type " ++ @typeName(Dist)),
    }
}
fn distDiv(num: Dist, denom: Dist) Dist {
    switch (@typeInfo(Dist)) {
        .Int => return @divTrunc(num, denom),
        .Float => return num / denom,
        else => @compileError("todo: handle Dist type " ++ @typeName(Dist)),
    }
}

// The camera contains a pos which is a point in 3d space.
// It works casting rays from pos for every pixel.
// The angle of each ray is determined by the pixel position
// on the screen and the length from the pos to the screen.
//
// | Screen |
//  \      /
//   \    /
//    \  /
//     \/  <--- camera pos
//     /\
//    /  \
//   /    \
//  / Scene\
// /        \
//
const Camera = struct {
    // camera is now attached to global.spheres[0]
    //pos: [3]Dist,
    pitch: f32,
    yaw: f32,
};

const Sphere = struct {
    center: [3]Dist,
    radius: Dist,
    velocity: [3]Dist,
    mass: Mass,
    rgb: Rgb,
    is_light_source: bool,
};

const earth_radius = kilometers(6371);

const max_sphere_count = 20;
const max_user_speed = 30;
const global = struct {
    pub var raytrace: bool = false;
    pub var user_speed: u8 = 4;

    pub var spheres = [_]Sphere{.{
        // player/camera
        .center = .{ 0, earth_radius + meters(1), 0 },
        .radius = meters(1),
        .velocity = .{ 0, 0, 0 },
        .mass = kilograms(68),
        .rgb = .{ .r = 0, .g = 255, .b = 0 },
        .is_light_source = false,
    }, .{
        // earth
        .center = .{  0, 0, 0 },
        .radius = kilometers(6371),
        .velocity = .{ 0, 0, 0 },
        .mass = kilograms(5.97219e24),
        .rgb = .{ .r = 102, .g = 51, .b = 0 },
        .is_light_source = false,
    }, .{
        // sun
        .center = .{  0, 0, kilometers(149_000_000) },
        .radius = kilometers(696_340),
        .velocity = .{ 0, 0, 0 },
        .mass = kilograms(1.9891e30),
        .rgb = .{ .r = 255, .g = 255, .b = 255 },
        .is_light_source = true,
    }, .{
        // basketball
        .center = .{  meters(-0.5), earth_radius + meters(2), meters(25) },
        .radius = centimeters(12),
        .velocity = .{ 0, 0, 0 },
        .mass = grams(600),
        .rgb = .{ .r = 212, .g = 102, .b = 38 },
        .is_light_source = false,
    }};
    pub var camera = Camera{
        .pitch = 0,
        .yaw = 0,
    };
    pub var user_input: struct {
        forward: ControlState = .up,
        backward: ControlState = .up,
        left: ControlState = .up,
        right: ControlState = .up,
        pitch_up: ControlState = .up,
        pitch_down: ControlState = .up,
        turn_left: ControlState = .up,
        turn_right: ControlState = .up,
    } = .{};
};

fn testLog(comptime fmt: []const u8, args: anytype) void {
    const stderr = std.io.getStdErr().writer();
    stderr.print(fmt ++ "\n", args) catch |err|
        std.debug.panic("testLog failed, error={s}", .{@errorName(err)});
}

fn intersectsSphere(
    pos: [3]Dist,
    dir: [3]Dist,
    center: [3]Dist,
    radius: Dist,
) [2]Dist {
    var a: Dist = 0;
    var b: Dist = 0;
    var c: Dist = -radius * radius;
    for (0 .. 3) |i| {
        a += dir[i] * dir[i];
        b += 2 * dir[i] * (pos[i] - center[i]);
        c += (pos[i] - center[i]) * (pos[i] - center[i]);
    }
    const discriminant = b*b - 4*a*c;
    if (discriminant < 0)
        return .{ -1, -1 }; // no intersection

    // Calculate the two solutions (t1 and t2) for the quadratic equation.
    const sqrtDiscriminant: Dist = distCast(std.math.sqrt(floatFromDist(f64, discriminant)));
    const t1 = distDiv(-b - sqrtDiscriminant, 2*a);
    const t2 = distDiv(-b + sqrtDiscriminant, 2*a);
    return .{ t1, t2 };
}

fn testIntersectsSphere(
    intersects: [2]Dist,
    pos: [3]Dist,
    dir: [3]Dist,
    center: [3]Dist,
    radius: Dist,
) !void {
    try std.testing.expectEqual(intersects, intersectsSphere(
        pos, dir, center, radius
    ));
    try std.testing.expectEqual(intersects, intersectsSphere(
        .{ pos[1], pos[2], pos[0] },
        .{ dir[1], dir[2], dir[0] },
        .{ center[1], center[2], center[0] },
        radius,
    ));
    try std.testing.expectEqual(intersects, intersectsSphere(
        .{ pos[2], pos[0], pos[1] },
        .{ dir[2], dir[0], dir[1] },
        .{ center[2], center[0], center[1] },
        radius,
    ));
}

test "line intersects sphere " {
    try testIntersectsSphere(
        .{9, 11},
        .{0, 0, 0}, // pos
        .{0, 0, 1}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
    try testIntersectsSphere(
        .{0, 0},
        .{0, 0, 0}, // pos
        .{0, 1, 11}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
    try testIntersectsSphere(
        .{0, 0},
        .{0, 0, 0}, // pos
        .{0, -1, 11}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
    try testIntersectsSphere(
        .{0, 0},
        .{0, 0, 0}, // pos
        .{0, 1, -11}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
    try testIntersectsSphere(
        .{-1, -1},
        .{0, 0, 0}, // pos
        .{0, 2, 11}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
    try testIntersectsSphere(
        .{-1, -1},
        .{0, 0, 0}, // pos
        .{0, -2, 11}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
}

fn calcMagnitude3d(comptime Float: type, v: [3]Dist) Float {
    return @sqrt(
        floatFromDist(Float, (v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]))
    );
}

fn calcDistance3d(comptime Float: type, a: [3]Dist, b: [3]Dist) Float {
    return calcMagnitude3d(Float, .{
        a[0] - b[0],
        a[1] - b[1],
        a[2] - b[2],
    });
}

fn calcNormal(comptime Float: type, v: [3]Dist) [3]Float {
    const magnitude = calcMagnitude3d(Float, v);
    return [3]Float{
        floatFromDist(Float, v[0]) / magnitude,
        floatFromDist(Float, v[1]) / magnitude,
        floatFromDist(Float, v[2]) / magnitude,
    };
}

fn filterColor(color: Rgb, light: Rgb) Rgb {
    return .{
        .r = @intFromFloat(@round(@as(f32, @floatFromInt(color.r)) * (@as(f32, @floatFromInt(light.r)) / 255.0))),
        .g = @intFromFloat(@round(@as(f32, @floatFromInt(color.g)) * (@as(f32, @floatFromInt(light.g)) / 255.0))),
        .b = @intFromFloat(@round(@as(f32, @floatFromInt(color.b)) * (@as(f32, @floatFromInt(light.b)) / 255.0))),
    };
}

fn getReflectDir(dir: [3]Dist, normal: [3]f32) [3]Dist {
    const scale = 2 * (
        floatFromDist(f32, dir[0]) * normal[0] +
        floatFromDist(f32, dir[1]) * normal[1] +
        floatFromDist(f32, dir[2]) * normal[2]
    );
    return .{
        distCast(floatFromDist(f32, dir[0]) - scale * normal[0]),
        distCast(floatFromDist(f32, dir[1]) - scale * normal[1]),
        distCast(floatFromDist(f32, dir[2]) - scale * normal[2]),
    };
}

fn raycast(pos: [3]Dist, dir: [3]Dist, depth: u32, maybe_exclude: ?*Sphere) ?Rgb {
    {
        var maybe_closest: ?struct {
            sphere: *Sphere,
            dist: Dist,
        } = null;
        // skip the first sphere since that's the player
        for (global.spheres[1..]) |*sphere| {
            if (maybe_exclude) |exclude| {
                if (sphere == exclude) continue;
            }

            const distances = intersectsSphere(
                pos, dir,
                sphere.center,
                sphere.radius,
            );
            for (distances) |dist| {
                if (dist >= 0) {
                    if (maybe_closest) |min| {
                        if (min.dist == dist) {
                            std.debug.panic("2 objects at the same distance {d} pos {d},{d},{d}!?!", .{
                                dist,
                                pos[0], pos[1], pos[2],
                            });
                        }
                    }
                    const new_min = if (maybe_closest) |min| dist < min.dist else true;
                    if (new_min) {
                        maybe_closest = .{ .sphere = sphere, .dist = dist };
                    }
                }
            }
        }

        if (maybe_closest) |closest| {
            if (!global.raytrace)
                return closest.sphere.rgb;
            if (closest.sphere.is_light_source)
                return closest.sphere.rgb;

            const max_depth = 6;
            if (depth >= max_depth) {
                @panic("TODO: max depth!");
                //return null;
            }

            const normal = calcNormal(f32, dir);
            const new_pos: [3]Dist = .{
                pos[0] + distCast(normal[0] * floatFromDist(f32, closest.dist)),
                pos[1] + distCast(normal[1] * floatFromDist(f32, closest.dist)),
                pos[2] + distCast(normal[2] * floatFromDist(f32, closest.dist)),
            };
            const reflect_normal = calcNormal(f32, .{
                new_pos[0] - closest.sphere.center[0],
                new_pos[1] - closest.sphere.center[1],
                new_pos[2] - closest.sphere.center[2],
            });
            const new_dir = getReflectDir(dir, reflect_normal);
            const light = raycast(new_pos, new_dir, depth + 1, closest.sphere) orelse return null;
            return filterColor(closest.sphere.rgb, light);
        }
    }

    return null;
}

pub const Control = enum {
    forward,
    backward,
    left,
    right,
    pitch_up,
    pitch_down,
    turn_left, // yaw
    turn_right, // yaw
};
pub const ControlState = enum { up, down };
pub fn onControl(control: Control, state: ControlState) void {
    std.log.info("control {s}: {s}", .{@tagName(control), @tagName(state)});
    const state_ref: *ControlState = switch (control) {
        .forward => &global.user_input.forward,
        .backward => &global.user_input.backward,
        .left => &global.user_input.left,
        .right => &global.user_input.right,
        .pitch_up => &global.user_input.pitch_up,
        .pitch_down => &global.user_input.pitch_down,
        .turn_left => &global.user_input.turn_left,
        .turn_right => &global.user_input.turn_right,
    };
    state_ref.* = state;
}

pub const ControlEvent = enum {
    speed_up,
    speed_down,
    toggle_raytrace,
};
pub fn onControlEvent(event: ControlEvent) void {
    switch (event) {
        .speed_up => {
            if (global.user_speed < max_user_speed) {
                global.user_speed += 1;
                std.log.info("user_speed = {}", .{global.user_speed});
            }
        },
        .speed_down => {
            if (global.user_speed > 0) {
                global.user_speed -= 1;
                std.log.info("user_speed = {}", .{global.user_speed});
            }
        },
        .toggle_raytrace => {
            global.raytrace = !global.raytrace;
            std.log.info("raytrace = {}", .{global.raytrace});
        },
    }
}

fn enforceNoOverlap() void {
    for (&global.spheres, 0..) |*sphere, i| {
        for (global.spheres[i + 1..]) |other_sphere| {
            const dist_vector = [3]Dist {
                sphere.center[0] - other_sphere.center[0],
                sphere.center[1] - other_sphere.center[1],
                sphere.center[2] - other_sphere.center[2],
            };
            const dist = calcMagnitude3d(f64, dist_vector);
            const min_dist = floatFromDist(f64, sphere.radius + other_sphere.radius);
            if (dist < min_dist)
                std.debug.panic("two spheres are overlapping!", .{});
        }
    }
}

pub fn init() void {
    enforceNoOverlap();
}

const gravity_constant: comptime_float = 0.000000000052;

fn applyGravity() void {
    for (&global.spheres, 0..) |*sphere, first_sphere_index| {
        for (global.spheres[first_sphere_index + 1..]) |*other_sphere| {
            const dist_vector = [3]Dist {
                sphere.center[0] - other_sphere.center[0],
                sphere.center[1] - other_sphere.center[1],
                sphere.center[2] - other_sphere.center[2],
            };
            const dist_magnitude = calcMagnitude3d(f64, dist_vector);
            const dist_vector_normalized = [3]f64{
                floatFromDist(f64, dist_vector[0]) / dist_magnitude,
                floatFromDist(f64, dist_vector[1]) / dist_magnitude,
                floatFromDist(f64, dist_vector[2]) / dist_magnitude,
            };
            //const force = gravity_constant * sphere.mass * other_sphere.mass / (dist_magnitude * dist_magnitude);
            //std.log.info("force {}", .{force});
            const dist_mult: f64 = gravity_constant / (dist_magnitude * dist_magnitude);
            //std.log.info("gravity {} {}: {}", .{first_sphere_index, j, dist_mult});
            const parts = [3]f64{
                dist_vector_normalized[0] * dist_mult,
                dist_vector_normalized[1] * dist_mult,
                dist_vector_normalized[2] * dist_mult,
            };
            sphere.velocity[0] -= distCast(parts[0] * other_sphere.mass / sphere.mass);
            sphere.velocity[1] -= distCast(parts[1] * other_sphere.mass / sphere.mass);
            sphere.velocity[2] -= distCast(parts[2] * other_sphere.mass / sphere.mass);
            //std.log.info("  {}: velocity {d},{d},{d}", .{first_sphere_index, sphere.velocity[0], sphere.velocity[1], sphere.velocity[2]});
            other_sphere.velocity[0] += distCast(parts[0] * sphere.mass / other_sphere.mass);
            other_sphere.velocity[1] += distCast(parts[1] * sphere.mass / other_sphere.mass);
            other_sphere.velocity[2] += distCast(parts[2] * sphere.mass / other_sphere.mass);
            //std.log.info("  {}: velocity {d},{d},{d}", .{j, other_sphere.velocity[0], other_sphere.velocity[1], other_sphere.velocity[2]});
        }
    }
}
var frame_count: u32 = 0;
fn move() void {
    // algorithm depends on this being true before beginning
    // to avoid infinite passes to resolve collisions
    //enforceNoOverlap();

    var new_centers: [max_sphere_count][3]Dist = undefined;

    for (&global.spheres, 0..) |*sphere, i| {
        new_centers[i] = [3]Dist {
            sphere.center[0] + sphere.velocity[0],
            sphere.center[1] + sphere.velocity[1],
            sphere.center[2] + sphere.velocity[2],
        };
    }

    var pass: u32 = 0;
    while (true) : (pass += 1) {
        var collisions_resolved: u32 = 0;
        for (&global.spheres, 0..) |*sphere, i| {
            for (global.spheres[i + 1..], i + 1..) |*other_sphere, j| {
                const dist_vector = [3]Dist {
                    new_centers[i][0] - new_centers[j][0],
                    new_centers[i][1] - new_centers[j][1],
                    new_centers[i][2] - new_centers[j][2],
                };
                const dist = calcMagnitude3d(f64, dist_vector);
                const min_dist = floatFromDist(f64, sphere.radius + other_sphere.radius);
                if (dist < min_dist) {
                    //std.log.info("COLLISION between {} and {}!", .{i, j});
                    collisions_resolved += 1;
                    // TODO: adjust velocity/etc better :)
                    sphere.velocity[0] = 0;
                    sphere.velocity[1] = 0;
                    sphere.velocity[2] = 0;
                    other_sphere.velocity[0] = 0;
                    other_sphere.velocity[1] = 0;
                    other_sphere.velocity[2] = 0;
                    // TODO: move both spheres close toward their current
                    //       location rather than just snapping back to
                    //       their original location
                    new_centers[i] = sphere.center;
                    new_centers[j] = other_sphere.center;
                }
            }
        }
        if (collisions_resolved == 0) break;
        //std.log.info("resolved {} collisions", .{collisions_resolved});
    }

    for (&global.spheres, 0..) |*sphere, i| {
        sphere.center = new_centers[i];
    }
    enforceNoOverlap();
}

fn getPlayerMove() ?[3]f32 {
    const maybe_z: ?f32 = if (global.user_input.forward == .down)
        (if (global.user_input.backward == .down) null else 1)
        else (if (global.user_input.backward == .down) -1 else null);
    const maybe_x: ?f32 = if (global.user_input.right == .down)
        (if (global.user_input.left == .down) null else 1)
        else (if (global.user_input.left == .down) -1 else null);
    if (maybe_z == null and maybe_x == null)
        return null;
    return calcNormal(f32, .{
        maybe_x orelse 0,
        0,
        maybe_z orelse 0,
    });
}

pub fn render(image: RenderImage, size: XY(usize)) void {
    applyGravity();
    move();

    if (global.user_input.pitch_up == .down) {
        global.camera.pitch -= 0.01;
    }
    if (global.user_input.pitch_down == .down) {
        global.camera.pitch += 0.01;
    }
    if (global.user_input.turn_left == .down) {
        global.camera.yaw -= 0.01;
    }
    if (global.user_input.turn_right == .down) {
        global.camera.yaw += 0.01;
    }
    const rotate_quat = zmath.quatFromRollPitchYaw(
        global.camera.pitch,
        global.camera.yaw,
        0,
    );

    if (getPlayerMove()) |player_move_unrotated| {
        const player_move_normal = zmath.rotate(rotate_quat, @Vector(4, f32){
            player_move_unrotated[0],
            player_move_unrotated[1],
            player_move_unrotated[2],
            1,
        });
        const user_speed: f32 = std.math.pow(f32, 2, @floatFromInt(global.user_speed));
        const new_player_pos = [3]Dist{
            global.spheres[0].center[0] + distCast(player_move_normal[0] * user_speed),
            global.spheres[0].center[1] + distCast(player_move_normal[1] * user_speed),
            global.spheres[0].center[2] + distCast(player_move_normal[2] * user_speed),
        };
        var move_has_collision = false;
        for (global.spheres[1..]) |other_sphere| {
            const dist_vector = [3]Dist {
                new_player_pos[0] - other_sphere.center[0],
                new_player_pos[1] - other_sphere.center[1],
                new_player_pos[2] - other_sphere.center[2],
            };
            const dist = calcMagnitude3d(f64, dist_vector);
            const min_dist = floatFromDist(f64, global.spheres[0].radius + other_sphere.radius);
            if (dist < min_dist) {
                std.log.info("player move collision", .{});
                move_has_collision = true;
                break;
            }
        }
        if (!move_has_collision) {
            enforceNoOverlap();
            global.spheres[0].center = new_player_pos;
            // sanity check
            enforceNoOverlap();
        }
    }

    const half_size: XY(Dist) = .{
        .x = distDiv(distCast(size.x), 2),
        .y = distDiv(distCast(size.y), 2),
    };
    const default_z_magnitude = (@abs(half_size.x) + @abs(half_size.y)) / 2;
    const focal_mult = 10.0;
    const z_dir: Dist = focal_mult * default_z_magnitude;

    // Put the camera at the top of our sphere
    // TODO: this won't be right once we add the ability to
    //       change the pitch/roll of the camera
    const camera_pos = [3]Dist{
        global.spheres[0].center[0],
        global.spheres[0].center[1] + global.spheres[0].radius,
        global.spheres[0].center[2],
    };

    for (0 .. size.y) |row| {
        for (0 .. size.x) |col| {
            const pixel_pos: XY(Dist) = .{
                .x = distCast(col),
                .y = distCast(row),
            };
            const direction: [3]Dist = .{
                pixel_pos.x - half_size.x,
                pixel_pos.y - half_size.y,
                distCast(z_dir),
            };
            const new_direction_vec = zmath.rotate(rotate_quat, @Vector(4, f32){
                floatFromDist(f32, direction[0]),
                floatFromDist(f32, direction[1]),
                floatFromDist(f32, direction[2]),
                1,
            });
            const new_direction = [3]Dist{
                distCast(new_direction_vec[0]),
                distCast(new_direction_vec[1]),
                distCast(new_direction_vec[2]),
            };
            const ray: Rgb = raycast(
                camera_pos,
                new_direction,
                0,
                null,
            ) orelse (
                if (global.raytrace) .{ .r = 0, .g = 0, .b = 0 }
                else .{ .r = 0, .g = 255, .b = 255 }
             );
            image.setPixel(col, row, ray);
        }
    }
}
