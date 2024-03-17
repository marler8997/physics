const builtin = @import("builtin");
const std = @import("std");

const RenderImage = @import("RenderImage.zig");
const Rgb = @import("Rgb.zig");
const XY = @import("xy.zig").XY;
const zmath = @import("zmathmini.zig");

// A World Unit
//const Unit = i64;
const Unit = f32;
fn unitCast(v: anytype) Unit {
    const V = @TypeOf(v);
    return switch (@typeInfo(Unit)) {
        .Int => switch (@typeInfo(V)) {
            .Int => @intCast(v),
            .Float => @intFromFloat(v),
            else => @compileError("todo: unitCast to " ++ @typeName(V)),
        },
        .Float => switch (@typeInfo(V)) {
            .Int => @floatFromInt(v),
            .Float => @floatCast(v),
            else => @compileError("todo: unitCast to " ++ @typeName(V)),
        },
        else => @compileError("todo: handle Unit type " ++ @typeName(Unit)),
    };
}
fn floatFromUnit(comptime Float: type, unit: Unit) Float {
    switch (@typeInfo(Unit)) {
        .Int => return @floatFromInt(unit),
        .Float => return @floatCast(unit),
        else => @compileError("todo: handle Unit type " ++ @typeName(Unit)),
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
    // initial position of every ray that is cast
    pos: [3]Unit,
    yaw: f32,
    //direction: [3]Unit,
    // distance from the pos to the screen
    len: Unit,
    //planks_per_pixel: Ratio(Unit),
};

const Sphere = struct {
    center: [3]Unit,
    radius: Unit,
    rgb: Rgb,
    is_light_source: bool,
};

const ground_radius = 2_000_000;

// 3 spheres
const global = struct {
    pub var raytrace: bool = false;
    pub var spheres = [_]Sphere{.{
        .center = .{ 0, -ground_radius, 0 },
        .radius = ground_radius,
        .rgb = .{ .r = 102, .g = 51, .b = 0 },
        .is_light_source = false,
    }, .{
        .center = .{  0, 0, 40_000 },
        .radius = 5_000,
        .rgb = .{ .r = 255, .g = 0, .b = 0 },
        .is_light_source = false,
    }, .{
        .center = .{ 20_000, 150_000, 50_000 },
        .radius = 120_000,
        .rgb = .{ .r = 255, .g = 255, .b = 255 },
        .is_light_source = true,
    }, .{
        .center = .{ -14_000, 4_000, 50_000 },
        .radius = 3_000,
        .rgb = .{ .r = 255, .g = 0, .b = 255 },
        .is_light_source = false,
    }};
    pub var camera = Camera{
        .pos = .{ 0, 1_000, 0 },
        .yaw = 0,
        .len = 10,
    };
    pub var user_input: struct {
        forward: ControlState = .up,
        backward: ControlState = .up,
        left: ControlState = .up,
        right: ControlState = .up,
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
    pos: [3]Unit,
    dir: [3]Unit,
    center: [3]Unit,
    radius: Unit,
) [2]Unit {
    var a: Unit = 0;
    var b: Unit = 0;
    var c: Unit = -radius * radius;
    for (0 .. 3) |i| {
        a += dir[i] * dir[i];
        b += 2 * dir[i] * (pos[i] - center[i]);
        c += (pos[i] - center[i]) * (pos[i] - center[i]);
    }
    const discriminant = b*b - 4*a*c;
    if (discriminant < 0)
        return .{ -1, -1 }; // no intersection

    // Calculate the two solutions (t1 and t2) for the quadratic equation.
    const sqrtDiscriminant: Unit = unitCast(std.math.sqrt(floatFromUnit(f32, discriminant)));
    const t1 = @divTrunc(-b - sqrtDiscriminant, 2*a);
    const t2 = @divTrunc(-b + sqrtDiscriminant, 2*a);
    return .{ t1, t2 };
}

fn testIntersectsSphere(
    intersects: [2]Unit,
    pos: [3]Unit,
    dir: [3]Unit,
    center: [3]Unit,
    radius: Unit,
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

fn calcNormal(comptime Float: type, v: [3]Unit) [3]Float {
    const length: f32 = std.math.sqrt(
        (v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2])
    );
    return [3]Float{
        floatFromUnit(Float, v[0]) / length,
        floatFromUnit(Float, v[1]) / length,
        floatFromUnit(Float, v[2]) / length,
    };
}

fn filterColor(color: Rgb, light: Rgb) Rgb {
    return .{
        .r = @intFromFloat(@round(@as(f32, @floatFromInt(color.r)) * (@as(f32, @floatFromInt(light.r)) / 255.0))),
        .g = @intFromFloat(@round(@as(f32, @floatFromInt(color.g)) * (@as(f32, @floatFromInt(light.g)) / 255.0))),
        .b = @intFromFloat(@round(@as(f32, @floatFromInt(color.b)) * (@as(f32, @floatFromInt(light.b)) / 255.0))),
    };
}

fn getReflectDir(dir: [3]Unit, normal: [3]f32) [3]Unit {
    const scale = 2 * (
        floatFromUnit(f32, dir[0]) * normal[0] +
        floatFromUnit(f32, dir[1]) * normal[1] +
        floatFromUnit(f32, dir[2]) * normal[2]
    );
    return .{
        unitCast(floatFromUnit(f32, dir[0]) - scale * normal[0]),
        unitCast(floatFromUnit(f32, dir[1]) - scale * normal[1]),
        unitCast(floatFromUnit(f32, dir[2]) - scale * normal[2]),
    };
}

fn raycast(pos: [3]Unit, dir: [3]Unit, depth: u32, maybe_exclude: ?*Sphere) ?Rgb {
    {
        var maybe_closest: ?struct {
            sphere: *Sphere,
            dist: Unit,
        } = null;
        for (&global.spheres) |*sphere| {
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
            const new_pos: [3]Unit = .{
                pos[0] + unitCast(normal[0] * floatFromUnit(f32, closest.dist)),
                pos[1] + unitCast(normal[1] * floatFromUnit(f32, closest.dist)),
                pos[2] + unitCast(normal[2] * floatFromUnit(f32, closest.dist)),
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
        .turn_left => &global.user_input.turn_left,
        .turn_right => &global.user_input.turn_right,
    };
    state_ref.* = state;
}

pub const ControlEvent = enum {
    toggle_raytrace,
};
pub fn onControlEvent(event: ControlEvent) void {
    std.log.info("control event {s}", .{@tagName(event)});
    switch (event) {
        .toggle_raytrace => global.raytrace = !global.raytrace,
    }
}

fn moveCamera(rotate_quat: zmath.Quat, vec: @Vector(4, f32)) void {
    const new_direction_vec = zmath.rotate(rotate_quat, vec);
    global.camera.pos[0] += unitCast(@round(new_direction_vec[0]));
    global.camera.pos[1] += unitCast(@round(new_direction_vec[1]));
    global.camera.pos[2] += unitCast(@round(new_direction_vec[2]));
}

pub fn render(image: RenderImage, size: XY(usize)) void {
    const rotate_quat = zmath.quatFromRollPitchYaw(0, global.camera.yaw, 0);

    if (global.user_input.forward == .down) {
        moveCamera(rotate_quat, @Vector(4, f32){ 0, 0, 1_000, 1});
    }
    if (global.user_input.backward == .down) {
        moveCamera(rotate_quat, @Vector(4, f32){ 0, 0, -1_000, 1});
    }
    if (global.user_input.left == .down) {
        moveCamera(rotate_quat, @Vector(4, f32){ -1_000, 0, 0, 1});
    }
    if (global.user_input.right == .down) {
        moveCamera(rotate_quat, @Vector(4, f32){ 1_000, 0, 0, 1});
    }
    if (global.user_input.turn_left == .down) {
        global.camera.yaw -= 0.1;
    }
    if (global.user_input.turn_right == .down) {
        global.camera.yaw += 0.1;
    }

    const half_size: XY(Unit) = .{
        .x = @divTrunc(unitCast(size.x), 2),
        .y = @divTrunc(unitCast(size.y), 2),
    };
    const z_dir: Unit = unitCast((@abs(half_size.x) + @abs(half_size.y)) / 2);

    for (0 .. size.y) |row| {
        for (0 .. size.x) |col| {
            const pixel_pos: XY(Unit) = .{
                .x = unitCast(col),
                .y = unitCast(row),
            };
            const direction: [3]Unit = .{
                pixel_pos.x - half_size.x,
                pixel_pos.y - half_size.y,
                z_dir,
            };
            const new_direction_vec = zmath.rotate(rotate_quat, @Vector(4, f32){
                floatFromUnit(f32, direction[0]),
                floatFromUnit(f32, direction[1]),
                floatFromUnit(f32, direction[2]),
                1,
            });
            const new_direction = [3]Unit{
                unitCast(new_direction_vec[0]),
                unitCast(new_direction_vec[1]),
                unitCast(new_direction_vec[2]),
            };
            const ray: Rgb = raycast(global.camera.pos, new_direction, 0, null) orelse (
                if (global.raytrace) .{ .r = 0, .g = 0, .b = 0 }
                else .{ .r = 0, .g = 255, .b = 255 }
             );
            image.setPixel(col, row, ray);
        }
    }
}
