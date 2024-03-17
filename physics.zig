const builtin = @import("builtin");
const std = @import("std");

const RenderImage = @import("RenderImage.zig");
const Rgb = @import("Rgb.zig");
const XY = @import("xy.zig").XY;
const zmath = @import("zmathmini.zig");

fn Xyz(comptime T: type) type {
    return struct {
        x: T,
        y: T,
        z: T,
    };
}

fn Ratio(comptime T: type) type {
    return struct { num: T, denom: T };
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
    pos: Xyz(i32),
    yaw: f32,
    //direction: Xyz(i32),
    // distance from the pos to the screen
    len: i32,
    //planks_per_pixel: Ratio(i32),
};

const Sphere = struct {
    center: Xyz(i32),
    radius: i32,
    rgb: Rgb,
};

const ground_radius = 2_000_000;

// 3 spheres
const global = struct {
    pub var spheres = [_]Sphere{.{
        .center = .{ .x =  0, .y = -ground_radius, .z = 0 },
        .radius = ground_radius,
        .rgb = .{ .r = 102, .g = 51, .b = 0 },
    }, .{
        .center = .{ .x =  0, .y = 0, .z = 40_000 },
        .radius = 5_000,
        .rgb = .{ .r = 255, .g = 0, .b = 0 },
    }, .{
        .center = .{ .x = 20_000, .y = 50_000, .z = 50_000 },
        .radius = 20_000,
        .rgb = .{ .r = 255, .g = 255, .b = 0 },
    }, .{
        .center = .{ .x = -14_000, .y = 4_000, .z = 50_000 },
        .radius = 3_000,
        .rgb = .{ .r = 255, .g = 0, .b = 255 },
    }};
    pub var camera = Camera{
        .pos = .{ .x = 0, .y = 1_000, .z = 0 },
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
    pos: [3]i64,
    dir: [3]i64,
    center: [3]i64,
    radius: i64,
) [2]i64 {
    var a: i64 = 0;
    var b: i64 = 0;
    var c: i64 = -radius * radius;
    for (0 .. 3) |i| {
        a += dir[i] * dir[i];
        b += 2 * dir[i] * (pos[i] - center[i]);
        c += (pos[i] - center[i]) * (pos[i] - center[i]);
    }
    const discriminant = b*b - 4*a*c;
    if (discriminant < 0)
        return .{ -1, -1 }; // no intersection

    // Calculate the two solutions (t1 and t2) for the quadratic equation.
    const sqrtDiscriminant: i64 = @intFromFloat(std.math.sqrt(@as(f64, @floatFromInt(discriminant))));
    const t1 = @divTrunc(-b - sqrtDiscriminant, 2*a);
    const t2 = @divTrunc(-b + sqrtDiscriminant, 2*a);
    return .{ t1, t2 };
}

fn testIntersectsSphere(
    intersects: [2]i64,
    pos: [3]i64,
    dir: [3]i64,
    center: [3]i64,
    radius: i64,
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

fn raycast(pos: Xyz(i32), dir: Xyz(i32)) Rgb {
    {
        var maybe_min: ?struct {
            rgb: Rgb,
            val: i64,
        } = null;
        for (global.spheres) |sphere| {
            const points = intersectsSphere(
                [3]i64{ pos.x, pos.y, pos.z },
                [3]i64{ dir.x, dir.y, dir.z },
                [3]i64{ sphere.center.x, sphere.center.y, sphere.center.z },
                sphere.radius,
            );
            for (points) |pt| {
                if (pt >= 0) {
                    const new_min = if (maybe_min) |min| pt < min.val else true;
                    if (new_min) {
                        maybe_min = .{ .rgb = sphere.rgb, .val = pt };
                    }
                }
            }
        }
        if (maybe_min) |min| return min.rgb;
    }

    return .{ .r = 0, .g = 255, .b = 255 };
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
pub fn onControl(key: Control, state: ControlState) void {
    std.log.info("key {s}: {s}", .{@tagName(key), @tagName(state)});
    const state_ref: *ControlState = switch (key) {
        .forward => &global.user_input.forward,
        .backward => &global.user_input.backward,
        .left => &global.user_input.left,
        .right => &global.user_input.right,
        .turn_left => &global.user_input.turn_left,
        .turn_right => &global.user_input.turn_right,
    };
    state_ref.* = state;
}


fn moveCamera(rotate_quat: zmath.Quat, vec: @Vector(4, f32)) void {
    const new_direction_vec = zmath.rotate(rotate_quat, vec);
    global.camera.pos.x += @intFromFloat(@round(new_direction_vec[0]));
    global.camera.pos.y += @intFromFloat(@round(new_direction_vec[1]));
    global.camera.pos.z += @intFromFloat(@round(new_direction_vec[2]));
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

    const size_i32: XY(i32) = .{
        .x = @intCast(size.x),
        .y = @intCast(size.y),
    };
    const half_size: XY(i32) = .{
        .x = @divTrunc(size_i32.x, 2),
        .y = @divTrunc(size_i32.y, 2),
    };
    const z_dir: i32 = @intCast((@abs(half_size.x) + @abs(half_size.y)) / 2);

    for (0 .. size.y) |row| {
        for (0 .. size.x) |col| {
            const pixel_pos: XY(i32) = .{
                .x = @intCast(col),
                .y = @intCast(row),
            };
            const direction: Xyz(i32) = .{
                .x = pixel_pos.x - half_size.x,
                .y = pixel_pos.y - half_size.y,
                .z = z_dir,
            };
            const new_direction_vec = zmath.rotate(rotate_quat, @Vector(4, f32){
                @floatFromInt(direction.x),
                @floatFromInt(direction.y),
                @floatFromInt(direction.z),
                1,
            });
            const new_direction = Xyz(i32){
                .x = @intFromFloat(new_direction_vec[0]),
                .y = @intFromFloat(new_direction_vec[1]),
                .z = @intFromFloat(new_direction_vec[2]),
            };
            image.setPixel(
                col, row,
                raycast(global.camera.pos, new_direction),
            );
        }
    }
}
