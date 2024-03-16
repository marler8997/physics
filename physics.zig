const builtin = @import("builtin");
const std = @import("std");

const RenderImage = @import("RenderImage.zig");
const Rgb = @import("Rgb.zig");
const XY = @import("xy.zig").XY;

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
    direction: Xyz(i32),
    // distance from the pos to the screen
    len: i32,
    //planks_per_pixel: Ratio(i32),
};

const Sphere = struct {
    center: Xyz(i32),
    radius: i32,
    rgb: Rgb,
};

// 3 spheres
const global = struct {
    pub var spheres = [_]Sphere{.{
        .center = .{ .x =  0, .y = 0, .z = 40_000 },
        .radius = 10_000,
        .rgb = .{ .r = 255, .g = 0, .b = 0 },
    }, .{
        .center = .{ .x = 20_000, .y = 20_000, .z = 50_000 },
        .radius = 10_000,
        .rgb = .{ .r = 255, .g = 255, .b = 0 },
    }};
    pub var camera = Camera{
        .pos = .{ .x = 0, .y = 0, .z = 0 },
        .direction = .{ .x = 0, .y = 0, .z = 1 },
        .len = 10,
    };
    pub var user_input: struct {
        forward: ControlState = .up,
        backward: ControlState = .up,
        left: ControlState = .up,
        right: ControlState = .up,
    } = .{};
};

fn intersectsSphere(
    pos: [3]i64,
    dir: [3]i64,
    center: [3]i64,
    radius: i64,
) bool {
    var a: i64 = 0;
    var b: i64 = 0;
    var c: i64 = -radius * radius;
    for (0 .. 3) |i| {
        a += dir[i] * dir[i];
        b += 2 * dir[i] * (pos[i] - center[i]);
        c += (pos[i] - center[i]) * (pos[i] - center[i]);
    }
    const discriminant = b*b - 4*a*c;
    const intersects = discriminant >= 0;
    if (intersects) {
        //std.log.info("pt {},{},{}", .{pos[0], pos[1], pos[2]});
        //std.log.info("dir {},{},{}", .{dir[0], dir[1], dir[2]});
    }
    return intersects;
}

fn testIntersectsSphere(
    intersects: bool,
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
        true,
        .{0, 0, 0}, // pos
        .{0, 0, 1}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
    try testIntersectsSphere(
        true,
        .{0, 0, 0}, // pos
        .{0, 1, 11}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
    try testIntersectsSphere(
        true,
        .{0, 0, 0}, // pos
        .{0, -1, 11}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
    try testIntersectsSphere(
        false,
        .{0, 0, 0}, // pos
        .{0, 2, 11}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
    try testIntersectsSphere(
        false,
        .{0, 0, 0}, // pos
        .{0, -2, 11}, // dir
        .{0, 0, 10}, // center
        1, // radius
    );
}


fn raycast(pos: Xyz(i32), dir: Xyz(i32)) Rgb {
    for (global.spheres) |sphere| {
        if (intersectsSphere(
            [3]i64{ pos.x, pos.y, pos.z },
            [3]i64{ dir.x, dir.y, dir.z },
            [3]i64{ sphere.center.x, sphere.center.y, sphere.center.z },
            sphere.radius,
        )) return sphere.rgb;
    }

    if (dir.x <= 0) {
        return if (dir.y <= 0)
            .{ .r = 0, .g = 0, .b = 0 }
        else
            .{ .r = 0, .g = 0, .b = 255 };
    } else {
        return if (dir.y <= 0)
            .{ .r = 0, .g = 255, .b = 0 }
        else
            .{ .r = 0, .g = 255, .b = 255 };
    }
}

pub const Control = enum {
    forward,
    backward,
    left,
    right,
};
pub const ControlState = enum { up, down };
pub fn onControl(key: Control, state: ControlState) void {
    std.log.info("key {s}: {s}", .{@tagName(key), @tagName(state)});
    const state_ref: *ControlState = switch (key) {
        .forward => &global.user_input.forward,
        .backward => &global.user_input.backward,
        .left => &global.user_input.left,
        .right => &global.user_input.right,
    };
    state_ref.* = state;
}


var render_state: u8 = 0;

pub fn render(image: RenderImage, size: XY(usize)) void {
    render_state +%= 1;

    if (global.user_input.forward == .down) {
        // TODO: move relative to camera direction
        global.camera.pos.z += 1_000;
    }
    if (global.user_input.backward == .down) {
        // TODO: move relative to camera direction
        global.camera.pos.z -= 1_000;
    }
    if (global.user_input.left == .down) {
        // TODO: move relative to camera direction
        global.camera.pos.x -= 1_000;
    }
    if (global.user_input.right == .down) {
        // TODO: move relative to camera direction
        global.camera.pos.x += 1_000;
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
            image.setPixel(
                col, row,
                raycast(global.camera.pos, direction),
            );
        }
    }
}
