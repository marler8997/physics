// Copied from:
//    https://raw.githubusercontent.com/zig-gamedev/zig-gamedev/a199efd5de1e861b60c92a7e98ac2fcb8f66bfde/libs/zmath/src/zmath.zig
const std = @import("std");
const math = std.math;

// 265
pub const F32x4 = @Vector(4, f32);
pub const F32x8 = @Vector(8, f32);
pub const F32x16 = @Vector(16, f32);
// 275
pub const Vec = F32x4;
// 276
pub const Quat = F32x4;

const builtin = @import("builtin");
const has_avx = if (cpu_arch == .x86_64) std.Target.x86.featureSetHas(builtin.cpu.features, .avx) else false;
const has_avx512f = if (cpu_arch == .x86_64) std.Target.x86.featureSetHas(builtin.cpu.features, .avx512f) else false;
const has_fma = if (cpu_arch == .x86_64) std.Target.x86.featureSetHas(builtin.cpu.features, .fma) else false;
// 284
const cpu_arch = builtin.cpu.arch;
// 293
pub inline fn f32x4(e0: f32, e1: f32, e2: f32, e3: f32) F32x4 {
    return .{ e0, e1, e2, e3 };
}
// 334
pub inline fn veclen(comptime T: type) comptime_int {
    return @typeInfo(T).Vector.len;
}

pub inline fn splat(comptime T: type, value: f32) T {
    return @splat(value);
}
// 634
pub inline fn andInt(v0: anytype, v1: anytype) @TypeOf(v0, v1) {
    const T = @TypeOf(v0, v1);
    const Tu = @Vector(veclen(T), u32);
    const v0u = @as(Tu, @bitCast(v0));
    const v1u = @as(Tu, @bitCast(v1));
    return @as(T, @bitCast(v0u & v1u)); // andps
}
// 658
pub inline fn andNotInt(v0: anytype, v1: anytype) @TypeOf(v0, v1) {
    const T = @TypeOf(v0, v1);
    const Tu = @Vector(veclen(T), u32);
    const v0u = @as(Tu, @bitCast(v0));
    const v1u = @as(Tu, @bitCast(v1));
    return @as(T, @bitCast(~v0u & v1u)); // andnps
}
// 680
pub inline fn orInt(v0: anytype, v1: anytype) @TypeOf(v0, v1) {
    const T = @TypeOf(v0, v1);
    const Tu = @Vector(veclen(T), u32);
    const v0u = @as(Tu, @bitCast(v0));
    const v1u = @as(Tu, @bitCast(v1));
    return @as(T, @bitCast(v0u | v1u)); // orps
}
// 875
pub fn round(v: anytype) @TypeOf(v) {
    const T = @TypeOf(v);
    if (cpu_arch == .x86_64 and has_avx) {
        if (T == F32x4) {
            return asm ("vroundps $0, %%xmm0, %%xmm0"
                : [ret] "={xmm0}" (-> T),
                : [v] "{xmm0}" (v),
            );
        } else if (T == F32x8) {
            return asm ("vroundps $0, %%ymm0, %%ymm0"
                : [ret] "={ymm0}" (-> T),
                : [v] "{ymm0}" (v),
            );
        } else if (T == F32x16 and has_avx512f) {
            return asm ("vrndscaleps $0, %%zmm0, %%zmm0"
                : [ret] "={zmm0}" (-> T),
                : [v] "{zmm0}" (v),
            );
        } else if (T == F32x16 and !has_avx512f) {
            const arr: [16]f32 = v;
            var ymm0 = @as(F32x8, arr[0..8].*);
            var ymm1 = @as(F32x8, arr[8..16].*);
            ymm0 = asm ("vroundps $0, %%ymm0, %%ymm0"
                : [ret] "={ymm0}" (-> F32x8),
                : [v] "{ymm0}" (ymm0),
            );
            ymm1 = asm ("vroundps $0, %%ymm1, %%ymm1"
                : [ret] "={ymm1}" (-> F32x8),
                : [v] "{ymm1}" (ymm1),
            );
            return @shuffle(f32, ymm0, ymm1, [16]i32{ 0, 1, 2, 3, 4, 5, 6, 7, -1, -2, -3, -4, -5, -6, -7, -8 });
        }
    } else {
        const sign = andInt(v, splatNegativeZero(T));
        const magic = orInt(splatNoFraction(T), sign);
        var r1 = v + magic;
        r1 = r1 - magic;
        const r2 = abs(v);
        const mask = r2 <= splatNoFraction(T);
        return select(mask, r1, v);
    }
}
// 1353
pub inline fn abs(v: anytype) @TypeOf(v) {
    return @abs(v); // load, andps
}

pub inline fn select(mask: anytype, v0: anytype, v1: anytype) @TypeOf(v0, v1) {
    return @select(f32, mask, v0, v1);
}
// 1425
pub const F32x4Component = enum { x, y, z, w };

pub inline fn swizzle(
    v: F32x4,
    comptime x: F32x4Component,
    comptime y: F32x4Component,
    comptime z: F32x4Component,
    comptime w: F32x4Component,
) F32x4 {
    return @shuffle(f32, v, undefined, [4]i32{ @intFromEnum(x), @intFromEnum(y), @intFromEnum(z), @intFromEnum(w) });
}
// 1458
pub fn modAngle(v: anytype) @TypeOf(v) {
    const T = @TypeOf(v);
    return switch (T) {
        //f32 => modAngle32(v),
        F32x4, F32x8, F32x16 => modAngle32xN(v),
        else => @compileError("zmath.modAngle() not implemented for " ++ @typeName(T)),
    };
}

pub inline fn modAngle32xN(v: anytype) @TypeOf(v) {
    const T = @TypeOf(v);
    return v - splat(T, math.tau) * round(v * splat(T, 1.0 / math.tau)); // 2 x vmulps, 2 x load, vroundps, vaddps
}

pub inline fn mulAdd(v0: anytype, v1: anytype, v2: anytype) @TypeOf(v0, v1, v2) {
    const T = @TypeOf(v0, v1, v2);
    if (false) {//@import("zmath_options").enable_cross_platform_determinism) {
        return v0 * v1 + v2; // Compiler will generate mul, add sequence (no fma even if the target supports it).
    } else {
        if (cpu_arch == .x86_64 and has_avx and has_fma) {
            return @mulAdd(T, v0, v1, v2);
        } else {
            // NOTE(mziulek): On .x86_64 without HW fma instructions @mulAdd maps to really slow code!
            return v0 * v1 + v2;
        }
    }
}
// 1610
pub fn sincos(v: anytype) [2]@TypeOf(v) {
    const T = @TypeOf(v);
    return switch (T) {
        //f32 => sincos32(v),
        F32x4, F32x8, F32x16 => sincos32xN(v),
        else => @compileError("zmath.sincos() not implemented for " ++ @typeName(T)),
    };
}
// 1637
fn sincos32xN(v: anytype) [2]@TypeOf(v) {
    const T = @TypeOf(v);

    var x = modAngle(v);
    var sign = andInt(x, splatNegativeZero(T));
    const c = orInt(sign, splat(T, math.pi));
    const absx = andNotInt(sign, x);
    const rflx = c - x;
    const comp = absx <= splat(T, 0.5 * math.pi);
    x = select(comp, x, rflx);
    sign = select(comp, splat(T, 1.0), splat(T, -1.0));
    const x2 = x * x;

    var sresult = mulAdd(splat(T, -2.3889859e-08), x2, splat(T, 2.7525562e-06));
    sresult = mulAdd(sresult, x2, splat(T, -0.00019840874));
    sresult = mulAdd(sresult, x2, splat(T, 0.0083333310));
    sresult = mulAdd(sresult, x2, splat(T, -0.16666667));
    sresult = x * mulAdd(sresult, x2, splat(T, 1.0));

    var cresult = mulAdd(splat(T, -2.6051615e-07), x2, splat(T, 2.4760495e-05));
    cresult = mulAdd(cresult, x2, splat(T, -0.0013888378));
    cresult = mulAdd(cresult, x2, splat(T, 0.041666638));
    cresult = mulAdd(cresult, x2, splat(T, -0.5));
    cresult = sign * mulAdd(cresult, x2, splat(T, 1.0));

    return .{ sresult, cresult };
}
// 1952
pub inline fn cross3(v0: Vec, v1: Vec) Vec {
    var xmm0 = swizzle(v0, .y, .z, .x, .w);
    var xmm1 = swizzle(v1, .z, .x, .y, .w);
    var result = xmm0 * xmm1;
    xmm0 = swizzle(xmm0, .y, .z, .x, .w);
    xmm1 = swizzle(xmm1, .z, .x, .y, .w);
    result = result - xmm0 * xmm1;
    return andInt(result, f32x4_mask3);
}
// 3033
pub fn rotate(q: Quat, v: Vec) Vec {
    const w = splat(F32x4, q[3]);
    const axis = f32x4(q[0], q[1], q[2], 0.0);
    const uv = cross3(axis, v);
    return v + ((uv * w) + cross3(axis, uv)) * splat(F32x4, 2.0);
}
// 3145
pub fn quatFromRollPitchYaw(pitch: f32, yaw: f32, roll: f32) Quat {
    return quatFromRollPitchYawV(f32x4(pitch, yaw, roll, 0.0));
}
pub fn quatFromRollPitchYawV(angles: Vec) Quat { // | pitch | yaw | roll | 0 |
    const sc = sincos(splat(Vec, 0.5) * angles);
    const p0 = @shuffle(f32, sc[1], sc[0], [4]i32{ ~@as(i32, 0), 0, 0, 0 });
    const p1 = @shuffle(f32, sc[0], sc[1], [4]i32{ ~@as(i32, 0), 0, 0, 0 });
    const y0 = @shuffle(f32, sc[1], sc[0], [4]i32{ 1, ~@as(i32, 1), 1, 1 });
    const y1 = @shuffle(f32, sc[0], sc[1], [4]i32{ 1, ~@as(i32, 1), 1, 1 });
    const r0 = @shuffle(f32, sc[1], sc[0], [4]i32{ 2, 2, ~@as(i32, 2), 2 });
    const r1 = @shuffle(f32, sc[0], sc[1], [4]i32{ 2, 2, ~@as(i32, 2), 2 });
    const q1 = p1 * f32x4(1.0, -1.0, -1.0, 1.0) * y1;
    const q0 = p0 * y0 * r0;
    return mulAdd(q1, r1, q0);
}
// 4450
const f32x4_mask3: F32x4 = F32x4{
    @as(f32, @bitCast(@as(u32, 0xffff_ffff))),
    @as(f32, @bitCast(@as(u32, 0xffff_ffff))),
    @as(f32, @bitCast(@as(u32, 0xffff_ffff))),
    0,
};

inline fn splatNegativeZero(comptime T: type) T {
    return @splat(@as(f32, @bitCast(@as(u32, 0x8000_0000))));
}
inline fn splatNoFraction(comptime T: type) T {
    return @splat(@as(f32, 8_388_608.0));
}
