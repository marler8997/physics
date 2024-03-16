const RenderImage = @This();
const Rgb = @import("Rgb.zig");
const XY = @import("xy.zig").XY;

ptr: []u8,
size: XY(usize),

pub fn setPixel(self: RenderImage, x: usize, y: usize, rgb: Rgb) void {
    const offset = 4 * ((y * self.size.x) + x);
    self.ptr[offset + 0] = rgb.b;
    self.ptr[offset + 1] = rgb.g;
    self.ptr[offset + 2] = rgb.r;
    self.ptr[offset + 3] = 255;
}
