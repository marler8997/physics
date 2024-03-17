const builtin = @import("builtin");
const std = @import("std");

const win32 = struct {
    usingnamespace @import("win32").zig;
    usingnamespace @import("win32").foundation;
    usingnamespace @import("win32").system.library_loader;
    usingnamespace @import("win32").ui.input.keyboard_and_mouse;
    usingnamespace @import("win32").ui.windows_and_messaging;
    usingnamespace @import("win32").graphics.gdi;
};
const XY = @import("xy.zig").XY;
const RenderImage = @import("RenderImage.zig");
const Rgb = @import("Rgb.zig");
const physics = @import("physics.zig");

const L = win32.L;
const CW_USEDEFAULT = win32.CW_USEDEFAULT;
const MSG = win32.MSG;
const HWND = win32.HWND;

const window_style_ex = win32.WINDOW_EX_STYLE{};
const window_style = win32.WS_OVERLAPPEDWINDOW;

const Bitmap = struct {
    ptr: []u8,
    size: XY(usize),
};

const global = struct {
    pub var hwnd: HWND = undefined;
    pub var maybe_bmp: ?Bitmap = null;
};

pub fn oom(e: error{OutOfMemory}) noreturn {
    std.log.err("{s}", .{@errorName(e)});
    _ = win32.MessageBoxA(null, "Out of memory", "Error", win32.MB_OK);
    std.os.exit(0xff);
}
pub fn fatal(comptime fmt: []const u8, args: anytype) noreturn {
    std.log.err(fmt, args);
    // TODO: detect if there is a console or not, only show message box
    //       if there is not a console
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    const msg = std.fmt.allocPrintZ(arena.allocator(), fmt, args) catch @panic("Out of memory");
    const result = win32.MessageBoxA(null, msg.ptr, null, win32.MB_OK);
    std.log.info("MessageBox result is {}", .{result});
    std.os.exit(0xff);
}

fn toColorRef(rgb: Rgb) u32 {
    return (@as(u32, rgb.r) << 0) | (@as(u32, rgb.g) << 8) | (@as(u32, rgb.b) << 16);
}

const TIMER_TICK = 1;

pub fn main() !void {
    const CLASS_NAME = L("Window");
    const wc = win32.WNDCLASSEXW{
        .cbSize = @sizeOf(win32.WNDCLASSEXW),
        .style = .{},
        .lpfnWndProc = WindowProc,
        .cbClsExtra = 0,
        .cbWndExtra = 0,
        .hInstance = win32.GetModuleHandleW(null),
        .hIcon = null,
        .hCursor = win32.LoadCursorW(null, win32.IDC_ARROW),
        .hbrBackground = null,
        .lpszMenuName = null,
        .lpszClassName = CLASS_NAME,
        .hIconSm = null,
    };
    const class_id = win32.RegisterClassExW(&wc);
    if (class_id == 0) {
        std.log.err("RegisterClass failed, error={}", .{win32.GetLastError()});
        std.os.exit(0xff);
    }

    global.hwnd = win32.CreateWindowExW(
        window_style_ex,
        CLASS_NAME, // Window class
        L("Physics"),
        window_style,
        CW_USEDEFAULT, CW_USEDEFAULT, // position
        400, 400, // size
        null, // Parent window
        null, // Menu
        win32.GetModuleHandleW(null), // Instance handle
        null // Additional application data
    ) orelse {
        std.log.err("CreateWindow failed with {}", .{win32.GetLastError()});
        std.os.exit(0xff);
    };

    const tick_ms = 16;
    if (0 == win32.SetTimer(global.hwnd, TIMER_TICK, tick_ms, null))
        fatal("SetTimer failed with {}", .{win32.GetLastError()});

    _ = win32.ShowWindow(global.hwnd, win32.SW_SHOW);
    var msg: MSG = undefined;
    while (win32.GetMessageW(&msg, null, 0, 0) != 0) {
        // No need for TranslateMessage since we don't use WM_*CHAR messages
        //_ = win32.TranslateMessage(&msg);
        _ = win32.DispatchMessageW(&msg);
    }
}

fn vkeyToControl(wParam: win32.VIRTUAL_KEY) ?physics.Control {
    return switch (wParam) {
        .LEFT => .turn_left,
        //.UP =>
        .RIGHT => .turn_right,
        // .DOWN =>
        @as(win32.VIRTUAL_KEY, @enumFromInt('W')) => .forward,
        @as(win32.VIRTUAL_KEY, @enumFromInt('S')) => .backward,
        @as(win32.VIRTUAL_KEY, @enumFromInt('A')) => .left,
        @as(win32.VIRTUAL_KEY, @enumFromInt('D')) => .right,
        else => null,
    };
}
fn vkeyToControlEvent(wParam: win32.VIRTUAL_KEY) ?physics.ControlEvent {
    return switch (wParam) {
        @as(win32.VIRTUAL_KEY, @enumFromInt('R')) => .toggle_raytrace,
        else => null,
    };
}

fn WindowProc(
    hwnd: HWND,
    uMsg: u32,
    wParam: win32.WPARAM,
    lParam: win32.LPARAM,
) callconv(std.os.windows.WINAPI) win32.LRESULT {
    switch (uMsg) {
        win32.WM_KEYDOWN => if (wParam == @intFromEnum(win32.VIRTUAL_KEY.ESCAPE)) {
            win32.PostQuitMessage(0);
        } else if (vkeyToControl(@enumFromInt(wParam))) |control| {
            physics.onControl(control, .down);
        } else if (vkeyToControlEvent(@enumFromInt(wParam))) |event| {
            physics.onControlEvent(event);
        },
        win32.WM_KEYUP => if (vkeyToControl(@enumFromInt(wParam))) |control| {
            physics.onControl(control, .up);
        },
        win32.WM_TIMER => {
            if (wParam != TIMER_TICK)
                fatal("WM_TIMER with unknown id {}", .{wParam});
            std.debug.assert(0 != win32.InvalidateRect(hwnd, null, 0));
        },
        win32.WM_PAINT => {
            paint(hwnd);
            return 0;
        },
        win32.WM_SIZE => {
            // since we "stretch" the image accross the full window, we
            // always invalidate the full client area on each window resize
            std.debug.assert(0 != win32.InvalidateRect(hwnd, null, 0));
        },
        win32.WM_DESTROY => {
            win32.PostQuitMessage(0);
            return 0;
        },
        else => {},
    }
    return win32.DefWindowProcW(hwnd, uMsg, wParam, lParam);
}

fn paint(hwnd: HWND) void {
    // TODO: render frames per second
    var ps: win32.PAINTSTRUCT = undefined;
    const hdc = win32.BeginPaint(hwnd, &ps);

    const client_size_i32 = getClientSize(hwnd);
    const client_size: XY(usize) = .{
        .x = @intCast(client_size_i32.x),
        .y = @intCast(client_size_i32.y),
    };

    const hbmp = win32.CreateCompatibleBitmap(hdc, client_size_i32.x, client_size_i32.y)
        orelse fatal("CreateCompatibleBitmap failed with {}", .{win32.GetLastError()});
    defer {
        if (0 == win32.DeleteObject(hbmp))
            fatal("delete bmp failed with {}", .{win32.GetLastError()});
    }
    const memdc = win32.CreateCompatibleDC(hdc);
    //if (memdc == 0)
    //fatal("CreateCompatibleDC failed with {}", .{win32.GetLastError()});
    defer if (0 == win32.DeleteDC(memdc))
        fatal("ReleaseDC failed with {}", .{win32.GetLastError()});
    _ = win32.SelectObject(memdc, hbmp);

    const bmp = blk: {
        if (global.maybe_bmp) |bmp| {
            if (bmp.size.x == client_size.x and bmp.size.y == client_size.y)
                break :blk bmp;
            std.log.info("releasing bitmap {}x{}!", .{bmp.size.x, bmp.size.y});
            std.heap.page_allocator.free(bmp.ptr);
            global.maybe_bmp = null;
        }
        const ptr = std.heap.page_allocator.alloc(
            u8, client_size.x * client_size.y * 4
        ) catch |e| oom(e);
        global.maybe_bmp = Bitmap{
            .ptr = ptr,
            .size = client_size,
        };
        break :blk global.maybe_bmp.?;
    };

    const image = RenderImage{
        .ptr = bmp.ptr,
        .size = bmp.size,
    };
    physics.render(image, client_size);
    {
        const bmp_info = win32.BITMAPINFO{
            .bmiHeader = .{
                .biSize = @sizeOf(win32.BITMAPINFOHEADER),
                .biWidth = client_size_i32.x,
                .biHeight = client_size_i32.y,
                .biPlanes = 1,
                .biBitCount = 32,
                .biCompression = win32.BI_RGB,
                .biSizeImage = 0,
                .biXPelsPerMeter = 0,
                .biYPelsPerMeter = 0,
                .biClrUsed = 0,
                .biClrImportant = 0,
            },
            .bmiColors = undefined,
        };
        const row_count = win32.SetDIBits(
            memdc,
            hbmp,
            0,
            @intCast(client_size.y),
            @ptrCast(bmp.ptr),
            &bmp_info,
            win32.DIB_RGB_COLORS,
        );
        if (row_count != client_size.y)
            fatal("SetDIBits failed, count={}, error={}", .{row_count, win32.GetLastError()});
    }

    if (0 == win32.BitBlt(
        hdc,
        0, 0,
        ps.rcPaint.right, ps.rcPaint.bottom,
        memdc,
        0, 0,
        .SRCCOPY,
    )) fatal("BitBlt failed with {}", .{win32.GetLastError()});

    _ = win32.EndPaint(hwnd, &ps);
}

fn getClientSize(hwnd: HWND) XY(i32) {
    var rect: win32.RECT = undefined;
    if (0 == win32.GetClientRect(hwnd, &rect))
        fatal("GetClientRect failed, error={}", .{win32.GetLastError()});
    return .{
        .x = rect.right - rect.left,
        .y = rect.bottom - rect.top,
    };
}
