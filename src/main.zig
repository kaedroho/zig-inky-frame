const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;
const time = rp2xxx.time;

// Compile-time pin configuration
const pin_config = rp2xxx.pins.GlobalConfiguration{
    .GPIO11 = .{
        .name = "led_a",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO12 = .{
        .name = "led_b",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO13 = .{
        .name = "led_c",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO14 = .{
        .name = "led_d",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO15 = .{
        .name = "led_e",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO6 = .{
        .name = "led_act",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO7 = .{
        .name = "led_conn",
        .function  = .SIO,
        .direction = .out,
    },
    // Inky display specific pins
    .GPIO17 = .{
        .name = "inky_cs",  // Aka. SPI_BG_FRONT_CS
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO28 = .{
        .name = "inky_dc",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO27 = .{
        .name = "inky_reset",
        .function  = .SIO,
        .direction = .out,
    },
    // SPI pins used by Inky display, SD card, and PSRAM
    .GPIO18 = .{
        .name = "spi_clk", // Aka. SPI_DEFAULT_SCK
        .function = .SPI0_SCK,
        .direction = .out,
    },
    .GPIO19 = .{
        .name = "spi_tx", // Aka. SPI_DEFAULT_MOSI
        .function = .SPI0_TX,
        .direction = .out,
    },
    // RX and two additional SPI data pins used for SD card and PSRAM but not Inky display
    .GPIO16 = .{
        .name = "spi_rx",
        .function = .SPI0_RX,
        .direction = .out,
    },
    .GPIO20 = .{
        .name = "spi_sd1",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO21 = .{
        .name = "spi_sd2",
        .function  = .SIO,
        .direction = .out,
    },
    // 74HC165D shift register used to mux inputs
    .GPIO8 = .{
        .name = "sr_clock",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO9 = .{
        .name = "sr_latch",
        .function  = .SIO,
        .direction = .out,
    },
    .GPIO10 = .{
        .name = "sr_data",
        .function  = .SIO,
        .direction = .in,
    },
};

const pins = pin_config.pins();
const spi = rp2xxx.spi.instance.SPI0;


/// Reads the inputs that are latched by the 74HC165D shift register
/// Outputs a U8 byte where each bit represents an input:
/// 0 - Switch A
/// 1 - Switch B
/// 2 - Switch C
/// 3 - Switch D
/// 4 - Switch E
/// 5 - RTC Alarm
/// 6 - External Trigger
/// 7 - Inky Busy
fn read_sr() u8 {
    // https://github.com/pimoroni/pimoroni-pico/blob/18d417b32919bd5f5486d5956eb703447970ef85/drivers/shiftregister/shiftregister.hpp
    pins.sr_latch.put(0);
    asm volatile ("nop");
    pins.sr_latch.put(1);
    asm volatile ("nop");

    var out: u8 = 0;
    for (0..8) |_| {
        out <<= 1;
        out |= pins.sr_data.read();
        pins.sr_clock.put(1);
        asm volatile ("nop");
        pins.sr_clock.put(0);
        asm volatile ("nop");
    }

    return out;
}

fn inky_busy() bool {
    return read_sr() & 128 == 0;
}

fn inky_busy_wait() void {
    pins.led_act.put(1);
    while (inky_busy()) {
        asm volatile ("nop");
    }
    pins.led_act.put(0);
}

fn inky_reset() void {
    inky_busy_wait();
    pins.inky_reset.put(0);
    time.sleep_ms(10);
    pins.inky_reset.put(1);
    time.sleep_ms(10);
    inky_busy_wait();
}

fn inky_init() !void {
    try spi.apply(.{ .clock_config = rp2xxx.clock_config });
    pins.inky_cs.put(1);
    pins.inky_reset.put(1);
}

const InkyCommand = enum(u8) {
    PSR = 0x00,
    PWR = 0x01,
    POF = 0x02,
    PFS = 0x03,
    PON = 0x04,
    BTST1 = 0x05,
    BTST2 = 0x06,
    DSLP = 0x07,
    BTST3 = 0x08,
    DTM1 = 0x10,
    DSP = 0x11,
    DRF = 0x12,
    IPC = 0x13,
    PLL = 0x30,
    TSC = 0x40,
    TSE = 0x41,
    TSW = 0x42,
    TSR = 0x43,
    CDI = 0x50,
    LPD = 0x51,
    TCON = 0x60,
    TRES = 0x61,
    DAM = 0x65,
    REV = 0x70,
    FLG = 0x71,
    AMV = 0x80,
    VV = 0x81,
    VDCS = 0x82,
    T_VDCS = 0x84,
    AGID = 0x86,
    CMDH = 0xAA,
    CCSET =0xE0,
    PWS = 0xE3,
    TSSET = 0xE6 // E5 or E6
};

fn inky_command(command: InkyCommand, data: []const u8) void {
    pins.inky_cs.put(0);

    // Enter command mode
    pins.inky_dc.put(0);

    // Send command
    spi.write_blocking(u8, &[_]u8{@intFromEnum(command)});

    if (data.len > 0) {
        // Enter data mode
        pins.inky_dc.put(1);

        // Send data
        spi.write_blocking(u8, data);
    }

    pins.inky_cs.put(1);
}

fn inky_setup() void {
    inky_reset();
    inky_command(.CMDH, &[_]u8{0x49, 0x55, 0x20, 0x08, 0x09, 0x18});

    inky_command(.PWR, &[_]u8{0x3F, 0x00, 0x32, 0x2A, 0x0E, 0x2A});
    //if (rotation == ROTATE_0) {
      inky_command(.PSR, &[_]u8{0x53, 0x69});
    //} else {
    //  inky_command(.PSR, &[_]u8{0x5F, 0x69});
    //}
    //inky_command(.PSR, &[_]u8{0x5F, 0x69});
    inky_command(.PFS, &[_]u8{0x00, 0x54, 0x00, 0x44});
    inky_command(.BTST1, &[_]u8{0x40, 0x1F, 0x1F, 0x2C});
    inky_command(.BTST2, &[_]u8{0x6F, 0x1F, 0x16, 0x25});
    inky_command(.BTST3, &[_]u8{0x6F, 0x1F, 0x1F, 0x22});
    inky_command(.IPC, &[_]u8{0x00, 0x04});
    inky_command(.PLL, &[_]u8{0x02});
    inky_command(.TSE, &[_]u8{0x00});
    inky_command(.CDI, &[_]u8{0x3F});
    inky_command(.TCON, &[_]u8{0x02, 0x00});
    inky_command(.TRES, &[_]u8{0x03, 0x20, 0x01, 0xE0});
    inky_command(.VDCS, &[_]u8{0x1E});
    inky_command(.T_VDCS, &[_]u8{0x00});
    inky_command(.AGID, &[_]u8{0x00});
    inky_command(.PWS, &[_]u8{0x2F});
    inky_command(.CCSET, &[_]u8{0x00});
    inky_command(.TSSET, &[_]u8{0x00});
}

fn inky_refresh() void {
    // Make sure inky isn't busy
    inky_busy_wait();

    // Power on
    inky_command(.PON, &[_]u8{0x00});
    inky_busy_wait();

    // Trigger refresh
    inky_command(.DRF, &[_]u8{0x00});
    inky_busy_wait();

    // Power off
    inky_command(.POF, &[_]u8{});
    inky_busy_wait();
}

fn inky_write_line(data: []const u8) void {
    pins.inky_cs.put(0);

    // Enter data mode
    pins.inky_dc.put(1);

    // Send data
    spi.write_blocking(u8, data);

    pins.inky_cs.put(1);
}

const INKY_WIDTH = 800;
const INKY_HEIGHT = 480;
const BORIS: *const [INKY_HEIGHT][INKY_WIDTH/8]u8 = @ptrCast(@embedFile("boris.bin"));

pub fn draw_blank(buffer: *[INKY_HEIGHT][INKY_WIDTH/2]u8) void {
    @memset(std.mem.asBytes(buffer), 0x11);
}

pub fn draw_stripes(buffer: *[INKY_HEIGHT][INKY_WIDTH/2]u8) void {
    for (0..INKY_HEIGHT) |y| {
        for (0..INKY_WIDTH/2) |x| {
            const colour: u8 = @intCast((y/10) % 16);
            buffer[y][x] = @intCast((colour << 4) + 0);
        }
    }
}

pub fn draw_boris(buffer: *[INKY_HEIGHT][INKY_WIDTH/2]u8) void {
    for (0..INKY_HEIGHT) |y| {
        for (0..INKY_WIDTH/2) |x| {
            const x_byte = (x*2) / 8;
            const x_bit: u3 = @intCast(7 - (x*2) % 8);

            const colour_left: u8 = if ((BORIS[y][x_byte] >> (x_bit-1)) & 0x01 != 0) 1 else 0;
            const colour_right: u8 = if ((BORIS[y][x_byte] >> (x_bit)) & 0x01 != 0) 1 else 0;

            buffer[INKY_HEIGHT - y][x] = @intCast((colour_left << 4) + colour_right);
        }
    }

}

pub fn inky_present(buffer: *[INKY_HEIGHT][INKY_WIDTH/2]u8) void {
    inky_command(.DTM1, &[_]u8{});

    for (0..INKY_HEIGHT) |line| {
        inky_write_line(&buffer[line]);
    }

    inky_refresh();
}

pub fn main() !void {
    pin_config.apply();
    try inky_init();

    inky_setup();

    var buffer: [INKY_HEIGHT][INKY_WIDTH/2]u8 = std.mem.zeroes([INKY_HEIGHT][INKY_WIDTH/2]u8);

    while (true) {
        pins.led_conn.toggle();
        const inputs = read_sr();

        if (inputs & 0x01 != 0) {
            draw_blank(&buffer);
            inky_present(&buffer);
        }

        if (inputs & 0x02 != 0) {
            draw_stripes(&buffer);
            inky_present(&buffer);
        }

        if (inputs & 0x04 != 0) {
            draw_boris(&buffer);
            inky_present(&buffer);
        }

        time.sleep_ms(100);
    }
}
