const std = @import("std");

/// A simple monitor for INA219 \
/// Based on https://github.com/adafruit/Adafruit_INA219 \
/// Datasheet: http://www.ti.com/lit/ds/symlink/ina219.pdf
const INA219 = struct {
    /// I2C Device file handle
    file: std.fs.File,
    /// I2C Address
    addr: u8,

    current_lsb: f32,
    power_lsb: f32,

    const BusVoltageRange = enum(u32) {
        /// Set Bus Voltage range to 16v
        RANGE_16V = 0x00,
        /// Set Bus Voltage range to 32v (default)
        RANGE_32V = 0x01,
    };

    const Gain = enum(u32) {
        /// Shunt prog. gain set to 1, 40 mV range
        DIV_1_40MV = 0x00,
        /// Shunt prog. gain set to /2, 80 mV range
        DIV_2_80MV = 0x01,
        /// shunt prog. gain set to /4, 160 mV range
        DIV_4_160MV = 0x02,
        /// shunt prog. gain set to /8, 320 mV range
        DIV_8_320MV = 0x03,
    };

    const AdcResolution = enum(u32) {
        /// 9 bit, 1 sample, 84us
        ADCRES_9BIT_1S = 0x00,
        /// 10 bit, 1 sample, 148us
        ADCRES_10BIT_1S = 0x01,
        /// 11 bit, 1 sample, 276us
        ADCRES_11BIT_1S = 0x02,
        /// 12 bit, 1 sample, 532us
        ADCRES_12BIT_1S = 0x03,
        /// 12 bit, 2 samples, 1.06ms
        ADCRES_12BIT_2S = 0x09,
        /// 12 bit, 4 samples, 2.13ms
        ADCRES_12BIT_4S = 0x0A,
        /// 12 bit, 8 samples, 4.26ms
        ADCRES_12BIT_8S = 0x0B,
        /// 12 bit, 16 samples, 8.51ms
        ADCRES_12BIT_16S = 0x0C,
        /// 12 bit, 32 samples, 17.02ms
        ADCRES_12BIT_32S = 0x0D,
        /// 12 bit, 64 samples, 34.05ms
        ADCRES_12BIT_64S = 0x0E,
        /// 12 bit, 128 samples, 68.10ms
        ADCRES_12BIT_128S = 0x0F,
    };

    const Mode = enum(u32) {
        /// power down
        POWERDOWN = 0x00,
        /// shunt voltage triggered
        SVOLT_TRIGGERED = 0x01,
        /// bus voltage triggered
        BVOLT_TRIGGERED = 0x02,
        /// shunt and bus voltage triggered
        SANDBVOLT_TRIGGERED = 0x03,
        /// ADC off
        ADCOFF = 0x04,
        /// shunt voltage continuous
        SVOLT_CONTINUOUS = 0x05,
        /// bus voltage continuous
        BVOLT_CONTINUOUS = 0x06,
        /// shunt and bus voltage continuous
        SANDBVOLT_CONTINUOUS = 0x07,
    };

    const Register = enum(u8) {
        /// Config Register (R/W)
        CONFIG = 0x00,
        /// Shunt Voltage Register (R)
        SHUNTVOLTAGE = 0x01,
        /// Bus Voltage Register (R)
        BUSVOLTAGE = 0x02,
        /// Power Register (R)
        POWER = 0x03,
        /// Current Register (R)
        CURRENT = 0x04,
        /// Calibration Register (R/W)
        CALIBRATION = 0x05,
    };

    const I2C_SLAVE = 0x0703;

    pub fn init(i2c_bus: u8, i2c_addr: u8) !INA219 {
        var new: INA219 = undefined;

        new.addr = i2c_addr;

        var buf: [15]u8 = undefined;
        const absolute_path = try std.fmt.bufPrint(&buf, "/dev/i2c-{}", .{i2c_bus});

        new.file = try std.fs.openFileAbsolute(absolute_path, .{ .mode = .read_write });
        errdefer new.file.close();

        if (std.os.linux.ioctl(new.file.handle, I2C_SLAVE, i2c_addr) < 0) return error.DeviceNotAvailable;

        // Calibrate

        const RSHUNT = 0.1; // Shunt resistor value in ohms

        // Determine max expected current
        const MaxExpectedI = 2.0;

        // Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
        const MinimumLSB = MaxExpectedI / 32_767.0; // 0.000_061 (61uA per bit)
        const MaximumLSB = MaxExpectedI / 4_096.0; // 0.000_488 (488uA per bit)

        // Choose an LSB between the min and max values
        // (Preferrably a roundish number close to MinimumLSB)
        const CurrentLSB = (MaximumLSB - MinimumLSB) / 4.0; // = 0.000100 (100uA per bit)
        new.current_lsb = CurrentLSB * 1_000; // Current LSB = 100uA per bit

        // Compute the calibration register
        // Cal = trunc (0.04096 / (Current_LSB * RSHUNT)) = 4096 (0x1000)
        const Cal: u16 = @trunc(0.04096 / (CurrentLSB * RSHUNT));

        try new.write(Register.CALIBRATION, Cal);

        // Calculate the power LSB
        // PowerLSB = 20 * CurrentLSB = 0.002 = 0.002 (2mW per bit)
        new.power_lsb = 20 * CurrentLSB;

        // Set Config register as per above settings
        const config: u16 = @intFromEnum(BusVoltageRange.RANGE_32V) << 13 |
            @intFromEnum(Gain.DIV_8_320MV) << 11 |
            @intFromEnum(AdcResolution.ADCRES_12BIT_32S) << 7 |
            @intFromEnum(AdcResolution.ADCRES_12BIT_32S) << 3 |
            @intFromEnum(Mode.SANDBVOLT_CONTINUOUS);

        try new.write(Register.CONFIG, config);

        return new;
    }

    fn deinit(self: *INA219) void {
        self.file.close();
    }

    /// Read 2 bytes ([2]u8) from the specified register and return it as a word (u16)
    pub fn read(self: *INA219, register: Register) !u16 {
        const written = try self.file.write(&.{@intFromEnum(register)});
        std.debug.assert(written == 1);

        var buf: [2]u8 = undefined;
        const readen = try self.file.read(&buf);
        std.debug.assert(readen == buf.len);

        return (@as(u16, buf[0]) << 8) | @as(u16, buf[1]);
    }

    /// Write a word (u16) to the specified register as 2 bytes ([2]u8)
    fn write(self: *INA219, register: Register, data: u16) !void {
        var buf: [3]u8 = .{
            @intFromEnum(register),
            @truncate(data >> 8),
            @truncate(data),
        };

        const written = try self.file.write(&buf);

        std.debug.assert(written == buf.len);
    }

    /// Returns shunt voltage in mV
    fn getShuntVoltage(self: *INA219) !f64 {
        var value: f64 = @floatFromInt(try self.read(Register.SHUNTVOLTAGE));
        if (value > 32767) value -= 65535; // recover signage

        return value * 0.01;
    }

    /// Returns bus voltage in V
    fn getBusVoltage(self: *INA219) !f64 {
        var value: f64 = @floatFromInt(try self.read(Register.BUSVOLTAGE) >> 3);
        if (value > 32767) value -= 65535; // recover signage

        return value * 0.004;
    }

    /// Returns current value in mA
    fn getCurrent(self: *INA219) !f64 {
        var value: f64 = @floatFromInt(try self.read(Register.CURRENT));
        if (value > 32767) value -= 65535; // recover signage

        return value * self.current_lsb;
    }

    /// Returns power value in W
    fn getPower(self: *INA219) !f64 {
        var value: f64 = @floatFromInt(try self.read(Register.POWER));
        if (value > 32767) value -= 65535; // recover signage

        return value * self.power_lsb;
    }
};

const vt100 = struct {
    pub const clear = "\x1b[2K\r";
    pub const up = "\x1b[A";
};

pub fn main() !void {
    const stdout = std.io.getStdOut().writer();

    var ina219 = try INA219.init(0x01, 0x42); // i2c bus 1, address 0x42
    defer ina219.deinit();

    while (true) {
        const bus_voltage = try ina219.getBusVoltage();
        try stdout.print(vt100.clear ++ "Bus Voltage: {d: >7.2} V\n", .{bus_voltage});

        const current = try ina219.getCurrent();
        try stdout.print(vt100.clear ++ "Current: {d: >11.2} mA\n", .{current});

        const power = try ina219.getPower();
        try stdout.print(vt100.clear ++ "Power: {d: >13.2} W\n", .{power});

        const battery = b: {
            var b = (bus_voltage - 6) / 2.15 * 100;
            if (b > 100) b = 100;
            if (b < 0) b = 0;
            break :b b;
        };
        try stdout.print(vt100.clear ++ "Battery: {d: >11.2} %\n", .{battery});

        std.time.sleep(200 * std.time.ns_per_ms); // sleep for 200ms

        try stdout.print(vt100.up ** 4, .{}); // go up 4 lines
    }
}
