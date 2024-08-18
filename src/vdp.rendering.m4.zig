//! Rendering routines for the 4 (M4) VDP mode.

const std = @import("std");
const vdp = @import("vdp.zig");
const opcodes = @import("opcodes.zig");
const Z80State = @import("cpu.zig").Z80State;
const VDPRegister0 = vdp.VDPRegister0;
const VDPRegister1 = vdp.VDPRegister1;
const VDPState = vdp.VDPState;
const BackgroundTile = vdp.BackgroundTile;
const expectEqual = std.testing.expectEqual;

const display_columns = 256;
const bitplanes_per_line = 4;
const pattern_columns = 8;
const pattern_lines = 8;
const cycles_per_frame = 228;

pub fn renderFrame(self: *VDPState, cpu: *Z80State) void {
    const scanlines_per_frame: u9 = switch (self.display_type) {
        .ntsc => vdp.ntsc_scanlines,
        .pal => vdp.pal_scanlines,
    };

    const r1: VDPRegister1 = @bitCast(self.registers[1]);

    for (0..scanlines_per_frame) |scanline| {
        self.scanline = @intCast(scanline);

        // Active display
        if (self.scanline < self.display_lines) {
            if (r1.display_on == true) {
                renderBackgroundLine(self);
            } else {
                const backdrop_color = 16 | (self.registers[7] & 15);
                for (0..display_columns) |x| {
                    self.display_buffer[(scanline * display_columns) + x] = backdrop_color;
                }
            }
        }

        // The vertical blanking period has started (inactive time)
        if (self.scanline == self.display_lines + 1) {
            // Set bit 7 of the status flags,
            // this bit will remain set until the control port is read.
            self.status_flags.frame_interrupt_pending = true;
        }

        self.processFrameInterrupt(cpu);
        self.processLineInterrupt(cpu);

        // cycles per frame
        for (0..cycles_per_frame) |_| {
            opcodes.execOne(cpu);
        }
    }
}

pub fn renderBackgroundLine(self: *VDPState) void {
    const scanline: u8 = @intCast(self.scanline);

    const display_buffer_y = @as(usize, scanline) * display_columns;

    // The vertical scroll value cannot be changed during the active display
    var y_scroll: u32 = @as(u32, scanline) + self.registers[9];

    if (self.display_lines == 192) {
        // In the regular 192-line display mode the name table is 32x28,
        // so the vertical scroll register wraps past 223
        y_scroll %= 224;
    } else {
        // In the 224 and 240-line display modes the name table is 32x32,
        // so the vertical scroll register wraps past 255
        y_scroll &= 255;
    }

    var y = y_scroll / 8;

    const r0: VDPRegister0 = @bitCast(self.registers[0]);

    // If bit 6 of VDP register 0 is set,
    // horizontal scrolling will be fixed at zero for scanlines 0-15
    var x_scroll: u8 = if (r0.fix_top_2_rows and scanline < 16) 0 else self.registers[8];
    const x = x_scroll & 7; // lower 3 bits of R8, fine scroll X
    x_scroll >>= 3;
    if (x_scroll == 0) {
        x_scroll = 32;
    }

    if (x > 0) { // Fine scroll > 0
        const backdrop = self.cram[self.registers[7]];
        if (r0.blank_col_0 == false) {
            for (0..x) |ix| {
                self.display_buffer[(@as(usize, scanline) * 256) + ix] = @bitCast(backdrop);
            }
        }
    }

    const name_table_addr = self.getNameTableBaseAddress();
    const name_table_size: usize = if (self.display_lines != 192) 32 * 32 else 32 * 28;
    const tiles: [*]align(1) BackgroundTile = @ptrCast(@constCast(self.vram[name_table_addr .. name_table_addr + name_table_size]));

    // foreach tile 0-31 on the current scanline
    for (0..32) |tile_x| {
        // The vertical scroll value will be fixed to zero when columns 24-31 are rendered
        if (r0.fix_right_8_cols and tile_x == 24) {
            y = scanline;
            y_scroll = scanline;
            y = y / 8;
        }

        // Tile from the Name Table (32 x H)
        const tile = tiles[(y * 32) + ((32 - x_scroll + tile_x) & 31)]; // 32 - x_scroll = starting column

        // The pattern index in VRAM.
        const base_pattern_addr = @as(usize, tile.pattern_idx) * 32;

        // The line's offset in the 8x8 pattern, each line is encoded in 4 bitplanes hence 32byte per pattern.
        const line_offset = ((y_scroll % 8) * 4);

        // The absolute line's address
        const pi = base_pattern_addr + line_offset;

        // Foreach column (pixel) 0-7 in the pattern's line,
        // merge the 4 bitplanes and get the color index.
        inline for (0..pattern_columns) |j| { // 4 bitplanes foreach pattern line
            const bp_0 = (self.vram[pi + 0] >> j) & 1; // bitplane 0
            const bp_1 = (self.vram[pi + 1] >> j) & 1; // bitplane 1
            const bp_2 = (self.vram[pi + 2] >> j) & 1; // bitplane 2
            const bp_3 = (self.vram[pi + 3] >> j) & 1; // bitplane 3

            var h_flip: u8 = 7;

            if (tile.horizontal_flip) {
                h_flip = 2 * j;
            }

            const ij_color = bp_0 | (bp_1 << 1) | (bp_2 << 2) | (bp_3 << 3);

            const palette_color = (16 * @as(u5, tile.palette_select)) + ij_color;

            self.display_buffer[display_buffer_y + (x + (tile_x * 8) + (h_flip - j))] = @bitCast(self.cram[palette_color]);
        }
    }
}
