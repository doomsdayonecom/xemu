/* RRDC backend for xemu's MEGA65 target.
 *
 * The Retro Remote Debug Controller contract (rrdc/README, SPEC.md) is the
 * HTTP surface a harness drives: screenshot the screen, read and write memory,
 * step the machine deterministically, inject input. The same pytest harness
 * already drives the forked x16emu, FAB Agon, Neo6502, ZEsarUX and Mednafen
 * ports; this makes the MEGA65 the next one, so an R* floor on this machine is
 * verified the same way as the other twelve rather than by eye.
 *
 * DETERMINISM IS THE WHOLE POINT, and it is the constraint that shapes this
 * file. Every state read must observe the machine at a frame boundary on the
 * EMULATOR thread, never a torn mid-instruction read from the HTTP thread. So
 * nothing here touches emulator state directly: the core calls these callbacks
 * from retro_control_service(), which mega65.c invokes from inside
 * emulation_loop once per frame, after vic4_close_frame_access().
 *
 * WHAT THIS TARGET GIVES US THAT OTHERS DID NOT
 *
 *   - debug_read_linear_byte() takes a 28-BIT linear address, so /mem can reach
 *     the whole address space including the framebuffer at $40000 without any
 *     banking ceremony on the harness side. On the 8-bit floors /mem needed a
 *     bank parameter; here it does not.
 *   - the VIC-IV renders into a flat 800x625 Uint32 texture, so the screenshot
 *     is a straight repack rather than a mode-dependent decode.
 */

#include "xemu/emutools.h"
#include "mega65.h"
#include "vic4.h"
#include "memory_mapper.h"
#include "input_devices.h"
#include "xemu/cpu65.h"
#include "xemu/emutools_hid.h"		/* KBD_PRESS_KEY / matrix, for /key code= */

#include "rrdc_core.h"

#include <string.h>
#include <stdio.h>

/* vic_frame_counter is vic4.c's; the pixel buffer is xemu's own global.
 *
 * NOT vic4.c's `pixel_start` -- that is static to the file, and reaching it
 * would mean either de-static'ing a symbol the emulator owns or adding an
 * accessor to a file this fork should touch as little as possible. A fork that
 * edits the upstream widely is a fork that is painful to rebase, and this one
 * has to track upstream indefinitely. */
extern unsigned int vic_frame_counter;

/* --- screenshot ----------------------------------------------------------
 * The core wants tightly packed RGB24. xemu's texture is Uint32 ARGB/XRGB in
 * host order, so this repacks rather than casting -- a cast would hand the
 * harness host-endian noise on a big-endian build and, more to the point, four
 * bytes per pixel where the contract says three.
 *
 * Static rather than malloc'd: this is called from the emulator thread once per
 * screenshot request, and a 1.5 MB allocation per request on that thread is a
 * frame-timing hazard for no gain. */
static uint8_t fb_rgb[TEXTURE_WIDTH * TEXTURE_HEIGHT * 3];
static int fb_valid;

/* CAPTURED, not read on demand. xemu's pixel pointer is only valid while the
 * frame is open: vic4_close_frame_access() calls xemu_update_screen(), which on
 * the locked-texture path (which is what a headless run uses) unlocks the
 * texture and NULLs the pointer. Reading it from the screenshot callback --
 * which runs after that -- got a null pointer and produced a valid, empty,
 * 0x0 PPM. An empty picture that parses is the worst possible failure here,
 * because a harness sees a well-formed response and asserts against nothing.
 *
 * So the repack happens in m65_rrdc_capture_frame(), called from the emulator
 * loop while the pointer is live, and the callback just hands back the result.
 * The cost is one 800x625 repack per frame while the control server is running,
 * which is test-only and opt-in. */
static void m65_get_framebuffer ( retro_framebuffer_t *out )
{
	if (!fb_valid) {
		out->pixels = NULL;
		out->width = out->height = 0;
		return;
	}
	out->pixels = fb_rgb;
	out->width  = TEXTURE_WIDTH;
	out->height = TEXTURE_HEIGHT;
}

void m65_rrdc_capture_frame ( void )
{
	const Uint32 *src = xemu_frame_pixel_access_p;
	uint8_t *dst = fb_rgb;
	if (!src)
		return;			/* keep the last good frame */
	for (int i = 0; i < TEXTURE_WIDTH * TEXTURE_HEIGHT; i++) {
		const Uint32 p = src[i];
		*dst++ = (uint8_t)((p >> 16) & 0xFF);	/* R */
		*dst++ = (uint8_t)((p >>  8) & 0xFF);	/* G */
		*dst++ = (uint8_t)( p        & 0xFF);	/* B */
	}
	fb_valid = 1;
}

/* --- memory --------------------------------------------------------------
 * 28-bit linear, which is the whole machine: chip RAM, the $40000 framebuffer,
 * attic RAM, I/O through the usual mapping. The harness passes an address and
 * gets those bytes; there is no bank parameter to get wrong.
 *
 * debug_read_linear_byte is the DEBUG accessor deliberately -- it does not
 * disturb the machine (no side-effect-on-read I/O, no cycle cost), which is
 * exactly what an observer must not do. */
static uint32_t m65_read_mem ( uint32_t addr, int32_t bank, uint32_t len,
			       uint8_t *out, uint32_t out_cap )
{
	(void)bank;			/* linear addressing: no banks here */
	if (len > out_cap)
		len = out_cap;
	for (uint32_t i = 0; i < len; i++)
		out[i] = debug_read_linear_byte((addr + i) & 0xFFFFFFFU);
	return len;
}

static uint32_t m65_write_mem ( uint32_t addr, int32_t bank, uint32_t len,
				const uint8_t *in )
{
	(void)bank;
	for (uint32_t i = 0; i < len; i++)
		debug_write_linear_byte((addr + i) & 0xFFFFFFFU, in[i]);
	return len;
}

/* --- registers ------------------------------------------------------------
 * Enough for a harness to assert on, in the shape the other ports emit. The
 * 45GS02's extended registers (B, Z, and the 32-bit indirection) are named
 * because a consumer chasing the far-pointer truncation trap will want them. */
static void m65_get_regs_json ( char *buf, size_t cap )
{
	snprintf(buf, cap,
		"{\"pc\":%u,\"a\":%u,\"x\":%u,\"y\":%u,\"z\":%u,"
		"\"b\":%u,\"sp\":%u,\"p\":%u,\"frame\":%u}",
		(unsigned)CPU65.pc, (unsigned)CPU65.a, (unsigned)CPU65.x,
		(unsigned)CPU65.y, (unsigned)CPU65.z, (unsigned)CPU65.bphi >> 8,
		(unsigned)(CPU65.sphi | CPU65.s), (unsigned)cpu65_get_pf(),
		vic_frame_counter);
}

static uint64_t m65_get_frame_count ( void )
{
	return (uint64_t)vic_frame_counter;
}

/* --- input ---------------------------------------------------------------
 * TWO keyboards, and they are not the same keyboard.
 *
 * text= feeds hwa_kbd_set_fake_key, the MEGA65's hardware-accelerated keyboard
 * queue ($D610). That is what MEGA65-native code reads -- HYPPO, the
 * on-boarding utility, a consumer's rim_key().
 *
 * code= drives the C64-style keyboard MATRIX, which is what a ROM's own scan
 * reads. Open ROMs' KERNAL, for one, reads only this; typing at its BASIC
 * prompt through the text path does nothing at all.
 *
 * Neither is a superset of the other, so both are here. */
static int m65_inject_key ( int is_text, uint32_t value, int action )
{
	if (is_text) {
		/* TAP and DOWN both queue the character: this queue is what the
		 * ROM reads, and it has no notion of a key being HELD, so a
		 * release is not something it can express. Answering 0 for UP is
		 * the contract's way of saying "not supported here" -- the
		 * harness reports that rather than believing a release happened.
		 *
		 * IT IS ALSO NOT THE WHOLE KEYBOARD. hwa_kbd_set_fake_key feeds
		 * the ROM's *keyboard queue*, so only code that asks the ROM for
		 * a character sees it. Anything scanning the matrix directly --
		 * HYPPO, the on-boarding utility, a game reading $DC01 -- sees
		 * nothing at all, and the request still answers "injected".
		 * That is why code= below exists. */
		if (action != RETRO_KEY_TAP && action != RETRO_KEY_DOWN)
			return 0;
		hwa_kbd_set_fake_key((Uint8)(value & 0xFF));
		return 1;
	}

	/* code= drives the KEYBOARD MATRIX instead: (row << 4) | bit, the
	 * encoding KBD_PRESS_KEY uses. This is what the machine's own hardware
	 * scan sees, so it reaches the code the text queue cannot -- and it is
	 * the only way past the on-boarding utility on a fresh SD card, which
	 * is the state every CI run starts in.
	 *
	 * A TAP presses and lets xemu's HID layer release on the next frame,
	 * the same press-and-autorelease pair mega65.c uses for -go64. Doing
	 * the release ourselves here would land in the SAME frame the press
	 * did, and a matrix scan that runs once a frame would never see it. */
	const int key = (int)(value & 0xFFU);
	if ((key & 0x0FU) > 7U)
		return 0;			/* bit is 0..7; refuse rather than alias */
	switch (action) {
		case RETRO_KEY_TAP:
			hid_set_autoreleased_key(key);
			KBD_PRESS_KEY(key);
			return 1;
		case RETRO_KEY_DOWN:
			KBD_PRESS_KEY(key);
			return 1;
		case RETRO_KEY_UP:
			KBD_RELEASE_KEY(key);
			return 1;
		default:
			return 0;
	}
}

/* --- /pad -> the CIA joystick lines (contract 0.5) -------------------------
 *
 * THE PORT MAPPING IS DELIBERATE AND WORTH READING TWICE. Canonical pad 0 is
 * the PRIMARY pad, and on this machine the primary game port is CONTROL PORT
 * 2 -- a C64 inheritance: port 1 shares its lines with the keyboard matrix,
 * so games read port 2 ($DC00/PRA). Pad index 1 is control port 1 ($DC01/PRB).
 * Mapping index 0 to port 1 "because zero comes first" would make every
 * injected test invisible to every real game, which is the PC Engine
 * data_ptr-NULL lesson wearing C64 clothes.
 *
 * THE IMAGE IS ACTIVE-LOW, like everything on these lines: bit CLEAR means
 * held. The canonical mask (bit0 LEFT, 1 RIGHT, 2 UP, 3 DOWN, 4 A) converts
 * here, in the backend, so a test never learns which machine it is on. Only
 * the five lines a real stick has exist -- B/X/Y/START and friends have no
 * wire and are deliberately dropped, not folded onto fire.
 *
 * The injected pad AND-merges with the host joystick at the CIA read
 * (input_devices.c), the same electrical model as two switches on one line --
 * and the same seam Hatari's Joy_GetStickData override uses, which is the
 * pattern that made the Atari ST the first floor whose pad injection actually
 * carried. */

static Uint8 rrdc_pad_cia[2]     = { 0xFF, 0xFF };   /* active-low CIA image */
static int   rrdc_pad_present[2] = { 0, 0 };
static int   rrdc_pad_mask[2]    = { 0, 0 };         /* canonical, for get_pad */

/* input_devices.c asks at every CIA port read. port: 1 or 2 (hardware
 * numbering, matching joystick_emu). 0xFF = nothing injected: the AND-merge
 * makes that a no-op, so local play is untouched while no test is driving. */
Uint8 m65_rrdc_pad_cia ( int port )
{
	const int idx = (port == 2) ? 0 : 1;   /* primary pad = port 2, see above */
	return rrdc_pad_present[idx] ? rrdc_pad_cia[idx] : 0xFF;
}

static int m65_set_pad ( int index, int buttons, int connected )
{
	if (index < 0 || index > 1)
		return 0;
	if (connected >= 0)
		rrdc_pad_present[index] = !!connected;
	if (buttons >= 0) {
		Uint8 cia = 0xFF;
		rrdc_pad_mask[index] = buttons;
		/* canonical -> CIA line, both directions spelled out so the
		 * polarity is checkable against the header comment above */
		if (buttons & 0x04) cia &= (Uint8)~0x01;   /* UP    -> bit 0 */
		if (buttons & 0x08) cia &= (Uint8)~0x02;   /* DOWN  -> bit 1 */
		if (buttons & 0x01) cia &= (Uint8)~0x04;   /* LEFT  -> bit 2 */
		if (buttons & 0x02) cia &= (Uint8)~0x08;   /* RIGHT -> bit 3 */
		if (buttons & 0x10) cia &= (Uint8)~0x10;   /* A     -> bit 4 (fire) */
		rrdc_pad_cia[index] = cia;
	}
	return 1;
}

static int m65_get_pad ( int index, int *buttons, int *connected )
{
	if (index < 0 || index > 1)
		return 0;
	if (buttons)
		*buttons = rrdc_pad_mask[index];
	if (connected)
		*connected = rrdc_pad_present[index];
	return 1;
}

static void m65_reset ( void )
{
	reset_mega65(RESET_MEGA65_HARD);
}

/* Start executing at addr. Called from retro_control_service(), which runs on
 * the emulator thread between frames, so the CPU is never mid-instruction.
 *
 * Only the PC is set: no stack frame is pushed, so the target must not expect
 * to RTS anywhere sensible. That suits what this is for -- entering a program
 * whose main() never returns -- and a target that does return will land
 * wherever the stack already pointed, which is the caller's problem to know. */
static void m65_set_pc ( uint32_t addr )
{
	cpu65.pc = (Uint16)(addr & 0xFFFF);
}

static const retro_control_backend_t m65_backend = {
	.platform        = "mega65",
	.emulator        = "xemu-xmega65",
	.read_mem        = m65_read_mem,
	.write_mem       = m65_write_mem,
	.get_regs_json   = m65_get_regs_json,
	.get_framebuffer = m65_get_framebuffer,
	.get_frame_count = m65_get_frame_count,
	.inject_key      = m65_inject_key,
	.set_pad         = m65_set_pad,
	.get_pad         = m65_get_pad,
	.reset           = m65_reset,
	.set_pc          = m65_set_pc,
};

int mega65_rrdc_start ( int port )
{
	return retro_control_start(port, &m65_backend);
}
