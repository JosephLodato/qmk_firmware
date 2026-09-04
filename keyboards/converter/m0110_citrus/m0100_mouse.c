/*
Copyright 2026 Joseph Lodato

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * Apple Macintosh M0100 mouse -- QMK custom pointing device.
 *
 * The M0100 has no protocol. It is a bare opto-mechanical quadrature encoder
 * (two phase pairs) plus one microswitch, wired straight to GPIO.
 *
 * Quadrature is decoded by pin-change interrupt on PORTB (PCINT0_vect), not
 * by INT0-INT3: those external interrupts live only on PD0-PD3, which are
 * taken by the keyboard's CLOCK (PD1) and DATA (PD0) lines. The 32u4's only
 * other external interrupt is INT6 on PE6, so four dedicated external
 * interrupts is impossible. PCINT0 is one vector shared by all of PORTB, so
 * the ISR reads PINB once and decodes both axes from that same sample.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "pointing_device.h"

#if defined(M0100_CALIBRATE) && defined(CONSOLE_ENABLE)
#    include "print.h"
#endif

/*
 * Pin assignments. These are physical wiring facts -- the board is already
 * built. Never reassign a pin to resolve a build error or a bug.
 *
 *   Mouse X1   SCK   PB1
 *   Mouse X2   MO    PB2
 *   Mouse BTN  MI    PB3
 *   Mouse Y1   D9    PB5
 *   Mouse Y2   D10   PB6
 */
#define M0100_X1_BIT  PB1
#define M0100_X2_BIT  PB2
#define M0100_BTN_BIT PB3
#define M0100_Y1_BIT  PB5
#define M0100_Y2_BIT  PB6

/* Set in config.h. Fallback so this file still builds standalone. */
#ifndef M0100_COUNT_DIVISOR
#    define M0100_COUNT_DIVISOR 1
#endif

/*
 * Fault bound on the fed-back leftover, NOT a scaling parameter -- that is
 * M0100_COUNT_DIVISOR. Legitimate motion never reaches it; see get_report().
 */
#define M0100_ACCUM_LIMIT 1000

/*
 * Consecutive agreeing samples required before the button's reported state
 * changes. Deliberately a sample counter, not a time-based delay: the poll
 * interval is not constant, because m0110.c error paths contain
 * _delay_ms(500), so a keyboard fault stretches the period by two orders of
 * magnitude. A counter degrades gracefully under that (slightly more
 * latency); a timer_elapsed() comparison would silently stop debouncing.
 */
#define M0100_BTN_DEBOUNCE 2

#define M0100_QUAD_MASK (_BV(M0100_X1_BIT) | _BV(M0100_X2_BIT) | _BV(M0100_Y1_BIT) | _BV(M0100_Y2_BIT))
#define M0100_ALL_MASK  (M0100_QUAD_MASK | _BV(M0100_BTN_BIT))

/*
 * Standard Gray-code quadrature decoder, indexed by (prev << 2) | cur, where
 * each 2-bit state is (phase1 << 1) | phase2.
 *
 * Zeros sit on the four diagonal entries -- no change: indices 0, 5, 10, 15 --
 * and on the four illegal double-transitions where both bits flip at once:
 * 00->11, 01->10, 10->01, 11->00, i.e. indices 3, 6, 9, 12. So noise and
 * missed samples contribute nothing rather than a wrong-direction count.
 *
 * Kept in RAM rather than PROGMEM on purpose: pgm_read_byte would add
 * instructions to the hot path, and 16 bytes of SRAM is not scarce here.
 */
static const int8_t m0100_qdec[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

/* Packed quadrature state: bits 1:0 = X (X1<<1|X2), bits 3:2 = Y (Y1<<1|Y2). */
static volatile uint8_t m0100_state = 0;

/* Decoded edge counts, 4x per quadrature cycle. Drained by get_report(). */
static volatile int16_t m0100_accum_x = 0;
static volatile int16_t m0100_accum_y = 0;

/*
 * Must stay very short. m0110.c bit-bangs the keyboard link with _delay_us(1)
 * busy-waits and ~200us sampling windows for ~165us bit cells; a long ISR
 * firing mid-bit can corrupt a byte.
 */
ISR(PCINT0_vect) {
    uint8_t pinb = PINB;

    uint8_t xs = ((pinb & _BV(M0100_X1_BIT)) ? 0x02 : 0) | ((pinb & _BV(M0100_X2_BIT)) ? 0x01 : 0);
    uint8_t ys = ((pinb & _BV(M0100_Y1_BIT)) ? 0x02 : 0) | ((pinb & _BV(M0100_Y2_BIT)) ? 0x01 : 0);

    uint8_t prev = m0100_state;

    /* (prev & 0x0c) is already the previous Y state shifted left by 2. */
    m0100_accum_x += m0100_qdec[((prev & 0x03) << 2) | xs];
    m0100_accum_y += m0100_qdec[(prev & 0x0c) | ys];

    m0100_state = (uint8_t)((ys << 2) | xs);
}

bool pointing_device_driver_init(void) {
    /* All five lines: input with internal pull-up. */
    DDRB &= (uint8_t)~M0100_ALL_MASK;
    PORTB |= M0100_ALL_MASK;

    /* Seed the previous state so the first ISR does not decode a bogus
       transition out of state 0. */
    uint8_t pinb = PINB;
    uint8_t xs   = ((pinb & _BV(M0100_X1_BIT)) ? 0x02 : 0) | ((pinb & _BV(M0100_X2_BIT)) ? 0x01 : 0);
    uint8_t ys   = ((pinb & _BV(M0100_Y1_BIT)) ? 0x02 : 0) | ((pinb & _BV(M0100_Y2_BIT)) ? 0x01 : 0);
    m0100_state  = (uint8_t)((ys << 2) | xs);

    PCIFR = _BV(PCIF0); /* write-1-to-clear; NOT &= ~_BV(...) */

    /* PCINT3 (the button) is deliberately left masked -- the button is polled
       in get_report(), so waking this ISR on button edges would only burn
       cycles inside the keyboard's bit-banged timing windows. */
    PCMSK0 |= _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT5) | _BV(PCINT6);
    PCICR |= _BV(PCIE0);

    return true;
}

report_mouse_t pointing_device_driver_get_report(report_mouse_t mouse_report) {
    xy_clamp_range_t dx, dy;

    /* A 16-bit read-then-clear is not atomic on AVR. This is a handful of
       cycles around an accumulator snapshot -- it is NOT cli() around a
       keyboard transaction, and does not weaken the interrupt-enabled
       busy-waits in m0110.c. */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        dx           = m0100_accum_x;
        m0100_accum_x = 0;
        dy           = m0100_accum_y;
        m0100_accum_y = 0;
    }

#if defined(M0100_CALIBRATE) && defined(CONSOLE_ENABLE)
    /* Bench calibration only. Gated on its own macro, NOT on CONSOLE_ENABLE
       alone: this board already ships "console": true in keyboard.json, so a
       bare CONSOLE_ENABLE guard would leave this compiled in and uprintf on
       every report with motion at ~150 Hz -- flash cost plus real runtime cost
       inside the loop that bit-bangs the keyboard.

       To calibrate: rebuild with -e EXTRAFLAGS=-DM0100_CALIBRATE, drag the
       mouse a measured four inches along one axis, sum the reported counts and
       divide by four to get true edges-per-inch. Near 400 means the M0100's
       ~100 CPI figure counts quadrature cycles, so M0100_COUNT_DIVISOR should
       be 4; near 100 means it already counts transitions and 1 is correct. */
    if (dx || dy) {
        uprintf("m0100 raw edges: x=%d y=%d\n", (int)dx, (int)dy);
    }
#endif

    /* Integer division truncates toward zero, so the remainder keeps the
       correct sign and the pointer never overshoots.

       The reference implementation this is based on (GuilleAcoustic/Yoe,
       geekhack thread 74340) reports 4x raw and was reported as under-resolved
       rather than too fast, so a divisor of 1 is the evidence-backed default. */
    xy_clamp_range_t qx = dx / M0100_COUNT_DIVISOR;
    xy_clamp_range_t qy = dy / M0100_COUNT_DIVISOR;
    xy_clamp_range_t ox = CONSTRAIN_HID_XY(qx);
    xy_clamp_range_t oy = CONSTRAIN_HID_XY(qy);

    /* Feed back the unreported leftover: the division remainder plus any
       excess the +/-127 clamp withheld, so a fast flick drains across several
       reports instead of being thrown away. */
    dx -= ox * M0100_COUNT_DIVISOR;
    dy -= oy * M0100_COUNT_DIVISOR;

    /* Bound that feedback. Legitimate motion never comes near the +/-127
       clamp: 4x decode at ~100 CPI is roughly 400 edges/inch, so even a fast
       20 in/s swipe is about 56 counts per 7 ms report. Hitting the clamp
       therefore indicates a fault, and on a 40-year-old encoder (dirty wheel,
       intermittent phototransistor) a noise burst would otherwise dump a large
       count into the accumulator and the pointer would glide at 127 units per
       report with the user's hand off the mouse. Clamping here discards only
       fault-generated counts and bounds ghost-drift to well under a second. */
    if (dx > M0100_ACCUM_LIMIT) {
        dx = M0100_ACCUM_LIMIT;
    } else if (dx < -M0100_ACCUM_LIMIT) {
        dx = -M0100_ACCUM_LIMIT;
    }
    if (dy > M0100_ACCUM_LIMIT) {
        dy = M0100_ACCUM_LIMIT;
    } else if (dy < -M0100_ACCUM_LIMIT) {
        dy = -M0100_ACCUM_LIMIT;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        m0100_accum_x += dx;
        m0100_accum_y += dy;
    }

    mouse_report.x = ox;
    mouse_report.y = oy;

    /* Button is polled here, not interrupt-driven -- PCINT3 stays masked. */
    static bool    btn_candidate = false;
    static uint8_t btn_stable    = 0;
    static bool    btn_state     = false;

    bool btn_raw = !(PINB & _BV(M0100_BTN_BIT)); /* active low, pulled up */

    if (btn_raw == btn_candidate) {
        if (btn_stable < M0100_BTN_DEBOUNCE) {
            btn_stable++;
        }
    } else {
        btn_candidate = btn_raw;
        btn_stable    = 0;
    }
    if (btn_stable >= M0100_BTN_DEBOUNCE) {
        btn_state = btn_candidate;
    }

    mouse_report.buttons = pointing_device_handle_buttons(mouse_report.buttons, btn_state, POINTING_DEVICE_BUTTON1);

    return mouse_report;
}

uint16_t pointing_device_driver_get_cpi(void) {
    return 100;
}

void pointing_device_driver_set_cpi(uint16_t cpi) {
    /* Fixed-resolution hardware. No scaling: see CLAUDE.md constraint 4. */
}
