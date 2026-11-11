# nuc140-pong

A two-paddle Pong game for the Nuvoton **Nu-LB-NUC140** learning board, written directly against the
NUC140 on-chip peripherals: a potentiometer moves the paddle, an I2C EEPROM keeps the random seed
between power cycles, a PWM buzzer plays per-event tones, and score/time are shown on both a
multiplexed 7-segment display and a 128x64 SPI LCD.

> Course project, 2024–2025. Course grade: A+

Written for the Microprocessors course, Department of Computer Science and Information
Engineering, Feng Chia University.

Demo video (English subtitles): https://youtu.be/Z9HS7O-trPg

Hardware: NUC140VE3CN (LQFP100) on Nu-LB-NUC140, plus a 24LC64 EEPROM on I2C1.

---

## Gameplay

- One paddle sits at the top of the screen (`p1`, `y = 0`), one at the bottom (`p2`, `y = 60`);
  the ball is an 8x8 sprite that advances by `dirX * movX` / `dirY * movY` every frame.
- The bottom paddle tracks the potentiometer on ADC channel 7; the top paddle's position arrives
  over UART0 (see the UART note below).
- Hitting a side wall flips `dirX` and plays a short tone; passing a paddle scores a point and
  plays a different tone. First to 3 points ends the match (`while (score1 != 3 && score2 != 3)`),
  after which the LCD clears and the buzzer loops a 90-note melody from `sound.h`.
- The 7-segment display shows the Timer0 second counter on digits 0-1 and the two scores on
  digits 2-3 (`digit[]` in `main.c`).

## Design & implementation notes

**I2C EEPROM access: one ISR, two transaction personalities.**
`I2C1_IRQHandler` does no protocol work itself. It reads the bus status register
(`I2C_GET_STATUS(I2C1)`) and forwards it through a function pointer:

```c
typedef void (*I2C_FUNC)(uint32_t u32Status);
static I2C_FUNC s_I2C1HandlerFn = NULL;
```

`EEPROM_Write()` installs `I2C_MasterTx` and `EEPROM_Read()` installs `I2C_MasterRx`, so switching
between a write and a read swaps the callback rather than editing the ISR. Each callback is a flat
state machine keyed on the I2C status codes: `0x08` (START sent) loads SLA+W, `0x18` / `0x28`
(address or data ACKed) push the next byte from `g_au8TxData[]`, and `0x20` (SLA+W NACKed) issues
`I2C_STOP()` followed by `I2C_START()` to restart the transfer. The read path additionally handles
`0x10` (repeated START) to turn the bus around into SLA+R, `0x40` (SLA+R ACKed) and `0x58` (data
received, NACK returned), which latches `g_u8RxData` and raises `g_u8EndFlag` - the standard 24LC64
random-read sequence of a 16-bit word address followed by a repeated START. Unhandled status codes
fall through to a `printf` rather than being silently dropped. Before dispatching, the ISR tests
`I2C_GET_TIMEOUT_FLAG(I2C1)` and, if set, clears it and skips the callback, so a timeout is never
mistaken for a protocol event; note that `Init_I2C()` does not enable the timeout counter itself,
so this branch is defensive rather than active.

**Boot-time EEPROM use, then teardown.**
`EEPROM_Read`/`EEPROM_Write` block on `while (g_u8EndFlag == 0)`, so they are only used before the
game loop: byte 0 of the EEPROM is read, offset by a constant, and used as `srand()` seed in
`boot_rdm()` to build a 4-digit value, which is written back to byte 0 and also selects one of the
four initial ball directions (`rdm_value % 4` into `setDirectionUp/Down/Left/Right`). The stored
value seeds the *next* power-up, so the serve direction differs across boots. Once that is done,
`Close_I2C()` disables the I2C interrupt, the NVIC line, the module and its clock
(`CLK_DisableModuleClock(I2C1_MODULE)`) before the real-time loop starts, and `s_I2C1HandlerFn` is
set back to `NULL`.

**ADC to paddle coordinate.**
`Init_ADC()` opens the ADC in single-end / single-cycle mode on `ADC_CH_7_MASK` (PA7, per
`MCU_init.h`), enables `ADC_ADF_INT` and the NVIC line. `ADC_IRQHandler` does nothing but set the
`volatile uint8_t u8ADF` flag and clear the interrupt flag; the game loop triggers a conversion with
`ADC_START_CONV(ADC)`, waits on the flag, and reads the result with
`ADC_GET_CONVERSION_DATA(ADC, 7)`. The 12-bit reading is mapped to screen space with
`p2.x = 128 - 16 - u32ADCvalue * 112 / 4096`, which folds in both the axis inversion and the 16-pixel
paddle width so the paddle stays fully on the 128-pixel-wide LCD across the whole travel of the
potentiometer.

**Timer1 as a 5 ms scheduler.**
`Init_Timer1()` runs TIMER1 in periodic mode at 200 Hz, and `TMR1_IRQHandler` is where all the
periodic housekeeping lives, keyed off `cnt_5ms`:

- every tick, `index_5ms = cnt_5ms % 4` selects one of the four 7-segment digits - the handler calls
  `CloseSevenSegment()` and then `ShowSevenSegment(index_5ms, digit[index_5ms])`, so each digit is
  refreshed at 50 Hz and only one common line (PC4-PC7) is ever driven at a time;
- every 20 ticks (100 ms), one of three keypad row lines (PA3/PA4/PA5) is pulled low and
  `GPAB_IRQn` is re-enabled;
- every 200 ticks, the 1-second counter advances.

TIMER0 runs separately at 1 Hz and only maintains the match clock (`count_down` modulo 60, split
into `digit[0]`/`digit[1]`) - the display refresh and the game clock are on different timers.

**Keypad: interrupt gated by the scan phase.**
The 3x3 keypad is wired as columns PA0-PA2 (level-low interrupts) against rows PA3-PA5, all in
quasi-bidirectional mode. `GPAB_IRQHandler` calls `NVIC_DisableIRQ(GPAB_IRQn)` as its first
statement, so a held key cannot re-enter the handler; the Timer1 scan tick is what re-arms the
interrupt after advancing to the next row. Inside the handler, the column is identified from
`PA->ISRC & BITn` and the row from whichever of `PA3/PA4/PA5` reads low, which resolves to
`KEY_Flag = 1..9`. The title screen simply spins on `while (KEY_Flag == 0)`.

**Debounce is done in hardware.**
Rather than sampling twice in software, the GPIO debounce unit is enabled on both interrupt-driven
input groups: `GPIO_ENABLE_DEBOUNCE(PA, BIT0|BIT1|BIT2)` for the keypad columns and
`GPIO_ENABLE_DEBOUNCE(PB, BIT15)` for the EINT1 button, both clocked from the LIRC via
`GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_LIRC, ...)`.

**Frame rendering.**
`draw_Bmp_axb(x, y, fgColor, bgColor, bitmap[], a, b)` is a generic 1-bit-per-pixel blitter: it walks
`(b + 7) / 8` byte rows per column, shifts out each bit, and calls `draw_Pixel()`, so paddles (16x4)
and the ball (8x8) use the same routine and any other sprite size would too. Animation is
erase-then-redraw - each frame paints the sprites in background colour at their old coordinates,
updates the positions, and repaints them in foreground colour.

**Sound.**
The buzzer is PWM1 CH0 (PB11). `playwall()`, `playpaddle()` and `playpoint()` each reconfigure the
channel to a different frequency (226 / 459 / 490 Hz) at 90% duty, hold it for a fixed
`CLK_SysTickDelay()`, then return the channel to 100% duty (silent), giving each game event its own
audible signature. `playgameover()` walks the note/duration tables in `sound.h`. Because the hold is
a busy delay, sound effects are synchronous with the game loop and stall the frame while they play.
PWM0 CH0/CH1 (PA12/PA13) are separately configured at 60 Hz / 50% duty and started at boot.

**UART0 paddle link.**
Every frame, the ADC-derived paddle position is split into three digits by `splitDigits()` and
written out as ASCII with `UART_Write(UART0, &ASSCII, 1)`. `UART02_IRQHandler` (RDA interrupt,
115200 baud) drains the RX FIFO with `while (!(UART0->FSR & UART_FSR_RX_EMPTY_Msk))`, rebuilds a
number from the incoming digits, and mirrors it with `p1.x = 128 - p1.x` so that the two boards see a
consistent playfield orientation. This is the wiring for a two-board head-to-head mode; on a single
board the RX side simply never fires. See the limitations below for a known defect in the digit
accumulator.

**Attribution.** The I2C handler structure and the debounce configuration follow the Nuvoton BSP
`I2C_EEPROM` sample and the course examples respectively; `main.c` still carries that sample's header
comment. The game logic, the ADC-to-screen mapping, the per-event tone choices, the two-timer split
and the rendering are project code.

## Peripherals driven

Seven distinct on-chip peripheral types, nine instances, and seven interrupt handlers:

| Block | Pins / config | Used for | Handler |
|---|---|---|---|
| I2C1 | PA11 SCL / PA10 SDA, 400 kHz | 24LC64 EEPROM (device addr 0x50) | `I2C1_IRQHandler` |
| ADC | ADC7 / PA7, single-end, single-cycle | potentiometer paddle input | `ADC_IRQHandler` |
| SPI3 | PD8-PD11 | 128x64 LCD (driver from the board library) | - |
| PWM1 | CH0 / PB11 | buzzer: event tones and end-of-game melody | - |
| PWM0 | CH0 / PA12, CH1 / PA13 | configured 60 Hz, 50% duty at boot | - |
| TIMER1 | periodic, 200 Hz | 7-segment multiplexing + keypad row scan | `TMR1_IRQHandler` |
| TIMER0 | periodic, 1 Hz | match clock, round countdown | `TMR0_IRQHandler` |
| UART0 | PB0 RX / PB1 TX, 115200, RDA int | paddle position link, `printf` output | `UART02_IRQHandler` |
| GPIO | PC4-PC7 + PE0-PE7 | 4-digit 7-segment display | - |
| GPIO | PA0-PA5, quasi mode, debounced | 3x3 matrix keypad | `GPAB_IRQHandler` |
| GPIO / EINT1 | PB15, rising edge, debounced | external button | `EINT1_IRQHandler` |

## Known limitations

Points found in the code that are worth stating plainly:

- `u8ADF` is set by `ADC_IRQHandler` but never cleared back to 0 in the game loop, so only the first
  frame actually waits for a conversion to finish; later frames read whatever is in the data register.
- `EEPROM_Write()` takes a `uint8_t`, so the 4-digit `rdm_value` is truncated to its low byte when
  the seed is stored back.
- `pindex` in `UART02_IRQHandler` is never reset, so the received-digit accumulator does not restart
  cleanly on each 3-digit packet.
- `playgameover()` passes the raw values from `game_over_song_beat[]` to `CLK_SysTickDelay()`, which
  takes microseconds, while the effect helpers pass millisecond values already scaled by 1000 - the
  melody therefore plays far faster than the note durations suggest.
- `used[10]` in `boot_rdm()` is not initialised before use, so the "distinct digits" check reads
  uninitialised stack.

## Building and running

This repository holds the application sources only; it is not a self-contained project. The code is
written against the Nuvoton NUC100/NUC140 BSP and the Nu-LB-NUC140 board library, and expects these
headers to be provided by the surrounding project: `NUC100Series.h`, `SYS_init.h`, `LCD.h`, `i2c.h`,
`Seven_Segment.h` and `Note_Freq.h` (included by `sound.h`).

1. Create a Nu-LB-NUC140 BSP sample project in Keil MDK-ARM (uVision) or NuEclipse and add the
   CMSIS + StdDriver sources for the NUC100 series.
2. Add `main.c` and `Seven_Segment_wang.c` to the project, and put `MCU_init.h` and `sound.h` on the
   include path. `MCU_init.h` declares the clock source (HXT) and every pin assignment listed above;
   `SYS_Init()` from the BSP consumes it.
3. Wire the hardware: 24LC64 EEPROM on I2C1 (PA11/PA10), a potentiometer on PA7, a buzzer on PB11.
   The LCD, 7-segment display and keypad are on-board.
4. Build and flash over Nu-Link, then press any keypad key at the title screen to start.

## Files

| File | Contents |
|---|---|
| `main.c` | everything: peripheral init, all seven ISRs, the I2C EEPROM driver, rendering helpers, game loop |
| `Seven_Segment_wang.c` | `OpenSevenSegment` / `ShowSevenSegment` / `CloseSevenSegment` and the segment pattern table for digits 0-F |
| `MCU_init.h` | clock source and per-peripheral pin/mode declarations consumed by `SYS_Init()` |
| `sound.h` | 93-entry note and duration tables for the end-of-game melody, plus the event tone constants |

## License

MIT - see [LICENSE](LICENSE).
