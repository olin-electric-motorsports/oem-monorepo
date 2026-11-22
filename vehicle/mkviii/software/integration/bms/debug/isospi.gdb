# isoSPI debug helpers for the MKVIII BMS (STM32G441)
#
# Terminal 1:
#   openocd -f interface/stlink.cfg -f target/stm32g4x.cfg
# Terminal 2 (from the repo root):
#   gdb-multiarch bazel-bin/vehicle/mkviii/software/integration/bms/bms.elf \
#       -x vehicle/mkviii/software/integration/bms/debug/isospi.gdb

set pagination off
set confirm off
target extended-remote localhost:3333
monitor reset halt
load
monitor reset halt
echo \nFirmware loaded and halted. Type 'continue' to run, Ctrl-C to halt.\n
echo Commands (run while halted): pins, spi, bms, rx, cs_low, cs_high\n\n

# Are the isoSPI pins set up the way the PCB needs?
define pins
  set $moda = *(unsigned int*)0x48000000
  set $modb = *(unsigned int*)0x48000400
  set $afrlb = *(unsigned int*)0x48000420
  set $idra = *(unsigned int*)0x48000010
  set $idrb = *(unsigned int*)0x48000410
  printf "PA15 CS    mode=%d (want 1 = output)       pin reads %d\n", ($moda>>30)&3, ($idra>>15)&1
  printf "PB3  SCK   mode=%d (want 2 = alt function) AF=%d (want 5)  pin reads %d\n", ($modb>>6)&3, ($afrlb>>12)&15, ($idrb>>3)&1
  printf "PB4  MISO  mode=%d (want 2)                AF=%d (want 5)  pin reads %d\n", ($modb>>8)&3, ($afrlb>>16)&15, ($idrb>>4)&1
  printf "PB5  MOSI  mode=%d (want 2)                AF=%d (want 5)  pin reads %d\n", ($modb>>10)&3, ($afrlb>>20)&15, ($idrb>>5)&1
end

# Is SPI1 on, in master mode, mode 3, at the right speed?
define spi
  set $apb2 = *(unsigned int*)0x40021060
  set $cr1 = *(unsigned int*)0x40013000
  set $sr = *(unsigned int*)0x40013008
  printf "SPI1 clock enabled=%d (want 1)\n", ($apb2>>12)&1
  printf "SPI1 enabled=%d master=%d CPOL=%d CPHA=%d (want 1 1 1 1)  BR=%d (want 6 = /128, 500 kHz)\n", ($cr1>>6)&1, ($cr1>>2)&1, ($cr1>>1)&1, $cr1&1, ($cr1>>3)&7
  printf "SPI1 status=0x%04x  busy=%d overrun=%d\n", $sr, ($sr>>7)&1, ($sr>>6)&1
end

# What does the BMS think is going on?
define bms
  printf "bms_fault_code = 0x%04x  (bit 10 = CSC lost, bit 7 = bad data / PEC)\n", bms_core.bms_fault_code
  printf "csc_mia = 0x%02x  voltage PEC errors = %d  temperature PEC errors = %d\n", bms_metrics.csc_mia, bms_metrics.voltage_pec_error_count, bms_metrics.temperature_pec_error_count
  echo Cell codes (100 uV units, 37000 = 3.7 V):\n
  print cell_data.cells[0].cells.c_codes
end

# Run to the next cell-voltage read and dump the raw bytes that came back
define rx
  tbreak voltage.c:50
  continue
  printf "Register %d raw bytes from the CSC:\n", cell_reg + 1
  x/8xb raw_data
  echo all ff  = nothing answering\n
  echo all 00  = MISO stuck low\n
  echo mixed   = the CSC is answering (6 data bytes + 2 PEC bytes)\n
end

# Drive CS by hand (CPU must be halted, or the firmware takes it back).
# Measure at J1 pin 5 on the BMS Core with a multimeter.
define cs_low
  set *(unsigned int*)0x48000018 = 0x80000000
  echo PA15 driven LOW: CS should read about 0 V\n
end

define cs_high
  set *(unsigned int*)0x48000018 = 0x8000
  echo PA15 driven HIGH: CS should read about 3.3 V\n
end
