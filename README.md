# ICFM

A lightweight partially 3d-printed guided rocket with four fin-type control surfaces.

## Stack

Software
- [PlatformIO](https://github.com/platformio/platformio-core) - Cross-platform and cross-architecture tooling for embedded software
- [arduino-esp32](https://github.com/espressif/arduino-esp32) - Arduino core for esp32

Hardware
- esp32-c6-devkitc-1
- 4x corona cs238mg servos
- ...

## Development

Refer to platformio core [documentation](https://docs.platformio.org/en/latest/core/index.html) for usage.

### vim/nvim users

`python pio_lsp.py` to generate necessary files, this will make the lsp work correctly most of the time.
Inspired by the `Pioinit` functionality from the [nvim-platformio](https://github.com/anurag3301/nvim-platformio.lua.git) project.

