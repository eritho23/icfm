<div align="center">
    <a href="https://github.com/eritho23/icfm">
        <img src="./images/icfm-blue.svg" alt="icfm" width="100" height="100"></img>
    </a>
    <h3></h3>
    <p align="center">A lightweight partially 3d-printed guided rocket.</p>
</div>

## Stack

Software
- [PlatformIO](https://github.com/platformio/platformio-core) - Cross-platform and cross-architecture tooling for embedded software
- [arduino-esp32](https://github.com/espressif/arduino-esp32) - Arduino core for esp32

Hardware
- esp32-c6-devkitc-1
- 4x corona cs238mg servos
- ...

## Development

Refer to platformio core [documentation](https://docs.platformio.org/en/latest/core/index.html) for building, uploading and more with the pio cli.

### Style

- Prefer simple c over cpp features
- snake_case (though libraries might use pascalCase)
- Use arenas to manage memory, include src/arena.h
- Header #include statements: use "" for self written headers and <> for others

### vim/nvim users

`python pio_lsp.py` to generate necessary files, this will make the lsp work correctly most of the time.
Inspired by the `Pioinit` functionality from the [nvim-platformio](https://github.com/anurag3301/nvim-platformio.lua.git) project.

