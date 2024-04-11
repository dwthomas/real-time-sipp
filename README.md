# Real Time SIPP

## Install

Dependencies (version tested):
- gcc (13.2.1)
- boost (1.83)
- zlib (1.3.1)

Compiling:
```bash
    meson setup --buildtype release  build
    meson compile -C build
    meson setup --buildtype debug build_debug
    meson compile -C build_debug
```

