# gd-real-sim Visualizer

Desktop visualizer built with TypeScript + Tauri using `gd-real-sim` as the Rust simulation engine.

## Features

- Home screen with:
  - local level discovery from `CCLocalLevels.dat`
  - official level search (`getGJLevels21.php`) + download (`downloadGJLevel22.php`)
  - stored click-tape bitstring library in `visualizer/bitstrings`
- Level view with:
  - level rendering
  - simulation trace playback
  - no-bitstring mode or attached bitstring mode
  - timeline scrub/play controls with zoom + pan canvas

## Run

```bash
npm install
npm run tauri dev
```

## Build

```bash
npm run tauri build -- --debug
```

## Backend Commands

Rust Tauri commands are registered in `src-tauri/src/lib.rs`:

- `parse_level`
- `simulate`
- `list_local_levels`
- `search_official_levels`
- `download_official_level`
- `list_bitstrings`
- `upsert_bitstring`
- `delete_bitstring`
