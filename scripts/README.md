# gd-real-sim Scripts

Shell helpers for building and running the Rust CLI plus the Tauri visualizer.

Run these from the repository root or from any other directory:

```sh
./scripts/build.sh
```

Builds:

- Rust CLI debug binary at `target/debug/gd-real-sim`
- Rust CLI release binary at `target/release/gd-real-sim`
- Tauri visualizer release app at `visualizer/src-tauri/target/release/gd-real-sim-visualizer`

```sh
./scripts/run-built.sh
```

Runs the built Tauri visualizer app. To run the built Rust CLI instead:

```sh
./scripts/run-built.sh --app rust --help
```

```sh
./scripts/dev.sh
```

Starts the Tauri development app with Vite and the Rust backend:

```sh
npm run tauri -- dev
```

Both `build.sh` and `dev.sh` install npm dependencies when `visualizer/node_modules` is missing. Set `SKIP_INSTALL=1` to skip that check:

```sh
SKIP_INSTALL=1 ./scripts/dev.sh
```
