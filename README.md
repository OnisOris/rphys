# rphys
Physics engine is written in RUST

## Browser demo (WASM + JS)

Build the wasm package:
```
wasm-pack build --target web --out-dir www/pkg
```

Serve the static files:
```
python3 -m http.server --directory www
```

Open http://localhost:8000 in the browser.

Rust dev server alternative:
```
cargo run --bin devserver -- --dir www --port 8000
```
