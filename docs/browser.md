# WebAssembly & ONNX Runtime with Trunk

This document details how to run `rust-sim` in the browser using [Trunk](https://trunkrs.dev/) (`wasm32-unknown-unknown`).

---

## 1. WebAssembly for ONNX

Running neural networks in a browser-targeted Bevy simulation requires careful library selection:

| Engine | Pure Rust? | `wasm32-unknown-unknown` Support | Best Used For |
| :--- | :---: | :---: | :--- |
| **`tract-onnx`** | **Yes** | **First-class (Native)** | **Recommended for Trunk / WASM builds** |
| **`ort`** | No (C++) | Complex (Needs Emscripten / JS glue) | Native desktop (`x86_64` / `aarch64`) |
| **Manual Forward Pass** | **Yes** | **Zero dependencies** | Ultra-small bundle size, fastest compile |
| **`onnxruntime-web`** | No (JS) | Requires `wasm-bindgen` JS bridge | Leveraging WebGPU / browser ORT |

### Why Not Standard `ort` in WASM?
The standard `ort` crate binds directly to Microsoft's C++ `libonnxruntime` library. Compiling C++ dependencies to `wasm32-unknown-unknown` (Trunk's default target) is complex and often fails due to missing POSIX APIs and threading assumptions.

### Recommended Engine: `tract-onnx`
[`tract`](https://github.com/sonos/tract) (developed by Sonos) is a **100% pure Rust** neural network engine:
- **Zero C/C++ dependencies**: Compiles seamlessly to `wasm32-unknown-unknown`.
- **Operator Support**: Fully implements the operators used by the Spot MLP policy (`Gemm`, `Elu`).
- **Memory friendly**: Can load directly from in-memory byte slices (`&[u8]`).

If you plan to support both Native (high performance) and Web (portable), you can use target-specific dependencies in `Cargo.toml`:
```toml
[target.'cfg(not(target_arch = "wasm32"))'.dependencies]
ort = "2.0.0-rc.9"

[target.'cfg(target_arch = "wasm32")'.dependencies]
tract-onnx = "0.21"
```

---

## 2. Trunk

`rust-sim` configures Trunk through [`index.html`](file:///home/kane-li/Documents/GitHub/rust-sim/index.html):
```html
<link data-trunk rel="rust" data-wasm-opt="z" />
<link data-trunk rel="copy-dir" href="assets" />
```

### Compile-Time Embedding
For assets needed immediately at startup, `rust-sim` embeds files directly into the WASM binary at compile time (such as `include_str!("../assets/spot.urdf")` in `scene.rs`). 

The ONNX policy can follow the same pattern:
```rust
const POLICY_BYTES: &[u8] = include_bytes!("../../assets/spot_policy.onnx");
```
This avoids runtime network requests and async asset fetch latency in the browser.

### Building & Running
Run the local development server:
```bash
trunk serve --release
```

Build the static distribution artifacts (written to `dist/`):
```bash
trunk build --release
```
