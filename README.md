# rust-sim

## Prerequisites (Web)

```bash
# Add WebAssembly target
rustup target add wasm32-unknown-unknown

# Install Trunk (build tool & dev server)
cargo install trunk --locked
```

## Running

### Web App
```bash
trunk serve
```
Open [http://127.0.0.1:8080](http://127.0.0.1:8080) in your browser.

### Native App
```bash
cargo run
```

