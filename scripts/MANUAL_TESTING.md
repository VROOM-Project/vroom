# Run the local VROOM binary (from project root)

## Build (macOS):

For typical testing from Trexity-generated requests, the matrix is always
included, so you can simplify the build by excluding routing:

```bash
./scripts/build-macos.sh --without-routing --debug
```

For a full build, omit `--without-routing`.

Run against example requests (matrix-only):

```bash
./bin/vroom-macos -t 4 -x 5 -i ./request3.example.json | jq .
```

Run with geometry (requires OSRM running for the profile):

```bash
# if OSRM car is at http://127.0.0.1:60050
./bin/vroom-macos -a car:127.0.0.1 -p car:60050 -g -t 4 -x 5 -i ./request3.example.json | jq .
```

Notes:
- Use the matrix-only command when the JSON includes a `matrices`/`matrix` section.
- Re-run the build script after code changes to refresh `bin/vroom-macos`.
