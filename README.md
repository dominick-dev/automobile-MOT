# automobile-MOT

A multi-object tracker for automotive radar. Takes radar detections in, produces tracks with stable IDs out.

**Status:** early. Setup and data loading are done; the tracker is in progress.

## Build

Requires a C++20 compiler (GCC 13+), CMake 3.21+, Ninja, HDF5, and Eigen3. HighFive, nlohmann/json, and GoogleTest are fetched by CMake.

```
sudo apt-get install ninja-build libhdf5-dev libeigen3-dev
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Presets: `debug`, `asan` (ASan + UBSan), `release`. Build output goes to `build/<preset>`.

## Data

The tracker runs on [RadarScenes](https://radar-scenes.com). The dataset is not included. Request it from the site, and note its license is non-commercial, so it can't be committed to this repo.

Set `RT_DATA_DIR` to the folder that contains the `sequence_N` directories:

```
export RT_DATA_DIR=/path/to/RadarScenes/data
```

## Run

```
build/debug/run sequence_1
```

The argument is either a sequence name, looked up under `RT_DATA_DIR`, or a path to a sequence directory.

## Python

Data exploration scripts live in `python/`.

```
python -m venv .venv && source .venv/bin/activate
pip install -r python/requirements.txt
```

## Tooling

- `.clang-format` and `.clang-tidy` for formatting and linting
- Tests use GoogleTest (`tests/`)

## CI

- **ci**: builds and runs tests on Ubuntu 24.04 for the `debug` and `asan` presets. Runs on pushes to `main` and on PRs.
- **Claude Code Review**: Claude reviews each PR and leaves comments (needs the `CLAUDE_CODE_OAUTH_TOKEN` secret).

## License

MIT. See [LICENSE](LICENSE).
