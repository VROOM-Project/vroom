# Agent notes

This is a fork of [VROOM](https://github.com/VROOM-Project/vroom).
Upstream does not accept LLM-written contributions (see
[AI_POLICY.md](AI_POLICY.md)), so changes made here are not meant to be
sent back upstream.

## Building on macOS

```sh
git submodule update --init
brew install asio pkgconf openssl
cd src
make CXXFLAGS="-I. -I/opt/homebrew/include -std=c++20 -O3 -DASIO_STANDALONE -DUSE_ROUTING=true" \
     LDLIBS="-L/opt/homebrew/opt/openssl/lib -lpthread -lssl -lcrypto"
```

The binary is written to `bin/vroom`. Leaving out `-DNDEBUG` keeps
`assert` checks on, which is what you want while testing changes.

## Testing

There is no unit test suite. Check changes by running `bin/vroom -i
<file>.json` on inputs that use custom `matrices`, so no routing server
is needed.

## Fork-specific behaviour

- A vehicle may have neither `start` nor `end`. It is anchored to a
  hidden zero-cost location added to every matrix
  (`Input::add_virtual_location`). Its `start`/`end` output steps are
  then moved to the first/last task (`Input::relocate_virtual_ends`).

## Docker image

`Dockerfile` builds this checkout together with
[vroom-express](https://github.com/VROOM-Project/vroom-express) (HTTP API
on port 3000). It is a drop-in replacement for `vroomvrp/vroom-docker`:
mount the config at `/conf/config.yml` and set `VROOM_ROUTER`.

`.github/workflows/image.yml` builds amd64 and arm64 images on every PR
and push. On `master` it publishes `ghcr.io/shipeedo/vroom` tagged
`sha-<commit>`, `master` and `latest`.
