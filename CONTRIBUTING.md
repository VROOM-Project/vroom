# Usage and bug reports

If you encounter any trouble while building or running the code,
please check:

- [the wiki](https://github.com/VROOM-Project/vroom/wiki) for build
and usage instructions
- the API documentation at [docs/API.md](docs/API.md)
- open or closed issues covering a similar problem.

# Contributing code

Contributions to the code-base are very welcome. If you plan to work
on something you want to submit a PR for, feel free to open an issue
in order to discuss implementation details or make sure your idea fits
with the scope or aim of the project.

## Branching

* The tip of the `master` branch is the current dev version.
* Separate tasks are done in branches, using explicit names like
`feature/*`, `experiment/*`, `refactor/*`, `fix/*` whenever possible.
* Branches are usually merged using a non-fast-forward merge.

## Coding conventions

To ensure a consistent formatting of the code-base, please run
`./scripts/format.sh` prior to submitting a PR (or run manually
`clang-format-3.8` using the provided `.clang-format` configuration
file).
