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

### Miscellaneous

- Custom types use upper CamelCase (e.g. `LocalSearch`)
- Variables and functions use lowercase with underscore (e.g `addition_cost`)

### Automatic formatting

To ensure a consistent formatting of the code-base, please run
`./scripts/format.sh` prior to submitting a PR. A convenient way to
automate code formatting is to use a commit hook. Create the file
`.git/hooks/pre-commit` containing:

```
#!/bin/sh

./scripts/format.sh
```

then make sure it's executable: `chmod +x .git/hooks/pre-commit`.
