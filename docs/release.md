# Release process

## New release branch and release candidate

- Check `CHANGELOG.md` and `docs/API.md`
- Create new release branch from `master`: `git checkout -b release/X.Y`
- Add `-DNDEBUG` to `CXXFLAGS` in `makefile` and commit changes
- Bump version number in `src/utils/version.h` and in `CHANGELOG.md`
  and commit changes
- Tag release candidate: `git tag vX.Y-rc.N`
- Push release branch and tag: `git push origin -u release/X.Y`, `git push origin vX.Y-rc.N`

## Final release

- Bump version number in `src/utils/version.h` and in `CHANGELOG.md`
  and commit changes
- Tag : `git tag vX.Y.0`
- Push release branch and tag: `git push origin release/X.Y`, `git push origin vX.Y.0`
- Draft a new release on github
- Close github milestone
- Update relevant wiki pages and links

## Master branch

- Bump dev version number in `src/utils/version.h` and update `CHANGELOG.md`


