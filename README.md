# lts-chassis

Chassis component package for the FSAE transient lap-time simulation: the
`+Chassis` classes (`SimpleChassis` sprung-mass heave/pitch/roll platform,
`ChassisState`), mounted into the main repository at
`src/+lts/+components/+Chassis`.

## Ownership

| | |
|---|---|
| Department | Chassis |
| Maintainer | *add GitHub handle* |
| Term | *e.g. 2026/27* |

## Running the tests

Requires MATLAB R2019b+ (CI pins R2026a) and the `lts-kit` submodule:

    git submodule update --init --recursive

Then in MATLAB, from the repository root: `run_tests`

The runner assembles a temporary `+lts` package sandbox in `build/`
(gitignored) — this repository's classes plus kit's `+util` — and runs
`tests/`. Nothing is installed into the main repository.

## Branch model and workflow

- `staging` — where PRs from forks land. `main` — stable, release-only.
- All development is done on forks; see [CONTRIBUTING.md](CONTRIBUTING.md).

## Contract with the main repository

- `SimpleChassis` reads only the geometry handed to its constructor (a
  struct with `totalMass`, `wheelbase`, `trackWidth`, `cgHeight`,
  `staticFrontWeight`, and an optional `tire` with a `FL.wheelRadius`
  works); gravity comes from `lts.util.PhysicalConstants`, not from the
  main repository's classes.
- The chassis↔suspension link is structural: `setSuspension` accepts any
  object exposing the four corner units, roll-center/anti-geometry
  properties, and `getAxleRollStiffness`. Nothing in this package
  references `+Suspension` (or any other component package) by name.
- Details: <https://jyjh.github.io/lts/repo-split/>
