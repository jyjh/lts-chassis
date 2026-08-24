# Contributing

All development happens on **forks**. `main` and `staging` here are
protected — you never push branches to this repository directly.

## The loop

1. **Fork** this repository once (GitHub → *Fork*, top right).
2. Clone your fork and initialize submodules:

   ```
   git clone <your-fork-URL>
   cd <this-repo>
   git submodule update --init --recursive
   ```

3. Branch from `staging`:

   ```
   git checkout staging
   git checkout -b your-name/short-description
   ```

4. Make your change, then run the tests in MATLAB (repo root): `run_tests`
   All green? If not, fix before continuing.
5. Push to your fork and open a **Pull Request targeting `staging`**, with
   the PR checklist filled in.
6. A maintainer reviews; CI must be green; they merge. You never merge your
   own PR.

## Branch model

- `staging` — where all work lands. PRs from forks target this branch.
- `main` — stable and release-only. Maintainers merge `staging` → `main`
  and tag a version (`v1.2.0`). The main integration repository pins its
  submodule of this repo to `main` tags (or `staging` tips while
  integrating).

## Rules

- Never edit anything under `kit/` (the lts-kit submodule) here — ask the
  integration lead instead.
- SI units everywhere; comments explain *why*, not *what*.
- Data files over 5 MB: open an issue and ask before committing.
- If CI is red on your PR: open the failing check, read the first error
  message — it is usually the answer. If not, paste it into the PR and ask.
  Stuck is normal; silent is not.

The full guidelines, PR/issue templates, and the repository-split plan live
in the main repository: <https://github.com/jyjh/lts> and
<https://jyjh.github.io/lts/repo-split/>.
