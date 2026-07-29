# Cutting a release

Releasing is one action: **push a tag.**

```bash
git tag -a v0.2.0 -m "0.2.0"
git push origin v0.2.0
```

That runs `.github/workflows/release.yml`, which builds everything, attaches it
to a GitHub Release, and folds the Debian packages into the
[apt repository](../install/apt.md) so every rover picks them up on its next
`apt upgrade`.

## What a tag produces

| Artifact | Built on | For |
|---|---|---|
| `roversoftware-robot_<v>_all.deb` | ubuntu-latest | Any Debian robot — the code is pure Python, so one build serves every Pi |
| `roversoftware-basestation_<v>_all.deb` | ubuntu-latest | Base stations and kiosks; bundles the built touch UI |
| `roversoftware-basestation-<v>-linux-x64` | ubuntu-latest | Desktop base station |
| `roversoftware-basestation-<v>-macos-x64` | macos-13 | Intel Macs |
| `roversoftware-basestation-<v>-macos-arm64` | macos-14 | Apple Silicon |
| `roversoftware-basestation-<v>-windows-x64.exe` | windows-latest | Windows |
| `roversoftware-<v>-py3-none-any.whl` + `.tar.gz` | ubuntu-latest | `pip install` |
| `SHA256SUMS` | — | `sha256sum -c SHA256SUMS` |

The desktop binaries come from `deno task bundle`, which is why they are built
on four runners rather than cross-compiled: a Deno desktop binary embeds a
platform-specific runtime.

## Version numbers

The tag is the single source of truth. `v0.2.0` becomes:

- the Debian `Version:` field, via `@VERSION@` in the control files
- the wheel and sdist version, stamped into `pyproject.toml` at build time
- the release name and the download paths

The `version` job validates it first, because a Debian version that does not
start with a digit produces a package nothing can ever upgrade from. Pre-release
tags work if you use Debian's ordering: `v0.2.0~rc1` sorts *before* `v0.2.0`,
which is what you want. `v0.2.0-rc1` sorts *after* it, which is not. Either is
marked as a GitHub pre-release.

`version = "0.1.0"` in `pyproject.toml` and `version := "0.1.0"` in the
`Justfile` are development defaults for local builds. They do not need bumping
to release, and CI does not read them.

## One-time setup

Two things, and only the first blocks anything today.

### 1 · Turn Pages on

**Settings → Pages → Source: Deploy from a branch → `gh-pages` / `(root)`.**

That is the whole deploy configuration. Both workflows publish to this
repository's own `gh-pages` branch using the built-in `GITHUB_TOKEN`, so there
is no personal access token to create and nothing to keep in sync with another
repository. The branch is created by the first successful run — you can set the
dropdown before or after it, whichever order you get to.

### 2 · A signing key for the apt repository

Only needed before the first tag; the handbook publishes without it.

Generate it somewhere you trust — a laptop, not a runner — and keep the private
half. Debian clients refuse an unsigned repository unless every rover is
configured with `[trusted=yes]`, which is worse.

```bash
gpg --batch --quick-gen-key "RoverSoftware Packages <you@example.com>" \
    ed25519 sign never
gpg --list-secret-keys --with-colons | awk -F: '/^fpr/{print $10; exit}'
```

Export the private key and paste it into a repository secret called
**`APT_GPG_PRIVATE_KEY`** (*Settings → Secrets and variables → Actions*):

```bash
gpg --armor --export-secret-keys <FINGERPRINT>
```

Back the key up offline. Losing it means every rover has to have its keyring
replaced by hand before it can update again.

## How the site is laid out

Both workflows write to the `gh-pages` branch of this repository, and neither
may clobber the other:

```text
gh-pages/
├── .nojekyll                   # so Jekyll leaves mdBook's output alone
├── index.html …                ← docs.yml      (the handbook)
└── apt/                        ← release.yml   (the package repository)
    ├── roversoftware-archive-keyring.asc
    ├── dists/stable/…
    └── pool/main/*.deb
```

Served at `https://next-competition.github.io/RoverSoftware/`. That path is
**case-sensitive** — it is the repository name, so the capital R and S matter in
a `sources.list` line.

The docs job syncs with `rsync --delete --exclude 'apt/'`, so publishing
documentation never unpublishes packages. Both jobs share the `pages-site`
concurrency group and retry a rejected push with a rebase, because two
concurrent pushes to one branch is a lost commit. Neither job assumes the branch
exists: if the clone fails they create it as an orphan.

The apt job is **incremental**: it clones the existing pool, adds the new
`.deb` files, and regenerates every index over all of it. That is what keeps
`apt-get install roversoftware-robot=0.1.0` working after 0.2.0 ships.

## Running it by hand

`workflow_dispatch` takes a version and an optional "skip apt" toggle — useful
for a test build that should not reach any rover.

```bash
gh workflow run release.yml -f version=0.2.0 -f publish_apt=false
```

## Building the same things locally

```bash
# packages
VERSION=0.2.0 ./packaging/build-deb.sh all

# an apt repository over them (unsigned unless you pass a key)
GPG_KEY_ID=<fingerprint> ./packaging/apt-repo.sh dist/*.deb

# desktop binary for this machine
cd basestation-ui && npm ci && npm run build && deno task bundle

# wheel + sdist
pipx run build
```

`packaging/apt-repo.sh` also honours `APT_ROOT`, so you can point it at a USB
stick and serve a field repository off a laptop when the venue has no useful
internet:

```bash
APT_ROOT=/media/usb/apt ./packaging/apt-repo.sh dist/*.deb
python3 -m http.server -d /media/usb/apt 8080
```

## The handbook deploys on its own

`docs.yml` runs on every push to `main` that touches `docs/`, and on pull
requests it builds without publishing — so a broken link or a missing screenshot
fails the PR rather than the site. You do not need to tag to update
documentation.

```bash
just book         # build into docs/book
just book-serve   # live-reloading preview on :3000
```

## A release checklist that has caught things

1. `pytest -q` is green.
2. `just book` builds with no warnings about missing links.
3. The simulator still comes up: `./start-basestation.sh`.
4. Tag, push, and watch the run.
5. On a spare Pi: `apt-get update && apt-get install roversoftware-robot` and
   confirm `apt-cache policy` shows the new version coming from the repository.
6. Check the handbook rendered at
   <https://next-competition.github.io/RoverSoftware/>.

If step 5 shows nothing, the usual cause is Pages not yet pointed at
`gh-pages` — the branch exists but nothing is serving it.
