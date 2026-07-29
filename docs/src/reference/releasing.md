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

Three things on the **RoverSoftware** repository. Without them the workflows
build fine but cannot publish — and each one fails with a step that tells you
exactly what is missing, rather than `Input required and not supplied: token`.

### 1 · A signing key for the apt repository

Generate it somewhere you trust — a laptop, not a runner — and keep the private
half. Debian clients will refuse an unsigned repository unless every rover is
configured with `[trusted=yes]`, which is worse.

```bash
gpg --batch --quick-gen-key "RoverSoftware Packages <you@example.com>" \
    ed25519 sign never
gpg --list-secret-keys --with-colons | awk -F: '/^fpr/{print $10; exit}'
```

Export the private key and paste it into a secret called
**`APT_GPG_PRIVATE_KEY`**:

```bash
gpg --armor --export-secret-keys <FINGERPRINT>
```

Back the key up offline. Losing it means every rover has to have its keyring
replaced by hand before it can update again.

### 2 · A token that can push to the site repo

The default `GITHUB_TOKEN` is scoped to this repository, so it cannot write to
`NEXT-Competition.github.io`. Create a **fine-grained personal access token**
with:

- Repository access: `NEXT-Competition/NEXT-Competition.github.io`
- Permissions: **Contents → Read and write**

Save it as **`SITE_DEPLOY_TOKEN`** under
*Settings → Secrets and variables → Actions*. A deploy key on the site repo
works too if you would rather not use a PAT — swap the `token:` line in both
workflows for `ssh-key:`.

Fine-grained tokens expire. When one does, both workflows fail at their
"Check the deploy token exists" step with the same instructions; generate a new
token and update the secret.

### 3 · Pages enabled on the site repo

In `NEXT-Competition.github.io` → Settings → Pages, serve from the default
branch, root. Both workflows commit into that branch; nothing else is needed.

## How the site is laid out

Two workflows write into one repository, and neither may clobber the other:

```text
NEXT-Competition.github.io/
├── .nojekyll                   # so Jekyll leaves mdBook's output alone
└── roversoftware/
    ├── index.html …            ← docs.yml       (the handbook)
    └── apt/                    ← release.yml    (the package repository)
        ├── roversoftware-archive-keyring.asc
        ├── dists/stable/…
        └── pool/main/*.deb
```

The docs job syncs with `rsync --delete --exclude 'apt/'`, so publishing
documentation never unpublishes packages. Both jobs share the
`pages-site` concurrency group and retry a rejected push with a rebase, because
two concurrent pushes to one branch is a lost commit.

The apt job is **incremental**: it checks out the existing pool, adds the new
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
   <https://next-competition.github.io/roversoftware/>.
