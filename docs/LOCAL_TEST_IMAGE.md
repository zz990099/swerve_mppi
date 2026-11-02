# Local test image

The repository publishes `ghcr.io/zz990099/swerve_mppi:jazzy-test` to its linked
GitHub Packages container package. It contains Ubuntu 24.04, ROS 2 Jazzy, Gazebo
Harmonic, plotting/test dependencies, MPPI sources, the CI-pinned chassis sources,
and a Release build with ROS plugin tests enabled. The platform is **linux/amd64**,
matching the GitHub Actions runner. This image does not target Humble or ARM64.

The chassis pin is `8f986bda64d5b48af012d37cf924328e0b127468`, matching the actual
`ci.yaml` and `trajectories.yaml` checkouts. `dependencies.repos` currently has a
different revision; this image deliberately reproduces the CI checkout.

## Pull and enter

```bash
docker pull ghcr.io/zz990099/swerve_mppi:jazzy-test
docker run --rm -it --shm-size=512m ghcr.io/zz990099/swerve_mppi:jazzy-test
```

The entrypoint sources ROS and the prebuilt workspace automatically. The default
working directory is `/opt/mppi_ws/src/swerve_mppi`; chassis sources are under
`/opt/mppi_ws/src/swerve_gazebo_sim`. No host ROS installation is needed.

If the package is private, first log in using your GitHub username and a personal
access token (classic) with `read:packages` as the password:

```bash
docker login ghcr.io -u zz990099
```

GitHub initially creates container packages as private. The owner can make the
package public in its package settings to allow anonymous pulls.

## Run a single physical case and save plots on the host

Run from an empty output directory (the runner rejects existing round directories):

```bash
mkdir -p artifacts
docker run --rm --shm-size=512m \
  -v "$PWD/artifacts:/artifacts" \
  ghcr.io/zz990099/swerve_mppi:jazzy-test \
  python3 swerve_mppi_ros/tests/run_trajectory_rounds.py \
    --chassis-root /opt/mppi_ws/src/swerve_gazebo_sim \
    --output /artifacts --rounds 1 --seed-base 42 \
    --min-path-length 12 --max-speed 1.0 --path-frame map \
    --cases straight
```

Remove `--cases straight` to run all 32 cases. PNGs, CSVs, JSON results and logs
are written to the mounted `artifacts/` directory. Files are created as root by
default. The supplied command uses headless Gazebo and does not need X11/GPU
forwarding. Container-local ROS discovery isolates the tests from host ROS nodes.
Test failures are preserved in the output and produce a nonzero exit code.
Publishing this image does not imply that all physical tracking quality gates pass.

## Run unit and ROS tests

```bash
docker run --rm ghcr.io/zz990099/swerve_mppi:jazzy-test bash -ec '
  python3 -m unittest discover -s swerve_mppi_ros/tests -p "test_*.py" -v
  cd /opt/mppi_ws
  colcon test --packages-select swerve_mppi_core swerve_mppi_ros \
    --return-code-on-test-failure --event-handlers console_direct+
  colcon test-result --all --verbose
'
```

For local source edits, copy a checkout into the container or mount it over
`/opt/mppi_ws/src/swerve_mppi`, then rebuild from `/opt/mppi_ws` with
`colcon build --packages-up-to swerve_mppi_ros --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON`.
Source `install/local_setup.bash` after rebuilding.

## Publication and reproducibility

[Publish local test image](../.github/workflows/test-image.yaml) builds on relevant
`main` pushes or manual dispatch. Pull requests build and test without publishing. Recording contract and core/ROS tests run inside
the built image before publication. The workflow then pulls from GHCR and checks
Python dependencies and installed MPPI/chassis packages.

Tags:

- `jazzy-test`: latest successfully tested image built by this workflow.
- `jazzy-test-sha-<full MPPI commit SHA>`: source-version tag.

The workflow summary includes the registry digest and both source revisions.
For an exact image, use `docker pull ghcr.io/zz990099/swerve_mppi@sha256:<digest>`;
a digest also pins the OS/dependency layers. Source tags identify source code but
can be rebuilt with newer upstream apt packages. The image is a local test
environment; physical/real-time performance depends on the host CPU.
