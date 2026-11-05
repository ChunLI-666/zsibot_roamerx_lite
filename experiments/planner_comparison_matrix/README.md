# Matrix planner comparison

This launcher starts only navigation, the existing safety gate and full MCAP
recording against an already running Matrix sensor bridge and native Lightning
localization. It never publishes replacement TF/odometry, sends goals, or starts
an SDK owner. Use the isolated domain 185 network environment. An independent
goal pilot sends each explicitly recorded Action.

Select `official`, `segmented`, or the auxiliary `hybrid` control with `--variant`.
Supply `--map`, `--footprint`, `--padding`, `--output`, the appropriate explicit
`--official-install` or `--segmented-install`, `--build-receipt`, and a custom-message
`--message-install` prefix. Preparation
is the default; `--run` starts the stack. The build receipt must bind selected
sources to a full local commit and list installation artifact SHA256 values.
Every listed artifact, including the segmented executable, is checked before
launch. The actual loaded Nav2 libraries and runtime parameters are also saved.
The message prefix is copied and applied only to the recorder process. Its
CustomMsg, projection debug, build metadata and Lightning debug types must load,
and recorder subscriptions must be present before READY. Do not substitute an
old full navigation overlay. A bag without these interfaces is incomplete even
when its raw sensors and front camera are available.

The fresh map retains 1 cm geometry and its mapping evidence uses 5 cm cells.
Navigation uses a 20 m rolling global costmap and 6 m local costmap at 1 cm.
Both publish full raw master costmaps; the recorder includes hidden topics,
actions, front camera and calibration, original sensors, commands, TF and diagnostics.
Each of the four complete frozen static-layer diagnostic topics is captured once
as its original CDR under `/experiment/static_layer_snapshot/...`, with a sidecar
hash and reliable delivery acknowledgement. The original `/map` and master
costmaps remain fully recorded. After closure, the MCAP must contain exactly one
message per snapshot alias with the original CDR hash. A map reload requires a new run. The old all-topic
v2 recording contained about 113 GB of uncompressed CDR, 92% from repeated complete
static-layer copies; that recording remains intact. The revised policy removes
those repeated subscriptions, and its exact exclusions are saved in each manifest.
The launcher writes a 0.5 s readiness lease and continuously revokes it for stale
odom/TF/scan/camera or invalid localization. The pilot must additionally check
current inputs immediately before each goal. Runtime command guards remain active
even if the observer exits.
Use a local DDS initial-peer range sufficient for all participants (the validated
November 5 environment uses 64). For the verified local fragmented CustomMsg loss,
the projector supports `sensor_input_reliable:=true`; its original default remains
false. This changes neither sampling stamps nor the 0.3 s stale-input rejection.

`matrix_profile.yaml` is an explicit open-area dynamic profile, separate from the
strict ideal-plant reference in `profile_template.yaml`. Before goal experiments,
Matrix native forward input 0.20 (nominal policy vx 0.4) measured about 0.441 m/s;
smaller inputs could remain stationary. Native yaw 0.30 measured about 0.2 rad/s.
Therefore the profile requests nominal vx 0.4, rotation 0.3, and does not silently
boost low commands downstream. Angular acceleration 6 permits a 0.3 policy step
at 20 Hz, within the tested input step amplitude; this is not a measured body
acceleration claim. The default final/phase XY tolerance is 0.15 m and yaw is
0.08 rad, applied identically to both schemes. Measured zero-command stopping
displacement was about 0.056 m; this sample is not a guaranteed bound. Each case
must still report exact settled original-goal error.

For the November 5 open-area runs, the conservative footprint is a square with
vertices at x/y = +/-0.74 m, plus 0.15 m padding. A triangle-inequality bound over
all model meshes and joint translations gives radius <=0.732237 m independently
of joint rotation. This deliberately conservative envelope does not establish
narrow-door capability. No result here establishes physical robot acceptance.

Keep every attempted run, including startup failures, network outages and
controller rejections. An Action success alone is insufficient: verify the
original goal after stopping, ordered turns, collision checks, camera continuity,
MCAP readability and concrete code/config/map identities.
