"""Generate one world file per trial from sydney_regatta.sdf with the sea-state,
direction and seed baked in (USVWind's random_seed is fixed per world file, so
without this every trial in a cell repeats the same gust sequence).

python3 make_trial_worlds.py --template /path/sydney_regatta.sdf --out worlds/ \
    --sea-states 3 --directions beneficial nominal --n-trials 20
Produces  worlds/trial_ss3_beneficial_seed0301.sdf ...  and prints the list.
Conventions: direction = the direction wind/waves travel TOWARD, degrees CCW
from +x (USVWind.cc: force along (cos, sin) of wind_direction). Beneficial:
wind 180, waves 170 (from astern of the westbound approach). Nominal: wind 135,
waves 120 (quartering). Verify the sign once (vessel heading west, beneficial:
/wamv/disturbance w_x > 0).

Units: USVWind converts wind_direction from degrees; the wavefield
(Wavefield.cc, cos(n*angle + direction)) takes direction in RADIANS with no
conversion, so the wave direction is converted here before it is written.

The <world name="..."> tag is rewritten to the file stem: vrx_gz builds the
IMU/GPS/LiDAR/joint-state bridges as /world/<basename of world:=>/model/wamv/...,
and Gazebo names those topics after the tag, so the two must agree.
"""
import argparse
import math
import pathlib
import re

SEA = {  # sea state -> (wind m/s, wave gain, period s, gust gain, gust tau)
    2: (3.5, 0.25, 3.5, 0.2, 5),
    3: (6.0, 0.50, 4.5, 0.2, 5),
    4: (9.0, 0.85, 5.5, 0.3, 10),
    5: (12.0, 1.30, 6.5, 0.3, 10),
}
DIR = {"beneficial": (180.0, 170.0), "nominal": (135.0, 120.0)}

WIND = """    <plugin filename="libUSVWind.so" name="vrx::USVWind">
      <wind_obj>
        <name>wamv</name>
        <link_name>wamv/base_link</link_name>
        <coeff_vector>0.5 0.5 0.33</coeff_vector>
      </wind_obj>
      <wind_direction>{wdir}</wind_direction>
      <wind_mean_velocity>{wv}</wind_mean_velocity>
      <var_wind_gain_constants>{gg}</var_wind_gain_constants>
      <var_wind_time_constants>{gt}</var_wind_time_constants>
      <random_seed>{seed}</random_seed>
      <update_rate>20</update_rate>
      <topic_wind_speed>/vrx/debug/wind/speed</topic_wind_speed>
      <topic_wind_direction>/vrx/debug/wind/direction</topic_wind_direction>
    </plugin>

"""
WAVE = """    <plugin filename="libPublisherPlugin.so" name="vrx::PublisherPlugin">
      <message type="gz.msgs.Param" topic="/vrx/wavefield/parameters" every="1.0">
        <params>
          <key>direction</key>
          <value><type>DOUBLE</type><double_value>{vdir:.6f}</double_value></value>
        </params>
        <params>
          <key>gain</key>
          <value><type>DOUBLE</type><double_value>{gain}</double_value></value>
        </params>
        <params>
          <key>period</key>
          <value><type>DOUBLE</type><double_value>{period}</double_value></value>
        </params>
        <params>
          <key>steepness</key>
          <value><type>DOUBLE</type><double_value>0.02</double_value></value>
        </params>
      </message>
    </plugin>

"""
A = "    <!-- Wind Configuration with Asymmetric Force -->"
B = "    <!-- Wave Field with Different Angle for Yaw Moment -->"
C = "    <!-- Alternative: Oscillating Wave Direction for Periodic Yaw Moments -->"

p = argparse.ArgumentParser()
p.add_argument("--template", required=True)
p.add_argument("--out", default="worlds")
p.add_argument("--sea-states", type=int, nargs="+", default=[3])
p.add_argument("--directions", nargs="+", default=["beneficial", "nominal"])
p.add_argument("--n-trials", type=int, default=20)
a = p.parse_args()

src = pathlib.Path(a.template).read_text()
assert A in src and B in src and C in src, "template markers not found"
out = pathlib.Path(a.out); out.mkdir(parents=True, exist_ok=True)
for ss in a.sea_states:
    wv, gain, period, gg, gt = SEA[ss]
    for d in a.directions:
        wdir, vdir_deg = DIR[d]
        vdir = math.radians(vdir_deg)          # wavefield direction is in radians
        for k in range(a.n_trials):
            seed = ss * 100 + (0 if d == "beneficial" else 50) + k
            s = src[:src.index(A)] + f"    <!-- trial world: SS{ss} {d} seed {seed} -->\n" \
                + WIND.format(wdir=wdir, wv=wv, gg=gg, gt=gt, seed=seed) \
                + WAVE.format(vdir=vdir, gain=gain, period=period) + src[src.index(C):]
            stem = f"trial_ss{ss}_{d}_seed{seed:04d}"
            s, n = re.subn(r'<world name="[^"]*">', f'<world name="{stem}">', s, count=1)
            assert n == 1, "<world name=...> tag not found in template"
            (out / f"{stem}.sdf").write_text(s)
            print(f"{stem}.sdf")
