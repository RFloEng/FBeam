"""Steering geometry and steady-state vehicle dynamics for the FBeam editor.

Pure Python, standard library only (runs in Pyodide). FBeam axes: X left, Y rear, Z up,
metres; SI units inside, mm and degrees in the result keys that say so. Uses the kinematic
corner solver (fbeam_suspension) and the full-car mass result (fbeam_mass). Informs 7.2
(steering) but raises no rule issues itself.

Steering (left turn): the rack moves both inner tie-rod points to the right (-X) by d.
The left corner is solved with its TRI moved by -d. The right corner is the mirror image
of the left one, so it is the left corner solved with +d. Steer angle is positive when
the front of the wheel points left.

Roll and load transfer (steady state, per g of lateral acceleration), classic
roll-axis method: roll stiffness per axle K = k_ride * t^2 / 2 (+ anti-roll bar), with
k_ride the wheel rate in series with the tyre; the sprung mass rolls about the axis
through the front and rear roll centres; each axle's lateral load transfer has an
elastic part (its share of roll stiffness), a geometric part (roll centre height) and
an unsprung part:
  dW_axle = (K_axle / (Kf + Kr) * ms g (hs - h_ra) + ms_axle g h_rc + m_u,axle g z_wc) / track
"""

import math

import fbeam_suspension as susp

G = 9.81
TYRE_RATE_N_PER_M = 200_000.0     # 165/70R14 vertical stiffness estimate; confirm from the spec tyre in-game


def _toe_with_rack(spec, points, d):
    """Toe (deg, positive toe-in) of the left corner at static height with TRI moved d (m) along X."""
    return susp.Corner(spec, points).toe_with_rack(d)


def steering(spec, points, wheelbase, rack_travel_mm=70.0, rack_mm_per_rev=65.0, steps=14):
    """Steer angles over rack travel, Ackermann, lock, turning circle and ratio (left turn).

    spec, points: the front corner as for fbeam_suspension.Corner. wheelbase in m.
    rack_travel_mm: rack travel each way from centre, swept in steps equal steps.
    rack_mm_per_rev: rack travel per steering-wheel turn (65 mm, the editor's default too).
    Returns lists over rack travel (rack_mm, inner_deg, outer_deg, ideal_inner_deg,
    ackermann_pct: 100 = true Ackermann, 0 = parallel steer) and lock_inner_deg,
    lock_outer_deg, ackermann_at_lock_pct, turning_circle_m (kerb to kerb, outer tyre edge),
    overall_ratio (steering-wheel deg per road-wheel deg near centre), rack_travel_total_m,
    lock_to_lock_turns, track_m, wheelbase_m. Raises ValueError if no rack step assembles.
    """
    track = 2 * points["WC"][0]                        # wheel-centre track, used for the Ackermann ideal
    out = {"rack_mm": [], "inner_deg": [], "outer_deg": [], "ideal_inner_deg": [], "ackermann_pct": []}
    # left turn = the left wheel's front points left. Tie rods behind the axle: the rack moves right (-X);
    # ahead of it: left (+X). Find which from a small first step.
    probe = rack_travel_mm / 1000 / steps / 4
    sgn = -1.0 if -_toe_with_rack(spec, points, -probe) > 0 else 1.0
    for i in range(1, steps + 1):
        d = rack_travel_mm / 1000 * i / steps
        try:
            dl = -_toe_with_rack(spec, points, sgn * d)   # left (inner) wheel steer angle = -toe, + = front points left
            dr = _toe_with_rack(spec, points, -sgn * d)   # right (outer) wheel: the mirrored corner; steer = +toe
        except ValueError:
            break
        inner, outer = dl, dr                          # left turn: the left wheel is the inner one
        if outer <= 0:
            continue
        # Ackermann: cot(outer) - cot(inner) = track / wheelbase
        ideal = math.degrees(math.atan(wheelbase / (wheelbase / math.tan(math.radians(outer)) - track)))
        ack = (inner - outer) / (ideal - outer) * 100 if ideal - outer > 1e-9 else 0.0
        out["rack_mm"].append(d * 1000)
        out["inner_deg"].append(inner)
        out["outer_deg"].append(outer)
        out["ideal_inner_deg"].append(ideal)
        out["ackermann_pct"].append(ack)
    if not out["rack_mm"]:
        raise ValueError("the steering does not assemble; check the tie-rod points")
    outer_lock, inner_lock = out["outer_deg"][-1], out["inner_deg"][-1]
    small = 0 if len(out["rack_mm"]) < 2 else 1        # ratio taken near centre (second step)
    mean_small = (out["inner_deg"][small] + out["outer_deg"][small]) / 2
    sw_small = out["rack_mm"][small] / rack_mm_per_rev * 360     # steering-wheel angle, deg
    out.update({
        "track_m": track, "wheelbase_m": wheelbase,
        "lock_inner_deg": inner_lock, "lock_outer_deg": outer_lock,
        "ackermann_at_lock_pct": out["ackermann_pct"][-1],
        # outer front wheel radius wheelbase / sin(outer), doubled, plus half a tyre width each side
        "turning_circle_m": 2 * wheelbase / math.sin(math.radians(outer_lock)) + susp.SPEC_TYRE["width"],
        "overall_ratio": sw_small / mean_small,
        "rack_travel_total_m": 2 * rack_travel_mm / 1000,
        "lock_to_lock_turns": 2 * rack_travel_mm / rack_mm_per_rev,
    })
    return out


def roll_and_load_transfer(mass, corners, unsprung_per_corner, wheelbase, arb_front=0.0, arb_rear=0.0):
    """Steady-state roll gradient, lateral load transfer distribution and ride frequencies.

    mass: fbeam_mass result (mass_total, cg, front_share). corners: {"front": res, "rear": res}
    from the suspension analysis (static wheel rate, track, roll centre, wheel centre height).
    unsprung_per_corner: kg, None or 0 if not set. wheelbase: m.
    arb_*: anti-roll bar roll stiffness per axle (N*m/rad), 0 if none.
    Returns sprung mass and CG height, roll axis height and arm (m), roll stiffness per axle
    (N*m/deg) and front share, roll gradient (deg/g), lateral load transfer per axle (N/g),
    LLTD front share, weight front share, ride frequencies (Hz) and the tyre rate used.
    """
    M, h = mass["mass_total"], mass["cg"][2]
    f_share = mass["front_share"]
    mu = unsprung_per_corner or 0.0
    st_f, st_r = corners["front"]["static"], corners["rear"]["static"]
    zf, zr = st_f["wheel_centre_mm"] / 1000, st_r["wheel_centre_mm"] / 1000
    ms = M - 4 * mu
    hs = (M * h - 2 * mu * zf - 2 * mu * zr) / ms      # sprung CG: total moment minus the unsprung lumps at the WCs
    # sprung mass front share: total front load minus the front unsprung
    ms_f = (M * f_share - 2 * mu) / ms
    a_s = wheelbase * (1 - ms_f)                        # sprung CG behind the front axle
    tf, tr = st_f["track_mm"] / 1000, st_r["track_mm"] / 1000
    rcf, rcr = (st_f["roll_centre_mm"] or 0) / 1000, (st_r["roll_centre_mm"] or 0) / 1000
    h_ra = rcf + (rcr - rcf) * a_s / wheelbase           # roll axis height under the sprung CG
    arm = hs - h_ra

    def ride(k_wheel):                                   # spring in series with the tyre
        return k_wheel * TYRE_RATE_N_PER_M / (k_wheel + TYRE_RATE_N_PER_M)

    if not st_f.get("wheel_rate_N_per_mm") or not st_r.get("wheel_rate_N_per_mm"):
        raise ValueError("roll: a corner has no wheel rate (its damper does not move with the wheel)")
    kf, kr = ride(st_f["wheel_rate_N_per_mm"] * 1000), ride(st_r["wheel_rate_N_per_mm"] * 1000)
    Kf = kf * tf * tf / 2 + arb_front                     # N*m/rad: two wheels at +-t/2, 2 k (t/2)^2
    Kr = kr * tr * tr / 2 + arb_rear
    roll_moment = ms * G * arm                            # per g
    # phi = ms g arm (1 + phi) / K  ->  phi = M / (K - M): includes the overturning of the rolled mass
    if Kf + Kr <= roll_moment:
        raise ValueError("roll: the springs and bars cannot hold the car's roll moment (roll unstable)")
    roll_grad = roll_moment / (Kf + Kr - roll_moment)     # rad/g
    # elastic share + geometric (sprung load at the roll centre) + unsprung (at the wheel centre)
    dWf = (Kf / (Kf + Kr) * roll_moment + ms * G * ms_f * rcf + 2 * mu * G * zf) / tf
    dWr = (Kr / (Kf + Kr) * roll_moment + ms * G * (1 - ms_f) * rcr + 2 * mu * G * zr) / tr
    m_corner_f = ms * ms_f / 2                            # sprung mass per corner, for f = sqrt(k / m) / 2 pi
    m_corner_r = ms * (1 - ms_f) / 2
    return {
        "sprung_mass": ms, "sprung_cg_height_m": hs, "roll_axis_height_m": h_ra, "roll_arm_m": arm,
        "roll_stiffness_front_Nm_per_deg": math.radians(Kf), "roll_stiffness_rear_Nm_per_deg": math.radians(Kr),
        "roll_stiffness_front_share": Kf / (Kf + Kr),
        "roll_gradient_deg_per_g": math.degrees(roll_grad),
        "load_transfer_front_N_per_g": dWf, "load_transfer_rear_N_per_g": dWr,
        "lltd_front": dWf / (dWf + dWr), "weight_front": f_share,
        "ride_freq_front_hz": math.sqrt(kf / m_corner_f) / (2 * math.pi),
        "ride_freq_rear_hz": math.sqrt(kr / m_corner_r) / (2 * math.pi),
        "tyre_rate_N_per_mm": TYRE_RATE_N_PER_M / 1000, "tyre_rate_is_estimate": True,
    }
