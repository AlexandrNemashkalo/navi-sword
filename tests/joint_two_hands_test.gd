extends SceneTree
# Both hands on the handle (a setting). At the guard the free hand is on the handle. A random
# minute of the pad's swings: it stays on it (half the time within a centimetre, nearly always
# within 4), the frames short. Wound up behind the head with both hands (the pad's nose up, the
# elbow bent): the sword arm goes up overhead (past 135°) with the free hand on the handle — the
# arms meet, the sword arm giving a little. Letting go (the other setting), wound far out to the
# right the free hand lets go, and takes the handle again back at the guard. One hand again: the
# handle short, the free arm where it was (only lifted clear of the body, as a free arm is).
# Run: godot --headless --path . --script res://tests/joint_two_hands_test.gd

func _initialize():
	call_deferred("run")

func step(lab, pad, stick = Vector2.ZERO):
	lab.DrivePad(pad, stick, 0.0, false)
	var t = Time.get_ticks_usec()
	lab._Process(1.0 / 60)
	return (Time.get_ticks_usec() - t) / 1000.0

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.TwoHanded = true
	lab.CentrePad()
	for i in 90:
		step(lab, Vector3.ZERO)
	var guard = lab.OffHandReach
	# A minute of swings.
	var reach = []
	var times = []
	for f in 900:
		var t = f / 60.0
		var pad = Vector3(30 * sin(t * 1.3) + 20 * sin(t * 0.4), 25 * sin(t * 0.9 + 1), 10 * sin(t * 0.7))
		times.append(step(lab, pad, Vector2(0, 0.4 * sin(t * 0.5))))
		reach.append(lab.OffHandReach)
	reach.sort()
	times.sort()
	# Behind the head.
	lab.CentrePad()
	for i in 60:
		step(lab, Vector3.ZERO)
	for f in 240:
		var up = 75.0 * smoothstep(0.0, 1.0, min(f / 90.0, 1.0))
		step(lab, lab.EvenSwing(Vector3(up, 0, 0)), Vector2(0, smoothstep(0.0, 1.0, min(f / 60.0, 1.0))))
	var overhead = lab.Angle(0, 1)
	var overhead_reach = lab.OffHandReach
	var give = lab.SwordGive
	# Letting go.
	lab.LetGo = true
	lab.CentrePad()
	for i in 60:
		step(lab, Vector3.ZERO)
	var least = 1.0
	for f in 120:
		step(lab, Vector3(0, 35.0 * min(1.0, f / 40.0), 0))
		least = min(least, lab.OffHandHold)
	for f in 120:
		step(lab, Vector3(0, 35.0 * max(0.0, 1.0 - f / 40.0), 0))
	var again = lab.OffHandHold
	var again_reach = lab.OffHandReach
	lab.LetGo = false
	lab.TwoHanded = false
	var free = lab.Angle(1, 1)
	step(lab, Vector3.ZERO)
	var stayed = abs(lab.Angle(1, 1) - free)
	print("guard %.3f m; swings: off the handle p50 %.3f p95 %.3f m, frames p50 %.1f p95 %.1f ms; behind the head: elevation %.0f°, off %.3f m, sword arm gave %.0f°; letting go: hold down to %.2f, again %.2f (off %.3f m); one hand: free arm moved %.1f°" % [guard, reach[450], reach[855], times[450], times[855], overhead, overhead_reach, give, least, again, again_reach, stayed])
	if guard < 0.01 and reach[450] < 0.01 and reach[855] < 0.04 and times[450] < 6 and overhead > 135 and overhead_reach < 0.04 and least < 0.1 and again > 0.99 and again_reach < 0.02 and stayed < 8:
		print("PASS two hands: on the handle at the guard, in swings and behind the head; letting go and taking hold again")
		quit()
	else:
		push_error("The two-handed grip slips, stalls overhead, or does not let go and take hold")
		quit(1)
