extends SceneTree
# (The pad's turns are as at its former even gain; EvenSwing keeps the poses this test means.)
# The arm forward and a little raised, elbow bent, the pad rolled to turn the shoulder in. Past
# 40° in, the raised arm is in the painful arc: the twist used to stop dead there (at the arc's
# edge the elevation does not change the pain, so the slide could only stop). Now the aim
# leaves the arc by the joint above — the elevation comes down a few degrees — and the twist
# goes on to its anatomical limit. No standing, no full solve, frames short.
# Run: godot --headless --path . --script res://tests/joint_gyro_twist_in_test.gd

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	var rng = RandomNumberGenerator.new()
	rng.seed = 3
	# Plane 53, elevation 55, elbow 70; roll in to 70° of twist over 2 s, hold, back.
	var pad_of = func(f):
		var roll = -58.0 * smoothstep(0.0, 1.0, clamp((f - 30) / 120.0, 0, 1) - clamp((f - 240) / 120.0, 0, 1))
		return Vector3(6.7, 10.8, roll)
	var stuck = 0
	var times = []
	var paths = {}
	var reversals = 0
	var last = []
	var prev = []
	for d in 7:
		last.append(0.0)
		prev.append(lab.Angle(0, d))
	for f in 400:
		lab.DrivePad(lab.EvenSwing(pad_of.call(f) + Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3))), Vector2.ZERO, 0.0, false)
		var start = Time.get_ticks_usec()
		lab._Process(1.0 / 60)
		if f >= 30:
			times.append((Time.get_ticks_usec() - start) / 1000.0)
		var moved = 0.0
		for d in 7:
			var v = lab.Angle(0, d) - prev[d]
			moved = max(moved, abs(v))
			if abs(v) > 0.3 and abs(last[d]) > 0.3 and sign(v) != sign(last[d]):
				reversals += 1
			if abs(v) > 0.3:
				last[d] = v
			prev[d] = lab.Angle(0, d)
		# Off what the arm can reach (the twist's own limit is not a stall).
		var off = abs(float(lab.Trace(2).split("aim ")[1].split(" ")[0]) - lab.Angle(0, 2))
		var path = lab.Trace(0).split("path ")[1]
		if f >= 30:
			paths[path] = paths.get(path, 0) + 1
		if off > 3 and moved < 0.2:
			stuck += 1
		if OS.get_environment("VERBOSE") == "1" and (f % 5 == 0 or path == "solve"):
			print("%3d moved %4.1f %-10s %s | %s" % [f, moved, path, lab.ArmTrace(0).split(" goal")[1].split(" planning")[0], lab.Contacts()])
	times.sort()
	print("paths ", paths, " stuck frames ", stuck, " reversals ", reversals, " p95 %.2f max %.2f" % [times[int(times.size() * 0.95)], times[times.size() - 1]])
	if paths.get("solve", 0) == 0 and stuck <= 5 and reversals <= 15 and times[int(times.size() * 0.95)] < 4:
		print("PASS gyro twist in: the elevation makes way, the twist goes on, no full solve")
		quit()
	else:
		push_error("Twisted in, the arm stalls")
		quit(1)
