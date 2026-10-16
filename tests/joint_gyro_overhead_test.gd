extends SceneTree
# The arm raised overhead with the elbow bent, so the hand and sword go behind the head, then
# swung across and out. Lifting it, the knuckles and the blade came up against the face and the
# guard against the chest: the slide, with one normal for every contact, stood still in that
# corner, and overhead the load limit was a wall with no slope to slide along. The arm must get
# behind the head with the full solver rare and without long stops.
# Run: godot --headless --path . --script res://tests/joint_gyro_overhead_test.gd

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	var warm = RandomNumberGenerator.new()
	for i in 360:
		lab.DrivePad(Vector3(warm.randf_range(-0.3, 0.3), warm.randf_range(-0.3, 0.3), 0), Vector2.ZERO, 0.0, false)
		lab._Process(1.0 / 60)
	var rng = RandomNumberGenerator.new()
	rng.seed = 3
	# Nose up 75° (elevation ~157°) with the elbow bending, then yaw sweeps across and out.
	var pad_of = func(f):
		var up = 75.0 * smoothstep(0.0, 1.0, min(f / 90.0, 1.0))
		var yaw = 0.0
		if f > 120:
			yaw = 30.0 * sin((f - 120) * TAU / 240.0)
		return Vector3(up, yaw, 0)
	var stick_of = func(f): return Vector2(0, smoothstep(0.0, 1.0, min(f / 60.0, 1.0)))
	var stuck = 0
	var slow = 0
	var times = []
	var paths = {}
	var behind = -1
	for f in 480:
		var p = pad_of.call(f) + Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3))
		lab.DrivePad(p, stick_of.call(f), 0.0, false)
		var before = []
		for d in 7:
			before.append(lab.Angle(0, d))
		var start = Time.get_ticks_usec()
		lab._Process(1.0 / 60)
		var ms = (Time.get_ticks_usec() - start) / 1000.0
		times.append(ms)
		var moved = 0.0
		var off = 0.0
		for d in 7:
			moved = max(moved, abs(lab.Angle(0, d) - before[d]))
			off = max(off, abs(lab.Target(0, d) - (lab.PlaneFromFront(0) if d == 0 else lab.Angle(0, d))))
		var path = lab.Trace(0).split("path ")[1]
		paths[path] = paths.get(path, 0) + 1
		if behind < 0 and lab.Angle(0, 1) >= 140:
			behind = f
		if off > 5 and moved < 0.2:
			stuck += 1
		if OS.get_environment("VERBOSE") == "1" and (f % 10 == 0 or ms > 6 or path == "solve"):
			print("%3d %5.2fms moved %4.1f off %5.1f %-16s %s | %s" % [f, ms, moved, off, path, lab.ArmTrace(0).split(" aim")[1].split(" planning")[0], lab.Contacts()])
	times.sort()
	print("behind the head at frame ", behind, " paths ", paths, " stuck frames ", stuck, " p50 %.2f p95 %.2f max %.2f" % [times[times.size() / 2], times[int(times.size() * 0.95)], times[times.size() - 1]])
	# Before this test: the full solver 6 times, 260 frames standing still, behind the head at
	# frame 173; now 5–6 solves (timing decides), ~145 standing, frame 168. Still open: lifted straight up, the bent arm waits at the face until the pad
	# swings it out (frame ~125) — the way round starts away from the target, which no slide
	# finds. Some standing is right: forward overhead the pad asks the upper arm through the head.
	var solves = paths.get("solve", 0)
	if behind >= 0 and behind <= 175 and solves <= 8 and stuck <= 180 and times[int(times.size() * 0.95)] < 4:
		print("PASS gyro overhead: the bent arm gets behind the head, full solve rare, frames short")
		quit()
	else:
		push_error("The bent arm stalls on its way behind the head")
		quit(1)
