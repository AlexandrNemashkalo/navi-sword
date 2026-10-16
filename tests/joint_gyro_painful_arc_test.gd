extends SceneTree
# Between two guards in the painful arc (raised 100–118°, the pad rolled to turn the shoulder in
# 57–80°): the twist cannot go past 40° there. Deep in the arc the elevation used to carry the
# arm out over its top and back down, 25° up and down, while the pad held level; now it leaves
# the arc only near its edge and the twist makes way for the rest. No standing, no full solve,
# few reversals. Run: godot --headless --path . --script res://tests/joint_gyro_painful_arc_test.gd

func _initialize():
	call_deferred("run")

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	var rng = RandomNumberGenerator.new()
	rng.seed = 3
	# The two guards (plane from the front, elevation, twist, elbow): (41,118,57,70) and (24,100,80,70), back and forth.
	var pad_of = func(f):
		var t = 0.5 - 0.5 * cos(clamp((f - 60) / 90.0, 0, 4) * PI)
		var a = Vector3(48.7, 15.6, -47.5)
		var b = Vector3(36.7, 22.4, -66.7)
		return a.lerp(b, t)
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
		lab.DrivePad(pad_of.call(f) + Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3)), Vector2.ZERO, 0.0, false)
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
	if paths.get("solve", 0) == 0 and stuck <= 5 and reversals <= 30 and times[int(times.size() * 0.95)] < 4:
		print("PASS gyro painful arc: smooth between the guards, no full solve")
		quit()
	else:
		push_error("In the painful arc the arm stalls or rocks")
		quit(1)
