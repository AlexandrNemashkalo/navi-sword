extends SceneTree
# Plays sixteen random smooth gyro motions, four seconds each, all over the sword arm's range,
# with a little sensor noise and the elbow on the stick, and checks the solver as a whole:
# the full solve is rare (free moves take the straight step, contacts the cheap slide), the
# arm reverses hardly more often than the pad asks it to, and frames stay short.
# Run: godot --headless --path . --script res://tests/joint_gyro_survey_test.gd

func _initialize():
	call_deferred("run")

func smooth(rng):
	var a1 = rng.randf_range(0.3, 1.0)
	var f1 = rng.randf_range(0.15, 0.5)
	var p1 = rng.randf_range(0, TAU)
	var a2 = rng.randf_range(0.0, 0.5)
	var f2 = rng.randf_range(0.5, 1.2)
	var p2 = rng.randf_range(0, TAU)
	return func(t): return a1 * sin(TAU * f1 * t + p1) + a2 * sin(TAU * f2 * t + p2)

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	var rng = RandomNumberGenerator.new()
	rng.seed = 11
	var extra = 0
	var frames = 0
	var solves = 0
	var times = []
	for k in 16:
		var sp = smooth(rng)
		var sy = smooth(rng)
		var sr = smooth(rng)
		var se = smooth(rng)
		var cp = rng.randf_range(-15, 30)
		var cy = rng.randf_range(-20, 45)
		var pad_of = func(t): return Vector3(cp + 30 * sp.call(t), cy + 35 * sy.call(t), 30 * sr.call(t))
		var stick_of = func(t): return Vector2(0, clamp(se.call(t), -1, 1))
		lab.CentrePad()
		for i in 45:
			lab.DrivePad(pad_of.call(0.0), stick_of.call(0.0), 0.0, false)
			lab._Process(1.0 / 60)
		lab.ResetSolverStats()
		var noise = RandomNumberGenerator.new()
		noise.seed = k
		var prev = []
		var last = []
		var target_prev = []
		var target_last = []
		for d in 7:
			prev.append(lab.Angle(0, d))
			last.append(0.0)
			target_prev.append(lab.Target(0, d))
			target_last.append(0.0)
		for f in 240:
			var t = f / 60.0
			var p = pad_of.call(t) + Vector3(noise.randf_range(-0.3, 0.3), noise.randf_range(-0.3, 0.3), noise.randf_range(-0.3, 0.3))
			lab.DrivePad(p, stick_of.call(t), 0.0, false)
			var start = Time.get_ticks_usec()
			lab._Process(1.0 / 60)
			times.append((Time.get_ticks_usec() - start) / 1000.0)
			for d in 7:
				# Reversals of the arm's joints, less those of the target itself.
				var v = lab.Angle(0, d) - prev[d]
				if abs(v) > 0.3 and abs(last[d]) > 0.3 and sign(v) != sign(last[d]):
					extra += 1
				if abs(v) > 0.3:
					last[d] = v
				prev[d] = lab.Angle(0, d)
				var tv = lab.Target(0, d) - target_prev[d]
				if abs(tv) > 0.3 and abs(target_last[d]) > 0.3 and sign(tv) != sign(target_last[d]):
					extra -= 1
				if abs(tv) > 0.3:
					target_last[d] = tv
				target_prev[d] = lab.Target(0, d)
		frames += 240
		solves += lab.SolverFrames(5)
	times.sort()
	var p95 = times[int(times.size() * 0.95)]
	print("full solves %d of %d frames, extra joint reversals %d over %d s, frame ms p50 %.2f p95 %.2f max %.2f" % [
		solves, frames, extra, frames / 60, times[times.size() / 2], p95, times[times.size() - 1]])
	if solves < frames * 0.03 and extra <= 80 and p95 < 4:
		print("PASS gyro survey: the solver is rarely needed, the arm moves smoothly, frames short")
		quit()
	else:
		push_error("The solver runs too often, the arm reverses too much, or frames are slow")
		quit(1)
