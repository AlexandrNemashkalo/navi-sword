extends SceneTree
# Plays the gyro: turns the pad right so the raised sword arm swings out and back behind the
# body, then brings it round again. Reports how far the arm trails the pad, frames where it
# stands still although the pad moves, and how long frames take.
# Run: godot --headless --path . --script res://tests/joint_gyro_behind_test.gd

func _initialize():
	call_deferred("run")

func play(lab, label, frames, pad):
	var rng = RandomNumberGenerator.new()
	rng.seed = 5
	var lag = 0.0
	var frozen = 0
	var moving = 0
	var times = []
	var previous = lab.PlaneFromFront(0)
	for f in frames:
		var p = pad.call(f) + Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3))
		var wanted_before = lab.Target(0, 0)
		lab.DrivePad(p, Vector2.ZERO, 0.0, false)
		var wanted = lab.Target(0, 0)
		var start = Time.get_ticks_usec()
		lab._Process(1.0 / 60)
		times.append((Time.get_ticks_usec() - start) / 1000.0)
		var now = lab.PlaneFromFront(0)
		# Frozen: the pad moved, the arm still has somewhere to go, and it did not move. (Held
		# at a joint's limit while the pad turns further is the limit, not a freeze.)
		if abs(wanted - wanted_before) > 0.3 and abs(lab.ReachablePlane(0) - previous) > 0.5:
			moving += 1
			if abs(now - previous) < 0.01:
				frozen += 1
		previous = now
		lag = max(lag, abs(now - lab.ReachablePlane(0)))
	times.sort()
	var p95 = times[int(times.size() * 0.95)]
	print("%s: worst lag behind the reachable %.1f°, frozen %d of %d moving frames, frame ms median %.2f p95 %.2f max %.2f; plane reached %.0f° (asked %.0f°)" % [
		label, lag, frozen, moving, times[times.size() / 2], p95, times[times.size() - 1], lab.PlaneFromFront(0), lab.Target(0, 0)])
	return [lag, frozen, p95, times[times.size() - 1]]

func still(lab, pad, stick):
	var rng = RandomNumberGenerator.new()
	rng.seed = 2
	lab.CentrePad()
	for i in 200:
		lab.DrivePad(pad, stick, 0.0, false)
		lab._Process(1.0 / 60)
	var previous = []
	for d in 7:
		previous.append(lab.Angle(0, d))
	var worst = 0.0
	for f in 180:
		lab.DrivePad(pad + Vector3(rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3), rng.randf_range(-0.3, 0.3)), stick, 0.0, false)
		lab._Process(1.0 / 60)
		for d in 7:
			worst = max(worst, abs(lab.Angle(0, d) - previous[d]))
			previous[d] = lab.Angle(0, d)
	return worst

func run():
	var lab = load("res://scenes/joint_lab.tscn").instantiate()
	root.add_child(lab)
	lab.set_process(false)
	lab.CentrePad()
	for i in 120:
		lab.DrivePad(Vector3(10, 0, 0), Vector2.ZERO, 0.0, false)
		lab._Process(1.0 / 60)
	# Out and behind: the pad turned 60° right over two seconds, held, and back.
	var turn = func(f): return Vector3(10, 60.0 * smoothstep(0.0, 1.0, min(f / 120.0, 1.0) - max((f - 180) / 120.0, 0.0)), 0)
	var behind = play(lab, "behind the back", 300, turn)
	# Quickly: the same in a third of a second each way.
	var quick = func(f): return Vector3(10, 60.0 * smoothstep(0.0, 1.0, min(f / 20.0, 1.0) - max((f - 40) / 20.0, 0.0)), 0)
	var flick = play(lab, "quick swing back", 90, quick)
	# Held still behind the back, elbow bent, the sensor noisy: the arm must not shake.
	var shake = still(lab, Vector3(-15, 60, 0), Vector2(0, 1))
	print("held still behind the back: biggest joint step %.2f° per frame" % shake)
	var ok = behind[1] <= 5 and flick[1] <= 5 and behind[2] < 12 and flick[2] < 12 and behind[3] < 25 and flick[3] < 25 and shake < 0.3
	if ok:
		print("PASS gyro behind the back: no freezing, no shaking, frames short")
		quit()
	else:
		push_error("Behind the back the arm freezes, shakes or frames are slow")
		quit(1)
